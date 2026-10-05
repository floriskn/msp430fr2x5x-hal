//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! LPM4, woken by a button: the CPU sleeps in LPM4, and each press of a button on P2.2 wakes it to toggle
//! an LED on P1.0.
//!
//! LPM4 stops every clock, so only an interrupt that needs none can end it, such as a pin's. The button
//! pulls P2.2 low against its internal pullup, and the falling edge requests the port 2 interrupt. Its
//! handler is declared with `wake_cpu`, so the CPU stays awake when it returns: the main loop toggles the
//! LED, waits for the button's bouncing to end, and calls `request_lpm4()` again. No peripheral uses SMCLK
//! or ACLK, which would keep the device in LPM0 or LPM3. On this device `request_lpm4()` also works around
//! the errata CS13 and PMM32, which can lock up the device on its way into LPM4, and a handler returning
//! into LPM4 is such a way in too. That's why the handler wakes the CPU and every sleep starts from the
//! main loop.
//! (LPM4: "CPU and all clocks are disabled": SLAU445I Table 1-2, p. 39; with a clock requested it's LPM3 or
//! LPM0: SLAU445I Table 1-3, p. 39. Only I/O, and CapTIvate, wake it from LPM4: SLASEE4C Table 6-1, p. 45.
//! The SR is saved on the stack during an interrupt, and a handler that changes it there returns to a
//! different operating mode: SLAU445I 1.4.2, p. 40. "e.g. during ISR exits": SLAZ705H CS13, p. 8; SLAZ705H
//! PMM32, p. 9. P1.0 and P2.2 are GPIO with PxSELx = 00: SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16,
//! p. 60. No board document covers the LED or the button: there is none for the MSP430FR25x2.)
//!
//! How to test (an LED, a resistor and a push button):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and a push button from P2.2 to
//!    GND. Both packages have P2.2 (SLASEE4C Table 4-1, p. 12).
//! 2. Flash this example.
//! 3. Expected: the LED is off, and each press of the button toggles it.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV},
    lpm::request_lpm4,
    pmm::Pmm,
    watchdog::Wdt,
};
use msp430fr25x2::{interrupt, P2};
use panic_msp430 as _;

static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The button on P2.2, with the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1,
    // p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0.to_output_low();
    let mut button = p2.pin2;
    with(|cs| P2IV.borrow_ref_mut(cs).replace(p2.pxiv));

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). No peripheral uses them, so
    // they stop in LPM4.
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The button pulls P2.2 low, so a press is a high-to-low transition (PxIES = 1: SLAU445I Table 8-16,
    // p. 336; PxIE: SLAU445I Table 8-17, p. 336). Set GIE, which masks every maskable interrupt while clear
    // (SLAU445I 1.3.3, p. 33).
    button.select_falling_edge_trigger().enable_interrupts();
    unsafe { enable_interrupts() };

    loop {
        // The PORT2 handler wakes the CPU, and the loop continues
        request_lpm4();
        led.toggle().ok();

        // Wait for the button's release and its bouncing to end, then forget the edges it caused
        while button.is_low().unwrap() {}
        delay.delay_ms(100);
        button.clear_ifg();
    }
}

// The port 2 interrupt vector, P2IFG.0 to P2IFG.6 through P2IV (FFE4h: SLASEE4C Table 6-2, p. 46).
// Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
// `wake_cpu` returns the CPU to active mode afterwards (an interrupt returns to another operating mode
// if its handler changes the SR saved on the stack: SLAU445I 1.4, p. 36).
#[interrupt(wake_cpu)]
fn PORT2() {
    with(|cs| {
        if let Some(ref mut p2iv) = *P2IV.borrow_ref_mut(cs) {
            let _: GpioVector = p2iv.get_interrupt_vector();
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
