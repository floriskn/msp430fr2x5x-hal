//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The supply current in LPM4, with the high-side supply voltage supervisor (SVSH) on and off. The device
//! sleeps in LPM4, and each press of a button on P2.3 switches SVSH, then sleeps again. A multimeter in
//! series with the supply shows the difference.
//! (LPM4 at 25 °C and 3 V: 0.64 µA typical with SVS, 0.48 µA without: SLASEE4C 5.7, p. 19. SVSHE turns
//! SVSH off in LPM2 to LPM4: SLAU445I Table 2-2, p. 91. P1.0, P2.0 and P2.3 are GPIO with PxSELx = 00:
//! SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16, p. 60. P2.3 only exists on the 20-pin RHL package:
//! SLASEE4C Table 4-2, p. 14. No board document covers the parts to connect: there is none for the
//! MSP430FR25x2.)
//!
//! After each press, an LED on P1.0 blinks if SVSH is now on, and one on P2.0 if it's off.
//!
//! How to test (multimeter, a 3.3-V supply, two LEDs and resistors, and a push button):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, another from P2.0 to GND, and a
//!    push button from P2.3 to GND (the internal pullup is on).
//! 2. Flash this example. Then disconnect the debugger, so that only the supply stays connected.
//! 3. Set the multimeter to DC current, with its leads in its current jacks (see its manual), and connect it
//!    in series with the 3.3-V supply of the MSP430FR2522. The MSP430FR2522 starts.
//! 4. Read the current: about 0.6 µA, with SVSH on. Press the button: the LED on P2.0 blinks and the
//!    current drops by about 0.16 µA. Press it again: the LED on P1.0 blinks and the current rises again.
//!
//! The current flickers while an LED blinks, so read it a few seconds after.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use msp430::interrupt::Mutex;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV},
    lpm::{request_lpm4_with_interrupts, SvsState},
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

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). No peripheral uses them, so
    // they stop in LPM4.
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);

    // No input may float (SLAU445I 8.3.2, p. 317: "To prevent a floating input and to reduce power
    // consumption"): the button's pin gets its pullup, and `pulldown_unused` gives every other pin its
    // pulldown (pullup and pulldown: PxDIR = 0, PxREN = 1, PxOUT = 1 or 0: SLAU445I Table 8-1, p. 313)
    let p1 = Batch::new(periph.p1).pulldown_unused().split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .pulldown_unused()
        .split(&pmm);

    let mut led_svsh_on = p1.pin0.to_output_low();
    let mut led_svsh_off = p2.pin0.to_output_low();
    let mut button = p2.pin3;
    with(|cs| P2IV.borrow_ref_mut(cs).replace(p2.pxiv));

    // The button pulls P2.3 low, so a press is a high-to-low transition (PxIES = 1: SLAU445I Table 8-16,
    // p. 336; PxIE: SLAU445I Table 8-17, p. 336). Port interrupts wake the device from LPM4 (SLASEE4C
    // Table 6-1, p. 45).
    button.select_falling_edge_trigger().enable_interrupts();

    // SVSH is on after reset (SVSHE = 1: SLAU445I Table 2-2, p. 91)
    let mut svsh = SvsState::Enabled;
    loop {
        // No clock is in use, so this is LPM4 (SLAU445I Table 1-3, p. 39). The PORT2 handler wakes the
        // CPU, and the loop continues. The entry works around errata CS13 and PMM32 (SLAZ705H CS13, p. 7 to
        // p. 8; SLAZ705H PMM32, p. 8 to p. 10).
        request_lpm4_with_interrupts();

        svsh = match svsh {
            SvsState::Enabled => SvsState::Disabled,
            SvsState::Disabled => SvsState::Enabled,
        };
        pmm.set_svsh(svsh);

        let led: &mut dyn OutputPin<Error = _> = match svsh {
            SvsState::Enabled => &mut led_svsh_on,
            SvsState::Disabled => &mut led_svsh_off,
        };
        led.set_high().ok();
        delay.delay_ms(100);
        led.set_low().ok();

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
