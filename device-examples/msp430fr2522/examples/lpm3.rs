//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! LPM3, woken by a timer: the CPU sleeps in LPM3, and once a second TA0's interrupt wakes it to toggle an
//! LED on P1.0.
//!
//! LPM3 stops MCLK, SMCLK and the FLL but keeps ACLK. TA0 counts ACLK from REFO, 32768 Hz, from 0 to 32767
//! in up mode, so it wraps around once a second and its TAIFG interrupt wakes the CPU. The handler is
//! declared with `wake_cpu`, so the CPU stays awake when it returns: the main loop toggles the LED and calls
//! `request_lpm3_with_interrupts()` again. No peripheral uses SMCLK, which would keep the device in LPM0.
//! On this device `request_lpm3_with_interrupts()` also works around the errata CS13 and PMM32, which can
//! lock up the device on its way into LPM3, and a handler returning into LPM3 is such a way in too. That's
//! why the handler wakes the CPU and every sleep starts from the main loop.
//! (LPM3: SLAU445I Table 1-2, p. 39; with SMCLK requested it's LPM0: SLAU445I Table 1-3, p. 39. ACLK and
//! TA0 in LPM3: SLASEE4C Table 6-1, p. 45. Up mode: SLAU445I 13.2.3.1, p. 371. The SR is saved on the stack
//! during an interrupt, and a handler that changes it there returns to a different operating mode:
//! SLAU445I 1.4.2, p. 40. "e.g. during ISR exits": SLAZ705H CS13, p. 8; SLAZ705H PMM32, p. 9. P1.0 is a GPIO
//! output with P1SELx = 00: SLASEE4C Table 6-15, p. 58. No board document covers the LED: there is none
//! for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example. Expected: the LED toggles once a second: on for 1 s, off for 1 s.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::digital::*;
use msp430::interrupt::Mutex;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    lpm::request_lpm3_with_interrupts,
    pmm::Pmm,
    timer::{TBxIV, TimerConfig, TimerParts3, TimerVector},
    watchdog::Wdt,
};
use msp430fr25x2::{interrupt, Ta0};
use panic_msp430 as _;

/// ACLK cycles per toggle of the LED: 1 s, with ACLK from REFO at 32768 Hz (SLASEE4C Table 5-7, p. 27)
const ACLK_CYCLES: u16 = 32_768;

static TA0IV: Mutex<RefCell<Option<TBxIV<Ta0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA0 counts ACLK (TASSEL = 01b: SLASEE4C Table 6-8, p. 49) from 0 to ACLK_CYCLES - 1 in up mode ("The
    // number of timer counts in the period is TAxCCR0 + 1": SLAU445I 13.2.3.1, p. 371), and TAIE requests
    // the interrupt for TAIFG, which is set when it wraps around to 0 (SLAU445I Table 13-4, p. 384)
    let parts = TimerParts3::new(periph.ta0, TimerConfig::aclk(&aclk));
    let mut timer = parts.timer;
    timer.start(ACLK_CYCLES - 1);
    timer.enable_interrupts();
    with(|cs| TA0IV.borrow_ref_mut(cs).replace(parts.tbxiv));

    loop {
        // Set GIE and enter LPM3 in one instruction (SLAU445I 1.4.2, p. 40). The TIMER0_A1 handler wakes
        // the CPU, and the loop continues.
        request_lpm3_with_interrupts();
        led.toggle().ok();
    }
}

// The TA0 vector of CCR1, CCR2 and TAIFG, decoded with TA0IV (FFF6h: SLASEE4C Table 6-2, p. 46). Reading
// TA0IV clears the flag it reports: "Any access, read or write, of the TAxIV register automatically resets
// the highest-pending interrupt flag" (SLAU445I 13.2.6.2, p. 380). `wake_cpu` returns the CPU to active
// mode afterwards (an interrupt returns to another operating mode if its handler changes the SR saved on
// the stack: SLAU445I 1.4, p. 36).
#[interrupt(wake_cpu)]
fn TIMER0_A1() {
    with(|cs| {
        if let Some(ta0iv) = TA0IV.borrow_ref_mut(cs).as_mut() {
            let _: TimerVector = ta0iv.interrupt_vector();
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
