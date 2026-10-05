//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! LPM3, woken by a timer: the CPU sleeps in LPM3, and once a second TB0's interrupt wakes it to toggle
//! LED1.
//!
//! LPM3 stops MCLK, SMCLK and the FLL but keeps ACLK. TB0 counts ACLK from REFO, 32768 Hz, from 0 to 32767
//! in up mode, so it wraps around once a second and its TBIFG interrupt wakes the CPU. The handler is
//! declared with `wake_cpu`, so the CPU stays awake when it returns: the main loop toggles LED1 and calls
//! `request_lpm3_with_interrupts()` again. No peripheral uses SMCLK, which would keep the device in LPM0.
//! On this device `request_lpm3_with_interrupts()` also works around the errata CS13 and PMM32, which can
//! lock up the device on its way into LPM3, and a handler returning into LPM3 is such a way in too. That's
//! why the handler wakes the CPU and every sleep starts from the main loop.
//! (LPM3: SLAU445I Table 1-2, p. 39; with SMCLK requested it's LPM0: SLAU445I Table 1-3, p. 39. ACLK and
//! TB0 in LPM3: SLASEC4D Table 6-1, p. 61 to p. 62. Up mode: SLAU445I 14.2.3.1, p. 394. The SR is saved
//! on the stack during an interrupt, and a handler that changes it there returns to a different operating
//! mode: SLAU445I 1.4.2, p. 40. "e.g. during ISR exits": SLAZ695J CS13, p. 8; SLAZ695J PMM32, p. 9. LED1
//! on P1.0 is red: SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example.
//! 2. Expected: LED1 toggles once a second: on for 1 s, off for 1 s.
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
use msp430fr2355::{interrupt, Tb0};
use panic_msp430 as _;

/// ACLK cycles per toggle of LED1: 1 s, with ACLK from REFO at 32768 Hz (SLASEC4D Table 5-7, p. 40)
const ACLK_CYCLES: u16 = 32_768;

static TB0IV: Mutex<RefCell<Option<TBxIV<Tb0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TB0 counts ACLK (TBSSEL = 01b: SLASEC4D Table 6-9, p. 68) from 0 to ACLK_CYCLES - 1 in up mode ("The
    // number of timer counts in the period is TBxCL0 + 1": SLAU445I 14.2.3.1, p. 394), and TBIE requests
    // the interrupt for TBIFG, which is set when it wraps around to 0 (SLAU445I Table 14-6, p. 410)
    let parts = TimerParts3::new(periph.tb0, TimerConfig::aclk(&aclk));
    let mut timer = parts.timer;
    timer.start(ACLK_CYCLES - 1);
    timer.enable_interrupts();
    with(|cs| TB0IV.borrow_ref_mut(cs).replace(parts.tbxiv));

    loop {
        // Set GIE and enter LPM3 in one instruction (SLAU445I 1.4.2, p. 40). The TIMER0_B1 handler wakes
        // the CPU, and the loop continues.
        request_lpm3_with_interrupts();
        led1.toggle().ok();
    }
}

// The TB0 vector of CCR1, CCR2 and TBIFG, decoded with TB0IV (FFF6h: SLASEC4D Table 6-2, p. 63). Reading
// TB0IV clears the flag it reports: "Any access, read or write, of the TBxIV register automatically resets
// the highest-pending interrupt flag" (SLAU445I 14.2.6.2, p. 405). `wake_cpu` returns the CPU to active
// mode afterwards (an interrupt returns to another operating mode if its handler changes the SR saved on
// the stack: SLAU445I 1.4, p. 36).
#[interrupt(wake_cpu)]
fn TIMER0_B1() {
    with(|cs| {
        if let Some(tb0iv) = TB0IV.borrow_ref_mut(cs).as_mut() {
            let _: TimerVector = tb0iv.interrupt_vector();
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
