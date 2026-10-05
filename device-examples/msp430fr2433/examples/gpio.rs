//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A GPIO input and output: LED2 lights while S2 is held down.
//!
//! The program polls P2.7, which S2 pulls low, and drives P1.1, LED2, high while it reads low.
//! (S2 on P2.7, with no pull-up on the board, and LED2 on P1.1 is green: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example.
//! 2. Expected: LED2 is off, and lights while S2 is pressed.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{gpio::Batch, pmm::Pmm, watchdog::Wdt};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    let _wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // P2.7 is an input with the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1,
    // p. 313), so it reads high until S2 is pressed
    let p2 = Batch::new(periph.p2)
        .config_pin7(|p| p.pullup())
        .split(&pmm);
    let p1 = Batch::new(periph.p1)
        .config_pin1(|p| p.to_output())
        .split(&pmm);

    let mut p2_7 = p2.pin7;
    let mut led2 = p1.pin1;

    loop {
        if p2_7.is_high().unwrap() {
            led2.set_low().ok();
        } else {
            led2.set_high().ok();
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
