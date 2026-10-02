#![no_main]
#![no_std]

use msp430_rt::entry;
use msp430_hal::{gpio::Batch, pmm::Pmm, watchdog::Wdt};
use panic_msp430 as _;

// Alternate GPIO mode demonstration

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // Convert P1.7 to its alternate function 1. On this device that is UCA0STE, not SMCLK (P1SELx = 01,
    // SLASEE4C Table 6-15, p. 58). SMCLK is output on P1.2, with P1SELx = 10 and P1DIR = 1
    // (SLASEE4C Table 6-15, p. 58).
    // Expect red LED to light up (no board document covers this LED: there is none for the MSP430FR25x2)
    p1.pin7.to_output().to_alternate1();

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
