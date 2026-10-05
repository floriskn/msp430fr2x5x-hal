//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A count of the starts, kept in program FRAM: `Fram::set_writable_program_fram` leaves the first KiB of
//! program FRAM writable, and each start adds 1 to a count in its first word. An LED on P1.0 shows whether
//! the count is odd, so it toggles at every reset and power-up.
//!
//! After every reset the program FRAM write protection (PFWP) covers all of the program FRAM. FRWPOA moves
//! the start of the protection up in steps of 1 KiB, and the part below it "can be used like RAM". FRAM
//! keeps its contents without power. The program itself must not be in that KiB, so `memory.x` must leave
//! it out, as below. The program checks this: the reset vector holds the address it starts at, the start of
//! ROM, as msp430-rt's `link.x` puts the reset handler first. If that is in the KiB, the program leaves the
//! count alone and lights an LED on P1.1.
//! (FRWPOA: SLAU445I 1.12.4.1, p. 53; SLAU445I Table 1-29, p. 80. PFWP: SLAU445I 1.9.3, p. 45. Program
//! FRAM, E300h to FFFFh: SLASEE4C Table 6-19, p. 62. FRAM is nonvolatile: SLAU445I 6.1, p. 301. The reset
//! vector at FFFEh: SLAU445I 1.2.1, p. 32. A low level on RST/NMI resets the device: SLAU445I 1.2, p. 30.
//! P1.0 and P1.1 are GPIO with P1SELx = 00: SLASEE4C Table 6-15, p. 58. No board document covers the LEDs:
//! there is none for the MSP430FR25x2.)
//!
//! Before building this example, change `ROM` in `memory.x` from `ORIGIN = 0xE300, LENGTH = 0x1C80` to
//! `ORIGIN = 0xE700, LENGTH = 0x1880`, so the program starts after the first KiB, E300h to E6FFh. Cargo
//! doesn't notice changes to `memory.x`, so clean the examples after the change, or they keep the old
//! layout: `cargo clean -p msp430fr25x2-hal-examples --target msp430-none-elf`. Change it back afterwards:
//! not every example fits in the smaller ROM.
//!
//! How to test (two LEDs and two resistors):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another from P1.1 to GND.
//! 2. Change `memory.x` and clean the examples as above, then flash this example. Expected: the LED on P1.1
//!    is off, and the LED on P1.0 is on or off, depending on what the first word of FRAM held before.
//! 3. Reset the device, with RST/NMI low for a moment. Expected: the LED on P1.0 toggles. It toggles at
//!    every reset.
//! 4. Switch the power off and on again. Expected: the LED on P1.0 toggles again: the count kept its value
//!    without power.
//! 5. Without the change to `memory.x`, the LED on P1.1 lights instead, and the one on P1.0 stays off.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{fram::Fram, gpio::Batch, pmm::Pmm, watchdog::Wdt};
use panic_msp430 as _;

/// The count: the first word of program FRAM (SLASEE4C Table 6-19, p. 62)
const COUNT: *mut u16 = 0xE300 as *mut u16;
/// The end of the first KiB of program FRAM, which FRWPOA = 1 leaves writable (SLAU445I Table 1-29, p. 80)
const WRITABLE_END: u16 = 0xE700;
/// The reset vector, which holds the address the program starts at (SLAU445I 1.2.1, p. 32)
const RESET_VECTOR: *const u16 = 0xFFFE as *const u16;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p1.pin1;
    led1.set_low().ok();
    led2.set_low().ok();

    // Without the change to memory.x, the program starts at E300h, in the KiB, and writing the count
    // would change its first instruction
    if unsafe { RESET_VECTOR.read_volatile() } < WRITABLE_END {
        led2.set_high().ok();
        loop {}
    }

    // FRWPOA = 1: E300h to E6FFh are writable, and PFWP protects the rest (SLAU445I Table 1-29, p. 80)
    fram.set_writable_program_fram(1);
    let count = unsafe { COUNT.read_volatile() }.wrapping_add(1);
    unsafe { COUNT.write_volatile(count) };

    led1.set_state((count % 2 == 1).into()).ok();
    loop {}
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
