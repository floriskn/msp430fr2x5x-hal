//! Why did the device reset? `Pmm::take_reset_cause` reads the reasons, highest priority first
//! (SYSRSTIV: SLAU445I 1.3.7, p. 36; values and priorities: SLASEO7C Table 9-10, p. 52),
//! and RGB LED2 shows the first one:
//!
//! | LED2                   | Reset                                                    |
//! |------------------------|----------------------------------------------------------|
//! | green                  | power-up: plug in the USB cable (brownout reset)         |
//! | blue                   | the reset button S3                                      |
//! | red                    | button S1, which calls `Pmm::software_bor()`             |
//! | yellow (red and green) | button S2, which calls `Pmm::software_por()`             |
//! | white                  | any other reason                                         |
//! | off, with LED1 on      | no reason: the debugger started the program after flashing it |
//!
//! The buttons reset the device when they are released.
//!
//! LED1 (P1.0) is green, LED2 is red on P5.1, green on P5.0 and blue on P4.7; S1 (P4.0) and S2 (P2.3)
//! pull their pins low, and S3 is the reset button on RST (SLAU802 Figure 19, p. 25). The resets:
//! brownout 02h, RST/NMI pin 04h, software BOR 06h, software POR 14h (SLASEO7C Table 9-10, p. 52).
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    gpio::Batch,
    pmm::{Pmm, ResetCause},
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);

    // Read the first reason, then the rest, which also clears them for the next reset
    // (reading SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.pullup())
        .config_pin7(|p| p.to_output())
        .split(&pmm);
    let p5 = Batch::new(periph.p5)
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut s1 = p4.pin0;
    let mut s2 = p2.pin3;
    let mut green = p5.pin0;
    let mut red = p5.pin1;
    let mut blue = p4.pin7;

    let (r, g, b) = match first {
        Some(ResetCause::Brownout) => (false, true, false),
        Some(ResetCause::ResetPin) => (false, false, true),
        Some(ResetCause::SoftwareBor) => (true, false, false),
        Some(ResetCause::SoftwarePor) => (true, true, false),
        Some(_) => (true, true, true),
        None => (false, false, false),
    };
    led1.set_state(first.is_none().into()).ok();
    red.set_state(r.into()).ok();
    green.set_state(g.into()).ok();
    blue.set_state(b.into()).ok();

    loop {
        if s1.is_low().unwrap() {
            while s1.is_low().unwrap() {}
            pmm.software_bor();
        }
        if s2.is_low().unwrap() {
            while s2.is_low().unwrap() {}
            pmm.software_por();
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
