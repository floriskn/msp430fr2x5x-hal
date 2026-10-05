//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! REFO in its low-power mode, `ClockConfig::refo_low_power()`: the board sleeps in LPM3 with ACLK from
//! REFO, and a multimeter shows how much less current REFO draws in that mode.
//!
//! LPM3 keeps ACLK running, and with it REFO, its source. REFO draws 15 µA in its normal mode and 1 µA in
//! its low-power mode, which only the enhanced clock system has. After a reset the example reads S2:
//! released, it puts REFO in its low-power mode and lights LED2 for a second; held, REFO stays in its
//! normal mode and LED1 lights. Then the CPU sleeps in LPM3 for good, as no interrupt is enabled to wake
//! it.
//! (LPM3 keeps ACLK: SLAU445I Table 1-2, p. 39. REFO runs while it's ACLK's source and ACLK is active:
//! SLAU445I 3.2.3, p. 103. REFOLP: SLAU445I Table 3-7, p. 116. REFO's current at 25 °C and 3 V: SLASEC4D
//! Table 5-7, p. 40. The enhanced clock system: SLAU445I Table 3-1, p. 98. LED1 on P1.0 is red, LED2 on
//! P6.6 green, and S2 on P2.3 connects the pin to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test (multimeter, following SLAU680 2.4, p. 12):
//! 1. Flash this example.
//! 2. On J101, remove the TXD, RXD, SBW RST, SBW TST and 3V3 jumpers. Keep GND. The board is now off (the
//!    jumpers: SLAU680 Table 2, p. 10).
//! 3. Set the multimeter to DC current, with its leads in its current jacks (see its manual), and hold
//!    the leads on the two 3V3 pins of J101. The board starts, and LED2 lights for a second.
//! 4. Read the current a few seconds after LED2 went off.
//! 5. Hold S2, press S3 (reset), and let go of S2: LED1 lights for a second. Now REFO runs in its normal
//!    mode, and the current is about 14 µA higher.
//! 6. Press S3 alone: LED2 lights, and the current drops again.
//!
//! Put the jumpers back afterwards: without them the board can't be flashed.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    lpm::request_lpm3,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    // "Make sure there are no floating inputs/outputs" (SLAU680 2.4, p. 13): S2 (P2.3) gets its pullup, and
    // `pulldown_unused` gives every other pin its pulldown (SLAU445I 8.3.2, p. 317; pullup and pulldown:
    // PxDIR = 0, PxREN = 1, PxOUT = 1 or 0: SLAU445I Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .pulldown_unused()
        .split(&pmm);
    let p1 = Batch::new(periph.p1).pulldown_unused().split(&pmm);
    let _p3 = Batch::new(periph.p3).pulldown_unused().split(&pmm);
    let _p4 = Batch::new(periph.p4).pulldown_unused().split(&pmm);
    let _p5 = Batch::new(periph.p5).pulldown_unused().split(&pmm);
    let p6 = Batch::new(periph.p6).pulldown_unused().split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p6.pin6.to_output_low();
    let mut s2 = p2.pin3;

    // S2 pulls P2.3 low while it's held
    let low_power = s2.is_high().unwrap();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118), with REFO in its low-power mode
    // (REFOLP = 1: SLAU445I Table 3-7, p. 116) unless S2 is held
    let clocks = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk();
    let clocks = if low_power { clocks.refo_low_power() } else { clocks };
    let (_smclk, _aclk, mut delay) = clocks.freeze(&mut fram);

    // Show the mode for a second
    if low_power {
        led2.set_high().ok();
    } else {
        led1.set_high().ok();
    }
    delay.delay_ms(1000);
    led1.set_low().ok();
    led2.set_low().ok();

    loop {
        // No interrupt is enabled, so the CPU stays in LPM3, where only ACLK, from REFO, keeps running. No
        // peripheral uses SMCLK, which would keep the device in LPM0 (SLAU445I Table 1-3, p. 39).
        request_lpm3();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
