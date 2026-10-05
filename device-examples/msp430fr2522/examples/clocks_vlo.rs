//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! MCLK from the VLO, the internal very-low-power oscillator, with SMCLK switched off: the CPU runs at
//! about 10 kHz, and an LED on P1.0 blinks slowly, timed by a delay loop.
//!
//! `mclk_vloclk` makes VLOCLK the source of MCLK, and `smclk_off` stops SMCLK, which always comes from the
//! same source as MCLK. MCLK comes out on P1.3 and SMCLK on P1.2, for the scope. The delay counts MCLK
//! cycles for the VLO's typical 10 kHz, but the VLO is only accurate to ±50 %, and at so slow a clock the
//! delay loop's own instructions count too: in a debug build `delay_ms(500)` lasts about 1.6 s at 10 kHz.
//! (SELMS = 011b selects VLOCLK: SLAU445I Table 3-8, p. 117. "SMCLK is derived from MCLK and always uses
//! the same clock source as MCLK": SLAU445I 3.2.1, p. 102. SMCLKOFF: SLAU445I Table 3-9, p. 118. VLOCLK
//! "10 kHz ±50%": SLASEE4C Table 6-8, p. 49. MCLK on P1.3 and SMCLK on P1.2 with P1SEL = 10 and P1DIR = 1,
//! and P1.0 a GPIO output with P1SELx = 00: SLASEE4C Table 6-15, p. 58. No board document covers the LED:
//! there is none for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, and optionally the scope):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example. Expected: the LED toggles slowly, about every 2 s.
//! 3. Scope on MCLK, P1.3, ground clip on GND: its counter shows the VLO's frequency, about 10 kHz. SMCLK,
//!    on P1.2, shows no clock: it's off.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, MclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led = p1.pin0.to_output_low();

    // MCLK on P1.3 and SMCLK on P1.2, each with P1SEL = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    let _smclk_out = p1.pin2.to_output().to_alternate2();

    // MCLK from VLOCLK, undivided (SELMS = 011b: SLAU445I Table 3-8, p. 117; DIVM = 000b: SLAU445I
    // Table 3-9, p. 118), SMCLK off (SMCLKOFF = 1: SLAU445I Table 3-9, p. 118), and ACLK from REFO, as
    // after reset (SELA = 01b: SLAU445I Table 3-8, p. 117). The delay counts MCLK cycles at 10 kHz.
    let (_aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_vloclk(MclkDiv::_1)
        .smclk_off()
        .freeze(&mut fram);

    loop {
        led.toggle().ok();
        delay.delay_ms(500);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
