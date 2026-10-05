//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! TA0 clocked from its external clock pin, TA0CLK on P1.6: the timer counts the rising edges of a signal
//! from the function generator. An LED on P1.0 toggles every 1000 edges.
//! (TA0CLK is P1.6, and P1.0 is a GPIO output: SLASEE4C Table 6-15, p. 58. No board document covers the
//! parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, an LED and a resistor, and optionally the scope):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Generator: square wave, 1 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load High-Z. Check the
//!    levels on the scope before connecting: a negative or >3.6 V signal can damage the pin.
//! 3. Connect the generator to P1.6, its ground to GND, and switch it on.
//! 4. Flash this example: the LED toggles once per second, on for 1 s and off for 1 s.
//! 5. Change the frequency: the LED follows, toggling every 1000 edges. With the scope on P1.0, the LED's
//!    signal has 1/2000 of the generator's frequency: 50 Hz at 100 kHz, for example. Without a signal, the
//!    LED stops.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    timer::{TimerConfig, TimerParts3},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Edges of the generator's signal per LED toggle
const EDGES: u16 = 1000;

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
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The timer doesn't use them.
    let (_smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // P1.6 is TA0CLK with P1SELx = 10 and P1DIR = 0 (SLASEE4C Table 6-15, p. 58). TASSEL = 00b selects it
    // (SLASEE4C Table 6-8, p. 49; SLAU445I Table 13-4, p. 384).
    let ta0clk = p1.pin6.to_alternate2();
    let mut timer = TimerParts3::new(periph.ta0, TimerConfig::tbclk(ta0clk)).timer;

    // Up mode counts 0 to EDGES - 1, so a period is EDGES edges (SLAU445I 13.2.3.1, p. 371)
    timer.start(EDGES - 1);

    loop {
        // TAIFG is set when the timer counts from TAxCCR0 to zero (SLAU445I 13.2.3.1, p. 371)
        if timer.wait().is_ok() {
            led.toggle().ok();
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
