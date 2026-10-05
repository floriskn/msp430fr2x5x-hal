//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! TA1 clocked from its external clock pin, TA1CLK on P1.6: the timer counts the rising edges of a signal
//! from the function generator. LED1 toggles every 1000 edges.
//! (TA1CLK is P1.6: SLASE59F Table 6-12, p. 51. TA0CLK, the other clock pin, is P1.0, LED1's pin:
//! SLASE59F Table 6-11, p. 50. LED1 on P1.0 is red: SLAU739 Figure 18, p. 23.)
//!
//! How to test (function generator, and optionally the scope):
//! 1. Generator: square wave, 1 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load High-Z. Check the
//!    levels on the scope before connecting: a negative or >3.6 V signal can damage the pin.
//! 2. Connect the generator to P1.6 (J1 pin 5), its ground to GND (J2 pin 20), and switch it on.
//! 3. Flash this example: LED1 toggles once per second, on for 1 s and off for 1 s.
//! 4. Change the frequency: LED1 follows, toggling every 1000 edges. With the scope on P1.0 (J1 pin 2),
//!    the LED's signal has 1/2000 of the generator's frequency: 50 Hz at 100 kHz, for example. Without
//!    a signal, the LED stops.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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

/// Edges of the generator's signal per LED1 toggle
const EDGES: u16 = 1000;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The timer doesn't use them.
    let (_smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // P1.6 is TA1CLK with P1SELx = 10 and P1DIR = 0 (SLASE59F Table 6-17, p. 55). TASSEL = 00b selects it
    // (SLAU445I Table 13-4, p. 384; SLASE59F Table 6-7, p. 46), and the timer counts on its rising edges
    // (SLAU445I 13.2.1, p. 370).
    let ta1clk = p1.pin6.to_alternate2();
    let mut timer = TimerParts3::new(periph.ta1, TimerConfig::tbclk(ta1clk)).timer;

    // Up mode counts 0 to EDGES - 1, so a period is EDGES edges (SLAU445I 13.2.3.1, p. 371)
    timer.start(EDGES - 1);

    loop {
        // TAIFG is set when the timer counts from TAxCCR0 to zero (SLAU445I 13.2.3.1, p. 371)
        if timer.wait().is_ok() {
            led1.toggle().ok();
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
