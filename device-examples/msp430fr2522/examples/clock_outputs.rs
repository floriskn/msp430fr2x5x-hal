//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! MCLK, SMCLK and ACLK on pins, to measure the DCO with the scope. MCLK runs from the DCO, locked by the FLL
//! to REFO, SMCLK is MCLK / 8, and ACLK is REFO. An LED on P1.0 lights once the clocks are set up.
//!
//! Change `DCO_FREQ` to check each DCO frequency. Every one except 16 MHz is trimmed in software at
//! start-up, and 16 MHz uses the factory trim: TI recommends the factory trim for the highest DCO range
//! and the software trim for the others (SLAU445I 3.2.11.1, p. 106). REFO is only accurate to ±3.5 %
//! (SLASEE4C Table 5-7, p. 27), and MCLK inherits that, but the FLL locks MCLK to a multiple of REFO,
//! fDCOCLKDIV = (FLLN + 1) × (fFLLREFCLK ÷ n) with n = 1 by default (SLAU445I 3.2.5, p. 104), so
//! MCLK / ACLK is the FLL multiplier:
//!
//! | `DCO_FREQ` | multiplier | nominal MCLK  |
//! |------------|------------|---------------|
//! | `_1MHz`    | 32         | 1.048576 MHz  |
//! | `_2MHz`    | 61         | 1.998848 MHz  |
//! | `_4MHz`    | 122        | 3.997696 MHz  |
//! | `_8MHz`    | 244        | 7.995392 MHz  |
//! | `_12MHz`   | 366        | 11.993088 MHz |
//! | `_16MHz`   | 488        | 15.990784 MHz |
//!
//! Measured on an MSP430FR2476, the FLL can settle MCLK slightly above that: up to about 1 % at 1 MHz,
//! and at most 0.12 % from 8 MHz up.
//! (MCLK on P1.3, SMCLK on P1.2 and ACLK on P1.1, and P1.0 as a GPIO output: SLASEE4C Table 6-15, p. 58.
//! No board document covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, the scope, and optionally the multimeter):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example: the LED lights.
//! 3. With the probe's ground clip on GND, measure with Analysis > Counter. For `_8MHz`:
//!    - P1.3, MCLK: 244 × REFO, 7.995 MHz nominal
//!    - P1.2, SMCLK: MCLK / 8, 999.4 kHz nominal, at the top of the multimeter's frequency range
//!    - P1.1, ACLK: REFO, 32.768 kHz nominal
//! 4. Expected: MCLK / ACLK = 244, even when both are a few percent off nominal.
//! 5. Change `DCO_FREQ`, flash again, and compare with the table.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// DCO frequency to measure
const DCO_FREQ: DcoclkFreqSel = DcoclkFreqSel::_8MHz;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let mut led = p1.pin0;
    led.set_low().ok();

    // MCLK on P1.3, SMCLK on P1.2 and ACLK on P1.1 each need P1SELx = 10 and P1DIR = 1
    // (SLASEE4C Table 6-15, p. 58)
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    let _smclk_out = p1.pin2.to_output().to_alternate2();
    let _aclk_out = p1.pin1.to_output().to_alternate2();

    // MCLK from DCOCLKDIV and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117).
    // SMCLK "directly derives from MCLK", here divided by 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118).
    // MCLK above 8 MHz needs one FRAM wait state (fSYSTEM: SLASEE4C 5.3, p. 17), which freeze() sets.
    let (_smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DCO_FREQ, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    led.set_high().ok();

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
