//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in bypass mode: a square wave from the function generator on XIN becomes XT1CLK, and ACLK runs from
//! it. An LED on P1.0 turns on once the clocks are set up, and ACLK on P1.1 follows the generator's
//! frequency.
//!
//! In bypass mode XT1's oscillator is off, and XIN takes a logic-level clock signal, so no crystal is needed.
//! MCLK (about 8 MHz, from the DCO) and SMCLK (MCLK / 8) don't depend on XT1; they come out on P1.3 and P1.2.
//! (Bypass mode: SLAU445I 3.2.4, p. 103. The input is a square wave with a 40 % to 60 % duty cycle:
//! SLASEE4C Table 5-4, p. 25. Any pin may see –0.3 V to VCC + 0.3 V at most: SLASEE4C 5.1, p. 17. XIN is
//! P2.1: SLASEE4C Table 6-16, p. 60. ACLK, SMCLK and MCLK are P1.1, P1.2 and P1.3: SLASEE4C Table 6-15,
//! p. 58. No board document covers the LED: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator and the scope, an LED and a resistor):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on.
//! 4. Flash this example. Expected: the LED turns on.
//! 5. Scope on ACLK, P1.1, ground clip on GND: its counter shows the generator's 32.768 kHz. With a 10X
//!    probe, MCLK on P1.3 shows about 8 MHz, and SMCLK on P1.2 about 1 MHz.
//! 6. Set the generator to 20 kHz: ACLK follows, while MCLK and SMCLK stay the same.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let mut led = p1.pin0;

    // Route the internal clocks to pins so they can be measured
    // (ACLK on P1.1, SMCLK on P1.2 and MCLK on P1.3, each with P1SELx = 10 and P1DIR = 1:
    // SLASEE4C Table 6-15, p. 58)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let _smclk_out = p1.pin2.to_output().to_alternate2();
    let _mclk_out = p1.pin3.to_output().to_alternate2();

    // Bypass mode only needs XIN; XOUT (P2.0) stays a normal GPIO
    // (SLAU445I 3.2.4, p. 103: "XT1OUT is configured as a general-purpose I/O". XIN is P2.1 with
    // P2SELx = 10: SLASEE4C Table 6-16, p. 60)
    let xin = p2.pin1.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in bypass mode
    // (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, _xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
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
