//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in bypass mode: a square wave from the function generator on XIN becomes XT1CLK, and ACLK runs from
//! it. LED1 turns on once the clocks are set up, and ACLK on P1.1 follows the generator's frequency.
//!
//! In bypass mode XT1's oscillator is off, and XIN takes a logic-level clock signal, so no crystal is needed.
//! MCLK (about 8 MHz, from the DCO) and SMCLK (MCLK / 8) don't depend on XT1; they come out on P3.0 and P3.4.
//! (Bypass mode: SLAU445I 3.2.4, p. 103. The input is a square wave with a 40 % to 60 % duty cycle:
//! SLASEC4D Table 5-3, p. 35. Any pin may see –0.3 V to VCC + 0.3 V at most: SLASEC4D 5.1, p. 27, and the
//! LaunchPad's VCC is 3.3 V: SLAU680 2.3.1, p. 12. LED1 on P1.0 is red: SLAU680 Figure 18, p. 26.)
//!
//! This test needs a board change: on the MSP-EXP430FR2355, XIN, P2.7, isn't on the BoosterPack headers but
//! goes only to the 32.768-kHz crystal Q1 and its capacitor C3 (SLAU680 Figure 18, p. 26). Desolder Q1, and
//! solder a wire to the pad of its pin 1, on the XIN side, for the generator. Solder Q1 back afterwards, for
//! the examples that use the crystal.
//!
//! How to test (a board change, the function generator and the scope):
//! 1. Make the board change above.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to the wire on XIN, its ground to GND (J2 pin 20), and switch the output on.
//! 4. Flash this example. Expected: LED1 turns on.
//! 5. Scope on ACLK, P1.1 (J3 pin 28), ground clip on GND (J3 pin 22): its counter shows the generator's
//!    32.768 kHz. With a 10X probe, MCLK on P3.0 (J2 pin 11) shows about 8 MHz, and SMCLK on P3.4 (J1 pin 8)
//!    about 1 MHz.
//! 6. Set the generator to 20 kHz: ACLK follows, while MCLK and SMCLK stay the same.
//! (Header pins: SLAU680 Figure 10, p. 15.)
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
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    let mut led = p1.pin0;

    // Route the internal clocks to pins so they can be measured
    // (MCLK on P3.0 and SMCLK on P3.4, each with P3SEL = 01 and P3DIR = 1: SLASEC4D Table 6-65, p. 100;
    // ACLK on P1.1 with P1SEL = 10 and P1DIR = 1: SLASEC4D Table 6-63, p. 96)
    let _mclk_out = p3.pin0.to_output().to_alternate1();
    let _smclk_out = p3.pin4.to_output().to_alternate1();
    let _aclk_out = p1.pin1.to_output().to_alternate2();

    // Bypass mode only needs XIN; XOUT (P2.6) stays a normal GPIO
    // (SLAU445I 3.2.4, p. 103: "XT1OUT is configured as a general-purpose I/O". XIN is P2.7 with
    // P2SEL = 10: SLASEC4D Table 6-64, p. 98)
    let xin = p2.pin7.to_alternate2();

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
