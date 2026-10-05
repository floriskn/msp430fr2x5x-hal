//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 as the FLL reference: the FLL locks the DCO to the generator's signal on XIN instead of REFO, so MCLK
//! on P1.3 runs at 244 times the generator's frequency and follows it.
//!
//! An LED on P1.0 shows an XT1 fault, which makes the FLL fall back to REFO, and an LED on P2.2 shows that
//! the FLL is unlocked: the DCO can't follow the reference any more. ACLK on P1.1 runs from REFO, as a fixed
//! reference. The HAL sets no FRAM wait states for this 7.995 MHz MCLK, so the generator must not go above
//! 32.768 kHz: MCLK would pass the 8 MHz that FRAM allows without them.
//! (SELREF picks XT1CLK or REFOCLK as the FLL reference, and fDCOCLKDIV = (FLLN + 1) × (fFLLREFCLK ÷ n):
//! SLAU445I 3.2.5, p. 104. FLLUNLOCK: SLAU445I 3.2.9, p. 105. An XT1 fault switches the FLL reference to
//! REFO: SLAU445I 3.2.13, p. 109. MCLK up to 8 MHz without FRAM wait states: SLASEE4C 5.3, p. 17. XIN is
//! P2.1: SLASEE4C Table 6-16, p. 60. ACLK, SMCLK and MCLK are P1.1, P1.2 and P1.3: SLASEE4C Table 6-15,
//! p. 58. No board document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator and the scope, two LEDs and resistors):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and another from P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on.
//! 4. Flash this example, and put the scope (10X probe) on MCLK, P1.3, ground clip on GND. Expected:
//!    244 × 32.768 kHz = 7.995 MHz, and both LEDs are off. SMCLK on P1.2 is MCLK / 8, 999.4 kHz, which is
//!    easier to measure precisely.
//! 5. Lower the generator's frequency: MCLK follows, 7.32 MHz at 30 kHz and 6.59 MHz at 27 kHz. Only go down,
//!    never above 32.768 kHz.
//! 6. Keep lowering it until the LED on P2.2 lights: the DCO can't go any lower, and the FLL is unlocked.
//! 7. Set the generator back to 32.768 kHz, and switch its output off: the LED on P1.0 turns on, and MCLK
//!    runs at about 8 MHz from REFO. Switch the output back on: the LED on P1.0 turns off, and MCLK is
//!    7.995 MHz again.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{fll_status, ClockConfig, DcoclkFreqSel, FllStatus, MclkDiv, SmclkDiv, Xt1Config},
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
    // The LEDs on P1.0 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.to_output())
        .split(&pmm);
    let mut led_fault = p1.pin0;
    let mut led_unlocked = p2.pin2;

    // ACLK on P1.1, SMCLK on P1.2 and MCLK on P1.3, each with P1SELx = 10 and P1DIR = 1 (SLASEE4C
    // Table 6-15, p. 58); XIN on P2.1 with P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let _smclk_out = p1.pin2.to_output().to_alternate2();
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from REFO (SELA = 01b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in bypass mode
    // (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120); XT1CLK as the FLL reference (SELREF = 00b:
    // SLAU445I Table 3-7, p. 116)
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .fll_ref_xt1()
        .aclk_refoclk()
        .freeze(&mut fram);

    loop {
        // Clearing the sticky fault flag also lets the FLL move back to XT1 once the signal is
        // healthy again (SLAU445I 3.2.13, p. 110)
        xt1clk.clear_fault();
        led_fault.set_state(xt1clk.is_faulted().into()).ok();
        // The FLL reports the DCO as too fast, too slow or out of range (CSCTL7.FLLUNLOCK: SLAU445I
        // Table 3-11, p. 121)
        led_unlocked.set_state((fll_status() != FllStatus::Locked).into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
