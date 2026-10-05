//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in crystal mode, with a 32.768-kHz crystal on XIN and XOUT: ACLK runs from it, an LED on P1.0 turns
//! on once it has started, and an LED on P2.2 lights while XT1 has a fault.
//!
//! In crystal mode XT1's oscillator drives the crystal on XIN and XOUT. `freeze()` waits until it runs
//! without a fault, about a second, and then the LED on P1.0 turns on. ACLK comes out on P1.1, so the scope
//! can count the crystal's frequency. The main loop keeps clearing XT1's sticky fault flag and shows on the
//! LED on P2.2 whether it comes back. The constants at the top set XT1's options.
//! (Crystal mode: SLAU445I 3.2.4, p. 103. Start-up time: 1000 ms typical, with the start counter's 1024
//! cycles: SLASEE4C Table 5-4, p. 25. The fault flag: SLAU445I 3.2.13, p. 109. XIN on P2.1 and XOUT on P2.0
//! with P2SEL = 10, and P2.2 a GPIO output with P2SELx = 00: SLASEE4C Table 6-16, p. 60. ACLK on P1.1 with
//! P1SEL = 10, and P1.0 a GPIO output with P1SELx = 00: SLASEE4C Table 6-15, p. 58. No board document
//! covers the crystal or the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (a 32.768-kHz crystal, two capacitors, two LEDs and resistors, and the scope):
//! 1. Connect a 32.768-kHz watch crystal from XIN (P2.1) to XOUT (P2.0), with short leads, and a capacitor
//!    from each of the two pins to GND, of the size the crystal's data sheet asks for (SLASEE4C Table 5-4,
//!    notes 1 and 7, p. 25). `DRIVE` below suits a 12.5-pF crystal (note 5).
//! 2. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another from P2.2 to GND.
//!    Both packages have P2.2 (SLASEE4C Table 4-1, p. 12).
//! 3. Flash this example. Expected: the LED on P1.0 turns on about a second after the start, and the LED
//!    on P2.2 stays off.
//! 4. Scope on ACLK, P1.1, ground clip on GND: its counter shows 32.768 kHz, as exact as the crystal.
//!    While XT1 has a fault, ACLK runs from REFO instead, which is only accurate to ±3.5 % (SLAU445I 3.2.13,
//!    p. 109; SLASEE4C Table 5-7, p. 27).
//! 5. Optional: change one constant and flash again. With `DISABLE_AGC`, `DISABLE_START_COUNTER` or
//!    `DISABLE_AUTO_OFF` set, steps 3 and 4 should look the same: without the start counter `freeze()`
//!    returns 1024 cycles (31 ms) sooner, and auto-off only acts while no clock uses XT1, and ACLK does. A
//!    lower `DRIVE` draws less current but suits crystals with a smaller load (SLAU445I Table 3-10, p. 119;
//!    SLASEE4C Table 5-4, note 5, p. 25), so with a 12.5-pF crystal the LED on P2.2 may report faults.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config, Xt1Drive},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The crystal's frequency (SLASEE4C Table 5-4, p. 25)
const XT1_FREQ_HZ: u32 = 32_768;
/// The drive strength once XT1 runs, as it starts at the highest (XT1DRIVE: SLAU445I 3.2.4, p. 103;
/// SLAU445I Table 3-10, p. 119). The highest, 3, suits a 12.5-pF crystal (SLASEE4C Table 5-4, note 5,
/// p. 25).
const DRIVE: Xt1Drive = Xt1Drive::Xt1drive3;
/// Switch the automatic gain control off (XT1AGCOFF = 1: SLAU445I Table 3-10, p. 120)
const DISABLE_AGC: bool = false;
/// Don't wait for the start counter's 1024 cycles of XT1 (ENSTFCNT1 = 0: SLAU445I Table 3-11, p. 121;
/// SLASEE4C Table 5-4, note 8, p. 25)
const DISABLE_START_COUNTER: bool = false;
/// Keep XT1 on even while no clock uses it (XT1AUTOOFF = 0: SLAU445I Table 3-10, p. 120)
const DISABLE_AUTO_OFF: bool = false;

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
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.to_output())
        .split(&pmm);
    let mut led_started = p1.pin0;
    let mut led_fault = p2.pin2;
    led_started.set_low().ok();
    led_fault.set_low().ok();

    // XIN on P2.1 and XOUT on P2.0, each with P2SEL = 10 (SLASEE4C Table 6-16, p. 60), and ACLK on P1.1
    // with P1SEL = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let xin = p2.pin1.to_alternate2();
    let xout = p2.pin0.to_alternate2();
    let _aclk_out = p1.pin1.to_output().to_alternate2();

    // XT1 in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120), with the options above
    let xt1 = Xt1Config::crystal(XT1_FREQ_HZ, xin, xout).with_drive(DRIVE);
    let xt1 = if DISABLE_AGC { xt1.disable_agc() } else { xt1 };
    let xt1 = if DISABLE_START_COUNTER { xt1.disable_start_counter() } else { xt1 };
    let xt1 = if DISABLE_AUTO_OFF { xt1.disable_auto_off() } else { xt1 };

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117). `freeze()` returns once XT1 runs without a fault.
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(xt1)
        .aclk_xt1clk()
        .freeze(&mut fram);
    led_started.set_high().ok();

    loop {
        // The fault flag is sticky, so clear it and see whether it comes straight back (SLAU445I 3.2.13,
        // p. 109. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10, p. 63.)
        xt1clk.clear_fault();
        led_fault.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
