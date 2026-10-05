//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in crystal mode, with the LaunchPad's 32.768-kHz crystal: ACLK runs from it, LED1 turns on once it
//! has started, and the red part of LED2 lights while XT1 has a fault. The crystal isn't connected as
//! shipped, so this needs a board change.
//!
//! In crystal mode XT1's oscillator drives the crystal on XIN and XOUT. `freeze()` waits until it runs
//! without a fault, about a second, and then LED1 turns on. ACLK comes out on P2.2, so the scope can count
//! the crystal's frequency. The main loop keeps clearing XT1's sticky fault flag and shows on LED2 whether
//! it comes back. The constants at the top set XT1's options.
//! (Crystal mode: SLAU445I 3.2.4, p. 103. Start-up time: 1000 ms typical, with the start counter's 1024
//! cycles: SLASEO7C 8.12.3.1, p. 27. The fault flag: SLAU445I 3.2.13, p. 109. The crystal is a 32.768-kHz,
//! 12.5-pF one: Q1 in SLAU802 2.5, p. 12, Y1 in the schematic, with the 12-pF capacitors C3 and C4: SLAU802
//! Figure 18, p. 24. LED1 on P1.0 is green, and the red part of LED2 is P5.1: SLAU802 Figure 19, p. 25.)
//!
//! How to test (a board change, and the scope):
//! 1. Connect the crystal. XIN (P2.1) reaches Y1 through R2 and XOUT (P2.0) through R3, which are not
//!    fitted, and the 0-Ω R1 and R4 connect the two pins to J2 pins 18 and 17 instead (SLAU802 Figure 18,
//!    p. 24). Fit 0-Ω resistors or solder bridges at R2 and R3, and remove R1 and R4, so the header's
//!    traces don't load the crystal ("Keep the trace between the device and the crystal as short as
//!    possible": SLASEO7C 8.12.3.1, note 1, p. 27). The XT1 bypass examples feed XIN from J2 pin 18: undo
//!    the change for them.
//! 2. Flash this example. Expected: LED1 turns on about a second after the start, and LED2 stays off.
//! 3. Scope on ACLK, P2.2 (J1 pin 5), ground clip on GND (J3 pin 22): its counter shows 32.768 kHz, as
//!    exact as the crystal. While XT1 has a fault, ACLK runs from REFO instead, which is only accurate to
//!    ±3.5 % (SLAU445I 3.2.13, p. 109; SLASEO7C 8.12.3.4, p. 30).
//! 4. Optional: change one constant and flash again. With `DISABLE_AGC`, `DISABLE_START_COUNTER` or
//!    `DISABLE_AUTO_OFF` set, steps 2 and 3 should look the same: without the start counter `freeze()`
//!    returns 1024 cycles (31 ms) sooner, and auto-off only acts while no clock uses XT1, and ACLK does. A
//!    lower `DRIVE` draws less current but suits crystals with a smaller load (SLAU445I Table 3-10, p. 119;
//!    SLASEO7C 8.12.3.1, note 5, p. 27), so with this crystal LED2 may report faults.
//! (Header pins: SLAU802 Figure 10, p. 13.)
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

/// The crystal's frequency (SLAU802 2.5, p. 12)
const XT1_FREQ_HZ: u32 = 32_768;
/// The drive strength once XT1 runs, as it starts at the highest (XT1DRIVE: SLAU445I 3.2.4, p. 103;
/// SLAU445I Table 3-10, p. 119). The highest, 3, suits the 12.5-pF crystal (SLASEO7C 8.12.3.1, note 5,
/// p. 27).
const DRIVE: Xt1Drive = Xt1Drive::Xt1drive3;
/// Switch the automatic gain control off (XT1AGCOFF = 1: SLAU445I Table 3-10, p. 120)
const DISABLE_AGC: bool = false;
/// Don't wait for the start counter's 1024 cycles of XT1 (ENSTFCNT1 = 0: SLAU445I Table 3-11, p. 121;
/// SLASEO7C 8.12.3.1, note 9, p. 27)
const DISABLE_START_COUNTER: bool = false;
/// Keep XT1 on even while no clock uses it (XT1AUTOOFF = 0: SLAU445I Table 3-10, p. 120)
const DISABLE_AUTO_OFF: bool = false;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p5 = Batch::new(periph.p5)
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2_red = p5.pin1;
    led1.set_low().ok();
    led2_red.set_low().ok();

    // XIN on P2.1 and XOUT on P2.0, each with P2SEL = 01, and ACLK on P2.2 with P2SEL = 10 and P2DIR = 1
    // (SLASEO7C Table 9-24, p. 66)
    let xin = p2.pin1.to_alternate1();
    let xout = p2.pin0.to_alternate1();
    let _aclk_out = p2.pin2.to_output().to_alternate2();

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
    led1.set_high().ok();

    loop {
        // The fault flag is sticky, so clear it and see whether it comes straight back (SLAU445I 3.2.13,
        // p. 109. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10, p. 63.)
        xt1clk.clear_fault();
        led2_red.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
