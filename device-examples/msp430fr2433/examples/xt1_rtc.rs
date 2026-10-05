//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The RTC clocked from XT1CLK: it counts 16384 cycles of the generator's signal on XIN between toggles of
//! LED1, so at 32.768 kHz LED1 toggles every 0.5 s.
//!
//! The fail-safe moves ACLK, MCLK, SMCLK and the FLL reference off a failed XT1, but not the RTC's XT1CLK
//! input: without a signal the RTC stops. The MSP430FR2476 version of this example also clocks the RTC
//! from ACLK; this device can't, as its RTCSS = 01b selects SMCLK.
//! (RTCSS = 10b selects XT1CLK and 01b SMCLK: SLASE59F Table 6-7, p. 46. SLAU445I 3.2.13, p. 109 to p. 110
//! describes the switch to REFO for MCLK, SMCLK, ACLK and the FLL reference only. A peripheral that
//! requests XT1 keeps it enabled: SLAU445I 3.2.4, p. 103. LED1 on P1.0 is red: SLAU739 Figure 18, p. 23.)
//!
//! How to test (function generator, and optionally the scope):
//! 1. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 2. Connect it to XIN, P2.1 (J2 pin 12), its ground to GND (J2 pin 20), and switch the output on.
//! 3. Flash this example. Expected: LED1 toggles every 0.5 s, a period of 1 s. The scope on LED1, P1.0
//!    (J1 pin 2), ground clip on GND (J3 pin 22), shows it exactly.
//! 4. Set the generator to 16.384 kHz: LED1 toggles every second. Set it back to 32.768 kHz.
//! 5. Switch the generator output off: LED1 stops, because the fail-safe doesn't cover XT1CLK. Switch it back
//!    on: LED1 carries on.
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    rtc::{Rtc, RtcDiv},
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;
/// XT1 cycles per LED toggle
const TICKS_PER_TOGGLE: u16 = 16_384;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let mut led = p1.pin0;

    // P2.1 = XIN with P2SELx = 01 (SLASE59F Table 6-18, p. 56)
    let xin = p2.pin1.to_alternate1();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from REFO (SELA = 01b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in bypass mode
    // (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_refoclk()
        .freeze(&mut fram);

    // RTCSS = 10b selects XT1CLK (SLASE59F Table 6-7, p. 46; SLAU445I Table 15-2, p. 420), and RTCPS = 000b
    // divides by 1 (SLAU445I Table 15-2, p. 420)
    let mut rtc = Rtc::new(periph.rtc).use_xt1clk(&xt1clk);
    rtc.set_clk_div(RtcDiv::_1);
    // A period lasts `count + 1` ticks
    // (The counter resets to 0 after reaching the modulo value: SLAU445I 15.2.1, p. 417;
    // SLAU445I Figure 15-2, p. 418)
    rtc.start(TICKS_PER_TOGGLE - 1);
    loop {
        // Waits for RTCIFG, which "can be cleared by reading RTCIV register" (SLAU445I Table 15-2, p. 420)
        block!(rtc.wait()).ok();
        led.toggle().ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
