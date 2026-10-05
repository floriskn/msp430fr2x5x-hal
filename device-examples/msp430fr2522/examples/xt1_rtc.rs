//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The RTC clocked from XT1, directly from XT1CLK or through ACLK: it counts 16384 cycles of the generator's
//! signal on XIN between toggles of an LED on P1.0, so at 32.768 kHz the LED toggles every 0.5 s.
//!
//! `RTC_FROM_ACLK` picks the path. The fail-safe covers ACLK but not the RTC's own XT1CLK input: without a
//! signal the RTC stops on XT1CLK, while through ACLK it counts REFO instead. The main loop clears the fault
//! flag at each toggle, which moves ACLK back to XT1 once the signal is back.
//! (RTCSS = 10b selects XT1CLK, and 01b SMCLK or ACLK, chosen by RTCCKSEL in SYSCFG2: SLASEE4C 6.10.11,
//! p. 55; SLASEE4C Table 6-12, p. 55; SLASEE4C Table 6-8, p. 49. SLAU445I 3.2.13, p. 109 to p. 110
//! describes the switch to REFO for MCLK, SMCLK, ACLK and the FLL reference only. XIN is P2.1: SLASEE4C
//! Table 6-16, p. 60. No board document covers the LED: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, an LED and a resistor, and optionally the scope):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on.
//! 4. Flash this example. Expected: the LED toggles every 0.5 s, a period of 1 s. The scope on the LED, P1.0,
//!    ground clip on GND, shows it exactly.
//! 5. Set the generator to 16.384 kHz: the LED toggles every second. Set it back to 32.768 kHz.
//! 6. Switch the generator output off: the LED stops, because the fail-safe doesn't cover XT1CLK. Switch it
//!    back on: the LED carries on.
//! 7. Set `RTC_FROM_ACLK` to true and flash again: the LED toggles the same way, but the RTC counts ACLK
//!    (RTCSS = 01b, RTCCKSEL = 1). The LED toggling every 16 ms instead would mean it counts SMCLK, 1 MHz.
//! 8. Switch the generator output off: the LED keeps toggling at nearly the same rate, because ACLK falls
//!    back to REFO. Set `RTC_FROM_ACLK` back to false afterwards.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config, Xt1clk},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    rtc::{Rtc, RtcClockSrc, RtcDiv},
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;
/// Clock the RTC from ACLK (sourced from XT1) instead of XT1CLK directly
const RTC_FROM_ACLK: bool = false;
/// XT1 cycles per LED toggle
const TICKS_PER_TOGGLE: u16 = 16_384;

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

    // P2.1 = XIN with P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let xin = p2.pin1.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in bypass mode
    // (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
        .freeze(&mut fram);

    let rtc = Rtc::new(periph.rtc);
    if RTC_FROM_ACLK {
        // RTCSS = 01b with RTCCKSEL = 1 selects ACLK (SLASEE4C Table 6-12, p. 55; RTCSS: SLAU445I
        // Table 15-2, p. 420; RTCCKSEL in SYSCFG2: SLAU445I Table 1-31, p. 82)
        let rtc = rtc.use_aclk(&aclk);
        run(rtc, &mut led, &mut xt1clk)
    } else {
        // RTCSS = 10b selects XT1CLK (SLASEE4C Table 6-12, p. 55; SLAU445I Table 15-2, p. 420)
        let rtc = rtc.use_xt1clk(&xt1clk);
        run(rtc, &mut led, &mut xt1clk)
    }
}

fn run<SRC: RtcClockSrc>(
    mut rtc: Rtc<SRC>,
    led: &mut impl StatefulOutputPin,
    xt1clk: &mut Xt1clk,
) -> ! {
    // RTCPS = 000b divides by 1 (SLAU445I Table 15-2, p. 420)
    rtc.set_clk_div(RtcDiv::_1);
    // A period lasts `count + 1` ticks
    // (The counter resets to 0 after reaching the modulo value: SLAU445I 15.2.1, p. 417;
    // SLAU445I Figure 15-2, p. 418)
    rtc.start(TICKS_PER_TOGGLE - 1);
    loop {
        // Waits for RTCIFG, which "can be cleared by reading RTCIV register" (SLAU445I Table 15-2, p. 420)
        block!(rtc.wait()).ok();
        led.toggle().ok();
        // Clearing the sticky fault flag lets ACLK move back from REFO to XT1 once the signal
        // is healthy again (SLAU445I 3.2.13, p. 109 to p. 110)
        xt1clk.clear_fault();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
