//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The RTC clocked from XT1, directly from XT1CLK or through ACLK: it counts 16384 cycles of the
//! LaunchPad's 32.768-kHz crystal between toggles of LED1, so LED1 toggles every 0.5 s.
//!
//! `RTC_FROM_ACLK` picks the path. The fail-safe covers ACLK but not the RTC's own XT1CLK input: without
//! XT1 the RTC stops on XT1CLK, while through ACLK it counts REFO instead. The crystal can't be stopped from
//! outside, so holding S1 stops it: the main loop then makes XIN a general-purpose I/O, which disables XT1,
//! as a broken crystal would stop (see `set_crystal_running` below). The main loop clears the fault flag at
//! each toggle, which moves ACLK back to XT1 once the crystal runs again.
//! (RTCSS = 10b selects XT1CLK, and 01b SMCLK or ACLK, chosen by RTCCKSEL in SYSCFG2: SLASEC4D Table 6-9,
//! p. 68; SLASEC4D Table 6-10, p. 68. SLAU445I 3.2.13, p. 109 to p. 110 describes the switch to REFO for
//! MCLK, SMCLK, ACLK and the FLL reference only. The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, LED1 on
//! P1.0 is red, and S1 connects P4.1 to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example. Expected, once the crystal has started, about a second after reset (1000 ms
//!    typical: SLASEC4D Table 5-3, p. 35): LED1 toggles every 0.5 s, a period of 1 s.
//! 2. Hold S1: LED1 stops, because the fail-safe doesn't cover XT1CLK. Release S1: about a second later,
//!    once the crystal has started again, LED1 carries on.
//! 3. Set `RTC_FROM_ACLK` to true and flash again: LED1 toggles the same way, but the RTC counts ACLK
//!    (RTCSS = 01b, RTCCKSEL = 1). LED1 toggling every 16 ms instead would mean it counts SMCLK, 1 MHz.
//! 4. Hold S1: LED1 keeps toggling at nearly the same rate, because ACLK falls back to REFO. Set
//!    `RTC_FROM_ACLK` back to false afterwards.
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
use panic_msp430 as _;

/// Frequency of the LaunchPad's crystal Q1 (SLAU680 2.5, p. 13)
const XT1_FREQ_HZ: u32 = 32_768;
/// Clock the RTC from ACLK (sourced from XT1) instead of XT1CLK directly
const RTC_FROM_ACLK: bool = false;
/// XT1 cycles per LED toggle
const TICKS_PER_TOGGLE: u16 = 16_384;

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
    // S1 on P4.1 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313)
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut s1 = p4.pin1;

    // XIN on P2.7 and XOUT on P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in crystal mode
    // (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120)
    let (_smclk, aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::crystal(XT1_FREQ_HZ, xin, xout))
        .aclk_xt1clk()
        .freeze(&mut fram);

    let rtc = Rtc::new(periph.rtc);
    if RTC_FROM_ACLK {
        // RTCSS = 01b with RTCCKSEL = 1 selects ACLK (SLASEC4D Table 6-9, p. 68; RTCSS: SLAU445I
        // Table 15-2, p. 420; RTCCKSEL in SYSCFG2: SLAU445I Table 1-26, p. 77)
        let rtc = rtc.use_aclk(&aclk);
        run(rtc, &mut led, &mut s1, &mut xt1clk)
    } else {
        // RTCSS = 10b selects XT1CLK (SLASEC4D Table 6-10, p. 68; SLAU445I Table 15-2, p. 420)
        let rtc = rtc.use_xt1clk(&xt1clk);
        run(rtc, &mut led, &mut s1, &mut xt1clk)
    }
}

fn run<SRC: RtcClockSrc>(
    mut rtc: Rtc<SRC>,
    led: &mut impl StatefulOutputPin,
    s1: &mut impl InputPin,
    xt1clk: &mut Xt1clk,
) -> ! {
    // RTCPS = 000b divides by 1 (SLAU445I Table 15-2, p. 420)
    rtc.set_clk_div(RtcDiv::_1);
    // A period lasts `count + 1` ticks
    // (The counter resets to 0 after reaching the modulo value: SLAU445I 15.2.1, p. 417;
    // SLAU445I Figure 15-2, p. 418)
    rtc.start(TICKS_PER_TOGGLE - 1);
    loop {
        // While S1 is held the crystal stands still
        set_crystal_running(s1.is_high().unwrap_or(true));
        // RTCIFG, which "can be cleared by reading RTCIV register" (SLAU445I Table 15-2, p. 420)
        if rtc.wait().is_ok() {
            led.toggle().ok();
            // Clearing the sticky fault flag lets ACLK move back from REFO to XT1 once the crystal
            // runs again (SLAU445I 3.2.13, p. 109 to p. 110)
            xt1clk.clear_fault();
        }
    }
}

/// Stop the crystal, as a broken one would stop, or let it start again: this test's stand-in for an XT1
/// fault, which the HAL has no function for. With the P2SEL1 bit of XIN, P2.7, cleared, "both XT1IN and
/// XT1OUT ports are configured as general-purpose I/Os, and XT1 is disabled"; setting it configures them
/// "for XT1 operation" again (SLAU445I 3.2.4, p. 103; XIN is P2SELx = 10: SLASEC4D Table 6-64, p. 98).
/// Meanwhile XIN is a GPIO output, held low, so that it passes no stray edges on to XT1CLK: the workaround
/// for erratum RTC15 toggles XIN as a GPIO output to clock XT1CLK (SLAZ695J RTC15, p. 11). PxDIR and PxOUT:
/// SLAU445I Table 8-1, p. 313.
fn set_crystal_running(running: bool) {
    const XIN: u8 = 1 << 7;
    let p2 = unsafe { &*msp430fr2355::P2::ptr() };
    if running {
        p2.p2dir().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2sel1().modify(|r, w| unsafe { w.bits(r.bits() | XIN) });
    } else {
        p2.p2sel1().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2out().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2dir().modify(|r, w| unsafe { w.bits(r.bits() | XIN) });
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
