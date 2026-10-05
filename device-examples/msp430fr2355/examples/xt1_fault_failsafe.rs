//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 fault detection and the fail-safe: when the crystal stops, ACLK switches from XT1 to REFO and LED1
//! lights red. When the crystal runs again, the example clears the fault and ACLK goes back to XT1.
//!
//! The LaunchPad's 32.768-kHz crystal can't be stopped from outside, so holding S1 stops it: the main loop
//! then makes XIN a general-purpose I/O, which disables XT1, as a broken crystal would stop (see
//! `set_crystal_running` below). The fault flags are sticky, and the fail-safe stays engaged until software
//! clears them, which the main loop does all the time, except while S2 is held.
//! (Fail-safe: SLAU445I 3.2.13, p. 109 to p. 110. REFO runs at 32.768 kHz ±3.5 %: SLASEC4D Table 5-7, p. 40.
//! The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, LED1 on P1.0 is red, and S1 and S2 connect P4.1 and P2.3
//! to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test (optionally the scope):
//! 1. Flash this example. Expected: LED1 is off, once the crystal has started, about a second after reset
//!    (1000 ms typical: SLASEC4D Table 5-3, p. 35).
//! 2. Scope on ACLK, P1.1 (J3 pin 28), ground clip on GND (J3 pin 22): its counter shows the crystal's
//!    32.768 kHz.
//! 3. Hold S1: LED1 lights, and ACLK keeps running, now from REFO, whose frequency can differ by up to 3.5 %.
//! 4. Release S1: about a second later, once the crystal has started again, LED1 turns off and ACLK runs
//!    from the crystal again.
//! 5. Hold S2, press and release S1, and keep S2 held for a few seconds: LED1 stays on, even once the
//!    crystal runs again, and ACLK stays on REFO. Release S2: LED1 turns off, and ACLK runs from the crystal
//!    again.
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

/// Frequency of the LaunchPad's crystal Q1 (SLAU680 2.5, p. 13)
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
    // S1 on P4.1 and S2 on P2.3 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1,
    // PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut s1 = p4.pin1;
    let mut s2 = p2.pin3;

    // ACLK on P1.1 with P1SEL = 10 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96); XIN on P2.7 and XOUT on
    // P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::crystal(XT1_FREQ_HZ, xin, xout))
        .aclk_xt1clk()
        .freeze(&mut fram);

    loop {
        // While S1 is held the crystal stands still
        set_crystal_running(s1.is_high().unwrap_or(true));
        // While S2 is held the flags are left alone, so the latched fault keeps ACLK on REFO
        // (SLAU445I 3.2.13, p. 109 to p. 110. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I
        // Table 1-10, p. 63.)
        let s2_held = s2.is_low().unwrap_or(false);
        if !s2_held {
            xt1clk.clear_fault();
        }
        led.set_state(xt1clk.is_faulted().into()).ok();
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
