//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 as the FLL reference: the FLL locks the DCO to the LaunchPad's 32.768-kHz crystal instead of REFO,
//! so MCLK on P3.0 runs at 244 times the crystal's frequency.
//!
//! LED1 shows an XT1 fault, which makes the FLL fall back to REFO, and LED2 that the FLL is locked. ACLK on
//! P1.1 runs from REFO, for comparison. The crystal can't be stopped from outside, so holding S1 stops it:
//! the main loop then makes XIN a general-purpose I/O, which disables XT1, as a broken crystal would stop
//! (see `set_crystal_running` below).
//! (SELREF picks XT1CLK or REFOCLK as the FLL reference, and fDCOCLKDIV = (FLLN + 1) × (fFLLREFCLK ÷ n):
//! SLAU445I 3.2.5, p. 104. FLLUNLOCK: SLAU445I 3.2.9, p. 105. An XT1 fault switches the FLL reference to
//! REFO: SLAU445I 3.2.13, p. 109. The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, LED1 on P1.0 is red,
//! LED2 on P6.6 green, and S1 connects P4.1 to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test (the scope):
//! 1. Flash this example, and put the scope (10X probe) on MCLK, P3.0 (J2 pin 11), ground clip on GND
//!    (J3 pin 22). Expected, once the crystal has started, about a second after reset (1000 ms typical:
//!    SLASEC4D Table 5-3, p. 35): 244 × 32.768 kHz = 7.995 MHz, within the ±0.5 % the FLL is specified to
//!    with an XT1 crystal as the reference (SLASEC4D Table 5-5, p. 37); LED1 is off and LED2 green. SMCLK
//!    on P3.4 (J1 pin 8) is MCLK / 8, 999.4 kHz, which is easier to measure precisely.
//! 2. Hold S1: LED1 lights red, and MCLK runs at 244 times REFO's frequency instead, which ACLK on P1.1
//!    (J3 pin 28) shows: REFO can differ from 32.768 kHz by up to 3.5 % (SLASEC4D Table 5-7, p. 40). LED2
//!    can go off for a moment while the FLL locks to the new reference.
//! 3. Release S1: about a second later, once the crystal has started again, LED1 turns off, and MCLK is
//!    7.995 MHz again.
//! (Header pins: SLAU680 Figure 10, p. 15.)
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
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    // S1 on P4.1 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313)
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let p6 = Batch::new(periph.p6)
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p6.pin6;
    let mut s1 = p4.pin1;

    // MCLK on P3.0 and SMCLK on P3.4, each with P3SEL = 01 and P3DIR = 1 (SLASEC4D Table 6-65, p. 100);
    // ACLK on P1.1 with P1SEL = 10 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96); XIN on P2.7 and XOUT on
    // P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let _mclk_out = p3.pin0.to_output().to_alternate1();
    let _smclk_out = p3.pin4.to_output().to_alternate1();
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b) and ACLK from REFO (SELA = 01b) (SLAU445I Table 3-8,
    // p. 117); SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118); XT1 in crystal mode
    // (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120); XT1CLK as the FLL reference (SELREF = 00b:
    // SLAU445I Table 3-7, p. 116)
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::crystal(XT1_FREQ_HZ, xin, xout))
        .fll_ref_xt1()
        .aclk_refoclk()
        .freeze(&mut fram);

    loop {
        // While S1 is held the crystal stands still
        set_crystal_running(s1.is_high().unwrap_or(true));
        // Clearing the sticky fault flag also lets the FLL move back to XT1 once the crystal runs
        // again (SLAU445I 3.2.13, p. 110)
        xt1clk.clear_fault();
        led1.set_state(xt1clk.is_faulted().into()).ok();
        // The FLL reports the DCO as locked, too fast, too slow or out of range (CSCTL7.FLLUNLOCK:
        // SLAU445I Table 3-11, p. 121)
        led2.set_state((fll_status() == FllStatus::Locked).into()).ok();
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
