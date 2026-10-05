//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! `try_freeze` with a timeout: if XT1 hasn't started within 3 s, the example falls back to the internal
//! oscillators. LED2 lights green when XT1 started, and LED1 red when the example gave up on it.
//!
//! The LaunchPad's 32.768-kHz crystal typically starts in a second, well within the 3 s. To see the
//! fallback, give it 10 ms instead: its start-up includes the start counter's 1024 cycles, 31 ms, so it
//! can't be ready in time.
//! (tSTART,LFXT, 1000 ms typical, "Includes startup counter of 1024 clock cycles": SLASEC4D Table 5-3,
//! note 8, p. 35. While XT1 isn't running, its fault flag XT1OFFG keeps being set again: SLAU445I 3.2.13,
//! p. 109. The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, LED1 on P1.0 is red and LED2 on P6.6 green:
//! SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example. Expected: about a second later LED2 lights green.
//! 2. Set `XT1_TIMEOUT_MS` to 10 and flash again. Expected: LED1 lights red at once, and LED2 stays off.
//!    Set it back to 3000 afterwards.
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
/// How long to wait for XT1 before falling back to REFO: three times the crystal's typical start-up time
const XT1_TIMEOUT_MS: u16 = 3000;

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
    let p6 = Batch::new(periph.p6)
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p6.pin6;
    led1.set_low().ok();
    led2.set_low().ok();

    // XIN on P2.7 and XOUT on P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120)
    let clocks = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::crystal(XT1_FREQ_HZ, xin, xout))
        .aclk_xt1clk();

    match clocks.try_freeze(&mut fram, XT1_TIMEOUT_MS) {
        Ok((_smclk, _aclk, _xt1clk, _delay)) => {
            led2.set_high().ok();
        }
        Err(clocks) => {
            // XT1 did not start: run everything that was sourced from XT1 from REFO instead
            // (XT1OFFG kept being set again while the fault lasted: SLAU445I 3.2.13, p. 109. ACLK from
            // REFO is SELA = 01b: SLAU445I Table 3-8, p. 117.)
            let (_smclk, _aclk, _delay) = clocks.xt1clk_off().freeze(&mut fram);
            led1.set_high().ok();
        }
    }

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
