//! XT1 fault detection and the fail-safe switch to REFO, using ACLK as the observable clock.
//!
//! The generator runs at 20 kHz instead of 32.768 kHz so the REFO fallback (32.768 kHz) is easy
//! to tell apart on the scope.
//!
//! Wiring: function generator -> P2.1/XIN (J2 pin 18), ground -> J2 pin 20. Square wave,
//! 20 kHz, 0 V to 3.3 V, 50 % duty, output load High-Z (see `xt1_bypass_aclk.rs`).
//!
//! Scope: the generator signal on CH3 through a BNC T-piece, CH1 on P2.2/ACLK (J1 pin 5).
//!
//! What to try:
//! 1. ACLK = 20 kHz, red LED1 off.
//! 2. Switch the generator output off: LED1 on, ACLK jumps to about 32.8 kHz (REFO).
//! 3. Switch it back on: the fault clears almost at once, LED1 turns off and ACLK returns to
//!    20 kHz. Bypass mode leaves the start counter off; with `.enable_start_counter()` the
//!    fault would only clear after 1024 clean cycles, 51 ms at 20 kHz.
//! 4. Repeat 2 and 3 while holding button S2 (P2.3): the fault flags are no longer cleared, so
//!    ACLK stays on REFO even after the signal is back. The fail-safe only releases once
//!    software clears the flags, which happens as soon as S2 is released.
//! 5. Lower the generator frequency step by step (10 kHz, 5 kHz, 3.5 kHz, 2 kHz, 1 kHz) to find
//!    where the fault detector trips. The datasheet only guarantees no fault above 3.5 kHz.
//! 6. Vary the duty cycle. The datasheet specifies 40 % to 60 % for bypass mode.
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
const XT1_FREQ_HZ: u32 = 20_000;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut button = p2.pin3;

    let _aclk_out = p2.pin2.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate1();

    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
        .freeze(&mut fram);

    loop {
        // While S2 is held the flags are left alone, so the latched fault keeps ACLK on REFO
        let s2_held = button.is_low().unwrap_or(false);
        if !s2_held {
            xt1clk.clear_fault();
        }
        led.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
