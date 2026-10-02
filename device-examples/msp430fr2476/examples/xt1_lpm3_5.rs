//! RTC clocked by XT1 through LPM3.5: the board sleeps in LPM3.5 and the RTC wakes it every
//! second. Each wake-up toggles red LED1, whose state is kept in the backup memory.
//!
//! After a wake-up, `Pmm::new_locked` keeps the pins, and XT1 with them, in their LPM3.5 state
//! until XIN and XT1 have been reconfigured, so XT1 never stops clocking the RTC.
//!
//! Wiring: function generator -> P2.1/XIN (J2 pin 18), ground -> J2 pin 20. Square wave,
//! 32.768 kHz, 0 V to 3.3 V, 50 % duty, output load High-Z (see `xt1_bypass_aclk.rs`).
//! Switch the generator on before the first start.
//!
//! When programming with mspdebug you need to unplug and replug the board for the example to
//! work (see `lpm3_5.rs`).
//!
//! Scope: P1.0/LED1 (J3 pin 27).
//!
//! What to try:
//! 1. LED1 toggles every second (a 2 s period). The period tracks the generator: at 16.384 kHz
//!    it doubles to 4 s, which shows the RTC runs from XT1 during LPM3.5.
//! 2. Switch the generator off: the RTC stops, so LED1 stops toggling. Switch it back on and
//!    it carries on.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430fr247x::{P3, P4, P5, P6};
use msp430_hal::{
    bak_mem::BackupMemory,
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    lpm::{enter_lpm3_5, enter_lpm3_5_unchecked, SvsState},
    pmm::Pmm,
    rtc::{Rtc, RtcDiv},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let wdt = Wdt::constrain(periph.wdt_a);
    let mut fram = Fram::new(periph.frctl);

    // After a wake-up from LPM3.5 the pins stay locked until XIN and XT1 are reconfigured, so
    // XT1 keeps clocking the RTC. After a cold start XT1 can only start once the pins are
    // unlocked, so unlock straight away.
    let woke_from_lpm3_5 = periph.sys.sysrstiv().read().sysrstiv().is_lpm5wu();
    let (mut pmm, _) = if woke_from_lpm3_5 {
        Pmm::new_locked(periph.pmm, periph.sys)
    } else {
        Pmm::new(periph.pmm, periph.sys)
    };

    // Configure the pins the same way every time. Floating input pins consume a *huge* amount
    // of energy (relatively speaking), so pull unused pins down.
    let port1 = Batch::new(periph.p1)
        .pulldown_all()
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let mut red_led = port1.pin0;
    let port2 = Batch::new(periph.p2)
        .pulldown_all()
        .config_pin1(|p| p.floating())
        .split(&pmm);
    let xin = port2.pin1.to_alternate1();
    init_unused_gpio(periph.p3, periph.p4, periph.p5, periph.p6, &pmm);

    let (_smclk, _aclk, xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .freeze(&mut fram);

    // XIN and XT1 are configured, so the pins can be released (already done after a cold start)
    pmm.unlock_lpm5();

    if woke_from_lpm3_5 {
        // Toggle the LED.
        // I/O registers have their values reset coming out of LPMx.5,
        // so we have to store state in the backup memory.
        let bak_mem = BackupMemory::as_u8s(periph.bkmem);

        let old_value = bak_mem[0] == 1;
        red_led.set_state(old_value.into()).ok();

        let new_value = if old_value { 0 } else { 1 };
        bak_mem[0] = new_value;

        // Clear RTC interrupt flag
        periph.rtc.rtciv().read();

        // Enter LPM3.5 (without having to configure the RTC, we did that already).
        unsafe { enter_lpm3_5_unchecked(wdt, SvsState::Svshe0) };
    }
    // Otherwise this is a fresh start. Configure the RTC.
    else {
        // Configure RTC for 1 Hz interrupt from XT1
        let mut rtc = Rtc::new(periph.rtc).use_xt1clk(&xt1clk);
        rtc.set_clk_div(RtcDiv::_1);
        // A period lasts `count + 1` ticks, so this gives a 1 Hz period
        rtc.start((XT1_FREQ_HZ - 1) as u16);
        rtc.enable_interrupts();
        // Global interrupts are enabled by `enter_lpm3_5()`
        // Leaving LPMx.5 requires a full system reset, so this function will never return.
        enter_lpm3_5(wdt, rtc, SvsState::Svshe0);
    }
}

/// Enable pulldowns on unused ports to massively reduce power usage.
fn init_unused_gpio(p3: P3, p4: P4, p5: P5, p6: P6, pmm: &Pmm) {
    Batch::new(p3).pulldown_all().split(pmm);
    Batch::new(p4).pulldown_all().split(pmm);
    Batch::new(p5).pulldown_all().split(pmm);
    Batch::new(p6).pulldown_all().split(pmm);
}

// Note: In this case we don't need an ISR when waking from LPMx.5, since power on disables interrupts
// and we clear the RTC interrupt flag before re-enabling interrupts.

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
