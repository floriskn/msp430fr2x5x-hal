#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
use panic_msp430 as _;

// Red LED should blink 1 second on, 1 second off
#[entry]
fn main() -> ! {
    let periph = msp430fr413x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.fram);
    let wdt = Wdt::constrain(periph.watchdog_timer);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let mut p1_0 = p1.pin0;
    let p4 = Batch::new(periph.p4)
        .split(&pmm);

    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_xt1clk(32_768, p4.pin1.to_alternate1(), p4.pin2.to_alternate1())
        .freeze(&mut fram);

    const DELAY: WdtClkPeriods = WdtClkPeriods::_32k;

    // blinks should be 1 second on, 1 second off
    let mut wdt = wdt.to_interval();
    p1_0.set_high().ok();
    wdt.set_aclk(&aclk).set_interval_and_start(DELAY);

    block!(wdt.wait()).ok();
    p1_0.set_low().ok();

    let mut wdt = wdt.to_watchdog();
    wdt.set_interval_and_start(DELAY);

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
