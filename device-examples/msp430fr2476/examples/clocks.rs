#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pin_mapping::DefaultMapping, pmm::Pmm, serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, StopBits}, watchdog::{Wdt, WdtClkPeriods}
};
use nb::block;
use panic_msp430 as _;

// Red LED should blink 1 second on, 1 second off
#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    let wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin7(|p| p.to_output().to_alternate2())
        .split(&pmm);
    let mut p1_0 = p1.pin0;
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.to_output().to_alternate2())
        .split(&pmm);

    let (mut smclk, mut aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_xt1clk(32_768, p2.pin0.to_alternate1(), p2.pin1.to_alternate1())
        .freeze(&mut fram);

    let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        // Launchpad UART-to-USB converter doesn't handle parity, so we don't use it
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_aclk(&aclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    // blinks should be 1 second on, 1 second off
   
   

    loop {
        delay.delay_ms(1000);
        p1_0.toggle().ok();
        delay.delay_ms(1000);
        p1_0.toggle().ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
