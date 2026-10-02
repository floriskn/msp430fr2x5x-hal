#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate}, gpio::Batch, pmm::Pmm, watchdog::Wdt
};
use nb::block;
use panic_msp430 as _;

// If pin 1.2 is between 1V and 2V, the LED on pin 1.0 should light up.
// P1.2 is ADC input A2 (SLASE59F Table 6-15, p. 53). On the LaunchPad it is header pin J1.10, and P1.0
// drives the red LED1 (both: SLAU739 Figure 18, p. 23).
#[entry]
fn main() -> ! {
    // Take peripherals and disable watchdog
    let periph = msp430fr2433::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(periph.watchdog_timer);

    // Configure GPIO
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let port1 = Batch::new(periph.p1).split(&pmm);
    let mut red_led = port1.pin0.to_output();
    red_led.set_low().ok();
    let mut adc_pin = port1.pin2.to_adc_mode(); // A2: ADCPCTL2 = 1 (SLASE59F Table 6-17, p. 55)

    // ADC setup
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::_8BIT,
        SamplingRate::_50KSPS,
        SampleTime::_4,
    )
    .use_modclk()
    .configure(periph.adc);

    loop {
        // Get ADC voltage, assuming the ADC reference voltage is 3300mV
        // (the reference is AVCC after reset, SLAU445I 21.3.6, Table 21-8, p. 567, which the LaunchPad
        // supplies at 3.3 V, SLAU739 2.3.1, p. 10)
        // It's infallible besides nb::WouldBlock, so it's safe to unwrap after block!()
        // If you want a raw count use adc.read_count() instead.
        let reading_mv = block!( adc.read_voltage_mv(&mut adc_pin, 3300) ).unwrap();

        // Turn on LED if voltage between 1000 and 2000mV
        if (1000..=2000).contains(&reading_mv) {
            red_led.set_high().ok();
        } else {
            red_led.set_low().ok();
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
