#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate},
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

// If pin 1.1 is between 1V and 2V, the LED on pin 1.0 should light up.
// LED1 is on P1.0 (SLAU802 Figure 19, p. 25). P1.1 is J3 pin 28 (SLAU802 Figure 10, p. 13), and on the
// LaunchPad it is also wired to the output of the TMP235 temperature sensor (SLAU802 2.2.5.1, p. 10).
#[entry]
fn main() -> ! {
    // Take peripherals and disable watchdog
    let periph = msp430fr247x::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Configure GPIO
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let port1 = Batch::new(periph.p1).split(&pmm);
    let mut led = port1.pin0.to_output();
    // P1.1 = analog input A1 with P1SEL = 11 (SLASEO7C Table 9-23, p. 65), ADC channel 1
    // (SLASEO7C Table 9-19, p. 62)
    let mut adc_pin = port1.pin1.to_alternate3();

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
        // (the ADC measures against AVCC after reset, ADCSREFx = 000b: SLAU445I 21.3.6, p. 567; the
        // LaunchPad supplies the MSP430 with 3.3 V: SLAU802 2.3.1, p. 10)
        // It's infallible besides nb::WouldBlock, so it's safe to unwrap after block!()
        // If you want a raw count use adc.read_count() instead.
        let reading_mv = block!(adc.read_voltage_mv(&mut adc_pin, 3300)).unwrap();

        // Turn on LED if voltage between 1000 and 2000mV
        if (1000..=2000).contains(&reading_mv) {
            led.set_high().ok();
        } else {
            led.set_low().ok();
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
