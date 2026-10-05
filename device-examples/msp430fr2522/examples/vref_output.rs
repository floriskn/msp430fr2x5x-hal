//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The 1.2 V reference on the VREF+ pin, P1.1. The device measures the pin with its ADC against the
//! internal 1.5 V reference, and turns an LED on P1.0 on while the result is within the data sheet's
//! 1.15 V to 1.23 V. A multimeter checks it independently.
//! (VREF+ is P1.1, A1, and "the ADC channel 1 can also be selected to monitor this voltage": SLASEE4C
//! 6.10.1, p. 49; SLASEE4C Table 6-15, p. 58. Its voltage: SLASEE4C Table 5-12, p. 31. P1.0 is a GPIO
//! output, P1SELx = 00 and P1DIR = 1: SLASEE4C Table 6-15, p. 58. No board document covers the parts to
//! connect: there is none for the MSP430FR25x2.)
//!
//! The internal reference gives the ADC a gain error of up to ±3 % (SLASEE4C Table 5-22, p. 39), so the
//! LED is a rough check; the multimeter is the precise one.
//!
//! How to test (an LED and a resistor, and the multimeter):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example. The LED should turn on.
//! 3. Set the multimeter to DC volts. Put the black probe on GND and the red probe on P1.1.
//!
//! Expected: 1.15 V to 1.23 V (1.19 V typical).
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, NegativeReference, PositiveReference, Predivider, Resolution, SampleTime, SamplingRate},
    gpio::Batch,
    pmm::{Pmm, ReferenceVoltage},
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The data sheet's range for VREF+ (SLASEE4C Table 5-12, p. 31: 1.15 V min, 1.23 V max)
const VREF_MIN_MV: u16 = 1150;
const VREF_MAX_MV: u16 = 1230;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led = p1.pin0.to_output_low();

    // EXTREFEN buffers the 1.2 V bandgap onto VREF+, P1.1 in its analog function, ADCPCTL1 = 1 (SLASEE4C
    // 6.10.1, p. 49; SLAU445I Table 2-4, p. 94; SLASEE4C Table 6-15, p. 58)
    let mut vref_out = pmm.enable_vref_output(p1.pin1.to_adc_mode());

    // The ADC measures the pin, which is input A1, against the internal 1.5 V reference. MODCLK clocks it
    // (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES = 01b: SLAU445I
    // Table 21-5, p. 565) and 256 ADCCLK cycles of sampling (ADCSHTx = 1000b: SLAU445I Table 21-3, p. 561):
    // at least 44 µs at MODCLK's 5.8 MHz maximum (SLASEE4C Table 5-9, p. 28), longer than the 30 µs the
    // internal reference may take to settle (SLAU445I 21.2.3.1, p. 542).
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V1_5).unwrap();
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles256,
    )
    .use_modclk()
    .configure(periph.adc);
    // ADCSREFx = 001b: VR+ = VREF and VR- = AVSS (SLAU445I Table 21-8, p. 567)
    let mut adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);

    loop {
        let count = block!(adc.read_count(&mut vref_out)).unwrap();
        let mv = adc.count_to_mv(count, 1500);
        led.set_state((VREF_MIN_MV..=VREF_MAX_MV).contains(&mv).into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
