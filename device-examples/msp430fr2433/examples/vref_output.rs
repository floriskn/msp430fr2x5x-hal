//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The 1.2 V reference on the VREF+ pin, P1.4. The board measures the pin with its ADC against the
//! internal 1.5 V reference, and turns LED1 on while the result is within the data sheet's 1.15 V to
//! 1.23 V, widened by the ADC's error. A multimeter checks it independently.
//! (VREF+ is P1.4 with ADCPCTL4 = 1: SLASE59F Table 6-17, p. 55; SLASE59F 6.10.1, p. 45. Its voltage:
//! SLASE59F Table 5-12, p. 29. LED1 on P1.0 is red: SLAU739 Figure 18, p. 23.)
//!
//! P1.4 is also the backchannel UART's TXD, which goes to the debug probe through the TXD jumper of J101
//! (SLAU739 Table 2, p. 8; SLAU739 Figure 18, p. 23). So this example prints nothing.
//!
//! How to test (multimeter):
//! 1. Remove the TXD jumper from J101, so the debug probe's UART input doesn't load the pin.
//! 2. Flash this example. LED1 should turn on.
//! 3. Set the multimeter to DC volts. Put the black probe on GND (J3 pin 22) and the red probe on P1.4
//!    (J1 pin 4). (Header pins: SLAU739 Figure 18, p. 23.)
//!
//! Expected: 1.15 V to 1.23 V (1.19 V typical). Put the jumper back afterwards, for the examples that
//! print.
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

/// The data sheet's range for VREF+ (SLASE59F Table 5-12, p. 29: 1.15 V min, 1.23 V max), widened by the
/// ADC's total unadjusted error with the internal 1.5 V reference, ±3.0 % (SLASE59F Table 5-22, p. 36)
const VREF_MIN_MV: u16 = 1115; // 1150 mV - 3 %
const VREF_MAX_MV: u16 = 1267; // 1230 mV + 3 %

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // EXTREFEN buffers the 1.2 V bandgap onto VREF+ (SLASE59F 6.10.1, p. 45; SLAU445I Table 2-4, p. 94)
    let mut vref_out = pmm.enable_vref_output(p1.pin4.to_adc_mode());

    // The ADC measures the pin, which is input A4 (SLASE59F Table 6-15, p. 53), against the internal 1.5 V
    // reference, the only internal reference for the ADC (SLASE59F 6.10.1, p. 45). MODCLK clocks it
    // (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES = 01b: SLAU445I
    // Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I Table 21-3, p. 561).
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V1_5).unwrap();
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc);
    // ADCSREFx = 001b: VR+ = VREF and VR- = AVSS (SLAU445I Table 21-8, p. 567)
    let mut adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);

    loop {
        let count = block!(adc.read_count(&mut vref_out)).unwrap();
        let mv = adc.count_to_mv(count, 1500);
        led1.set_state((VREF_MIN_MV..=VREF_MAX_MV).contains(&mv).into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
