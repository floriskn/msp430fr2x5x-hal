//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The ADC's two internal supply channels: the ADC converts channel 14, DVSS, and channel 15, DVCC, over and
//! over, and an LED on P1.0 lights while both results are right and `adc_is_busy()` found the second
//! conversion running. An LED on P1.1 lights otherwise.
//!
//! The reference is AVCC, as after reset, so the two channels are the ends of the ADC's range: DVSS converts
//! to 0 and DVCC to the full-scale count, 1023 for 10-bit results. The program accepts results up to 16
//! counts away from these, far more than the ADC's offset and gain errors. A long sample time, 1024 ADCCLK
//! cycles, keeps each conversion going long enough for the loop that polls ADCBUSY to find it set.
//! (Channels 14 and 15: SLASEE4C Table 6-13, p. 56. DVCC and DVSS supply the analog modules too: SLASEE4C
//! 1.4, p. 4. The range: SLAU445I 21.2.1, p. 541. AVCC as the reference after reset: SLAU445I Table 21-8,
//! p. 567. ADCBUSY: SLAU445I Table 21-4, p. 564. The ADC's offset and gain errors, up to 6.5 mV and 2 LSB:
//! SLASEE4C Table 5-22, p. 39. No board document covers the LEDs: there is none for the MSP430FR25x2. P1.0
//! and P1.1 are GPIO outputs, P1SELx = 00 and P1DIR = 1: SLASEE4C Table 6-15, p. 58.)
//!
//! How to test (two LEDs and two resistors):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another one from P1.1 to
//!    GND.
//! 2. Flash this example.
//! 3. Expected: the LED on P1.0 lights. The one on P1.1 lights instead if DVSS or DVCC converts to more than
//!    16 counts away from 0 or 1023, or if `adc_is_busy()` never found the ADC busy.
#![no_main]
#![no_std]

use embedded_hal::digital::OutputPin;
use msp430_rt::entry;
use msp430_hal::{
    adc::{
        adc_ch14_vss, adc_ch15_vcc, AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate,
    },
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The full-scale count of 10-bit results (SLAU445I 21.2.1, p. 541)
const FULL_SCALE: u16 = 1023;
/// How far from 0 and from the full scale the results may be
const TOLERANCE: u16 = 16;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut pass_led = p1.pin0.to_output_low();
    let mut fail_led = p1.pin1.to_output_low();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES =
    // 01b: SLAU445I Table 21-5, p. 565) and 1024 ADCCLK cycles of sampling (ADCSHTx = 1100b: SLAU445I
    // Table 21-3, p. 561)
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles1024,
    )
    .use_modclk()
    .configure(periph.adc);

    // Channels 14 and 15 have no pins, so these stand for them (ADCINCHx = 1110b and 1111b: SLASEE4C
    // Table 6-13, p. 56)
    let mut dvss = adc_ch14_vss();
    let mut dvcc = adc_ch15_vcc();

    loop {
        let dvss_count = block!(adc.read_count(&mut dvss)).unwrap();

        // The first `read_count()` starts the conversion and returns `WouldBlock`. Count the reads of ADCBUSY
        // that find it still running, then fetch the result.
        adc.read_count(&mut dvcc).ok();
        let mut polls: u16 = 0;
        while adc.adc_is_busy() {
            polls += 1;
        }
        let dvcc_count = block!(adc.read_count(&mut dvcc)).unwrap();

        let right = dvss_count <= TOLERANCE && dvcc_count >= FULL_SCALE - TOLERANCE && polls > 0;
        pass_led.set_state(right.into()).ok();
        fail_led.set_state((!right).into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
