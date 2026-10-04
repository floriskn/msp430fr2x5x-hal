#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, NegativeReference, PositiveReference, Predivider, Resolution, SampleTime, SamplingRate},
    gpio::Batch,
    pmm::{Pmm, ReferenceVoltage},
    tlv::TempSensorCalibration,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

// Turn on P1.0 if temp between 20 and 25C (LED1, red: SLAU680 Figure 18, p. 26)
#[entry]
fn main() -> ! {
    // Take peripherals and disable watchdog
    let periph = msp430fr2355::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Configure GPIO
    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let port1 = Batch::new(periph.p1).split(&pmm);
    let mut led = port1.pin0.to_output();
    led.set_low().ok();

    // ADC setup.
    // Temp sensor needs >= 30 us sample time (SLAU445I 21.2.7.8, p. 556: the sample period must be
    // greater than 30 us).
    // MODCLK is < ~4.6MHz, so 256 cycles / 4.6 MHz = 55 us sample time (fMODOSC is 4.6 MHz at most:
    // SLASEC4D Table 5-9, p. 41).
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
        SamplingRate::Max200ksps,
        SampleTime::Cycles256,
    )
    .use_modclk()
    .configure(periph.adc);

    let vref = pmm.enable_internal_reference(ReferenceVoltage::V1_5).unwrap();
    let mut t_sense = pmm.enable_internal_temp_sensor(&vref).unwrap();

    // The device descriptors (TLV) hold the sensor readings measured in the factory at two temperatures,
    // against the internal 1.5 V reference at full resolution, so measure the same way. This is much more
    // accurate than the typical sensor voltage and slope from the data sheet.
    // (Calibration: SLASEC4D Table 6-70, p. 108, ADC internal shared 1.5-V reference at 30 C and at a
    // high temperature, 105 C; SLAU445I 1.13.3.3, p. 60. Typical values: VSENSOR 788 mV at 30 C and
    // TCSENSOR 2.32 mV per degree C, SLASEC4D Table 5-10, p. 41.)
    let mut adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);
    let calibration = TempSensorCalibration::new(ReferenceVoltage::V1_5);

    loop {
        let count = block!(adc.read_count(&mut t_sense)).unwrap();
        let temp_decicelsius = calibration.decicelsius(count);

        // Turn on LED if temp between 20 and 25C (LED1 on P1.0: SLAU680 Figure 18, p. 26)
        if (200..=250).contains(&temp_decicelsius) {
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
