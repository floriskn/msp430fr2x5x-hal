//! Factory calibration data from the device descriptor table (TLV)
//!
//! Each device stores calibration values measured during production in its device descriptors
//! (data sheets: Device Descriptors). [`TempSensorCalibration`] converts temperature sensor readings
//! with them, which is much more accurate than the typical sensor voltage and slope from the data sheet.

use crate::pmm::ReferenceVoltage;

#[inline(always)]
fn read(addr: usize) -> u16 { unsafe { core::ptr::read_volatile(addr as *const u16) } }

const ADC_GAIN_FACTOR: usize = 0x1A16;
const ADC_OFFSET: usize = 0x1A18;
const TEMP_15V_30C: usize = 0x1A1A;

/// The factory calibration of the internal temperature sensor: the ADC counts it gave at 30 °C and at
/// a high temperature, 105 °C on the MSP430FR2x5x and MSP430FR247x and 85 °C on the MSP430FR2433 and
/// MSP430FR25x2, measured against the internal reference.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct TempSensorCalibration {
    /// The count at 30 °C
    pub count_30c: u16,
    /// The count at [`high_celsius`](TempSensorCalibration::high_celsius)
    pub count_high: u16,
    /// The high calibration temperature in °C
    pub high_celsius: i16,
}

impl TempSensorCalibration {
    /// The calibration for readings against the internal reference at `vref`.
    #[inline]
    pub fn new(vref: ReferenceVoltage) -> Self {
        // Pairs of counts at 30 °C and at the high temperature, one pair per reference level
        let addr = TEMP_15V_30C + 4 * vref as usize;
        #[cfg(feature = "enhanced_ref")]
        let high_celsius = 105;
        #[cfg(not(feature = "enhanced_ref"))]
        let high_celsius = 85;
        TempSensorCalibration { count_30c: read(addr), count_high: read(addr + 2), high_celsius }
    }

    /// Convert a reading of the temperature sensor to tenths of a degree Celsius.
    ///
    /// The reading must be taken as the calibration was: against the same internal reference
    /// ([`PositiveReference::Internal`](crate::adc::PositiveReference::Internal)), at the ADC's full
    /// resolution (12-bit, or 10-bit on the MSP430FR2433 and MSP430FR25x2), in the unsigned format and
    /// with a sample time of at least 30 µs (user's guide 21.2.7.8).
    #[inline]
    pub fn decicelsius(&self, count: u16) -> i16 {
        let span = self.count_high as i32 - self.count_30c as i32;
        let delta = count as i32 - self.count_30c as i32;
        (delta * (self.high_celsius as i32 - 30) * 10 / span + 300) as i16
    }
}

/// The ADC gain correction factor, in 1/32768ths: multiply results by it and divide by 32768. The
/// factory measured it at 2.4 V and room temperature, with external references on VeREF+ and VeREF-
/// (data sheets: Device Descriptors), so other settings can need a different factor.
#[inline]
pub fn adc_gain_factor() -> u16 { read(ADC_GAIN_FACTOR) }

/// The ADC offset correction, in counts: add it to results after the gain correction. Measured as
/// [`adc_gain_factor()`] was.
#[inline]
pub fn adc_offset() -> i16 { read(ADC_OFFSET) as i16 }

/// The correction factor of the internal reference at `vref`, in 1/32768ths: multiply results
/// measured against it by the factor and divide by 32768.
#[inline]
pub fn reference_factor(vref: ReferenceVoltage) -> u16 {
    #[cfg(feature = "enhanced_ref")]
    const REF_15V_FACTOR: usize = 0x1A28;
    #[cfg(not(feature = "enhanced_ref"))]
    const REF_15V_FACTOR: usize = 0x1A20;
    read(REF_15V_FACTOR + 2 * vref as usize)
}
