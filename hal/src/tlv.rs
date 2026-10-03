//! Factory calibration data from the device descriptor table (TLV)
//!
//! Each device stores calibration values measured during production in its device descriptors
//! (data sheets: Device Descriptors, SLASEC4D Table 6-70, p. 107 to p. 108; SLASEO7C Table 9-30, p. 71 to
//! p. 72; SLASE59F Table 6-22, p. 60 to p. 61; SLASEE4C Table 6-18, p. 61 to p. 62; how to use them:
//! SLAU445I 1.13.3, p. 59 to p. 60). [`TempSensorCalibration`] converts temperature sensor readings
//! with them, which is much more accurate than the typical sensor voltage and slope from the data sheet
//! (SLAU445I 21.2.7.8, p. 556: "The temperature sensor offset error can be large and must be calibrated for
//! most applications"; SLASE59F Table 5-22 note 2, p. 36, and SLASEE4C Table 5-22 note 2, p. 39: "for
//! higher accuracy").

use crate::pmm::ReferenceVoltage;

#[inline(always)]
fn read(addr: usize) -> u16 { unsafe { core::ptr::read_volatile(addr as *const u16) } }

// ADC calibration addresses, the same on all devices (SLASEC4D Table 6-70, p. 108; SLASEO7C Table 9-30,
// p. 72; SLASE59F Table 6-22, p. 60; SLASEE4C Table 6-18, p. 61)
const ADC_GAIN_FACTOR: usize = 0x1A16;
const ADC_OFFSET: usize = 0x1A18;
const TEMP_15V_30C: usize = 0x1A1A;

/// The factory calibration of the internal temperature sensor: the ADC counts it gave at 30 °C and at
/// a high temperature, 105 °C on the MSP430FR2x5x and MSP430FR247x and 85 °C on the MSP430FR2433 and
/// MSP430FR25x2, measured against the internal reference. (SLAU445I 1.13.3.3, p. 60: "The temperature
/// sensor is calibrated using the internal voltage references"; 105 °C: SLASEC4D Table 6-70 note 3, p. 108,
/// and SLASEO7C Table 9-30, p. 72; 85 °C: SLASE59F Table 6-22, p. 60, and SLASEE4C Table 6-18, p. 61)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct TempSensorCalibration {
    count_30c: u16,
    count_high: u16,
    high_celsius: i16,
    /// Tenths of a degree per count, in 1/65536ths: worked out once, so that each conversion takes a
    /// multiplication instead of a division, which the MSP430 has no hardware for
    scale_q16: i32,
}

impl TempSensorCalibration {
    /// The calibration for readings against the internal reference at `vref`.
    #[inline]
    pub fn new(vref: ReferenceVoltage) -> Self {
        // Pairs of counts at 30 °C and at the high temperature, one pair per reference level (1.5 V at
        // 1A1Ah, 2.0 V at 1A1Eh and 2.5 V at 1A22h: SLASEC4D Table 6-70, p. 108; SLASEO7C Table 9-30, p. 72)
        let addr = TEMP_15V_30C + 4 * vref as usize;
        // The high calibration temperature: 105 °C (SLASEC4D Table 6-70 note 3, p. 108; SLASEO7C Table 9-30,
        // p. 72) or 85 °C (SLASE59F Table 6-22, p. 60; SLASEE4C Table 6-18, p. 61)
        #[cfg(feature = "enhanced_ref")]
        let high_celsius = 105;
        #[cfg(not(feature = "enhanced_ref"))]
        let high_celsius = 85;
        let count_30c = read(addr);
        let count_high = read(addr + 2);
        let span = count_high as i32 - count_30c as i32;
        // 75 or 55 °C over the counts between the two calibration points, in tenths of a degree (SLAU445I
        // 1.13.3.3, p. 60: Equations 9 and 10)
        // A blank calibration (both counts equal) gives 0
        let scale_q16 = ((high_celsius as i32 - 30) * 10 * 65536).checked_div(span).unwrap_or(0);
        TempSensorCalibration { count_30c, count_high, high_celsius, scale_q16 }
    }

    /// The count at 30 °C
    #[inline]
    pub fn count_30c(&self) -> u16 { self.count_30c }

    /// The count at [`high_celsius`](TempSensorCalibration::high_celsius)
    #[inline]
    pub fn count_high(&self) -> u16 { self.count_high }

    /// The high calibration temperature in °C
    #[inline]
    pub fn high_celsius(&self) -> i16 { self.high_celsius }

    /// Convert a reading of the temperature sensor to tenths of a degree Celsius, rounded to the
    /// nearest tenth (SLAU445I 1.13.3.3, p. 60: Equations 9 and 10).
    ///
    /// The reading must be taken as the calibration was: against the same internal reference
    /// ([`PositiveReference::Internal`](crate::adc::PositiveReference::Internal)), at the ADC's full
    /// resolution (12-bit, or 10-bit on the MSP430FR2433 and MSP430FR25x2), in the unsigned format and
    /// with a sample time of at least 30 µs (SLAU445I 21.2.7.8, p. 556: "the sample period must be greater
    /// than 30 µs").
    #[inline]
    pub fn decicelsius(&self, count: u16) -> i16 {
        let delta = count as i32 - self.count_30c as i32;
        // Wrapping, so a garbled calibration gives a garbled value rather than a panic
        let tenths = delta.wrapping_mul(self.scale_q16).wrapping_add(1 << 15) >> 16;
        // Plus 30 °C, the lower calibration point (SLAU445I 1.13.3.3, p. 60: Equations 9 and 10)
        (tenths + 300) as i16
    }
}

/// The ADC gain correction factor, in 1/32768ths: multiply results by it and divide by 32768 (SLAU445I
/// 1.13.3.2, p. 60: Equation 6). The factory measured it at 2.4 V and room temperature, with external
/// references on VeREF+ and VeREF- (SLASEO7C Table 9-30 note 3, p. 72; the other data sheets don't give
/// these conditions), so other settings can need a different factor.
#[inline]
pub fn adc_gain_factor() -> u16 { read(ADC_GAIN_FACTOR) }

/// The ADC offset correction, in counts: add it to results after the gain correction (SLAU445I 1.13.3.2,
/// p. 60: Equations 4 and 7; it is "stored as a twos-complement number"). Measured as
/// [`adc_gain_factor()`] was (SLASEO7C Table 9-30 note 4, p. 72).
#[inline]
pub fn adc_offset() -> i16 { read(ADC_OFFSET) as i16 }

/// The correction factor of the internal reference at `vref`, in 1/32768ths: multiply results
/// measured against it by the factor and divide by 32768 (SLAU445I 1.13.3, p. 59, in "1.5-V Reference
/// Calibration": Equations 2 and 3).
#[inline]
pub fn reference_factor(vref: ReferenceVoltage) -> u16 {
    // The 1.5 V, 2.0 V and 2.5 V factors at 1A28h, 1A2Ah and 1A2Ch (SLASEC4D Table 6-70, p. 108; SLASEO7C
    // Table 9-30, p. 72)
    #[cfg(feature = "enhanced_ref")]
    const REF_15V_FACTOR: usize = 0x1A28;
    // The 1.5 V factor only, at 1A20h (SLASE59F Table 6-22, p. 61; SLASEE4C Table 6-18, p. 62)
    #[cfg(not(feature = "enhanced_ref"))]
    const REF_15V_FACTOR: usize = 0x1A20;
    read(REF_15V_FACTOR + 2 * vref as usize)
}
