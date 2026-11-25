//! Analog to Digital Converter (ADC)
//!
//! Begin configuration by calling [`AdcConfig::new()`] or [`::default()`](AdcConfig::default()).
//! Once fully configured an [`Adc`] will be returned.
//!
//! [`Adc`] can read from a channel by calling [`read_count()`](Adc::read_count()) to return an ADC count.
//! Counts are `u16`, or `i16` if [`AdcConfig::signed()`] selected signed results (see [`ResultFormat`]).
//!
//! The [`count_to_mv()`](Adc::count_to_mv()) method is available to convert an ADC count to a voltage in millivolts,
//! given a reference voltage.
//!
//! As a convenience, [`read_voltage_mv()`](Adc::read_voltage_mv()) combines [`read_count()`](Adc::read_count()) and
//! [`count_to_mv()`](Adc::count_to_mv()).
//!
//! The ADC measures against AVCC, the operating voltage of the MSP430, unless
//! [`with_reference()`](Adc::with_reference()) selects the internal shared reference or an external one on the
//! VeREF+ (P1.0) and VeREF- (P1.2) pins (ADCSREFx, reset to AVCC and AVSS: SLAU445I Table 21-8, p. 567; the
//! pins are A0/Veref+ and A2/Veref- in the ADC channel tables listed below).
//!
//! [`read_count()`](Adc::read_count()) takes a reference to the GPIO pin corresponding to the relevant ADC channel
//! to ensure it's been correctly configured. The ADC inputs are (data sheets, pin function tables):
//!
//! | Device       | Resolution | Channels 0 to 7 | Channels 8 to 11            | Pin mode          |
//! |--------------|------------|-----------------|-----------------------------|-------------------|
//! | MSP430FR2x5x | 12-bit     | P1.0 to P1.7    | P5.0 to P5.3                | `to_alternate3()` |
//! | MSP430FR247x | 12-bit     | P1.0 to P1.7    | P4.3, P4.4, P5.3, P5.4      | `to_alternate3()` |
//! | MSP430FR2433 | 10-bit     | P1.0 to P1.7    |                             | `to_adc_mode()`   |
//! | MSP430FR25x2 | 10-bit     | P1.0 to P1.3, P2.2 to P2.5 |                  | `to_adc_mode()`   |
//!
//! The table follows each data sheet's ADC section with its ADC Channel Connections table, and its pin
//! function tables (`to_alternate3()` is PxSELx = 11 there, the tertiary module function: SLAU445I
//! Table 8-3, p. 314):
//! - MSP430FR2x5x: SLASEC4D 6.10.12, p. 77; SLASEC4D Table 6-21, p. 77; SLASEC4D Table 6-63, p. 96;
//!   SLASEC4D Table 6-67, p. 104
//! - MSP430FR247x: SLASEO7C 9.10.12, p. 62; SLASEO7C Table 9-19, p. 62; SLASEO7C Table 9-23, p. 65;
//!   SLASEO7C Table 9-26, p. 68; SLASEO7C Table 9-27, p. 69
//! - MSP430FR2433: SLASE59F 6.10.12, p. 53; SLASE59F Table 6-15, p. 53; SLASE59F Table 6-17, p. 55
//! - MSP430FR25x2: SLASEE4C 6.10.12, p. 55; SLASEE4C Table 6-13, p. 55 to p. 56; SLASEE4C Table 6-15,
//!   p. 58; SLASEE4C Table 6-16, p. 60
//!
//! On the MSP430FR2433 and MSP430FR25x2 the analog inputs are enabled through SYSCFG2.ADCPCTLx instead of the
//! pin's function select bits, which is what `to_adc_mode()` does (SLASE59F Table 6-17 note 2, p. 55;
//! SLASEE4C Table 6-15, p. 58; SLAU445I Table 1-31, p. 82).
//!
//! ADC channels 12 to 15 are not associated with external pins, so instead channels 12 and 13 can be read by passing a
//! reference to [`InternalTempSensor`] or [`InternalVRef`] respectively. Channels 14 and 15 require no prior
//! configuration, so the two functions below provide a reference that can be used to read from these channels.
//! The ADC channel tables listed above give channel 12 as the temperature sensor, 13 as the internal
//! reference, 14 as DVSS and 15 as DVCC.

use crate::_pac;
use crate::{
    clock::{Aclk, Smclk},
    pmm::{InternalTempSensor, InternalVRef, VrefOutput},
};
use core::{convert::Infallible, marker::PhantomData};

#[cfg(feature = "embedded-hal-02")]
pub use embedded_hal_02::adc::Channel;

#[cfg(not(feature = "embedded-hal-02"))]
/// A marker trait to identify MCU pins that can be used as inputs to an ADC channel.
///
/// This marker trait denotes an object, i.e. a GPIO pin, that is ready for use as an input to the
/// ADC. As ADCs channels can be supplied by multiple pins, this trait defines the relationship
/// between the physical interface and the ADC sampling buffer.
///
/// ```
/// # use std::marker::PhantomData;
/// # use embedded_hal::adc::Channel;
///
/// struct Adc1; // Example ADC with single bank of 8 channels
/// struct Gpio1Pin1<MODE>(PhantomData<MODE>);
/// struct Analog(()); // marker type to denote a pin in "analog" mode
///
/// // GPIO 1 pin 1 can supply an ADC channel when it is configured in Analog mode
/// impl Channel<Adc1> for Gpio1Pin1<Analog> {
///     fn channel() -> u8 { 7 } // GPIO pin 1 is connected to ADC channel 7
/// }
/// ```
pub trait Channel<ADC> {
    /// Type denoting the method used to identify ADC channels. This may be an integer (e.g. single ADC bank), or a tuple (multiple ADC banks), etc.
    /// On the MSP430 this is a `u8`, but this type remains generic for compatibility reasons with embedded-hal v0.2.7.
    type ID;
    /// Channel ID type
    ///
    /// A type used to identify this ADC channel. For example, if the ADC has eight channels, this
    /// might be a `u8`. If the ADC has multiple banks of channels, it could be a tuple, like
    /// `(u8: bank_id, u8: channel_id)`.
    /// Get the specific ID that identifies this channel, for example `0_u8` for the first ADC channel
    fn channel() -> u8;
}
/// Marker trait that marks a pin as being capable of being an ADC input via ADCPCTLx (SYSCFG2: SLAU445I
/// Table 1-31, p. 82).
// This trait is used to mark a pin as being capable of moving between PxSEL modes and ADCPCTLx mode.
pub trait AdcPctlCapable {
    /// The corresponding ADCPCTL bit that represents this pin: ADCPCTLx enables input Ax (SLAU445I
    /// Table 1-31, p. 82).
    const ADCPCTLX: u8;
}

/// How many ADCCLK cycles the ADC's sample-and-hold stage will last for (ADCSHTx: SLAU445I Table 21-3,
/// p. 561).
///
/// Default: 8 cycles
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum SampleTime {
    /// Sample for 4 ADCCLK cycles
    _4 = 0b0000,
    /// Sample for 8 ADCCLK cycles
    #[default]
    _8 = 0b0001,
    /// Sample for 16 ADCCLK cycles
    _16 = 0b0010,
    /// Sample for 32 ADCCLK cycles
    _32 = 0b0011,
    /// Sample for 64 ADCCLK cycles
    _64 = 0b0100,
    /// Sample for 96 ADCCLK cycles
    _96 = 0b0101,
    /// Sample for 128 ADCCLK cycles
    _128 = 0b0110,
    /// Sample for 192 ADCCLK cycles
    _192 = 0b0111,
    /// Sample for 256 ADCCLK cycles
    _256 = 0b1000,
    /// Sample for 384 ADCCLK cycles
    _384 = 0b1001,
    /// Sample for 512 ADCCLK cycles (ADCSHTx = 1010b: SLAU445I Table 21-3, p. 561)
    _512 = 0b1010,
    /// Sample for 768 ADCCLK cycles (ADCSHTx = 1011b: SLAU445I Table 21-3, p. 561)
    _768 = 0b1011,
    /// Sample for 1024 ADCCLK cycles (ADCSHTx = 1100b; 1101b to 1111b give 1024 cycles too: SLAU445I
    /// Table 21-3, p. 561)
    _1024 = 0b1100,
}

impl SampleTime {
    #[inline(always)]
    fn adcsht(self) -> u8 { self as u8 }
}

/// How much the ADC input clock will be divided by after being divided by the predivider (ADCDIVx: SLAU445I
/// Table 21-4, p. 563)
///
/// Default: Divide by 1
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum ClockDivider {
    /// Divide the input clock by 1
    #[default]
    _1 = 0b000,
    /// Divide the input clock by 2
    _2 = 0b001,
    /// Divide the input clock by 3
    _3 = 0b010,
    /// Divide the input clock by 4
    _4 = 0b011,
    /// Divide the input clock by 5
    _5 = 0b100,
    /// Divide the input clock by 6
    _6 = 0b101,
    /// Divide the input clock by 7
    _7 = 0b110,
    /// Divide the input clock by 8
    _8 = 0b111,
}

impl ClockDivider {
    #[inline(always)]
    fn adcdiv(self) -> u8 { self as u8 }
}

// ADCSSELx (SLAU445I Table 21-4, p. 564)
#[derive(Default, Copy, Clone, PartialEq, Eq)]
enum ClockSource {
    /// Use MODCLK as the ADC input clock
    #[default]
    ModClk = 0b00,
    /// Use ACLK as the ADC input clock
    AClk = 0b01,
    /// Use SMCLK as the ADC input clock
    SmClk = 0b10,
}

impl ClockSource {
    #[inline(always)]
    fn adcssel(self) -> u8 { self as u8 }
}

/// How much the ADC input clock will be divided by prior to being divided by the ADC clock divider (ADCPDIVx:
/// SLAU445I Table 21-5, p. 565)
///
/// Default: Divide by 1
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum Predivider {
    /// Divide the input clock by 1
    #[default]
    _1 = 0b00,
    /// Divide the input clock by 4
    _4 = 0b01,
    /// Divide the input clock by 64
    _64 = 0b10,
}

impl Predivider {
    #[inline(always)]
    fn adcpdiv(self) -> u8 { self as u8 }
}

/// The output resolution of the ADC conversion. Also determines how many ADCCLK cycles the conversion step takes
/// (ADCRES: SLAU445I Table 21-5, p. 565).
///
/// Default: 10-bit resolution
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum Resolution {
    /// 8-bit ADC conversion result. The conversion step takes 10 ADCCLK cycles.
    _8BIT = 0b00,
    /// 10-bit ADC conversion result. The conversion step takes 12 ADCCLK cycles.
    #[default]
    _10BIT = 0b01,
    #[cfg(feature = "adc12bit")]
    /// 12-bit ADC conversion result. The conversion step takes 14 ADCCLK cycles.
    _12BIT = 0b10,
}

impl Resolution {
    #[inline(always)]
    fn adcres(self) -> u8 { self as u8 }
}

/// Selects the drive capability of the ADC reference buffer, which can increase the maximum sampling speed at the cost of increased power draw.
/// (ADCSR: SLAU445I Table 21-5, p. 565, and SLAU445I 21.2.3.1, p. 542)
///
/// Default: 200ksps
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum SamplingRate {
    /// Maximum of 50 ksps. Lower power usage.
    _50KSPS,
    /// Maximum of 200 ksps. Higher power usage.
    #[default]
    _200KSPS,
}

impl SamplingRate {
    #[inline(always)]
    fn adcsr(self) -> bool {
        match self {
            SamplingRate::_200KSPS => false,
            SamplingRate::_50KSPS => true,
        }
    }
}

/// The format of the conversion results (ADCDF: SLAU445I Table 21-5, p. 565): [`Unsigned`], the default,
/// or [`Signed`], selected with [`AdcConfig::signed()`]. It is a type parameter of [`AdcConfig`] and
/// [`Adc`], so the methods that return or take results use `u16` for unsigned results and `i16` for signed
/// ones.
pub trait ResultFormat: sealed::SealedResultFormat {
    /// The type of a result: `u16` for [`Unsigned`], `i16` for [`Signed`]
    type Count: Copy;
    #[doc(hidden)]
    const ADCDF: bool;
    #[doc(hidden)]
    fn from_bits(bits: u16) -> Self::Count;
    #[doc(hidden)]
    fn to_bits(count: Self::Count) -> u16;
    /// The unsigned count of `count`, for results of `resolution` bits
    #[doc(hidden)]
    fn to_unsigned(count: Self::Count, resolution: u32) -> u16;
}

/// Typestate for unsigned, right-aligned results, as after reset: from 0 at VR- up to 255, 1023 or 4095
/// at VR+ (SLAU445I 21.2.1, p. 541, and SLAU445I 21.3.4, p. 566)
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Unsigned;

/// Typestate for signed results, two's complement and left-aligned in an `i16`: from -32768 at VR- up to
/// just below 32768 at VR+. The low bits are 0: 8 bits for an 8-bit result, 6 for 10-bit and 4 for 12-bit
/// (SLAU445I 21.3.5, p. 566; ADCDF in SLAU445I Table 21-5, p. 565: "-VREF results in 8000h, and ... +VREF
/// results in 7FC0h").
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Signed;

impl ResultFormat for Unsigned {
    type Count = u16;
    const ADCDF: bool = false;
    #[inline(always)]
    fn from_bits(bits: u16) -> u16 { bits }
    #[inline(always)]
    fn to_bits(count: u16) -> u16 { count }
    #[inline(always)]
    fn to_unsigned(count: u16, _resolution: u32) -> u16 { count }
}

impl ResultFormat for Signed {
    type Count = i16;
    const ADCDF: bool = true;
    #[inline(always)]
    fn from_bits(bits: u16) -> i16 { bits as i16 }
    #[inline(always)]
    fn to_bits(count: i16) -> u16 { count as u16 }
    // Right-aligned again, and offset by half the range (SLAU445I 21.3.5, p. 566)
    #[inline(always)]
    fn to_unsigned(count: i16, resolution: u32) -> u16 {
        ((count >> (16 - resolution)) + (1 << (resolution - 1))) as u16
    }
}

mod sealed {
    pub trait SealedResultFormat {}
    impl SealedResultFormat for super::Unsigned {}
    impl SealedResultFormat for super::Signed {}
}

// Pins corresponding to an ADC channel. Pin types can have `::channel()` called on them to get their ADC channel index.
macro_rules! impl_adc_channel_pin {
    ($port: ty, $pin: ty, $mode:tt => $channel: literal ) => {
        impl<DIR> Channel<Adc> for Pin<$port, $pin, $mode<DIR>> {
            type ID = u8;

            fn channel() -> Self::ID { $channel }
        }
        // On the MSP430FR2433 and MSP430FR25x2 ADC functionality is done via ADCPCTLx instead of
        // Alternate1/2/3 (SLASE59F Table 6-17, p. 55; SLASEE4C Table 6-15, p. 58, and SLASEE4C Table 6-16,
        // p. 60). The MSP430FR247x has no SAC either, but uses PxSELx = 11 (SLASEO7C Table 9-23, p. 65).
        // ADCPCTLx enables input Ax (SLAU445I Table 1-31, p. 82).
        // Implement this for all modes
        #[cfg(feature = "adcpctl")]
        impl<MODE> AdcPctlCapable for Pin<$port, $pin, MODE> {
            const ADCPCTLX: u8 = $channel;
        }
    };
}
pub(crate) use impl_adc_channel_pin;

// A few ADC channels don't correspond to pins.
macro_rules! impl_adc_channel_extra {
    ($type: ty, $channel: literal ) => {
        impl Channel<Adc> for $type {
            type ID = u8;

            fn channel() -> Self::ID { $channel }
        }
    };
}

// Channel 12 is the temperature sensor, 13 the internal reference (SLASEC4D Table 6-21, p. 77; SLASEO7C
// Table 9-19, p. 62; SLASE59F Table 6-15, p. 53; SLASEE4C Table 6-13, p. 55; ADCINCHx = 1100b for the
// sensor: SLAU445I 21.2.7.8, p. 556)
impl_adc_channel_extra!(InternalTempSensor<'_>, 12);
impl_adc_channel_extra!(InternalVRef, 13);

// The VREF+ output is measured through its pin's channel (SLASEC4D 6.10.1, p. 67: "ADC channel 7 can also be
// selected to monitor this voltage"; SLASEO7C Table 9-19 note 1, p. 62; SLASE59F 6.10.1, p. 45; SLASEE4C
// 6.10.1, p. 49)
impl<PIN: Channel<Adc, ID = u8>> Channel<Adc> for VrefOutput<PIN> {
    type ID = u8;

    fn channel() -> Self::ID { PIN::channel() }
}

// Users needn't deal with the structs themselves so it just adds noise to the docs. We instead document the functions below.
#[doc(hidden)]
pub struct AdcVssChannel;
impl_adc_channel_extra!(AdcVssChannel, 14);
/// ADC channel 14, tied to VSS (DVSS: SLASEC4D Table 6-21, p. 77; SLASEO7C Table 9-19, p. 62; SLASE59F
/// Table 6-15, p. 53; SLASEE4C Table 6-13, p. 56). Pass this function's output to `read_count()`.
#[inline(always)]
pub fn adc_ch14_vss() -> AdcVssChannel { AdcVssChannel }

#[doc(hidden)]
pub struct AdcVccChannel;
impl_adc_channel_extra!(AdcVccChannel, 15);
/// ADC channel 15, tied to VCC (DVCC: SLASEC4D Table 6-21, p. 77; SLASEO7C Table 9-19, p. 62; SLASE59F
/// Table 6-15, p. 53; SLASEE4C Table 6-13, p. 56). Pass this function's output to `read_count()`.
#[inline(always)]
pub fn adc_ch15_vcc() -> AdcVccChannel { AdcVccChannel }

/// Typestate for an ADC configuration with no clock source selected
pub struct NoClockSet;
/// Typestate for an ADC configuration with a clock source selected
pub struct ClockSet(ClockSource);

/// Configuration object for an ADC.
///
/// The default configuration is based on the default register values (SLAU445I Table 21-3, p. 561 to
/// p. 562; SLAU445I Table 21-4, p. 563 to p. 564; SLAU445I Table 21-5, p. 565):
/// - Predivider = 1 and clock divider = 1
/// - 10-bit resolution
/// - 8 cycle sample time, the reset value of the 12-bit ADC (the 10-bit ADC of the MSP430FR2433 and
///   MSP430FR25x2 resets to 4 cycles: SLAU445I Table 21-3 note 1, p. 561)
/// - Max 200 ksps sample rate
/// - Unsigned results, see [`signed()`](AdcConfig::signed())
#[derive(Clone, PartialEq, Eq)]
pub struct AdcConfig<STATE, FMT = Unsigned> {
    state: STATE,
    /// How much the input clock is divided by, after the predivider (ADCDIVx: SLAU445I Table 21-4, p. 563).
    pub clock_divider: ClockDivider,
    /// How much the input clock is initially divided by, before the clock divider (ADCPDIVx: SLAU445I
    /// Table 21-5, p. 565).
    pub predivider: Predivider,
    /// How many bits the conversion result is. Also defines the number of ADCCLK cycles required to do the conversion step.
    /// (ADCRES: SLAU445I Table 21-5, p. 565)
    pub resolution: Resolution,
    /// Sets the maximum sampling rate of the ADC. Lower values use less power. (ADCSR: SLAU445I Table 21-5,
    /// p. 565)
    pub sampling_rate: SamplingRate,
    /// Determines the number of ADCCLK cycles the sampling time takes (ADCSHTx: SLAU445I Table 21-3, p. 561).
    pub sample_time: SampleTime,
    format: PhantomData<FMT>,
}

// Only implement Default for NoClockSet
impl Default for AdcConfig<NoClockSet> {
    fn default() -> Self {
        Self {
            state: NoClockSet,
            clock_divider: Default::default(),
            predivider: Default::default(),
            resolution: Default::default(),
            sampling_rate: Default::default(),
            sample_time: Default::default(),
            format: PhantomData,
        }
    }
}

impl AdcConfig<NoClockSet> {
    /// Creates an ADC configuration. A default implementation is also available through `::default()`
    pub fn new(
        clock_divider: ClockDivider,
        predivider: Predivider,
        resolution: Resolution,
        sampling_rate: SamplingRate,
        sample_time: SampleTime,
    ) -> AdcConfig<NoClockSet> {
        AdcConfig {
            state: NoClockSet,
            clock_divider,
            predivider,
            resolution,
            sampling_rate,
            sample_time,
            format: PhantomData,
        }
    }
}

impl<STATE> AdcConfig<STATE> {
    /// Return signed results, as `i16`, instead of unsigned ones (ADCDF: SLAU445I Table 21-5, p. 565). See
    /// [`Signed`].
    pub fn signed(self) -> AdcConfig<STATE, Signed> {
        AdcConfig {
            state: self.state,
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            format: PhantomData,
        }
    }
}

impl<FMT> AdcConfig<NoClockSet, FMT> {
    /// Configure the ADC to use SMCLK (ADCSSELx: SLAU445I Table 21-4, p. 564)
    pub fn use_smclk(self, _smclk: &Smclk) -> AdcConfig<ClockSet, FMT> {
        AdcConfig {
            state: ClockSet(ClockSource::SmClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            format: self.format,
        }
    }
    /// Configure the ADC to use ACLK (ADCSSELx: SLAU445I Table 21-4, p. 564)
    ///
    /// On the MSP430FR2433 and MSP430FR25x2, temperature sensor readings taken in LPM3 with ACLK as the
    /// ADC clock may be wrong: "When ACLK is used as ADC clock source and device is in LPM3 mode while
    /// sampling the on-chip temperature sensor, the ADC may generate erroneous conversion results". The
    /// erratum's workarounds are SMCLK or MODCLK as the ADC clock, with "A 100us sampling time" if the
    /// conversion is triggered from LPM3, or LPM0 or active mode (SLAZ664S ADC50; SLAZ705H ADC50).
    pub fn use_aclk(self, _aclk: &Aclk) -> AdcConfig<ClockSet, FMT> {
        AdcConfig {
            state: ClockSet(ClockSource::AClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            format: self.format,
        }
    }
    /// Configure the ADC to use MODCLK (ADCSSELx: SLAU445I Table 21-4, p. 564)
    pub fn use_modclk(self) -> AdcConfig<ClockSet, FMT> {
        AdcConfig {
            state: ClockSet(ClockSource::ModClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            format: self.format,
        }
    }
}
impl<FMT: ResultFormat> AdcConfig<ClockSet, FMT> {
    /// Applies this ADC configuration to hardware registers, and returns an ADC.
    pub fn configure(self, mut adc_reg: _pac::Adc) -> Adc<AvccReference, FMT> {
        // Disable the ADC before we set the other bits. Some can only be set while the ADC is disabled.
        // (SLAU445I 21.2.1, p. 541: "the ADC control bits can be modified only when ADCENC = 0")
        disable_adc_reg(&mut adc_reg);

        let adcsht = self.sample_time.adcsht();
        adc_reg.adcctl0().write(|w| unsafe { w.adcsht().bits(adcsht) });
        // AVCC and AVSS as reference, as the returned `Adc` says, and channel 0 (ADCSREFx = 000b,
        // ADCINCHx = 0: SLAU445I Table 21-8, p. 567)
        adc_reg.adcmctl0().write(|w| unsafe { w.bits(0) });

        let adcssel = self.state.0.adcssel();
        let adcdiv = self.clock_divider.adcdiv();
        // ADCSHP = 1: the sampling timer sets the sample time, pulse sample mode (SLAU445I 21.2.5.2, p. 544)
        adc_reg.adcctl1().write(|w| { unsafe { w
            .adcssel().bits(adcssel)
            .adcshp().set_bit()
            .adcdiv().bits(adcdiv) 
        }});

        let adcpdiv = self.predivider.adcpdiv();
        let adcres = self.resolution.adcres();
        let adcsr = self.sampling_rate.adcsr();
        // ADCDF as a bit: the PACs name its values differently, and the MSP430FR2433 PAC has no enum for it
        adc_reg.adcctl2().write(|w| { unsafe { w
            .adcpdiv().bits(adcpdiv)
            .adcres().bits(adcres)
            .adcdf().bit(FMT::ADCDF)
            .adcsr().bit(adcsr)
        }});

        Adc { adc_reg, pending: None, reference: PhantomData, format: PhantomData }
    }
}

/// Typestate for an ADC that measures against AVCC and AVSS, as after reset (ADCSREFx: SLAU445I Table 21-8,
/// p. 567)
pub struct AvccReference;
/// Typestate for an ADC with a reference selected by [`Adc::with_reference()`], which borrows the
/// internal reference or the VeREF pins for `'a`
pub struct SelectedReference<'a>(PhantomData<&'a ()>);

/// Marker trait for the VeREF+ pin (P1.0) in its analog mode, which supplies an external positive reference
/// (A0/Veref+: SLASEC4D Table 6-21, p. 77; SLASEO7C Table 9-19, p. 62; SLASE59F Table 6-15, p. 53; SLASEE4C
/// Table 6-13, p. 55)
pub trait VeRefPlusPin {}
/// Marker trait for the VeREF- pin (P1.2) in its analog mode, which supplies an external negative reference
/// (A2/Veref-: SLASEC4D Table 6-21, p. 77; SLASEO7C Table 9-19, p. 62; SLASE59F Table 6-15, p. 53; SLASEE4C
/// Table 6-13, p. 55)
pub trait VeRefMinusPin {}

/// The positive reference of the ADC, VR+: an input at or above it converts to the full-scale count (ADCSREF,
/// SLAU445I 21.2.1, p. 541, SLAU445I 21.2.3, p. 542, and SLAU445I Table 21-8, p. 567)
pub enum PositiveReference<'a> {
    /// AVCC, as after reset
    Avcc,
    /// The internal shared reference, see [`Pmm::enable_internal_reference()`](crate::pmm::Pmm::enable_internal_reference)
    Internal(&'a InternalVRef),
    /// An external reference on the VeREF+ pin, through the ADC's reference buffer
    ExternalBuffered(&'a dyn VeRefPlusPin),
    /// An external reference on the VeREF+ pin, unbuffered
    External(&'a dyn VeRefPlusPin),
}

/// The negative reference of the ADC, VR-: an input at or below it converts to 0 (ADCSREF, SLAU445I 21.2.1,
/// p. 541, SLAU445I 21.2.3, p. 542, and SLAU445I Table 21-8, p. 567)
pub enum NegativeReference<'a> {
    /// AVSS, as after reset
    Avss,
    /// An external reference on the VeREF- pin
    External(&'a dyn VeRefMinusPin),
}

/// Controls the onboard ADC. The `read()` method is available through the embedded_hal `OneShot` trait.
///
/// `REF` tracks the reference selected with [`Adc::with_reference()`], and `FMT` the [`ResultFormat`].
pub struct Adc<REF = AvccReference, FMT = Unsigned> {
    adc_reg: _pac::Adc,
    /// Channel of the conversion that was started but not read yet
    pending: Option<u8>,
    reference: PhantomData<REF>,
    format: PhantomData<FMT>,
}

impl<REF, FMT: ResultFormat> Adc<REF, FMT> {
    /// Whether the ADC is currently sampling or converting (ADCBUSY: SLAU445I Table 21-4, p. 564).
    pub fn is_busy(&self) -> bool {
        self.adc_reg.adcctl1().read().adcbusy().bit_is_set()
    }

    /// Gets the latest ADC conversion result (ADCMEM0: SLAU445I 21.3.4, p. 566).
    pub fn get_result(&self) -> FMT::Count { FMT::from_bits(self.adc_reg.adcmem0().read().bits()) }

    /// Enables this ADC, ready to start conversions (ADCON: SLAU445I Table 21-3, p. 561).
    pub fn enable(&mut self) {
        unsafe {
            self.adc_reg.adcctl0().set_bits(|w| w.adcon().set_bit());
        }
    }

    /// Disables this ADC to save power (SLAU445I 21.2.1, p. 541: "The ADC can be turned off when not in
    /// use to save power").
    pub fn disable(&mut self) { disable_adc_reg(&mut self.adc_reg); }

    /// Selects which pin to sample (ADCINCHx: SLAU445I Table 21-8, p. 567).
    fn set_pin<PIN>(&mut self, _pin: &PIN)
    where PIN: Channel<Adc, ID = u8> {
        self.adc_reg.adcmctl0().modify(|_, w|
            unsafe { w.adcinch().bits(PIN::channel()) }
        );
    }

    /// Starts an ADC conversion (SLAU445I Table 21-3, p. 562: "ADCSC and ADCENC may be set together with one
    /// instruction").
    fn start_conversion(&mut self) {
        unsafe {
            self.adc_reg.adcctl0().set_bits(|w| w
                .adcenc().set_bit()
                .adcsc().set_bit());
        }
    }

    /// Begins a single ADC conversion if one isn't already underway, enabling the ADC in the process.
    ///
    /// If the result is ready it is returned as an ADC count, otherwise returns `WouldBlock`
    ///
    /// A conversion that is still pending for another channel is finished first and its result
    /// discarded.
    pub fn read_count<PIN>(&mut self, pin: &mut PIN) -> nb::Result<FMT::Count, Infallible>
    where PIN: Channel<Adc, ID = u8> {
        if let Some(pending) = self.pending {
            if self.is_busy() {
                return Err(nb::Error::WouldBlock);
            }
            self.pending = None;
            if pending == PIN::channel() {
                return Ok(self.get_result());
            }
        }
        self.disable();
        self.set_pin(pin);
        self.enable();

        self.start_conversion();
        self.pending = Some(PIN::channel());
        Err(nb::Error::WouldBlock)
    }

    /// Convert an ADC count to a voltage value in millivolts.
    ///
    /// `ref_voltage_mv` is the reference voltage of the ADC in millivolts. As in the conversion formula of
    /// SLAU445I 21.2.1, p. 541, the reference voltage corresponds to a count of 256, 1024 or 4096. With an
    /// external negative reference, the result is the voltage above VR-. [`Signed`] counts are converted
    /// too.
    pub fn count_to_mv(&self, count: FMT::Count, ref_voltage_mv: u16) -> u16 {
        use crate::_pac::adc::adcctl2::Adcres;
        let bits = match self.adc_reg.adcctl2().read().adcres().variant() {
            Adcres::Adcres0 => 8,
            Adcres::Adcres1 => 10,
            Adcres::Adcres2 => 12,
            Adcres::Adcres3 => 12, // Reserved, unreachable
        };
        let count = FMT::to_unsigned(count, bits);
        ((count as u32 * ref_voltage_mv as u32) >> bits) as u16
    }

    /// Begins a single ADC conversion if one isn't already underway, enabling the ADC in the process.
    ///
    /// If the result is ready it is returned as a voltage in millivolts based on `ref_voltage_mv`, otherwise returns `WouldBlock`.
    ///
    /// If you instead want a raw count you should use the `.read_count()` method.
    pub fn read_voltage_mv<PIN: Channel<Adc, ID = u8>>(
        &mut self,
        pin: &mut PIN,
        ref_voltage_mv: u16,
    ) -> nb::Result<u16, Infallible> {
        self.read_count(pin).map(|count| self.count_to_mv(count, ref_voltage_mv))
    }

    /// Select the reference voltages the ADC measures against (ADCSREF: SLAU445I Table 21-8, p. 567). An
    /// input at or below the negative reference converts to 0, one at or above the positive reference to the
    /// full-scale count (SLAU445I 21.2.1, p. 541).
    ///
    /// The internal reference must stay enabled, and the VeREF pins in their analog mode, while the ADC
    /// uses them, so the returned ADC borrows them (SLAU445I 21.2.3.1, p. 542: "The on-chip reference from
    /// the PMM module must be enabled by software"). Waits for a conversion in progress to finish.
    pub fn with_reference<'a>(
        mut self,
        positive: PositiveReference<'a>,
        negative: NegativeReference<'a>,
    ) -> Adc<SelectedReference<'a>, FMT> {
        // ADCSREFx: bits 1-0 of the field select VR+, bit 2 selects VEREF- as VR- (SLAU445I Table 21-8,
        // p. 567)
        let vr_plus: u8 = match positive {
            PositiveReference::Avcc => 0b00,
            PositiveReference::Internal(_) => 0b01,
            PositiveReference::ExternalBuffered(_) => 0b10,
            PositiveReference::External(_) => 0b11,
        };
        let vr_minus: u8 = match negative {
            NegativeReference::Avss => 0,
            NegativeReference::External(_) => 1,
        };
        // "It is not recommended to change this setting while a conversion is ongoing" (SLAU445I Table 21-8,
        // p. 567)
        while self.is_busy() {}
        self.disable();
        self.pending = None;
        self.adc_reg.adcmctl0().modify(|_, w| w.adcsref().set(vr_minus << 2 | vr_plus));
        Adc { adc_reg: self.adc_reg, pending: None, reference: PhantomData, format: PhantomData }
    }
}

// Clears ADCON and ADCENC (SLAU445I Table 21-3, p. 561 to p. 562)
fn disable_adc_reg(adc: &mut _pac::Adc) {
    unsafe {
        adc.adcctl0().clear_bits(|w| w
            .adcon().clear_bit()
            .adcenc().clear_bit());
    }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use super::*;
    use embedded_hal_02::adc::{Channel, OneShot};

    impl<REF, FMT: ResultFormat, PIN> OneShot<Adc, FMT::Count, PIN> for Adc<REF, FMT>
    where PIN: Channel<Adc, ID = u8>
    {
        type Error = Infallible; // Only returns WouldBlock

        /// Begins a single ADC conversion if one isn't already underway, enabling the ADC in the process.
        ///
        /// If the result is ready it is returned as an ADC count, otherwise returns `WouldBlock`
        #[inline(always)]
        fn read(&mut self, pin: &mut PIN) -> nb::Result<FMT::Count, Self::Error> { self.read_count(pin) }
    }
}
