//! Analog to Digital Converter (ADC)
//!
//! Begin configuration by calling [`AdcConfig::new()`] or [`::default()`](AdcConfig::default()).
//! Once fully configured an [`Adc`] will be returned.
//!
//! [`Adc`] can read from a channel by calling [`read_count()`](Adc::read_count()) to return an ADC count.
//!
//! The [`count_to_mv()`](Adc::count_to_mv()) method is available to convert an ADC count to a voltage in millivolts,
//! given a reference voltage.
//!
//! As a convenience, [`read_voltage_mv()`](Adc::read_voltage_mv()) combines [`read_count()`](Adc::read_count()) and
//! [`count_to_mv()`](Adc::count_to_mv()).
//!
//! The ADC measures against AVCC, the operating voltage of the MSP430, unless
//! [`with_reference()`](Adc::with_reference()) selects the internal shared reference or an external one on the
//! VeREF+ (P1.0) and VeREF- (P1.2) pins.
//!
//! Besides single conversions with [`read_count()`](Adc::read_count()), [`start()`](Adc::start()) converts a
//! sequence of channels and repeats conversions, started by software, the RTC, a timer or the comparator. The
//! window comparator ([`set_window()`](Adc::set_window())) flags results outside or inside a range, and
//! [`enable_interrupts()`](Adc::enable_interrupts()) requests the `ADC` interrupt for these events.
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
//! On the MSP430FR2433 and MSP430FR25x2 the analog inputs are enabled through SYSCFG2.ADCPCTLx instead of the
//! pin's function select bits, which is what `to_adc_mode()` does.
//!
//! ADC channels 12 to 15 are not associated with external pins, so instead channels 12 and 13 can be read by passing a
//! reference to [`InternalTempSensor`] or [`InternalVRef`] respectively. Channels 14 and 15 require no prior
//! configuration, so the two functions below provide a reference that can be used to read from these channels.

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
/// Marker trait that marks a pin as being capable of being an ADC input via ADCPCTLx.
// This trait is used to mark a pin as being capable of moving between PxSEL modes and ADCPCTLx mode.
pub trait AdcPctlCapable {
    /// The corresponding ADCPCTL bit that represents this pin.
    const ADCPCTLX: u8;
}

/// How many ADCCLK cycles the ADC's sample-and-hold stage will last for.
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
    /// Sample for 512 ADCCLK cycles
    _512 = 0b1010,
    /// Sample for 768 ADCCLK cycles
    _768 = 0b1011,
    /// Sample for 1024 ADCCLK cycles
    _1024 = 0b1100,
}

impl SampleTime {
    #[inline(always)]
    fn adcsht(self) -> u8 { self as u8 }
}

/// How much the ADC input clock will be divided by after being divided by the predivider
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

/// How much the ADC input clock will be divided by prior to being divided by the ADC clock divider
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

/// The output resolution of the ADC conversion. Also determines how many ADCCLK cycles the conversion step takes.
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

/// How conversion results and window comparator thresholds are formatted (ADCDF).
///
/// Default: unsigned
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum DataFormat {
    /// Unsigned and right-aligned: from 0 at VR- up to 255, 1023 or 4095 at VR+.
    #[default]
    Unsigned,
    /// Two's complement and left-aligned, as an `i16` would read it: from -32768 at VR- up to just below 32768 at VR+.
    /// The low bits are 0: 8 bits for an 8-bit result, 6 for 10-bit and 4 for 12-bit.
    Signed,
}

// Pins corresponding to an ADC channel. Pin types can have `::channel()` called on them to get their ADC channel index.
macro_rules! impl_adc_channel_pin {
    ($port: ty, $pin: ty, $mode:tt => $channel: literal ) => {
        impl<DIR> Channel<Adc> for Pin<$port, $pin, $mode<DIR>> {
            type ID = u8;

            fn channel() -> Self::ID { $channel }
        }
        // If the device doesn't have SAC, then ADC functionality is done via ADCPCTLx instead of Alternate1/2/3.
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

impl_adc_channel_extra!(InternalTempSensor<'_>, 12);
impl_adc_channel_extra!(InternalVRef, 13);

// The VREF+ output is measured through its pin's channel
impl<PIN: Channel<Adc, ID = u8>> Channel<Adc> for VrefOutput<PIN> {
    type ID = u8;

    fn channel() -> Self::ID { PIN::channel() }
}

// Users needn't deal with the structs themselves so it just adds noise to the docs. We instead document the functions below.
#[doc(hidden)]
pub struct AdcVssChannel;
impl_adc_channel_extra!(AdcVssChannel, 14);
/// ADC channel 14, tied to VSS. Pass this function's output to `read_count()`.
#[inline(always)]
pub fn adc_ch14_vss() -> AdcVssChannel { AdcVssChannel }

#[doc(hidden)]
pub struct AdcVccChannel;
impl_adc_channel_extra!(AdcVccChannel, 15);
/// ADC channel 15, tied to VCC. Pass this function's output to `read_count()`.
#[inline(always)]
pub fn adc_ch15_vcc() -> AdcVccChannel { AdcVccChannel }

/// Typestate for an ADC configuration with no clock source selected
pub struct NoClockSet;
/// Typestate for an ADC configuration with a clock source selected
pub struct ClockSet(ClockSource);

/// `x / (2^BITS - 1)`, rounded down, from shifts: the MSP430 has no divider, and a library division
/// takes several hundred cycles. `(x + x / 2^BITS) / 2^BITS` is never above the quotient, and at most
/// one below it for the products `count_to_mv` divides; the loop makes up the difference.
#[inline(always)]
fn div_by_full_scale<const BITS: u32>(x: u32) -> u32 {
    let full_scale = (1 << BITS) - 1;
    let mut quotient = (x + (x >> BITS)) >> BITS;
    let mut remainder = x - ((quotient << BITS) - quotient);
    while remainder >= full_scale {
        quotient += 1;
        remainder -= full_scale;
    }
    quotient
}

/// Configuration object for an ADC.
///
/// The default configuration is based on the default register values:
/// - Predivider = 1 and clock divider = 1
/// - 10-bit resolution
/// - 8 cycle sample time
/// - Max 200 ksps sample rate
/// - Unsigned results
#[derive(Clone, PartialEq, Eq)]
pub struct AdcConfig<STATE> {
    state: STATE,
    /// How much the input clock is divided by, after the predivider.
    pub clock_divider: ClockDivider,
    /// How much the input clock is initially divided by, before the clock divider.
    pub predivider: Predivider,
    /// How many bits the conversion result is. Also defines the number of ADCCLK cycles required to do the conversion step.
    pub resolution: Resolution,
    /// Sets the maximum sampling rate of the ADC. Lower values use less power.
    pub sampling_rate: SamplingRate,
    /// Determines the number of ADCCLK cycles the sampling time takes.
    pub sample_time: SampleTime,
    /// The format of conversion results and window comparator thresholds.
    pub data_format: DataFormat,
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
            data_format: Default::default(),
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
            data_format: DataFormat::Unsigned,
        }
    }
    /// Configure the ADC to use SMCLK
    pub fn use_smclk(self, _smclk: &Smclk) -> AdcConfig<ClockSet> {
        AdcConfig {
            state: ClockSet(ClockSource::SmClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            data_format: self.data_format,
        }
    }
    /// Configure the ADC to use ACLK
    pub fn use_aclk(self, _aclk: &Aclk) -> AdcConfig<ClockSet> {
        AdcConfig {
            state: ClockSet(ClockSource::AClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            data_format: self.data_format,
        }
    }
    /// Configure the ADC to use MODCLK
    pub fn use_modclk(self) -> AdcConfig<ClockSet> {
        AdcConfig {
            state: ClockSet(ClockSource::ModClk),
            clock_divider: self.clock_divider,
            predivider: self.predivider,
            resolution: self.resolution,
            sampling_rate: self.sampling_rate,
            sample_time: self.sample_time,
            data_format: self.data_format,
        }
    }
}
impl AdcConfig<ClockSet> {
    /// Applies this ADC configuration to hardware registers, and returns an ADC.
    pub fn configure(self, mut adc_reg: _pac::Adc) -> Adc {
        // Disable the ADC before we set the other bits. Some can only be set while the ADC is disabled.
        disable_adc_reg(&mut adc_reg);

        let adcsht = self.sample_time.adcsht();
        adc_reg.adcctl0().write(|w| unsafe { w.adcsht().bits(adcsht) });
        // AVCC and AVSS as reference, as the returned `Adc` says, and channel 0
        adc_reg.adcmctl0().write(|w| unsafe { w.bits(0) });

        let adcssel = self.state.0.adcssel();
        let adcdiv = self.clock_divider.adcdiv();
        adc_reg.adcctl1().write(|w| { unsafe { w
            .adcssel().bits(adcssel)
            .adcshp().set_bit()
            .adcdiv().bits(adcdiv) 
        }});

        let adcpdiv = self.predivider.adcpdiv();
        let adcres = self.resolution.adcres();
        let adcsr = self.sampling_rate.adcsr();
        let adcdf = self.data_format == DataFormat::Signed;
        adc_reg.adcctl2().write(|w| { unsafe { w
            .adcpdiv().bits(adcpdiv)
            .adcres().bits(adcres)
            .adcdf().bit(adcdf)
            .adcsr().bit(adcsr)
        }});

        Adc { adc_reg, pending: None, reference: PhantomData }
    }
}

/// Typestate for an ADC that measures against AVCC and AVSS, as after reset
pub struct AvccReference;
/// Typestate for an ADC with a reference selected by [`Adc::with_reference()`], which borrows the
/// internal reference or the VeREF pins for `'a`
pub struct SelectedReference<'a>(PhantomData<&'a ()>);

/// Marker trait for the VeREF+ pin (P1.0) in its analog mode, which supplies an external positive reference
pub trait VeRefPlusPin {}
/// Marker trait for the VeREF- pin (P1.2) in its analog mode, which supplies an external negative reference
pub trait VeRefMinusPin {}

/// The positive reference of the ADC, VR+: an input at or above it converts to the full-scale count (ADCSREF,
/// user's guide 21.2.3)
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

/// The negative reference of the ADC, VR-: an input at or below it converts to 0 (ADCSREF, user's guide 21.2.3)
pub enum NegativeReference<'a> {
    /// AVSS, as after reset
    Avss,
    /// An external reference on the VeREF- pin
    External(&'a dyn VeRefMinusPin),
}

/// How conversions repeat (ADCCONSEQ, user's guide 21.2.7)
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum ConversionMode {
    /// Convert the channel once. With a hardware trigger, start again for the next conversion.
    #[default]
    Single,
    /// Convert the channels from the selected one down to channel 0, once
    Sequence,
    /// Convert the channel once for each trigger, until stopped
    RepeatSingle,
    /// Convert the channels from the selected one down to channel 0 for each trigger, until stopped
    RepeatSequence,
}

/// Marker trait for the timer whose capture/compare register 1 output starts conversions with
/// [`TriggerSource::Timer`]: TB1 on the MSP430FR2x5x, TA1 on the other devices (data sheets: ADC Trigger Signal
/// Connections)
pub trait AdcTriggerTimer {}

/// What starts conversions (ADCSHS, data sheets: ADC Trigger Signal Connections)
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum TriggerSource {
    /// Software, through [`Adc::start()`] (ADCSC)
    #[default]
    Software,
    /// RTC counter overflows
    Rtc,
    /// The output of capture/compare register 1 of TB1 on the MSP430FR2x5x, or TA1 on the other devices. Set
    /// that timer up for PWM, with a pin or with
    /// [`PwmUninit::into_adc_trigger()`](crate::pwm::PwmUninit::into_adc_trigger).
    Timer,
    /// The output of eCOMP0
    #[cfg(feature = "ecomp")]
    Comparator,
}

/// How a trigger controls the sampling (ADCSHP, ADCISSH, user's guide 21.2.5)
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum SampleMode {
    /// A rising edge starts sampling for the configured sample time (pulse sample mode)
    #[default]
    RisingEdge,
    /// A falling edge starts sampling for the configured sample time (pulse sample mode, inverted trigger)
    FallingEdge,
    /// Sample while the trigger is high, and convert when it goes low (extended sample mode). The trigger must
    /// stay high for at least 4 ADCCLK cycles. Hardware triggers only.
    WhileHigh,
    /// Sample while the trigger is low, and convert when it goes high (extended sample mode, inverted trigger).
    /// Hardware triggers only.
    WhileLow,
}

/// Settings for [`Adc::start()`]. The default converts once, started by software.
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub struct ConversionConfig {
    /// How conversions repeat
    pub mode: ConversionMode,
    /// What starts the conversions
    pub trigger: TriggerSource,
    /// How the trigger controls sampling
    pub sample_mode: SampleMode,
    /// In the sequence and repeat modes, convert back to back after the first trigger, as fast as possible,
    /// instead of waiting for a trigger for each conversion (ADCMSC). In the repeat modes the conversions
    /// then continue until stopped.
    pub back_to_back: bool,
}

bitflags::bitflags! {
    /// ADC interrupt sources, for [`Adc::enable_interrupts()`] and [`Adc::interrupt_flags()`] (ADCIE, ADCIFG)
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct AdcInterruptFlags: u16 {
        /// ADCIFG0. A conversion result is ready. Reading it clears this flag.
        const ResultReady  = 1 << 0;
        /// ADCINIFG. The result is inside the window: from the low threshold up to the high threshold.
        const InsideWindow = 1 << 1;
        /// ADCLOIFG. The result is below the low threshold of the window.
        const BelowWindow  = 1 << 2;
        /// ADCHIIFG. The result is above the high threshold of the window.
        const AboveWindow  = 1 << 3;
        /// ADCOVIFG. A result overwrote one that hadn't been read.
        const Overflow     = 1 << 4;
        /// ADCTOVIFG. A trigger arrived before the conversion had finished.
        const TimeOverflow = 1 << 5;
    }
}

/// The highest-priority pending ADC interrupt, as read from ADCIV by [`Adc::interrupt_source()`]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcVector {
    /// No interrupt pending
    None,
    /// A result overwrote one that hadn't been read
    Overflow,
    /// A trigger arrived before the conversion had finished
    TimeOverflow,
    /// The result is above the high threshold of the window
    AboveWindow,
    /// The result is below the low threshold of the window
    BelowWindow,
    /// The result is inside the window
    InsideWindow,
    /// A conversion result is ready. This flag stays set until the result is read.
    ResultReady,
}

// ADCCTL0, ADCCTL1 and ADCMCTL0 fields (user's guide 21.3)
const ADCMSC: u16 = 1 << 7;
const ADCENC: u16 = 1 << 1;
const ADCSC: u16 = 1 << 0;
const ADCSHS_MASK: u16 = 0b11 << 10;
const ADCSHP: u16 = 1 << 9;
const ADCISSH: u16 = 1 << 8;
const ADCCONSEQ_MASK: u16 = 0b11 << 1;
const ADCSREF_MASK: u16 = 0b111 << 4;
const ADCINCH_MASK: u16 = 0b1111;

/// Controls the onboard ADC. The `read()` method is available through the embedded_hal `OneShot` trait.
///
/// `REF` tracks the reference selected with [`Adc::with_reference()`].
pub struct Adc<REF = AvccReference> {
    adc_reg: _pac::Adc,
    /// Channel of the conversion that was started but not read yet
    pending: Option<u8>,
    reference: PhantomData<REF>,
}

impl<REF> Adc<REF> {
    /// Whether the ADC is currently sampling or converting.
    pub fn adc_is_busy(&self) -> bool {
        self.adc_reg.adcctl1().read().adcbusy().bit_is_set()
    }

    /// Gets the latest ADC conversion result.
    pub fn adc_get_result(&self) -> u16 { self.adc_reg.adcmem0().read().bits() }

    /// Enables this ADC, ready to start conversions.
    pub fn enable(&mut self) {
        unsafe {
            self.adc_reg.adcctl0().set_bits(|w| w.adcon().set_bit());
        }
    }

    /// Disables this ADC to save power.
    pub fn disable(&mut self) { disable_adc_reg(&mut self.adc_reg); }

    /// Selects which pin to sample.
    fn set_pin<PIN>(&mut self, _pin: &PIN)
    where PIN: Channel<Adc, ID = u8> {
        self.adc_reg.adcmctl0().modify(|_, w|
            unsafe { w.adcinch().bits(PIN::channel()) }
        );
    }

    /// Starts an ADC conversion.
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
    pub fn read_count<PIN>(&mut self, pin: &mut PIN) -> nb::Result<u16, Infallible>
    where PIN: Channel<Adc, ID = u8> {
        if let Some(pending) = self.pending {
            if self.adc_is_busy() {
                return Err(nb::Error::WouldBlock);
            }
            self.pending = None;
            if pending == PIN::channel() {
                return Ok(self.adc_get_result());
            }
        }
        self.disable();
        // A single conversion started by software, as `start()` may have set otherwise
        self.adc_reg.adcctl1().modify(|r, w| unsafe {
            w.bits(r.bits() & !(ADCSHS_MASK | ADCISSH | ADCCONSEQ_MASK) | ADCSHP)
        });
        self.adc_reg.adcctl0().modify(|r, w| unsafe { w.bits(r.bits() & !ADCMSC) });
        self.set_pin(pin);
        self.enable();

        self.start_conversion();
        self.pending = Some(PIN::channel());
        Err(nb::Error::WouldBlock)
    }

    /// Convert an ADC count to a voltage value in millivolts, rounded down.
    ///
    /// `ref_voltage_mv` is the reference voltage of the ADC in millivolts. The full-scale count
    /// (255, 1023 or 4095) corresponds to the reference voltage (user's guide, ADC conversion
    /// formula). With an external negative reference, this is the voltage above VR-. A count in the
    /// signed [`DataFormat`] is converted too.
    pub fn count_to_mv(&self, count: u16, ref_voltage_mv: u16) -> u16 {
        use crate::_pac::adc::adcctl2::Adcres;
        let ctl2 = self.adc_reg.adcctl2().read();
        let bits = match ctl2.adcres().variant() {
            Adcres::Adcres0 => 8,
            Adcres::Adcres1 => 10,
            Adcres::Adcres2 => 12,
            Adcres::Adcres3 => 12, // Reserved, unreachable
        };
        let count = if ctl2.adcdf().bit_is_set() {
            // Left-aligned two's complement, offset by half the range
            (((count as i16) >> (16 - bits)) + (1 << (bits - 1))) as u16
        } else {
            count
        };
        let product = count as u32 * ref_voltage_mv as u32;
        let mv = match bits {
            8 => div_by_full_scale::<8>(product),
            10 => div_by_full_scale::<10>(product),
            _ => div_by_full_scale::<12>(product),
        };
        mv as u16
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

    /// Select the reference voltages the ADC measures against (ADCSREF). An input at or below the
    /// negative reference converts to 0, one at or above the positive reference to the full-scale count.
    ///
    /// The internal reference must stay enabled, and the VeREF pins in their analog mode, while the ADC
    /// uses them, so the returned ADC borrows them. Waits for a conversion in progress to finish.
    pub fn with_reference<'a>(
        mut self,
        positive: PositiveReference<'a>,
        negative: NegativeReference<'a>,
    ) -> Adc<SelectedReference<'a>> {
        let vr_plus: u16 = match positive {
            PositiveReference::Avcc => 0b00,
            PositiveReference::Internal(_) => 0b01,
            PositiveReference::ExternalBuffered(_) => 0b10,
            PositiveReference::External(_) => 0b11,
        };
        let vr_minus: u16 = match negative {
            NegativeReference::Avss => 0,
            NegativeReference::External(_) => 1,
        };
        while self.adc_is_busy() {}
        self.disable();
        self.pending = None;
        self.adc_reg.adcmctl0().modify(|r, w| unsafe {
            w.bits(r.bits() & !ADCSREF_MASK | (vr_minus << 6 | vr_plus << 4))
        });
        Adc { adc_reg: self.adc_reg, pending: None, reference: PhantomData }
    }

    /// Start conversions of `pin`'s channel, or in the sequence modes of the channels from it down to
    /// channel 0, as `config` describes. Read the results with [`result()`](Adc::result()).
    ///
    /// A sequence converts every channel down to 0, so their pins should be in their analog mode too.
    /// Conversions already running are stopped first, and their results discarded.
    pub fn start<PIN>(&mut self, _pin: &mut PIN, config: ConversionConfig)
    where PIN: Channel<Adc, ID = u8> {
        self.disable();
        self.pending = None;

        let shs: u16 = match config.trigger {
            TriggerSource::Software => 0b00,
            TriggerSource::Rtc => 0b01,
            TriggerSource::Timer => 0b10,
            #[cfg(feature = "ecomp")]
            TriggerSource::Comparator => 0b11,
        };
        let sample = match (config.trigger, config.sample_mode) {
            // The software trigger is a pulse
            (TriggerSource::Software, _) => ADCSHP,
            (_, SampleMode::RisingEdge) => ADCSHP,
            (_, SampleMode::FallingEdge) => ADCSHP | ADCISSH,
            (_, SampleMode::WhileHigh) => 0,
            (_, SampleMode::WhileLow) => ADCISSH,
        };
        let conseq: u16 = match config.mode {
            ConversionMode::Single => 0b00,
            ConversionMode::Sequence => 0b01,
            ConversionMode::RepeatSingle => 0b10,
            ConversionMode::RepeatSequence => 0b11,
        };
        self.adc_reg.adcctl1().modify(|r, w| unsafe {
            w.bits(r.bits() & !(ADCSHS_MASK | ADCSHP | ADCISSH | ADCCONSEQ_MASK) | shs << 10 | sample | conseq << 1)
        });
        let msc = if config.back_to_back { ADCMSC } else { 0 };
        self.adc_reg.adcctl0().modify(|r, w| unsafe { w.bits(r.bits() & !ADCMSC | msc) });
        self.adc_reg.adcmctl0().modify(|r, w| unsafe {
            w.bits(r.bits() & !ADCINCH_MASK | PIN::channel() as u16)
        });
        // Discard results and flags of earlier conversions
        self.adc_reg.adcifg().write(|w| unsafe { w.bits(0) });

        self.enable();
        let start = match config.trigger {
            TriggerSource::Software => ADCENC | ADCSC,
            _ => ADCENC,
        };
        self.adc_reg.adcctl0().modify(|r, w| unsafe { w.bits(r.bits() | start) });
    }

    /// The next result of the conversions started with [`start()`](Adc::start()), or `WouldBlock` if
    /// none is ready (ADCIFG0). Reading a result clears the flag.
    ///
    /// Results that aren't read before the next one arrives are lost, see
    /// [`AdcInterruptFlags::Overflow`].
    pub fn result(&mut self) -> nb::Result<u16, Infallible> {
        if self.adc_reg.adcifg().read().adcifg0().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(self.adc_get_result())
    }

    /// Stop the conversions started with [`start()`](Adc::start()), after the current conversion in the
    /// single modes and after the current sequence in the sequence modes (user's guide 21.2.7.6).
    pub fn stop(&mut self) {
        let single = self.adc_reg.adcctl1().read().bits() & ADCCONSEQ_MASK == 0;
        if single {
            // Clearing ADCENC would cut a single conversion short
            while self.adc_is_busy() {}
        }
        self.adc_reg.adcctl0().modify(|r, w| unsafe { w.bits(r.bits() & !ADCENC) });
    }

    /// Set the window comparator thresholds (ADCLO, ADCHI), in the configured [`DataFormat`]. Each result
    /// then sets one of the flags [`AdcInterruptFlags::BelowWindow`], [`InsideWindow`](AdcInterruptFlags::InsideWindow)
    /// and [`AboveWindow`](AdcInterruptFlags::AboveWindow).
    ///
    /// The ADC only sets these flags, so clear them with [`clear_interrupt_flags()`](Adc::clear_interrupt_flags())
    /// once handled.
    pub fn set_window(&mut self, low: u16, high: u16) {
        self.adc_reg.adclo().write(|w| unsafe { w.bits(low) });
        self.adc_reg.adchi().write(|w| unsafe { w.bits(high) });
    }

    /// Request the `ADC` interrupt for `flags`, besides those already enabled (ADCIE).
    pub fn enable_interrupts(&mut self, flags: AdcInterruptFlags) {
        self.adc_reg.adcie().modify(|r, w| unsafe { w.bits(r.bits() | flags.bits()) });
    }

    /// Stop requesting the `ADC` interrupt for `flags` (ADCIE).
    pub fn disable_interrupts(&mut self, flags: AdcInterruptFlags) {
        self.adc_reg.adcie().modify(|r, w| unsafe { w.bits(r.bits() & !flags.bits()) });
    }

    /// The interrupt flags that are set, whether or not their interrupt is enabled (ADCIFG).
    pub fn interrupt_flags(&self) -> AdcInterruptFlags {
        AdcInterruptFlags::from_bits_truncate(self.adc_reg.adcifg().read().bits())
    }

    /// Clear `flags` (ADCIFG).
    pub fn clear_interrupt_flags(&mut self, flags: AdcInterruptFlags) {
        self.adc_reg.adcifg().modify(|r, w| unsafe { w.bits(r.bits() & !flags.bits()) });
    }

    /// The highest-priority pending interrupt among the enabled ones (ADCIV). Reading it clears its flag,
    /// except [`AdcVector::ResultReady`], which reading the result clears.
    pub fn interrupt_source(&mut self) -> AdcVector {
        match self.adc_reg.adciv().read().bits() {
            0x02 => AdcVector::Overflow,
            0x04 => AdcVector::TimeOverflow,
            0x06 => AdcVector::AboveWindow,
            0x08 => AdcVector::BelowWindow,
            0x0A => AdcVector::InsideWindow,
            0x0C => AdcVector::ResultReady,
            _ => AdcVector::None,
        }
    }
}

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

    impl<REF, PIN> OneShot<Adc, u16, PIN> for Adc<REF>
    where PIN: Channel<Adc, ID = u8>
    {
        type Error = Infallible; // Only returns WouldBlock

        /// Begins a single ADC conversion if one isn't already underway, enabling the ADC in the process.
        ///
        /// If the result is ready it is returned as an ADC count, otherwise returns `WouldBlock`
        #[inline(always)]
        fn read(&mut self, pin: &mut PIN) -> nb::Result<u16, Self::Error> { self.read_count(pin) }
    }
}
