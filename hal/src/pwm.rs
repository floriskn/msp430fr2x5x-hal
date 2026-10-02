//! PWM ports
//!
//! Configures the board's TimerB peripherals into PWM ports. Each PWM port consists of multiple PWM
//! pins which all share the same period but have their own duty cycles.
//!
//! Each PWM pin starts off in an "uninitialized" state and must be initialized by passing in the
//! appropriate alternate-function GPIO pin. Only initialized pins can be used for PWM.
//!
//! The outputs go high at the start of each period with [`PwmParts3::new`], or are centered on the timer's
//! return to 0 with [`PwmParts3::new_center_aligned`]. [`Pwm::set_polarity`] makes an output active low. On a
//! Timer_B a new duty cycle takes effect at the start of the next period, so no period is cut short.
//!
//! # Timer_B outputs and the comparators
//!
//! After reset the output of an eCOMP comparator switches all outputs of a Timer_B to high impedance while it is
//! high: eCOMP0 for TB0 and TB1, eCOMP1 for TB2 and TB3 (data sheets: TBxOUTH). Measured on an MSP430FR2476, a TB0
//! PWM output stops whenever eCOMP0's output is high, even with the comparator used for something else.
//! [`TimerConfig::high_impedance_trigger`](crate::timer::TimerConfig::high_impedance_trigger) selects the TBxTRG
//! pin instead, or nothing.

use crate::gpio::AlternatePin;
use crate::hw_traits::timer_base::{CCRn, Outmod, RunningMode, TimerBase};
use crate::pin_mapping::{DefaultMapping, PinMap};
use crate::timer::{CapCmpTimer3, CapCmpTimer7};
use core::marker::PhantomData;

pub use crate::timer::{
    CapCmp, CascadeOutput, TimerConfig, TimerDiv, TimerExDiv, TimerPeriph, CCR0, CCR1, CCR2, CCR3,
    CCR4, CCR5, CCR6,
};

// Sealed by CapCmp
/// Associates PWM pins with specific GPIO pins, for pin mapping `M`
pub trait PwmPeriph<C, M: PinMap = DefaultMapping>: CapCmp<C> + CapCmp<CCR0> + TimerBase {
    /// GPIO type, in the alternate function that outputs the PWM signal
    type Gpio: AlternatePin;
}

fn setup_pwm<T: TimerPeriph<M>, M: PinMap>(timer: &T, config: TimerConfig<T, M>, period: u16) {
    config.write_regs(timer);
    CCRn::<CCR0>::set_ccrn(timer, period);
    CCRn::<CCR0>::config_outmod(timer, Outmod::Toggle);
}

/// Whether PWM outputs go high at the start of each period, or are centered on the timer's return to 0
#[derive(Copy, Clone, PartialEq, Eq)]
enum Alignment {
    Edge,
    Center,
}

/// Configure a PWM channel: its output mode, and on Timer_B when its compare latch loads the duty cycle,
/// so a new duty cycle starts with a period instead of cutting one short
fn setup_channel<T: CapCmp<C>, C>(timer: &T, alignment: Alignment) {
    match alignment {
        Alignment::Edge => {
            CCRn::<C>::config_outmod(timer, Outmod::ResetSet);
            // Load when the timer counts to 0
            CCRn::<C>::set_clld(timer, 0b01);
        }
        Alignment::Center => {
            CCRn::<C>::config_outmod(timer, Outmod::ToggleReset);
            // Load when the timer counts to 0 or to the top, so both halves of a period match
            CCRn::<C>::set_clld(timer, 0b10);
        }
    }
}

/// Collection of uninitialized PWM pins derived from timer peripheral with 3 capture-compare registers
pub struct PwmParts3<T: CapCmpTimer3<M>, M: PinMap = DefaultMapping> {
    /// Square wave output of capture-compare register 0, see [`PeriodOutputUninit`]
    pub period_output: PeriodOutputUninit<T, M>,
    /// PWM pin 1 (derived from capture-compare register 1)
    pub pwm1: PwmUninit<T, CCR1, M>,
    /// PWM pin 2 (derived from capture-compare register 2)
    pub pwm2: PwmUninit<T, CCR2, M>,
    _pin_map: PhantomData<M>,
}

impl<T: CapCmpTimer3<M>, M: PinMap> PwmParts3<T, M> {
    /// Create uninitialized PWM pins with the same period. The timer counts from 0 up to and
    /// including `period`, so each PWM period is `period + 1` timer clock cycles, and each output is
    /// high at the start of the period.
    pub fn new(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        setup_channel::<T, CCR1>(&timer, Alignment::Edge);
        setup_channel::<T, CCR2>(&timer, Alignment::Edge);
        // Start the timer to run PWM
        timer.upmode();
        Self::parts()
    }

    /// Create uninitialized center-aligned PWM pins with the same period. The timer counts from 0 up
    /// to `period` and back down (up/down mode), so each PWM period is `2 * period` timer clock
    /// cycles, and each output is high for a stretch centered on the timer's return to 0. Duty cycles go
    /// up to `period`. Two outputs with nearly the same duty cycle, one of them active low, drive the two
    /// sides of a half bridge with a dead time between them (user's guide 13.2.3.5).
    pub fn new_center_aligned(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        setup_channel::<T, CCR1>(&timer, Alignment::Center);
        setup_channel::<T, CCR2>(&timer, Alignment::Center);
        timer.updown_mode();
        Self::parts()
    }

    fn parts() -> Self {
        Self {
            period_output: PeriodOutputUninit(PhantomData, PhantomData),
            pwm1: PwmUninit::new(),
            pwm2: PwmUninit::new(),
            _pin_map: PhantomData,
        }
    }
}

/// Collection of uninitialized PWM pins derived from timer peripheral with 7 capture-compare registers
pub struct PwmParts7<T: CapCmpTimer7<M>, M: PinMap = DefaultMapping> {
    /// Square wave output of capture-compare register 0, see [`PeriodOutputUninit`]
    pub period_output: PeriodOutputUninit<T, M>,
    /// PWM pin 1 (derived from capture-compare register 1)
    pub pwm1: PwmUninit<T, CCR1, M>,
    /// PWM pin 2 (derived from capture-compare register 2)
    pub pwm2: PwmUninit<T, CCR2, M>,
    /// PWM pin 3 (derived from capture-compare register 3)
    pub pwm3: PwmUninit<T, CCR3, M>,
    /// PWM pin 4 (derived from capture-compare register 4)
    pub pwm4: PwmUninit<T, CCR4, M>,
    /// PWM pin 5 (derived from capture-compare register 5)
    pub pwm5: PwmUninit<T, CCR5, M>,
    /// PWM pin 6 (derived from capture-compare register 6)
    pub pwm6: PwmUninit<T, CCR6, M>,
    _pin_map: PhantomData<M>,
}

impl<T: CapCmpTimer7<M>, M: PinMap> PwmParts7<T, M> {
    /// Create uninitialized PWM pins with the same period. The timer counts from 0 up to and
    /// including `period`, so each PWM period is `period + 1` timer clock cycles, and each output is
    /// high at the start of the period.
    pub fn new(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        Self::setup_channels(&timer, Alignment::Edge);
        timer.upmode();
        Self::parts()
    }

    /// Create uninitialized center-aligned PWM pins with the same period, see
    /// [`PwmParts3::new_center_aligned`].
    pub fn new_center_aligned(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        Self::setup_channels(&timer, Alignment::Center);
        timer.updown_mode();
        Self::parts()
    }

    fn setup_channels(timer: &T, alignment: Alignment) {
        setup_channel::<T, CCR1>(timer, alignment);
        setup_channel::<T, CCR2>(timer, alignment);
        setup_channel::<T, CCR3>(timer, alignment);
        setup_channel::<T, CCR4>(timer, alignment);
        setup_channel::<T, CCR5>(timer, alignment);
        setup_channel::<T, CCR6>(timer, alignment);
    }

    fn parts() -> Self {
        Self {
            period_output: PeriodOutputUninit(PhantomData, PhantomData),
            pwm1: PwmUninit::new(),
            pwm2: PwmUninit::new(),
            pwm3: PwmUninit::new(),
            pwm4: PwmUninit::new(),
            pwm5: PwmUninit::new(),
            pwm6: PwmUninit::new(),
            _pin_map: PhantomData,
        }
    }
}

/// The uninitialized output of capture-compare register 0, which toggles once per PWM period: a square wave at half
/// the PWM frequency (a quarter of it for center-aligned PWM), useful as a clock for other circuits.
///
/// Only some timers have a pin for it: on the MSP430FR247x TA2, TA3 and TB0 (data sheet: TA2.0, TA3.0, TB0.0).
pub struct PeriodOutputUninit<T, M = DefaultMapping>(PhantomData<T>, PhantomData<M>);

impl<T: PwmPeriph<CCR0, M>, M: PinMap> PeriodOutputUninit<T, M> {
    /// Initializes the output by passing in the appropriately configured GPIO pin.
    #[inline]
    pub fn init(self, pin: <T as PwmPeriph<CCR0, M>>::Gpio) -> PeriodOutput<T, M> { PeriodOutput { pin } }
}

/// The output of capture-compare register 0 on its pin, see [`PeriodOutputUninit`]
pub struct PeriodOutput<T: PwmPeriph<CCR0, M>, M: PinMap = DefaultMapping> {
    pin: <T as PwmPeriph<CCR0, M>>::Gpio,
}

impl<T: PwmPeriph<CCR0, M>, M: PinMap> PeriodOutput<T, M> {
    /// Disconnect the pin from the timer. It then drives its GPIO output level (PxOUT).
    #[inline]
    pub fn disable(&mut self) { self.pin.set_function_gpio(); }

    /// Connect the pin to the timer again.
    #[inline]
    pub fn enable(&mut self) { self.pin.set_function_from_type(); }
}

/// The active level of a PWM output: the level it has for the duty cycle
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum Polarity {
    /// High for the duty cycle, as set up by `PwmParts`
    #[default]
    ActiveHigh,
    /// Low for the duty cycle
    ActiveLow,
}

/// Uninitialized PWM pin
pub struct PwmUninit<T, C, M = DefaultMapping>(PhantomData<T>, PhantomData<C>, PhantomData<M>);

impl<T: PwmPeriph<C, M>, C, M: PinMap> PwmUninit<T, C, M> {
    /// Initializes the PWM pin by passing in the appropriately configured GPIO pin.
    #[inline]
    pub fn init(self, pin: <T as PwmPeriph<C, M>>::Gpio) -> Pwm<T, C, M> {
        Pwm { _timer: PhantomData, _ccrn: PhantomData, _pin_map: PhantomData, pin }
    }
}

impl<T, C, M> PwmUninit<T, C, M> {
    #[inline]
    fn new() -> Self { Self(PhantomData, PhantomData, PhantomData) }
}

impl<T: CapCmp<CCR2>, M> PwmUninit<T, CCR2, M> {
    /// Use this PWM output to clock a cascaded timer instead of a pin, see
    /// [`TimerConfig::cascade`]. The cascaded timer then counts PWM periods.
    #[inline]
    pub fn into_cascade_output(self) -> CascadeOutput<T> { CascadeOutput::new() }
}

#[cfg(feature = "adc")]
impl<T: CapCmp<CCR1> + crate::adc::AdcTriggerTimer, M> PwmUninit<T, CCR1, M> {
    /// Use this PWM output to start ADC conversions with
    /// [`TriggerSource::Timer`](crate::adc::TriggerSource::Timer) instead of driving a pin. The output is high
    /// for the first `high_cycles` timer cycles of each period, so it rises at the start of each period; it
    /// needs to be at least 1. With [`SampleMode::WhileHigh`](crate::adc::SampleMode::WhileHigh) it sets the
    /// sample time.
    #[inline]
    pub fn into_adc_trigger(self, high_cycles: u16) -> AdcTriggerOutput<T> {
        let mut output = AdcTriggerOutput(PhantomData);
        output.set_high_cycles(high_cycles);
        output
    }
}

impl<T: CapCmp<CCR2> + crate::ir::IrInputTimer, M> PwmUninit<T, CCR2, M> {
    /// Use this PWM output as an input of the infrared modulator, see [`crate::ir`], instead of driving a pin.
    /// The output is high for the first `high_cycles` timer cycles of each period.
    #[inline]
    pub fn into_ir_input(self, high_cycles: u16) -> crate::ir::IrInput<T> {
        let timer = unsafe { T::steal() };
        CCRn::<CCR2>::set_ccrn(&timer, high_cycles);
        crate::ir::IrInput(PhantomData)
    }
}

/// A PWM output that starts ADC conversions, see [`PwmUninit::into_adc_trigger()`]
pub struct AdcTriggerOutput<T>(PhantomData<T>);

impl<T: CapCmp<CCR1>> AdcTriggerOutput<T> {
    /// Change how many timer cycles the output stays high at the start of each period.
    #[inline]
    pub fn set_high_cycles(&mut self, high_cycles: u16) {
        let timer = unsafe { T::steal() };
        CCRn::<CCR1>::set_ccrn(&timer, high_cycles);
    }
}

/// The duty cycle of 100 %, in timer clock cycles. In up mode it's the period: the timer counts from 0 up
/// to and including CCR0. With CCR0 at 65535 the period is 65536 cycles, which a `u16` can't hold, so 100 %
/// isn't reachable. For center-aligned PWM (up/down mode) it's CCR0.
#[inline]
fn max_duty<T: CapCmp<CCR0> + TimerBase>() -> u16 {
    let timer = unsafe { T::steal() };
    let ccr0 = CCRn::<CCR0>::get_ccrn(&timer);
    if timer.mode_rd() == RunningMode::UpDown as u8 {
        ccr0
    } else {
        ccr0.saturating_add(1)
    }
}

/// An initialized Pwm pin
pub struct Pwm<T: PwmPeriph<C, M>, C, M: PinMap = DefaultMapping> {
    _timer: PhantomData<T>,
    _ccrn: PhantomData<C>,
    _pin_map: PhantomData<M>,
    pin: <T as PwmPeriph<C, M>>::Gpio,
}

impl<T: PwmPeriph<C, M>, C, M: PinMap> Pwm<T, C, M> {
    /// The duty cycle in timer clock cycles: the output is high for this many cycles of each period.
    #[inline]
    pub fn duty(&self) -> u16 {
        let timer = unsafe { T::steal() };
        CCRn::<C>::get_ccrn(&timer)
    }

    /// Disconnect the pin from the timer. It then drives its GPIO output level (PxOUT), for example low if it was
    /// set up with [`to_output_low()`](crate::gpio::Pin::to_output_low). The timer keeps running.
    #[inline]
    pub fn disable(&mut self) { self.pin.set_function_gpio(); }

    /// Connect the pin to the timer again.
    #[inline]
    pub fn enable(&mut self) { self.pin.set_function_from_type(); }

    /// Select the level the output has for the duty cycle. The change takes effect at once, without
    /// passing through other output modes (user's guide 13.2.5.1.3).
    #[inline]
    pub fn set_polarity(&mut self, polarity: Polarity) {
        let timer = unsafe { T::steal() };
        // Edge-aligned PWM uses reset/set (7) or set/reset (3), center-aligned toggle/reset (2) or
        // toggle/set (6). The top output mode bit picks between each pair.
        let center = CCRn::<C>::outmod_rd(&timer) & 0b011 == 0b010;
        let high_bit = (polarity == Polarity::ActiveHigh) != center;
        CCRn::<C>::set_outmod_high_bit(&timer, high_bit);
    }
}

mod ehal1 {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::pwm::{ErrorType, SetDutyCycle};

    impl<T: PwmPeriph<C, M>, C, M: PinMap> ErrorType for Pwm<T, C, M> {
        type Error = Infallible;
    }

    impl<T: PwmPeriph<C, M>, C, M: PinMap> SetDutyCycle for Pwm<T, C, M> {
        /// The PWM period in timer clock cycles, `period + 1`. A duty cycle of 0 keeps the output
        /// low and the maximum keeps it high.
        #[inline]
        fn max_duty_cycle(&self) -> u16 { max_duty::<T>() }

        /// Set the duty cycle to `duty / max_duty`.
        ///
        /// The caller is responsible for ensuring that the duty cycle value is less than or equal to the maximum duty cycle value,
        /// as reported by `max_duty_cycle`.
        ///
        /// As the error type is `Infallible` this can be safely unwrapped.
        #[inline]
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            let timer = unsafe { T::steal() };
            CCRn::<C>::set_ccrn(&timer, duty);
            Ok(())
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use super::*;
    use embedded_hal_02::PwmPin;

    impl<T: PwmPeriph<C, M>, C, M: PinMap> PwmPin for Pwm<T, C, M> {
        /// Number of cycles
        type Duty = u16;

        #[inline]
        fn set_duty(&mut self, duty: Self::Duty) {
            let timer = unsafe { T::steal() };
            CCRn::<C>::set_ccrn(&timer, duty);
        }

        #[inline]
        fn get_duty(&self) -> Self::Duty { self.duty() }

        /// The PWM period in timer clock cycles, `period + 1`. A duty of 0 keeps the output low and
        /// the maximum keeps it high.
        #[inline]
        fn get_max_duty(&self) -> Self::Duty { max_duty::<T>() }

        #[inline]
        fn disable(&mut self) { Pwm::disable(self) }

        #[inline]
        fn enable(&mut self) { Pwm::enable(self) }
    }
}
