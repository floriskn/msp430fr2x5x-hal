//! PWM ports
//!
//! Configures the board's TimerB peripherals into PWM ports. Each PWM port consists of multiple PWM
//! pins which all share the same period but have their own duty cycles.
//!
//! Each PWM pin starts off in an "uninitialized" state and must be initialized by passing in the
//! appropriate alternate-function GPIO pin. Only initialized pins can be used for PWM.

use crate::gpio::AlternatePin;
use crate::hw_traits::timer_base::{CCRn, Outmod};
use crate::pin_mapping::{DefaultMapping, PinMap};
use crate::timer::{CapCmpTimer3, CapCmpTimer7};
use core::marker::PhantomData;

pub use crate::timer::{
    CapCmp, CascadeOutput, TimerConfig, TimerDiv, TimerExDiv, TimerPeriph, CCR0, CCR1, CCR2, CCR3,
    CCR4, CCR5, CCR6,
};

// Sealed by CapCmp
/// Associates PWM pins with specific GPIO pins, for pin mapping `M`
pub trait PwmPeriph<C, M: PinMap = DefaultMapping>: CapCmp<C> + CapCmp<CCR0> {
    /// GPIO type, in the alternate function that outputs the PWM signal
    type Gpio: AlternatePin;
}

fn setup_pwm<T: TimerPeriph<M>, M: PinMap>(timer: &T, config: TimerConfig<T, M>, period: u16) {
    config.write_regs(timer);
    CCRn::<CCR0>::set_ccrn(timer, period);
    CCRn::<CCR0>::config_outmod(timer, Outmod::Toggle);
}

/// Collection of uninitialized PWM pins derived from timer peripheral with 3 capture-compare registers
pub struct PwmParts3<T: CapCmpTimer3<M>, M: PinMap = DefaultMapping> {
    /// PWM pin 1 (derived from capture-compare register 1)
    pub pwm1: PwmUninit<T, CCR1, M>,
    /// PWM pin 2 (derived from capture-compare register 2)
    pub pwm2: PwmUninit<T, CCR2, M>,
    _pin_map: PhantomData<M>,
}

impl<T: CapCmpTimer3<M>, M: PinMap> PwmParts3<T, M> {
    /// Create uninitialized PWM pins with the same period. The timer counts from 0 up to and
    /// including `period`, so each PWM period is `period + 1` timer clock cycles.
    pub fn new(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        // Configure PWM ports
        CCRn::<CCR1>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR2>::config_outmod(&timer, Outmod::ResetSet);
        // Start the timer to run PWM
        timer.upmode();
        Self { pwm1: PwmUninit::new(), pwm2: PwmUninit::new(), _pin_map: PhantomData }
    }
}

/// Collection of uninitialized PWM pins derived from timer peripheral with 7 capture-compare registers
pub struct PwmParts7<T: CapCmpTimer7<M>, M: PinMap = DefaultMapping> {
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
    /// including `period`, so each PWM period is `period + 1` timer clock cycles.
    pub fn new(timer: T, config: TimerConfig<T, M>, period: u16) -> Self {
        setup_pwm(&timer, config, period);
        // Configure PWM ports
        CCRn::<CCR1>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR2>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR3>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR4>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR5>::config_outmod(&timer, Outmod::ResetSet);
        CCRn::<CCR6>::config_outmod(&timer, Outmod::ResetSet);
        // Start the timer to run PWM
        timer.upmode();
        Self {
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

/// The PWM period in timer clock cycles: the timer counts from 0 up to and including CCR0. With
/// CCR0 at 65535 the period is 65536 cycles, which a `u16` can't hold, so 100 % isn't reachable.
#[inline]
fn max_duty<T: CapCmp<CCR0>>() -> u16 {
    let timer = unsafe { T::steal() };
    CCRn::<CCR0>::get_ccrn(&timer).saturating_add(1)
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
