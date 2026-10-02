//! Countdown timers
//!
//! Configures the board's TimerB peripherals into periodic countdown timers. Each peripheral
//! consists of a main timer and multiple "sub-timers". Sub-timers have their own thresholds and
//! interrupts but share their countdowns with their main timer.
//!
//! This module also contains traits used by other HAL modules that depend on TimerB, such as
//! `Capture` and `Pwm`.

use crate::clock::{Aclk, Smclk};
use crate::hw_traits::timer_base::{CCRn, Outmod, RunningMode, Tbssel, TimerBase};
use crate::pin_mapping::*;
use core::convert::Infallible;
use core::marker::PhantomData;

pub use crate::hw_traits::timer_base::{
    TimerDiv, TimerExDiv, CCR0, CCR1, CCR2, CCR3, CCR4, CCR5, CCR6,
};

// Trait effectively sealed by CCRn
/// Trait indicating that the peripheral can be used as a sub-timer, PWM, or capture
pub trait CapCmp<C>: CCRn<C> {}
impl<T: CCRn<C>, C> CapCmp<C> for T {}

// Trait effectively sealed by TimerB
/// Trait indicating that the peripheral can be used as a timer
pub trait TimerPeriph<M: PinMap = DefaultMapping>: TimerBase + CapCmp<CCR0> {
    /// Pin type used for external TBxCLK of this timer, or [`NoTbxclkPin`] if it has none
    type Tbxclk;

    /// Additional configuration
    #[inline(always)]
    fn configure_pin_mapping() {}
}

/// The external TBxCLK pin of timers that don't have one, such as TA2 and TA3 on the
/// MSP430FR2433
///
/// It has no values, so [`TimerConfig::tbclk`] can't be called for these timers.
pub enum NoTbxclkPin {}

// Traits effectively sealed by CCRn
/// Trait indicating that the peripheral has 2 capture compare registers
pub trait CapCmpTimer2<M: PinMap = DefaultMapping>: TimerPeriph<M> + CapCmp<CCR1> {}
/// Trait indicating that the peripheral has 3 capture compare registers
pub trait CapCmpTimer3<M: PinMap = DefaultMapping>:
    TimerPeriph<M> + CapCmp<CCR1> + CapCmp<CCR2>
{}
/// Trait indicating that the peripheral has 7 capture compare registers
pub trait CapCmpTimer7<M: PinMap = DefaultMapping>:
    TimerPeriph<M>
    + CapCmp<CCR1>
    + CapCmp<CCR2>
    + CapCmp<CCR3>
    + CapCmp<CCR4>
    + CapCmp<CCR5>
    + CapCmp<CCR6>
{}

// Traits effectively sealed by TimerBase
/// Trait indicating that the timer can be clocked from VLOCLK, see [`TimerConfig::vloclk`]
///
/// A timer's fourth clock input, INCLK, is wired differently on each device and each timer. These
/// timers have the VLO on it (data sheet, timer signal connections):
///
/// | Device                     | Timers   |
/// |----------------------------|----------|
/// | MSP430FR2475, MSP430FR2476 | TA0, TA2 |
/// | MSP430FR2512, MSP430FR2522 | TA0      |
/// | Other devices              | None     |
pub trait VloclkTimer: TimerBase {}

/// Trait indicating that the timer can be clocked by another timer, see [`TimerConfig::cascade`]
///
/// A timer's fourth clock input, INCLK, is wired differently on each device and each timer. These
/// timers have the CCR2 output of their `Source` timer on it (data sheet, timer signal
/// connections):
///
/// | Device                     | Timer ← `Source`     |
/// |----------------------------|----------------------|
/// | MSP430FR2475, MSP430FR2476 | TA1 ← TA0, TA3 ← TA2 |
/// | MSP430FR2512, MSP430FR2522 | TA1 ← TA0            |
/// | MSP430FR2x5x               | TB1 ← TB0            |
/// | MSP430FR2433               | None                 |
pub trait CascadedTimer: TimerBase {
    /// Timer whose CCR2 output clocks this timer
    type Source: CapCmp<CCR2>;
}

/// The CCR2 output of timer `T`, set up to clock a [`CascadedTimer`] with
/// [`TimerConfig::cascade`]
///
/// While `T` runs, the output is high for the first count of each period, so the cascaded timer
/// counts as `T` wraps around to 0. This needs a period of at least 2 counts.
pub struct CascadeOutput<T>(PhantomData<T>);

impl<T: CapCmp<CCR2>> CascadeOutput<T> {
    #[inline]
    pub(crate) fn new() -> Self {
        let timer = unsafe { T::steal() };
        // Reset/set mode sets the output as the timer wraps around to 0, and resets it when the
        // timer reaches CCR2. With CCR2 at 0 both happen at once and the output stays low.
        CCRn::<CCR2>::set_ccrn(&timer, 1);
        CCRn::<CCR2>::config_outmod(&timer, Outmod::ResetSet);
        CascadeOutput(PhantomData)
    }
}

/// Configuration object for the TimerB peripheral
///
/// Used to configure `Timer`, `Capture`, and `Pwm`, which all use the TimerB peripheral.
pub struct TimerConfig<T, M = DefaultMapping>
where
    T: TimerPeriph<M>,
    M: PinMap,
{
    _timer: PhantomData<T>,
    sel: Tbssel,
    div: TimerDiv,
    ex_div: TimerExDiv,
    _pin_map: PhantomData<M>,
}

impl<T, M> TimerConfig<T, M>
where
    T: TimerPeriph<M>,
    M: PinMap,
{
    #[inline]
    fn with_clock(sel: Tbssel) -> Self {
        TimerConfig {
            _timer: PhantomData,
            sel,
            div: TimerDiv::_1,
            ex_div: TimerExDiv::_1,
            _pin_map: PhantomData,
        }
    }

    /// Configure timer clock source to ACLK
    #[inline]
    pub fn aclk(_aclk: &Aclk) -> Self { Self::with_clock(Tbssel::Aclk) }

    /// Configure timer clock source to SMCLK
    #[inline]
    pub fn smclk(_smclk: &Smclk) -> Self { Self::with_clock(Tbssel::Smclk) }

    /// Configure timer clock source to TBCLK
    #[inline]
    pub fn tbclk(_pin: T::Tbxclk) -> Self { Self::with_clock(Tbssel::Tbxclk) }

    /// Configure the normal clock divider and expansion clock divider settings
    #[inline]
    pub fn clk_div(self, div: TimerDiv, ex_div: TimerExDiv) -> Self {
        TimerConfig {
            _timer: PhantomData,
            sel: self.sel,
            div,
            ex_div,
            _pin_map: PhantomData,
        }
    }

    #[inline]
    pub(crate) fn write_regs(self, timer: &T) {
        T::configure_pin_mapping();
        timer.reset();
        timer.set_tbidex(self.ex_div);
        timer.config_clock(self.sel, self.div);
    }
}

impl<T, M> TimerConfig<T, M>
where
    T: TimerPeriph<M> + VloclkTimer,
    M: PinMap,
{
    /// Configure timer clock source to VLOCLK, which runs at about 10 kHz but is only accurate to
    /// ±50 % (data sheet). Only some timers have this option, see [`VloclkTimer`].
    #[inline]
    pub fn vloclk() -> Self { Self::with_clock(Tbssel::Inclk) }
}

impl<T, M> TimerConfig<T, M>
where
    T: TimerPeriph<M> + CascadedTimer,
    M: PinMap,
{
    /// Configure the timer to be clocked by its source timer (cascading): it counts once per
    /// period of the source timer, while that timer runs. Only some timers have this option, see
    /// [`CascadedTimer`].
    ///
    /// `source` is the source timer's CCR2 output, from [`SubTimer::into_cascade_output`] or
    /// [`PwmUninit::into_cascade_output`](crate::pwm::PwmUninit::into_cascade_output). For
    /// example, a source timer with a period of 1 s lets this timer count seconds.
    #[inline]
    pub fn cascade(_source: &CascadeOutput<T::Source>) -> Self { Self::with_clock(Tbssel::Inclk) }
}

/// Main timer and sub-timer for timer peripherals with 2 capture-compare registers
pub struct TimerParts2<T, M = DefaultMapping>
where
    T: CapCmpTimer2<M>,
    M: PinMap,
{
    /// Main timer
    pub timer: Timer<T, M>,
    /// Timer interrupt vector
    pub tbxiv: TBxIV<T>,
    /// Sub-timer 1 (derived from CCR1 register)
    pub subtimer1: SubTimer<T, CCR1>,
}

impl<T, M> TimerParts2<T, M>
where
    T: CapCmpTimer2<M>,
    M: PinMap,
{
    /// Create new set of timers out of a TBx peripheral
    #[inline(always)]
    pub fn new(_timer: T, config: TimerConfig<T, M>) -> Self {
        config.write_regs(unsafe { &T::steal() });
        Self {
            timer: Timer::new(),
            tbxiv: TBxIV(PhantomData),
            subtimer1: SubTimer::new(),
        }
    }
}

/// Main timer and sub-timers for timer peripherals with 3 capture-compare registers
pub struct TimerParts3<T, M = DefaultMapping>
where
    T: CapCmpTimer3<M>,
    M: PinMap,
{
    /// Main timer
    pub timer: Timer<T, M>,
    /// Timer interrupt vector
    pub tbxiv: TBxIV<T>,
    /// Sub-timer 1 (derived from CCR1 register)
    pub subtimer1: SubTimer<T, CCR1>,
    /// Sub-timer 2 (derived from CCR2 register)
    pub subtimer2: SubTimer<T, CCR2>,
}

impl<T, M> TimerParts3<T, M>
where
    T: CapCmpTimer3<M>,
    M: PinMap,
{
    /// Create new set of timers out of a TBx peripheral
    #[inline(always)]
    pub fn new(_timer: T, config: TimerConfig<T, M>) -> Self {
        config.write_regs(unsafe { &T::steal() });
        Self {
            timer: Timer::new(),
            tbxiv: TBxIV(PhantomData),
            subtimer1: SubTimer::new(),
            subtimer2: SubTimer::new(),
        }
    }
}

/// Main timer and sub-timers for timer peripherals with 7 capture-compare registers
pub struct TimerParts7<T, M = DefaultMapping>
where
    T: CapCmpTimer7<M>,
    M: PinMap,
{
    /// Main timer
    pub timer: Timer<T, M>,
    /// Timer interrupt vector
    pub tbxiv: TBxIV<T>,
    /// Sub-timer 1 (derived from CCR1 register)
    pub subtimer1: SubTimer<T, CCR1>,
    /// Sub-timer 2 (derived from CCR2 register)
    pub subtimer2: SubTimer<T, CCR2>,
    /// Sub-timer 3 (derived from CCR3 register)
    pub subtimer3: SubTimer<T, CCR3>,
    /// Sub-timer 4 (derived from CCR4 register)
    pub subtimer4: SubTimer<T, CCR4>,
    /// Sub-timer 5 (derived from CCR5 register)
    pub subtimer5: SubTimer<T, CCR5>,
    /// Sub-timer 6 (derived from CCR6 register)
    pub subtimer6: SubTimer<T, CCR6>,
}

impl<T, M> TimerParts7<T, M>
where
    T: CapCmpTimer7<M>,
    M: PinMap,
{
    /// Create new set of timers out of a TBx peripheral
    #[inline(always)]
    pub fn new(_timer: T, config: TimerConfig<T, M>) -> Self {
        config.write_regs(unsafe { &T::steal() });
        Self {
            timer: Timer::new(),
            tbxiv: TBxIV(PhantomData),
            subtimer1: SubTimer::new(),
            subtimer2: SubTimer::new(),
            subtimer3: SubTimer::new(),
            subtimer4: SubTimer::new(),
            subtimer5: SubTimer::new(),
            subtimer6: SubTimer::new(),
        }
    }
}

/// Main periodic countdown timer
pub struct Timer<T: TimerPeriph<M>, M: PinMap = DefaultMapping>(PhantomData<T>, PhantomData<M>);

impl<T, M> Timer<T, M>
where
    T: TimerPeriph<M>,
    M: PinMap,
{
    fn new() -> Self { Self(PhantomData, PhantomData) }
}

/// Sub-timer associated with a main timer
///
/// Each sub-timer has its own interrupt mechanism and threshold, but shares its countdown value
/// with its main timer.
pub struct SubTimer<T: CapCmp<C>, C>(PhantomData<T>, PhantomData<C>);

impl<T: CapCmp<C>, C> SubTimer<T, C> {
    fn new() -> Self { Self(PhantomData, PhantomData) }
}

/// Indicates which sub/main timer caused the interrupt to fire
pub enum TimerVector {
    /// No pending interrupt
    NoInterrupt,
    /// Interrupt caused by sub-timer 1
    SubTimer1,
    /// Interrupt caused by sub-timer 2
    SubTimer2,
    /// Interrupt caused by sub-timer 3
    SubTimer3,
    /// Interrupt caused by sub-timer 4
    SubTimer4,
    /// Interrupt caused by sub-timer 5
    SubTimer5,
    /// Interrupt caused by sub-timer 6
    SubTimer6,
    /// Interrupt caused by main timer overflow
    MainTimer,
}

#[inline]
pub(crate) fn read_tbxiv<T: TimerBase>(timer: &T) -> TimerVector {
    match timer.tbxiv_rd() {
        0 => TimerVector::NoInterrupt,
        2 => TimerVector::SubTimer1,
        4 => TimerVector::SubTimer2,
        6 => TimerVector::SubTimer3,
        8 => TimerVector::SubTimer4,
        10 => TimerVector::SubTimer5,
        12 => TimerVector::SubTimer6,
        14 => TimerVector::MainTimer,
        _ => unsafe { core::hint::unreachable_unchecked() },
    }
}

/// Interrupt vector register for determining which timer caused an ISR
pub struct TBxIV<T>(PhantomData<T>);

impl<T: TimerBase> TBxIV<T> {
    #[inline]
    /// Read the timer interrupt vector. Automatically resets corresponding interrupt flag.
    pub fn interrupt_vector(&mut self) -> TimerVector {
        let timer = unsafe { T::steal() };
        read_tbxiv(&timer)
    }
}

impl<T, M> Timer<T, M>
where
    T: TimerPeriph<M>,
    M: PinMap,
{
    /// Enable timer countdown expiration interrupts
    #[inline(always)]
    pub fn enable_interrupts(&mut self) {
        let timer = unsafe { T::steal() };
        timer.tbie_set();
    }

    /// Disable timer countdown expiration interrupts
    #[inline(always)]
    pub fn disable_interrupts(&mut self) {
        let timer = unsafe { T::steal() };
        timer.tbie_clr();
    }

    #[inline]
    /// Clears the timer, sets the count, and starts the timer in upcounting mode.
    pub fn start(&mut self, count: u16) {
        let timer = unsafe { T::steal() };
        timer.stop();
        timer.set_ccrn(count);
        timer.upmode();
    }

    #[inline]
    /// Checks if the timer has reached the target value. Returns `Ok(())` if so, otherwise `WouldBlock`.
    pub fn wait(&mut self) -> nb::Result<(), Infallible> {
        let timer = unsafe { T::steal() };
        if timer.tbifg_rd() {
            timer.tbifg_clr();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline]
    /// Pause the timer at the current value
    pub fn pause(&mut self) {
        let timer = unsafe { T::steal() };
        timer.stop();
    }

    #[inline]
    /// Resume counting from the current value
    pub fn resume(&mut self) {
        let timer = unsafe { T::steal() };
        timer.resume(RunningMode::Up);
    }

    #[inline]
    /// Get the current timer value
    pub fn count(&mut self) -> u16 {
        let timer = unsafe { T::steal() };
        timer.get_tbxr()
    }
}

impl<T: CapCmp<C>, C> SubTimer<T, C> {
    #[inline]
    /// Set the threshold for one of the sub-timers. Once the main timer counts to this threshold
    /// the sub-timer will fire. Note that the main timer resets once it counts to its own
    /// threshold, not the sub-timer thresholds. It follows that the sub-timer threshold must be
    /// less than the main threshold for it to fire.
    pub fn set_count(&mut self, count: u16) {
        let timer = unsafe { T::steal() };
        timer.set_ccrn(count);
        timer.ccifg_clr();
    }

    #[inline]
    /// Wait for the sub-timer to fire
    pub fn wait(&mut self) -> nb::Result<(), Infallible> {
        let timer = unsafe { T::steal() };
        if timer.ccifg_rd() {
            timer.ccifg_clr();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline(always)]
    /// Enable the sub-timer interrupts
    pub fn enable_interrupts(&mut self) {
        let timer = unsafe { T::steal() };
        timer.ccie_set();
    }

    #[inline(always)]
    /// Disable the sub-timer interrupts
    pub fn disable_interrupts(&mut self) {
        let timer = unsafe { T::steal() };
        timer.ccie_clr();
    }
}

impl<T: CapCmp<CCR2>> SubTimer<T, CCR2> {
    #[inline]
    /// Use CCR2 to clock a cascaded timer instead, see [`TimerConfig::cascade`]
    pub fn into_cascade_output(self) -> CascadeOutput<T> { CascadeOutput::new() }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use super::*;
    use embedded_hal_02::timer::{Cancel, CountDown, Periodic};

    impl<T, M> CountDown for Timer<T, M>
    where
        T: TimerPeriph<M> + CapCmp<CCR0>,
        M: PinMap,
    {
        type Time = u16;

        #[inline]
        fn start<U: Into<Self::Time>>(&mut self, count: U) { self.start(count.into()) }

        #[inline]
        fn wait(&mut self) -> nb::Result<(), void::Void> {
            self.wait().map_err(|_| nb::Error::WouldBlock)
        }
    }

    impl<T, M> Cancel for Timer<T, M>
    where
        T: TimerPeriph<M> + CapCmp<CCR0>,
        M: PinMap,
    {
        type Error = void::Void;

        #[inline(always)]
        fn cancel(&mut self) -> Result<(), Self::Error> {
            self.pause();
            Ok(())
        }
    }

    impl<T, M> Periodic for Timer<T, M>
    where
        T: TimerPeriph<M>,
        M: PinMap,
    {}
}
