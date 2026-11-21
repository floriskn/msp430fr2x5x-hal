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
    /// Pin type used for external TBxCLK of this timer, or [`NoTbxclkPin`] if it has none (TASSEL/TBSSEL =
    /// 00b: SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
    type Tbxclk;

    /// Additional configuration
    #[inline(always)]
    fn configure_pin_mapping() {}
}

/// The external TBxCLK pin of timers that don't have one, such as TA2 and TA3 on the
/// MSP430FR2433 (SLASE59F Table 6-13, p. 51; SLASE59F Table 6-14, p. 52)
///
/// It has no values, so [`TimerConfig::tbclk`] can't be called for these timers.
pub enum NoTbxclkPin {}

// Traits effectively sealed by CCRn
/// Trait indicating that the peripheral has 2 capture compare registers: TA2 and TA3 on the MSP430FR2433
/// (SLASE59F 6.10.8, p. 51: "two capture/compare registers each")
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

// Trait effectively sealed by TimerBase
/// Trait indicating a Timer_B. Its counter length can be changed, see [`TimerConfig::counter_length`], and its
/// compare registers are buffered, which PWM uses so that a duty cycle change never spoils a period
/// (SLAU445I 14.1.1, p. 391; 14.2.4.2.1, p. 400). Erratum TB25 makes two of the load modes load at once in up
/// mode on the MSP430FR2x5x and MSP430FR247x, so edge-aligned PWM uses one it doesn't list (SLAZ695J TB25,
/// p. 11; SLAZ726B TB25, p. 8; see [`crate::pwm`]).
pub trait TimerB: TimerBase {}

/// The number of bits a Timer_B counts with (CNTL), which sets its highest count in continuous mode
/// (SLAU445I 14.2.1.1, p. 393; SLAU445I Table 14-6, p. 409)
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum CounterLength {
    /// 16 bits, up to 0xFFFF, as after reset (CNTL = 00b, reset value 0h: SLAU445I Table 14-6, p. 409)
    #[default]
    _16Bit = 0,
    /// 12 bits, up to 0x0FFF (CNTL = 01b: SLAU445I Table 14-6, p. 409)
    _12Bit = 1,
    /// 10 bits, up to 0x03FF (CNTL = 10b: SLAU445I Table 14-6, p. 409)
    _10Bit = 2,
    /// 8 bits, up to 0x00FF (CNTL = 11b: SLAU445I Table 14-6, p. 409)
    _8Bit = 3,
}

// Traits effectively sealed by TimerBase
/// Trait indicating that the timer can be clocked from VLOCLK, see [`TimerConfig::vloclk`]
///
/// A timer's fourth clock input, INCLK (TASSEL/TBSSEL = 11b: SLAU445I Table 13-4, p. 384; SLAU445I
/// Table 14-6, p. 409), is wired differently on each device and each timer. These timers have the VLO on
/// it (data sheets, timer signal connections and clock distribution):
///
/// | Device                     | Timers   | Reference                                              |
/// |----------------------------|----------|--------------------------------------------------------|
/// | MSP430FR2475, MSP430FR2476 | TA0, TA2 | SLASEO7C Table 9-12, p. 55; SLASEO7C Table 9-14, p. 58 |
/// | MSP430FR2512, MSP430FR2522 | TA0      | SLASEE4C Figure 6-2, p. 54; SLASEE4C Table 6-8, p. 49  |
/// | Other devices              | None     | SLASEC4D Table 6-9, p. 68; SLASE59F Table 6-7, p. 46   |
pub trait VloclkTimer: TimerBase {}

/// Trait indicating that the timer can be clocked by another timer, see [`TimerConfig::cascade`]
///
/// A timer's fourth clock input, INCLK, is wired differently on each device and each timer. These
/// timers have the CCR2 output of their `Source` timer on it (data sheets, timer signal
/// connections):
///
/// | Device                     | Timer ← `Source`     | Reference                                              |
/// |----------------------------|----------------------|--------------------------------------------------------|
/// | MSP430FR2475, MSP430FR2476 | TA1 ← TA0, TA3 ← TA2 | SLASEO7C Table 9-13, p. 56; SLASEO7C Table 9-14, p. 58 |
/// | MSP430FR2512, MSP430FR2522 | TA1 ← TA0            | SLASEE4C Figure 6-2, p. 54                             |
/// | MSP430FR2x5x               | TB1 ← TB0            | SLASEC4D Table 6-17, p. 74                             |
/// | MSP430FR2433               | None                 | SLASE59F Tables 6-11 to 6-14, p. 50 to p. 52           |
pub trait CascadedTimer: TimerBase {
    /// Timer whose CCR2 output clocks this timer
    type Source: CapCmp<CCR2>;
}

/// The CCR2 output of timer `T`, set up to clock a [`CascadedTimer`] with
/// [`TimerConfig::cascade`]
///
/// While `T` runs, the output is high for the first count of each period, so the cascaded timer,
/// which counts on the rising edges of its clock (SLAU445I 13.2.1, p. 370), counts as `T` wraps around
/// to 0. This needs a period of at least 2 counts.
pub struct CascadeOutput<T>(PhantomData<T>);

impl<T: CapCmp<CCR2>> CascadeOutput<T> {
    #[inline]
    pub(crate) fn new() -> Self {
        let timer = unsafe { T::steal() };
        // Reset/set mode sets the output as the timer wraps around to 0, and resets it when the
        // timer reaches CCR2 (SLAU445I Table 13-2, p. 376; 13.2.5.1.1, p. 377). With CCR2 at 0 both
        // happen at once and the output stays low.
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
    cntl: u8,
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
            cntl: CounterLength::_16Bit as u8,
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
        TimerConfig { div, ex_div, ..self }
    }

    #[inline]
    pub(crate) fn write_regs(self, timer: &T) {
        T::configure_pin_mapping();
        timer.reset();
        timer.set_tbidex(self.ex_div);
        timer.config_clock(self.sel, self.div);
        timer.set_cntl(self.cntl);
    }
}

impl<T, M> TimerConfig<T, M>
where
    T: TimerPeriph<M> + TimerB,
    M: PinMap,
{
    /// Set how many bits this Timer_B counts with (CNTL). In continuous mode it then counts up to 0xFF,
    /// 0x3FF, 0xFFF or 0xFFFF before it starts over (SLAU445I 14.2.1.1, p. 393; 14.2.3.2, p. 395).
    #[inline]
    pub fn counter_length(self, length: CounterLength) -> Self { TimerConfig { cntl: length as u8, ..self } }
}

impl<T, M> TimerConfig<T, M>
where
    T: TimerPeriph<M> + VloclkTimer,
    M: PinMap,
{
    /// Configure timer clock source to VLOCLK, which runs at about 10 kHz but is only accurate to
    /// ±50 % (VLOCLK "10 kHz ±50%": SLASEO7C Table 9-8, p. 50; SLASEE4C Table 6-8, p. 49). Only some
    /// timers have this option, see [`VloclkTimer`]. It selects INCLK (TASSEL = 11b: SLAU445I Table 13-4,
    /// p. 384).
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
    /// [`CascadedTimer`]. It selects INCLK (TASSEL/TBSSEL = 11b: SLAU445I Table 13-4, p. 384; SLAU445I
    /// Table 14-6, p. 409).
    ///
    /// `source` is the source timer's CCR2 output, from [`SubTimer::into_cascade_output`] or
    /// [`PwmUninit::into_cascade_output`](crate::pwm::PwmUninit::into_cascade_output). For
    /// example, a source timer with a period of 1 s lets this timer count seconds.
    #[inline]
    pub fn cascade(_source: &CascadeOutput<T::Source>) -> Self { Self::with_clock(Tbssel::Inclk) }
}

/// Main timer and sub-timer for timer peripherals with 2 capture-compare registers
///
/// The timers with 2 capture/compare registers are TA2 and TA3 on the MSP430FR2433 (SLASE59F Table 6-13,
/// p. 51; SLASE59F Table 6-14, p. 52).
pub struct TimerParts2<T, M = DefaultMapping>
where
    T: CapCmpTimer2<M>,
    M: PinMap,
{
    /// Main timer
    pub timer: Timer<T, M>,
    /// Timer interrupt vector (TAxIV: SLAU445I Table 13-8, p. 388)
    pub tbxiv: TBxIV<T>,
    /// Sub-timer 1 (derived from CCR1 register, TAxCCR1: SLAU445I Table 13-7, p. 388)
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
pub struct Timer<T: TimerPeriph<M>, M: PinMap = DefaultMapping> {
    /// The mode it counts in once started, for `resume()`
    mode: RunningMode,
    _timer: PhantomData<T>,
    _pin_map: PhantomData<M>,
}

impl<T, M> Timer<T, M>
where
    T: TimerPeriph<M>,
    M: PinMap,
{
    fn new() -> Self { Self { mode: RunningMode::Up, _timer: PhantomData, _pin_map: PhantomData } }
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
        self.mode = RunningMode::Up;
    }

    #[inline]
    /// Clears the timer and starts it counting up to `count` and back down to 0 (up/down mode), so a
    /// period lasts `2 * count` timer cycles. [`wait()`](Timer::wait) returns once per period, when the
    /// count gets back to 0 (SLAU445I 13.2.3.4, p. 373; 14.2.3.4, p. 396), and sub-timers fire twice per
    /// period, on the way up and on the way down (SLAU445I Figure 13-14, p. 379; SLAU445I Figure 14-14,
    /// p. 404).
    pub fn start_up_down(&mut self, count: u16) {
        let timer = unsafe { T::steal() };
        // CCR0 is updated while the timer is stopped (SLAU445I 13.2.3.4.1, p. 373)
        timer.stop();
        timer.set_ccrn(count);
        timer.updown_mode();
        self.mode = RunningMode::UpDown;
    }

    #[inline]
    /// Checks if the timer has reached the target value. Returns `Ok(())` if so, otherwise `WouldBlock`.
    ///
    /// It checks TBIFG, which is set as the count goes from the target value to 0 (SLAU445I 13.2.3.1,
    /// p. 371; 14.2.3.1, p. 394), or in up/down mode as it gets back down to 0.
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
    /// Resume counting from the current value, in the direction and mode it was counting in (the
    /// count direction is latched: SLAU445I 13.2.3.4, p. 373; 14.2.3.4, p. 396)
    pub fn resume(&mut self) {
        let timer = unsafe { T::steal() };
        timer.resume(self.mode);
    }

    #[inline]
    /// Get the current timer value.
    ///
    /// A timer clocked asynchronously to MCLK (from ACLK, for example) can return a wrong value
    /// when read while it counts, so this takes the median of three reads, as the user's guide
    /// suggests (SLAU445I 13.2.1, p. 370; 14.2.1, p. 393, notes "Accessing TAxR" and "Accessing TBxR":
    /// "TBxR can be read multiple times while the timer is running, and a majority vote taken in
    /// software").
    pub fn count(&mut self) -> u16 {
        let timer = unsafe { T::steal() };
        let (a, b, c) = (timer.get_tbxr(), timer.get_tbxr(), timer.get_tbxr());
        a.min(b).max(a.max(b).min(c))
    }
}

impl<T: CapCmp<C>, C> SubTimer<T, C> {
    #[inline]
    /// Set the threshold for one of the sub-timers. Once the main timer counts to this threshold
    /// the sub-timer will fire (SLAU445I 13.2.4.2, p. 376; 14.2.4.2, p. 399). Note that the main timer
    /// resets once it counts to its own threshold, not the sub-timer thresholds. It follows that the
    /// sub-timer threshold must not be more than the main threshold for it to fire: the main timer
    /// counts up to and including its threshold (SLAU445I 13.2.3.1, p. 371; 14.2.3.1, p. 394).
    ///
    /// It writes TAxCCRn/TBxCCRn (SLAU445I Table 13-7, p. 388; SLAU445I Table 14-9, p. 413) and clears
    /// CCIFG (SLAU445I Table 13-6, p. 387; SLAU445I Table 14-8, p. 412). A Timer_A is stopped for the write,
    /// as the user's guide says "the timer should be stopped by writing the MC bits to zero (MC = 0) before
    /// writing new data to TAxCCRn" (SLAU445I 13.2.4.2, p. 376), so the main timer misses the few timer
    /// clocks that takes.
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
    /// Use CCR2 to clock a cascaded timer instead, see [`TimerConfig::cascade`]. The CCR2 output drives the
    /// cascaded timer's INCLK (SLASEC4D Table 6-17, p. 74; SLASEO7C Table 9-13, p. 56; SLASEO7C Table 9-14,
    /// p. 58; SLASEE4C Figure 6-2, p. 54).
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
