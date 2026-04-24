//! Capacitive Touch implementation for MSP430FR4133

pub use crate::_pac::rtc::rtcctl::Rtcps as RtcDiv;
pub use crate::_pac::wdt_a::wdtctl::Wdtis as WdtClkPeriods;
use crate::{
    _pac::{self, rtc::rtcctl::Rtcss, wdt_a::wdtctl::Wdtssel},
    hw_traits::{capacitive::*, timer_a::*},
    lpm::*,
    pin_mapping::*,
    timer::*,
    watchdog::*,
};
use core::{cell::Cell, marker::PhantomData, mem::MaybeUninit, num::NonZero};
use msp430::interrupt::{disable as disable_interrupts, enable as enable_interrupts};


pub use crate::hw_traits::timer_base::Tbssel;
// --- Traits ---

pub trait CaptivateIoTimer {}
pub trait CapacitiveCapable {
    const CHANNEL_SELECT: u16;
}

pub trait Gate<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap = DefaultMapping> {
    type Interval: Copy;

    fn prepare(&self, timer: &T, interval: Self::Interval, is_roi: bool);
    fn capture(&self, timer: &T, interval: Self::Interval, is_roi: bool) -> u16;
    fn release(&self, timer: &T, is_roi: bool);
}

// --- Gating Implementations ---

/// 1. WDT Gate (Borrowed)
pub struct WdtGate {
    wdt: _pac::WdtA,
    context_save_sr: Cell<u16>,
    context_save_sfrie1: Cell<u16>,
    context_save_wdtctl: Cell<u16>,
    context_save_txnctl: Cell<u16>,
    context_save_txcctl0: Cell<u16>,
    context_save_txccr0: Cell<u16>,
}
impl WdtGate {
    #[inline(always)]
    pub fn new(wdt: _pac::WdtA) -> Self {
        Self {
            wdt,
            context_save_sr: Cell::new(0),
            context_save_sfrie1: Cell::new(0),
            context_save_wdtctl: Cell::new(0),
            context_save_txnctl: Cell::new(0),
            context_save_txcctl0: Cell::new(0),
            context_save_txccr0: Cell::new(0),
        }
    }
}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for WdtGate {
    type Interval = (WdtClkPeriods, Wdtssel);

    #[inline(always)]
    fn prepare(&self, timer: &T, _: Self::Interval, _is_roi: bool) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };

        self.context_save_sr
            .set(msp430::register::sr::read().bits());
        self.context_save_sfrie1.set(sfr.sfrie1().read().bits());
        self.context_save_wdtctl
            .set(self.wdt.wdtctl().read().bits() & 0xff);
        self.context_save_txnctl.set(timer.get_ctl());
        self.context_save_txcctl0
            .set(CCRn::<CCR0>::get_cctln(timer));
        self.context_save_txccr0.set(CCRn::<CCR0>::get_ccrn(timer));

        timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
        timer.continuous();

        CCRn::<CCR0>::config_cap_mode(timer, Cm::BothEdges, Ccis::Gnd);

        unsafe { sfr.sfrie1().set_bits(|w| w.wdtie().set_bit()) };

        unsafe { enable_interrupts() };
    }

    #[inline(always)]
    fn capture(&self, timer: &T, interval: Self::Interval, _is_roi: bool) -> u16 {
        timer.reset();
        timer.tbifg_clr();

        // Halt timer first, as specified in the user's guide
        self.wdt.wdtctl().write(|w| {
            Wdt::<IntervalMode>::prewrite(w, 0)
                .wdthold()
                .hold()
                // Also reset timer
                .wdtcntcl()
                .set_bit()
        });
        // Set clock src and keep timer halted
        self.wdt.wdtctl().write(|w| {
            Wdt::<IntervalMode>::prewrite(w, 0)
                .wdtssel()
                .variant(interval.1)
                .wdthold()
                .hold()
        });

        self.wdt.wdtctl().modify(|r, w| {
            Wdt::<IntervalMode>::prewrite(w, r.bits())
                .wdtcntcl()
                .set_bit()
                .wdthold()
                .unhold()
                .wdtis()
                .variant(interval.0)
        });

        if interval.1 == Wdtssel::Aclk {
            request_lpm3();
        } else {
            enter_lpm0();
        }

        CCRn::<CCR0>::trigger_sw(timer);
        self.wdt
            .wdtctl()
            .modify(|r, w| Wdt::<IntervalMode>::prewrite(w, r.bits()).wdthold().hold());

        if timer.tbifg_rd() {
            0
        } else {
            CCRn::<CCR0>::get_ccrn(timer)
        }
    }

    #[inline(always)]
    fn release(&self, timer: &T, _is_roi: bool) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };

        if self.context_save_sr.get() & (1 << 3) == 0 {
            disable_interrupts();
        }

        sfr.sfrie1()
            .write(|w| unsafe { w.bits(self.context_save_sfrie1.get()) });

        self.wdt
            .wdtctl()
            .write(|w| Wdt::<IntervalMode>::prewrite(w, self.context_save_wdtctl.get()));
        timer.set_ctl(self.context_save_txnctl.get());
        CCRn::<CCR0>::set_cctln(timer, self.context_save_txcctl0.get());
        CCRn::<CCR0>::set_ccrn(timer, self.context_save_txccr0.get());
    }
}

/// 3. Timer Gate (Using Locked)
pub struct TimerGate<TG, MG = DefaultMapping>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    timer: TG,
    context_save_sr: Cell<u16>,
    context_save_t0nctl: Cell<u16>,
    context_save_t0cctl0: Cell<u16>,
    context_save_t0ccr0: Cell<u16>,
    context_save_t1nctl: Cell<u16>,
    context_save_t1cctl0: Cell<u16>,
    context_save_t1ccr0: Cell<u16>,
    _pin_map: PhantomData<MG>,
}

impl<TG, MG> TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    #[inline(always)]
    pub fn new(timer: TG) -> Self {
        Self {
            timer,
            context_save_sr: Cell::new(0),
            context_save_t0nctl: Cell::new(0),
            context_save_t0cctl0: Cell::new(0),
            context_save_t0ccr0: Cell::new(0),
            context_save_t1nctl: Cell::new(0),
            context_save_t1cctl0: Cell::new(0),
            context_save_t1ccr0: Cell::new(0),
            _pin_map: PhantomData,
        }
    }
}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap, TG, MG> Gate<T, M> for TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    type Interval = (Tbssel, TimerDiv, u16);

    #[inline(always)]
    fn prepare(&self, timer: &T, interval: Self::Interval, is_roi: bool) {
        self.context_save_sr
            .set(msp430::register::sr::read().bits());
        self.context_save_t0nctl.set(timer.get_ctl());
        self.context_save_t0cctl0
            .set(CCRn::<CCR0>::get_cctln(timer));
        self.context_save_t0ccr0.set(CCRn::<CCR0>::get_ccrn(timer));

        self.context_save_t1nctl.set(self.timer.get_ctl());
        self.context_save_t1cctl0
            .set(CCRn::<CCR0>::get_cctln(&self.timer));
        self.context_save_t1ccr0
            .set(CCRn::<CCR0>::get_ccrn(&self.timer));

        if is_roi {
            timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
            timer.continuous();
            CCRn::<CCR0>::config_cap_mode(timer, Cm::BothEdges, Ccis::Gnd);

            CCRn::<CCR0>::set_ccrn(&self.timer, interval.2);
            self.timer.config_clock(interval.0, interval.1);
            CCRn::<CCR0>::ccie_set(&self.timer);
        } else {
            self.timer.config_clock(interval.0, TimerDiv::_1);
            self.timer.continuous();
            CCRn::<CCR0>::config_cap_mode(&self.timer, Cm::BothEdges, Ccis::Gnd);

            CCRn::<CCR0>::set_ccrn(timer, interval.2);
            timer.config_clock(Tbssel::Inclk, interval.1);
            CCRn::<CCR0>::ccie_set(timer);
        }

        unsafe { enable_interrupts() };
    }

    #[inline(always)]
    fn capture(&self, timer: &T, interval: Self::Interval, is_roi: bool) -> u16 {
        
        if is_roi {
            timer.reset();
            timer.tbifg_clr();

            self.timer.upmode();
        } else {
            self.timer.reset();
            self.timer.tbifg_clr();

            timer.upmode();
        }


        if interval.0 == Tbssel::Aclk {
            request_lpm3();
        } else {
            enter_lpm0();
        }

        if is_roi {
            CCRn::<CCR0>::trigger_sw(timer);
            self.timer.stop();

            if timer.tbifg_rd() {
                0
            } else {
                CCRn::<CCR0>::get_ccrn(timer)
            }
        } else {
            CCRn::<CCR0>::trigger_sw(&self.timer);
            timer.stop();

            if self.timer.tbifg_rd() {
                0
            } else {
                CCRn::<CCR0>::get_ccrn(&self.timer)
            }
        }
    }

    #[inline(always)]
    fn release(&self, timer: &T, _is_roi: bool) {
        if self.context_save_sr.get() & (1 << 3) == 0 {
            disable_interrupts();
        }

        timer.set_ctl(self.context_save_t0nctl.get());
        CCRn::<CCR0>::set_cctln(timer, self.context_save_t0cctl0.get());
        CCRn::<CCR0>::set_ccrn(timer, self.context_save_t0ccr0.get());

        self.timer.set_ctl(self.context_save_t1nctl.get());
        CCRn::<CCR0>::set_cctln(&self.timer, self.context_save_t1cctl0.get());
        CCRn::<CCR0>::set_ccrn(&self.timer, self.context_save_t1ccr0.get());
    }
}

/// 4. RTC Gate (Using Locked)
pub struct RtcGate {
    pub rtc: _pac::Rtc,
    context_save_sr: Cell<u16>,
    context_save_rtcctl: Cell<u16>,
    context_save_txnctl: Cell<u16>,
    // context_save_txcctl0: Cell<u16>,
    // context_save_txccr0: Cell<u16>,
}
impl RtcGate {
    #[inline(always)]
    pub fn new(rtc: _pac::Rtc) -> Self {
        Self {
            rtc,
            context_save_sr: Cell::new(0),
            context_save_rtcctl: Cell::new(0),
            context_save_txnctl: Cell::new(0),
            // context_save_txcctl0: Cell::new(0),
            // context_save_txccr0: Cell::new(0),
        }
    }
}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for RtcGate {
    type Interval = (Rtcss, RtcDiv, u16);

    #[inline(always)]
    fn prepare(&self, timer: &T, _interval: Self::Interval, _is_roi: bool) {
        self.context_save_sr
            .set(msp430::register::sr::read().bits());
        self.context_save_rtcctl
            .set(self.rtc.rtcctl().read().bits());
        self.context_save_txnctl.set(timer.get_ctl());
        // self.context_save_txcctl0
        //     .set(CCRn::<CCR0>::get_cctln(timer));
        // self.context_save_txccr0.set(CCRn::<CCR0>::get_ccrn(timer));

        // timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
        // timer.continuous();

        // CCRn::<CCR0>::config_cap_mode(timer, Cm::BothEdges, Ccis::Gnd);

        unsafe { enable_interrupts() };
    }

    #[inline(always)]
    fn capture(&self, timer: &T, interval: Self::Interval, _is_roi: bool) -> u16 {
        timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
        timer.continuous();

        // timer.reset();
        // timer.tbifg_clr();

        self.rtc.rtcctl().write(|w| w.rtcsr().set_bit());
        // Need to clear interrupt flag from last timer run
        self.rtc.rtciv().read();
        self.rtc.rtcmod().write(|w| unsafe { w.bits(interval.2) });
        self.rtc.rtcctl().modify(|r, w| {
            unsafe { w.bits(r.bits()) }
                .rtcss()
                .variant(interval.0)
                .rtcsr()
                .set_bit()
                .rtcps()
                .variant(interval.1)
                .rtcie()
                .set_bit()
        });

        if interval.0 == Rtcss::Smclk {
            enter_lpm0();
        } else {
            request_lpm3();
        }

        // CCRn::<CCR0>::trigger_sw(timer);
        timer.stop();
        self.rtc.rtcctl().write(|w| w.rtcsr().set_bit());

        if timer.tbifg_rd() {
            0
        } else {
            // CCRn::<CCR0>::get_ccrn(timer)
            timer.get_tbxr()
        }
    }

    #[inline(always)]
    fn release(&self, timer: &T, _is_roi: bool) {
        if self.context_save_sr.get() & (1 << 3) == 0 {
            disable_interrupts();
        }

        self.rtc
            .rtcctl()
            .write(|w| unsafe { w.bits(self.context_save_rtcctl.get()) });

        timer.set_ctl(self.context_save_txnctl.get());
        // CCRn::<CCR0>::set_cctln(timer, self.context_save_txcctl0.get());
        // CCRn::<CCR0>::set_ccrn(timer, self.context_save_txccr0.get());
    }
}

/// 5. Software Gate (No Peripherals)
pub struct SwGate {
    context_save_txnctl: Cell<u16>,
    context_save_txcctl0: Cell<u16>,
}

impl SwGate {
    #[inline(always)]
    pub fn new() -> Self {
        Self {
            context_save_txnctl: Cell::new(0),
            context_save_txcctl0: Cell::new(0),
        }
    }
}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for SwGate {
    type Interval = u16;

    #[inline(always)]
    fn prepare(&self, timer: &T, accumulation_cycles: Self::Interval, _is_roi: bool) {
        self.context_save_txnctl.set(timer.get_ctl());
        self.context_save_txcctl0
            .set(CCRn::<CCR0>::get_cctln(timer));

        CCRn::<CCR0>::set_ccrn(timer, accumulation_cycles);
        // timer.set_ccrn(count);
    }

    #[inline(always)]
    fn capture(&self, timer: &T, _: Self::Interval, _is_roi: bool) -> u16 {
        let mut j = 0u16;
        timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
        timer.upmode();

        while !timer.tbifg_rd() {
            j = j.saturating_add(1);
        }

        timer.stop();

        return j;
    }

    #[inline(always)]
    fn release(&self, timer: &T, _is_roi: bool) {
        timer.set_ctl(self.context_save_txnctl.get());
        CCRn::<CCR0>::set_cctln(timer, self.context_save_txcctl0.get());
    }
} // Marker traits

// Define markers for the Sensor struct to use
pub trait MeasurementMethod {
    const IS_ROI: bool;
}

pub struct RO;
impl MeasurementMethod for RO {
    const IS_ROI: bool = true;
}

pub struct FRO;
impl MeasurementMethod for FRO {
    const IS_ROI: bool = false;
}

pub trait RoCapable {}
pub trait FroCapable {}

// NEW: Combined traits that act as the "Trait Object" type
pub trait RoGate<T, M>: Gate<T, M> + RoCapable
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
}

pub trait FroGate<T, M>: Gate<T, M> + FroCapable
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
}

// WDT and RTC only support RO
impl RoCapable for WdtGate {}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> RoGate<T, M> for WdtGate {}
impl RoCapable for RtcGate {}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> RoGate<T, M> for RtcGate {}

// SW only supports fRO
impl FroCapable for SwGate {}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> FroGate<T, M> for SwGate {}

// TimerGate supports BOTH
impl<TG, MG> RoCapable for TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
}
impl<TG, MG, T, M> RoGate<T, M> for TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
}

impl<TG, MG> FroCapable for TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
}
impl<TG, MG, T, M> FroGate<T, M> for TimerGate<TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
}

// --- Elements & Sensors (Same as before) ---

pub struct CapacitiveElement {
    pub pin_id: u16,
    pub threshold: u16,
    pub max_response: u16,
    pub range: NonZero<u16>, // Store the pre-calculated 400 here
}

impl CapacitiveElement {
    #[inline(always)]
    pub fn new<P: CapacitiveCapable, const THRESH: u16, const MAX: u16>(_pin: P) -> Self {
        // This is evaluated at compile-time
        const {
            assert!(MAX > THRESH, "Max response must be greater than threshold");
        };

        const {
            assert!((MAX - THRESH) != 0, "Cannot be 0");
        };

        const {
            assert!((MAX - THRESH) <= 655, "X");
        };

        Self {
            pin_id: P::CHANNEL_SELECT,
            threshold: THRESH,
            max_response: MAX,
            range: unsafe { core::num::NonZeroU16::new_unchecked(MAX - THRESH) },
        }
    }
}

pub struct CapacitiveParts3<T, M = DefaultMapping>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
    pub timer: T,
    _pin_map: PhantomData<M>,
}
impl<T, M> CapacitiveParts3<T, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
    #[inline(always)]
    pub fn new(timer: T) -> Self {
        Self {
            timer,
            _pin_map: PhantomData,
        }
    }
}

pub enum TrackingRate {
    VerySlow,
    Slow,
    Medium,
    Fast,
}

pub enum DriftRate {
    VerySlow,
    Slow,
    Medium,
    Fast,
}

pub enum DirectionOfInterest {
    Decrement,
    Increment,
}

// Marker types for sensor kind
pub struct Button;
pub struct Slider {
    pub sensor_threshold: u8,
    pub points: u8,
}
pub struct Wheel {
    pub sensor_threshold: u8,
    pub points: u8,
}

impl Button {
    #[inline(always)]
    pub const fn new() -> Self {
        Self
    }
}

impl Slider {
    #[inline(always)]
    pub const fn new(sensor_threshold: u8, points: u8) -> Self {
        Self {
            sensor_threshold,
            points,
        }
    }
}

impl Wheel {
    #[inline(always)]
    pub const fn new(sensor_threshold: u8, points: u8) -> Self {
        Self {
            sensor_threshold,
            points,
        }
    }
}

// Seal them behind a trait
pub trait SensorKind {
    fn process<const N: usize>(&self, index: usize, meas_cnt: &[u16]) -> Option<i16>;
}

impl SensorKind for Button {
    #[inline(always)]
    fn process<const N: usize>(&self, index: usize, _meas_cnt: &[u16]) -> Option<i16> {
        Some(index as i16)
    }
}

impl SensorKind for Slider {
    // TODO: still panics
    #[inline(always)]
    fn process<const N: usize>(&self, index: usize, meas_cnt: &[u16]) -> Option<i16> {
        let pts = self.points as i16;
        let num_els = N as i16;
        let pts_per_el = pts / num_els;

        // Determine cumulative position/strength
        let strength: i16 = if index == 0 {
            (meas_cnt[0] + meas_cnt[1]) as i16
        } else if index == N - 1 {
            (meas_cnt[N - 1] + meas_cnt[N - 2]) as i16
        } else {
            (meas_cnt[index] + meas_cnt[index + 1] + meas_cnt[index - 1]) as i16
        };

        if strength < self.sensor_threshold as i16 {
            return None;
        }

        // Interpolation Logic
        let mut pos = (index as i16 * pts_per_el) + (pts_per_el / 2);

        if index == 0 {
            if meas_cnt[1] > 0 {
                pos += (meas_cnt[1] as i16 * pts_per_el) / 100;
            } else {
                pos = (meas_cnt[0] as i16 * (pts_per_el / 2)) / 100;
            }
        } else if index == N - 1 {
            if meas_cnt[index - 1] > 0 {
                pos -= (meas_cnt[index - 1] as i16 * pts_per_el) / 100;
            } else {
                pos = pts - (meas_cnt[index] as i16 * (pts_per_el / 2)) / 100;
            }
        } else {
            pos += (meas_cnt[index + 1] as i16 * pts_per_el) / 100;
            pos -= (meas_cnt[index - 1] as i16 * pts_per_el) / 100;
        }

        if pos > pts || pos < 0 {
            None
        } else {
            Some(pos)
        }
    }
}

impl SensorKind for Wheel {
    // TODO: still panics
    #[inline(always)]
    fn process<const N: usize>(&self, index: usize, meas_cnt: &[u16]) -> Option<i16> {
        let pts = self.points as i16;
        let num_els = N as i16;
        let pts_per_el = pts / num_els;

        // Wheel wraps around: check neighbors with modulo/index wrapping
        let strength: i16 = if index == 0 {
            (meas_cnt[0] + meas_cnt[N - 1] + meas_cnt[1]) as i16
        } else if index == N - 1 {
            (meas_cnt[index] + meas_cnt[0] + meas_cnt[index - 1]) as i16
        } else {
            (meas_cnt[index] + meas_cnt[index + 1] + meas_cnt[index - 1]) as i16
        };

        if strength <= self.sensor_threshold as i16 {
            return None;
        }

        let mut pos = (index as i16 * pts_per_el) + (pts_per_el / 2);

        if index == 0 {
            pos += (meas_cnt[1] as i16 * pts_per_el) / 100;
            pos -= (meas_cnt[N - 1] as i16 * pts_per_el) / 100;
            if pos < 0 {
                pos += pts + 1;
            }
        } else if index == N - 1 {
            pos += (meas_cnt[0] as i16 * pts_per_el) / 100;
            pos -= (meas_cnt[index - 1] as i16 * pts_per_el) / 100;
            if pos >= pts {
                pos -= pts;
            }
        } else {
            pos += (meas_cnt[index + 1] as i16 * pts_per_el) / 100;
            pos -= (meas_cnt[index - 1] as i16 * pts_per_el) / 100;
        }

        if pos > pts || pos < 0 {
            None
        } else {
            Some(pos)
        }
    }
}

// Trait to enforce the minimum N constraint per kind
pub trait MultiElement: SensorKind {}
impl MultiElement for Slider {}
impl MultiElement for Wheel {}

pub struct CapacitiveSensor<'a, T, const N: usize, C, Method, G, S, M = DefaultMapping>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
    G: Gate<T, M>,
    S: SensorKind,
{
    pub parts: &'a CapacitiveParts3<T, M>,
    pub elements: [CapacitiveElement; N],

    // Use the combined trait here!
    pub gate: &'a G,

    pub base_count: [u16; N],
    capacitive: &'a C,
    interval: G::Interval,

    // Fixed configuration set in constructor
    tracking_rate: TrackingRate,
    drift_rate: DriftRate,
    doi: DirectionOfInterest,

    had_event: bool,
    sensor: S,
    _method: PhantomData<Method>,
}

// Internal sealed trait to handle construction-time validation
pub trait ConstructValidate {
    fn validate<const N: usize>();
}

impl ConstructValidate for Button {
    #[inline(always)]
    fn validate<const N: usize>() {} // No constraint
}

impl ConstructValidate for Slider {
    #[inline(always)]
    fn validate<const N: usize>() {
        const { assert!(N >= 3, "Slider requires at least 3 elements") };
    }
}

impl ConstructValidate for Wheel {
    #[inline(always)]
    fn validate<const N: usize>() {
        const { assert!(N >= 3, "Wheel requires at least 3 elements") };
    }
}

// Single shared constructor body — not public
impl<'a, T, const N: usize, C, G, S, M, Method> CapacitiveSensor<'a, T, N, C, Method, G, S, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
    G: Gate<T, M>,
    S: SensorKind + ConstructValidate,
{
    #[inline(always)]
    fn build(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        interval: G::Interval,
        tracking: TrackingRate,
        drift: DriftRate,
        doi: DirectionOfInterest,
        sensor: S,
    ) -> Self {
        S::validate::<N>();
        Self {
            parts,
            elements,
            gate,
            base_count: unsafe { MaybeUninit::uninit().assume_init() },
            capacitive,
            interval,
            tracking_rate: tracking,
            drift_rate: drift,
            doi,
            had_event: false,
            _method: PhantomData,
            sensor,
        }
    }
}

impl<'a, T, const N: usize, C, G, S, M> CapacitiveSensor<'a, T, N, C, RO, G, S, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
    G: RoGate<T, M>,
    S: SensorKind + ConstructValidate,
{
    #[inline(always)]
    pub fn new_ro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        interval: G::Interval,
        tracking: TrackingRate,
        drift: DriftRate,
        doi: DirectionOfInterest,
        sensor: S,
    ) -> Self {
        Self::build(
            parts, elements, gate, capacitive, interval, tracking, drift, doi, sensor,
        )
    }
}

impl<'a, T, const N: usize, C, G, S, M> CapacitiveSensor<'a, T, N, C, FRO, G, S, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
    G: FroGate<T, M>,
    S: SensorKind + ConstructValidate,
{
    #[inline(always)]
    pub fn new_fro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        interval: G::Interval,
        tracking: TrackingRate,
        drift: DriftRate,
        doi: DirectionOfInterest,
        sensor: S,
    ) -> Self {
        Self::build(
            parts, elements, gate, capacitive, interval, tracking, drift, doi, sensor,
        )
    }
}

impl<'a, T, const N: usize, C, G, S, Method, M> CapacitiveSensor<'a, T, N, C, Method, G, S, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    Method: MeasurementMethod,
    C: Capacitive,
    G: Gate<T, M>,
    S: SensorKind,
{
    #[inline(always)]
    fn apply_tracking(base: &mut u16, temp: &mut u16, tracking: &TrackingRate, is_roi: bool) {
        let mut remainder = 0;

        match tracking {
            TrackingRate::Fast => {
                *temp /= 2;
                *base /= 2;
            }
            TrackingRate::Medium => {
                *temp /= 4;
                *base = 3 * (*base / 4);
            }
            TrackingRate::Slow => {
                remainder = (((*base & 0x3F) * 63) + (*temp & 0x3F)) >> 6;
                *temp /= 64;
                *base = 63 * (*base / 64);
            }
            TrackingRate::VerySlow => {
                remainder = (((*base & 0x7F) * 127) + (*temp & 0x7F)) >> 7;
                *temp /= 128;
                *base = 127 * (*base / 128);
            }
        }

        *base = base.saturating_add(*temp + remainder);

        if is_roi {
            *base = base.saturating_add(1);
        } else {
            *base = base.saturating_sub(1);
        }
    }

    #[inline(always)]
    fn normalize_to_percent(cnt: &mut u16, element: &CapacitiveElement) {
        if *cnt > element.max_response {
            *cnt = element.max_response;
        }
        let diff = cnt.saturating_sub(element.threshold);
        if (diff as u32 * 100) > u16::MAX as u32 {
            // SAFETY: enforced by element invariant
            unsafe {
                core::hint::unreachable_unchecked();
            }
        }
        *cnt = (100 * diff) / element.range;
    }

    #[inline(always)]
    fn apply_drift(base: &mut u16, temp: &mut u16, drift: &DriftRate, is_roi: bool) {
        let mut remainder = 1;

        match drift {
            DriftRate::VerySlow => {
                *temp = 0;
            }
            DriftRate::Slow => {
                remainder = 2;
                *temp = 0;
            }
            DriftRate::Medium => {
                *temp /= 4;
                *base = 3 * (*base / 4);
            }
            DriftRate::Fast => {
                *temp /= 2;
                *base /= 2;
            }
        }

        *base = base.saturating_add(*temp);

        if is_roi {
            *base = base.saturating_sub(remainder);
        } else {
            *base = base.saturating_add(remainder);
        }
    }

    /// Unified hardware sweep — drives the gate/IO sequence and calls `logic`
    /// once per element with `(index, element, raw_count)`.
    #[inline(always)]
    fn perform_hardware_sweep<F>(
        gate: &G,
        timer: &T,
        capacitive: &C,
        elements: &[CapacitiveElement; N],
        interval: G::Interval,
        mut logic: F,
    ) where
        F: FnMut(usize, &CapacitiveElement, u16),
    {
        gate.prepare(timer, interval, Method::IS_ROI);
        let context_save = capacitive.captioxctl_rd();
        for (i, element) in elements.iter().enumerate() {
            capacitive.captioxctl_set(element.pin_id);
            capacitive.enable();
            logic(i, element, gate.capture(timer, interval, Method::IS_ROI));
        }
        capacitive.captioxctl_set(context_save);
        gate.release(timer, Method::IS_ROI);
    }

    #[inline(always)]
    pub fn init(&mut self) {
        let base_count = &mut self.base_count;
        Self::perform_hardware_sweep(
            self.gate,
            &self.parts.timer,
            self.capacitive,
            &self.elements,
            self.interval,
            |i, _el, val| base_count[i] = val,
        );
    }

    #[inline(always)]
    pub fn update(&mut self, number_of_averages: u8) {
        let base_count = &mut self.base_count;
        for _ in 0..number_of_averages {
            Self::perform_hardware_sweep(
                self.gate,
                &self.parts.timer,
                self.capacitive,
                &self.elements,
                self.interval,
                |i, _el, val| {
                    base_count[i] = (val >> 1) + (base_count[i] >> 1);
                },
            );
        }
    }

    #[inline(always)]
    pub fn sensor(&mut self) -> Option<i16> {
        let mut has_event = false;
        let mut meas_cnt: [u16; N] = unsafe { MaybeUninit::uninit().assume_init() };
        let mut dominant = 0usize;
        let mut peak_delta = 0u16;

        // Split-borrow self so the closure can mutate base_count / had_event
        // while perform_hardware_sweep uses the remaining fields.
        let base_count = &mut self.base_count;
        let tracking = &self.tracking_rate;
        let drift = &self.drift_rate;
        let doi = &self.doi;
        let had_event = &mut self.had_event;

        Self::perform_hardware_sweep(
            self.gate,
            &self.parts.timer,
            self.capacitive,
            &self.elements,
            self.interval,
            |i, element, raw| {
                meas_cnt[i] = raw;

                if raw == 0 {
                    return;
                }

                let is_roi = Method::IS_ROI;
                let is_doi = matches!(doi, DirectionOfInterest::Increment);

                let mut temp_cnt = raw;

                if is_doi == is_roi {
                    if base_count[i] < raw {
                        meas_cnt[i] = 0;
                        if element.threshold != 0 {
                            let ceil = base_count[i].saturating_add(element.threshold >> 1);
                            if ceil < temp_cnt {
                                temp_cnt = ceil;
                            }
                        }
                    } else {
                        meas_cnt[i] = base_count[i].saturating_sub(raw);
                    }
                } else {
                    if base_count[i] > raw {
                        meas_cnt[i] = 0;
                        if element.threshold != 0 {
                            let floor = base_count[i].saturating_sub(element.threshold >> 1);
                            if floor > temp_cnt {
                                temp_cnt = floor;
                            }
                        }
                    } else {
                        meas_cnt[i] -= base_count[i];
                    }
                }

                // ── Update baseline / flag event ───────────────────────────
                if meas_cnt[i] == 0 {
                    Self::apply_tracking(&mut base_count[i], &mut temp_cnt, tracking, is_roi);
                } else if meas_cnt[i] < element.threshold && !*had_event {
                    Self::apply_drift(&mut base_count[i], &mut temp_cnt, drift, is_roi);
                } else if meas_cnt[i] >= element.threshold {
                    has_event = true;
                    *had_event = true;
                }

                // ── Normalise to 0–100 % and track dominant element ────────
                if meas_cnt[i] >= element.threshold {
                    Self::normalize_to_percent(&mut meas_cnt[i], element);
                    if meas_cnt[i] >= peak_delta {
                        peak_delta = meas_cnt[i];
                        dominant = i;
                    }
                } else {
                    meas_cnt[i] = 0;
                }
            },
        );

        if !has_event {
            *had_event = false;

            return None;
        }

        self.sensor.process::<N>(dominant, &meas_cnt)
    }
}
