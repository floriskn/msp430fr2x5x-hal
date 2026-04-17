//! Capacitive Touch implementation for MSP430FR4133

use crate::{
    _pac,
    hw_traits::{
        capacitive::*,
        timer_a::{CCRn, Tbssel},
    },
    pin_mapping::{DefaultMapping, PinMap},
    timer::*,
    watchdog::{IntervalMode, Wdt},
};
use core::{cell::Cell, marker::PhantomData, mem::MaybeUninit, num::NonZero};

// --- Traits ---

pub trait CaptivateIoTimer {}
pub trait CapacitiveCapable {
    const CHANNEL_SELECT: u16;
}

pub trait Gate<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap = DefaultMapping> {
    fn prepare(&self, timer: &T, accumulation_cycles: u16);
    fn capture(&self, timer: &T) -> u16;
    fn release(&self, timer: &T);
}

// --- Gating Implementations ---

/// 1. WDT Gate (Borrowed)
pub struct WdtGate<'a>(&'a Wdt<IntervalMode>);
impl<'a> WdtGate<'a> {
    pub fn new(wdt: &'a Wdt<IntervalMode>) -> Self {
        Self(wdt)
    }
}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for WdtGate<'a> {
    fn prepare(&self, timer: &T, accumulation_cycles: u16) {}
    fn capture(&self, timer: &T) -> u16 {
        0
    }
    fn release(&self, timer: &T) {}
}

/// 2. Generic Locked Wrapper (Used for Timer and RTC)
/// This consumes the peripheral so it cannot be used elsewhere.
pub struct Locked<T>(pub T);
impl<T> Locked<T> {
    pub fn new(peripheral: T) -> Self {
        Self(peripheral)
    }
}

/// 3. Timer Gate (Using Locked)
pub struct TimerGate<'a, TG, MG = DefaultMapping>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    pub locked: &'a Locked<TG>,
    pub config: TimerConfig<TG, MG>,
}
impl<'a, TG, MG> TimerGate<'a, TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    pub fn new(locked: &'a Locked<TG>, config: TimerConfig<TG, MG>) -> Self {
        Self { locked, config }
    }
}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap, TG, MG> Gate<T, M>
    for TimerGate<'a, TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
    fn prepare(&self, timer: &T, accumulation_cycles: u16) {}
    fn capture(&self, timer: &T) -> u16 {
        0
    }
    fn release(&self, timer: &T) {}
}

/// 4. RTC Gate (Using Locked)
pub struct RtcGate<'a> {
    pub locked: &'a Locked<_pac::Rtc>,
    pub timeout: u16,
}
impl<'a> RtcGate<'a> {
    pub fn new(locked: &'a Locked<_pac::Rtc>, timeout: u16) -> Self {
        Self { locked, timeout }
    }
}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for RtcGate<'a> {
    fn prepare(&self, timer: &T, accumulation_cycles: u16) {}
    fn capture(&self, timer: &T) -> u16 {
        0
    }
    fn release(&self, timer: &T) {}
}

/// 5. Software Gate (No Peripherals)
pub struct SwGate {
    context_save_ta0ctl: Cell<u16>,
    context_save_tacctl0: Cell<u16>,
}

impl SwGate {
    pub fn new() -> Self {
        Self {
            context_save_ta0ctl: Cell::new(0),
            context_save_tacctl0: Cell::new(0),
        }
    }
}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> Gate<T, M> for SwGate {
    fn prepare(&self, timer: &T, accumulation_cycles: u16) {
        self.context_save_ta0ctl.set(timer.get_ctl());
        self.context_save_tacctl0
            .set(CCRn::<CCR0>::get_cctln(timer));

        CCRn::<CCR0>::set_ccrn(timer, accumulation_cycles);
        // timer.set_ccrn(count);
    }
    fn capture(&self, timer: &T) -> u16 {
        let mut j = 0u16;
        timer.config_clock(Tbssel::Inclk, TimerDiv::_1);
        timer.upmode();

        while !timer.tbifg_rd() {
            j = j.saturating_add(1);
        }

        timer.stop();

        return j;
    }
    fn release(&self, timer: &T) {
        timer.set_ctl(self.context_save_ta0ctl.get());
        CCRn::<CCR0>::set_cctln(timer, self.context_save_tacctl0.get());
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
impl<'a> RoCapable for WdtGate<'a> {}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> RoGate<T, M> for WdtGate<'a> {}
impl<'a> RoCapable for RtcGate<'a> {}
impl<'a, T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> RoGate<T, M> for RtcGate<'a> {}

// SW only supports fRO
impl FroCapable for SwGate {}
impl<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap> FroGate<T, M> for SwGate {}

// TimerGate supports BOTH
impl<'a, TG, MG> RoCapable for TimerGate<'a, TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
}
impl<'a, TG, MG, T, M> RoGate<T, M> for TimerGate<'a, TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
{
}

impl<'a, TG, MG> FroCapable for TimerGate<'a, TG, MG>
where
    TG: TimerPeriph<MG>,
    MG: PinMap,
{
}
impl<'a, TG, MG, T, M> FroGate<T, M> for TimerGate<'a, TG, MG>
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
    pub const fn new() -> Self {
        Self
    }
}

impl Slider {
    pub const fn new(sensor_threshold: u8, points: u8) -> Self {
        Self { sensor_threshold, points }
    }
}

impl Wheel {
    pub const fn new(sensor_threshold: u8, points: u8) -> Self {
        Self { sensor_threshold, points }
    }
}

// Seal them behind a trait
pub trait SensorKind {
    fn process<const N: usize>(&self, index: usize, meas_cnt: &[u16]) -> Option<i16>;
}

impl SensorKind for Button {
    fn process<const N: usize>(&self, index: usize, _meas_cnt: &[u16]) -> Option<i16> {
        Some(index as i16)
    }
}

impl SensorKind for Slider {
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
    accumulation_cycles: u16,

    // Fixed configuration set in constructor
    tracking_rate: TrackingRate,
    drift_rate: DriftRate,
    doi: DirectionOfInterest,

    cts_status_reg: u8,
    sensor: S,
    _method: PhantomData<Method>,
}

// Internal sealed trait to handle construction-time validation
pub trait ConstructValidate {
    fn validate<const N: usize>();
}

impl ConstructValidate for Button {
    fn validate<const N: usize>() {} // No constraint
}

impl ConstructValidate for Slider {
    fn validate<const N: usize>() {
        const { assert!(N >= 3, "Slider requires at least 3 elements") };
    }
}

impl ConstructValidate for Wheel {
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
    fn build(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        accumulation_cycles: u16,
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
            base_count: [0u16; N],
            capacitive,
            accumulation_cycles,
            tracking_rate: tracking,
            drift_rate: drift,
            doi,
            cts_status_reg: 0,
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
    pub fn new_ro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        accumulation_cycles: u16,
        tracking: TrackingRate,
        drift: DriftRate,
        doi: DirectionOfInterest,
        sensor: S,
    ) -> Self {
        Self::build(parts, elements, gate, capacitive, accumulation_cycles, tracking, drift, doi, sensor)
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
    pub fn new_fro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [CapacitiveElement; N],
        gate: &'a G,
        capacitive: &'a C,
        accumulation_cycles: u16,
        tracking: TrackingRate,
        drift: DriftRate,
        doi: DirectionOfInterest,
        sensor: S,
    ) -> Self {
        Self::build(parts, elements, gate, capacitive, accumulation_cycles, tracking, drift, doi, sensor)
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
    /// THE UNIFIED HARDWARE LOOP
    /// This is a "Static" helper - it does not take &self.
    fn perform_hardware_sweep<F>(
        gate: &dyn Gate<T, M>,
        timer: &T,
        capacitive: &C,
        elements: &[CapacitiveElement; N],
        cycles: u16,
        mut logic: F,
    ) where
        F: FnMut(usize, u16),
    {
        gate.prepare(timer, cycles);
        let context_save = capacitive.captioxctl_rd();

        for (i, element) in elements.iter().enumerate() {
            capacitive.captioxctl_set(element.pin_id);
            capacitive.enable();
            let raw_val = gate.capture(timer);
            logic(i, raw_val);
        }

        capacitive.captioxctl_set(context_save);
        gate.release(timer);
    }

    pub fn init(&mut self) {
        let base_counts = &mut self.base_count;

        Self::perform_hardware_sweep(
            self.gate,
            &self.parts.timer,
            self.capacitive,
            &self.elements,
            self.accumulation_cycles,
            |i, val| {
                base_counts[i] = val;
            },
        );
    }

    pub fn update(&mut self, number_of_averages: u8) {
        let base_counts = &mut self.base_count;

        for _ in 0..number_of_averages {
            Self::perform_hardware_sweep(
                self.gate,
                &self.parts.timer,
                self.capacitive,
                &self.elements,
                self.accumulation_cycles,
                |i, val| {
                    // Now base_counts is a local unique reference, no longer tied to 'self' borrow
                    base_counts[i] = (val >> 1) + (base_counts[i] >> 1);
                },
            );
        }
    }

    fn raw(&mut self, out: &mut [u16]) {
        Self::perform_hardware_sweep(
            self.gate,
            &self.parts.timer,
            self.capacitive,
            &self.elements,
            self.accumulation_cycles,
            |i, val| out[i] = val,
        );
    }

    fn custom(&mut self, delta_cnt: &mut [u16]) {
        const EVNT: u8 = 0x01;
        const PAST_EVNT: u8 = 0x04;
        // ctsStatusReg &= ~ EVNT;

        self.cts_status_reg &= !EVNT;

        // let mut delta_cnt = [0u16; N];
        self.raw(delta_cnt);

        for (i, element) in self.elements.iter().enumerate() {
            let mut temp_cnt = delta_cnt[i];
            if temp_cnt == 0 {
                continue;
            }

            let is_roi = Method::IS_ROI;

            let is_doi = matches!(self.doi, DirectionOfInterest::Increment);

            // Interest in decrease vs increase logic
            // Logic: (DOI && RO) || (!DOI && !RO)
            if is_doi == is_roi {
                if self.base_count[i] < delta_cnt[i] {
                    delta_cnt[i] = 0;

                    if element.threshold != 0 {
                        let temp = self.base_count[i].saturating_add(element.threshold >> 1);

                        if temp < temp_cnt {
                            temp_cnt = temp;
                        }
                    }
                } else {
                    delta_cnt[i] = self.base_count[i].wrapping_sub(delta_cnt[i]);
                }
            } else {
                if self.base_count[i] > delta_cnt[i] {
                    delta_cnt[i] = 0;
                    if element.threshold != 0 {
                        let temp = self.base_count[i].saturating_sub(element.threshold >> 1);
                        if temp > temp_cnt {
                            temp_cnt = temp;
                        }
                    }
                } else {
                    delta_cnt[i] -= self.base_count[i];
                }
            }

            if delta_cnt[i] == 0 {
                let mut remainder: u16 = 0;
                match self.tracking_rate {
                    TrackingRate::Fast => {
                        temp_cnt /= 2;
                        self.base_count[i] /= 2;
                    }
                    TrackingRate::Medium => {
                        temp_cnt /= 4;
                        self.base_count[i] = 3 * (self.base_count[i] / 4);
                    }
                    TrackingRate::Slow => {
                        remainder = (0x3F & self.base_count[i]) * 63;
                        remainder = (remainder + (0x3F & temp_cnt)) >> 6;
                        temp_cnt /= 64;
                        self.base_count[i] = 63 * (self.base_count[i] / 64);
                    }
                    TrackingRate::VerySlow => {
                        // VSLOW
                        remainder = (0x7F & self.base_count[i]) * 127;
                        remainder = (remainder + (0x7F & temp_cnt)) >> 7;
                        temp_cnt /= 128;
                        self.base_count[i] = 127 * (self.base_count[i] / 128);
                    }
                }
                self.base_count[i] = self.base_count[i].saturating_add(temp_cnt + remainder);

                // Adjust baseline by 1 to prevent "sticking"
                if is_roi {
                    self.base_count[i] = self.base_count[i].saturating_add(1);
                } else {
                    self.base_count[i] = self.base_count[i].saturating_sub(1);
                }
            } else if delta_cnt[i] < element.threshold && self.cts_status_reg & PAST_EVNT == 0 {
                // Drift following (Slow tracking while not in a touch event)
                let mut remainder = 1;
                match self.drift_rate {
                    DriftRate::VerySlow => {
                        temp_cnt = 0;
                    }
                    DriftRate::Slow => {
                        remainder = 2;
                        temp_cnt = 0;
                    }
                    DriftRate::Medium => {
                        temp_cnt /= 4;
                        self.base_count[i] = 3 * (self.base_count[i] / 4);
                    }
                    DriftRate::Fast => {
                        // fast
                        temp_cnt /= 2;
                        self.base_count[i] /= 2;
                    } // FAST
                }
                self.base_count[i] = self.base_count[i].saturating_add(temp_cnt);
                if is_roi {
                    self.base_count[i] = self.base_count[i].saturating_sub(remainder);
                } else {
                    self.base_count[i] = self.base_count[i].saturating_add(remainder);
                }
            } else if delta_cnt[i] >= element.threshold {
                self.cts_status_reg |= EVNT;
                self.cts_status_reg |= PAST_EVNT;
            }
        }

        // 0100 1001
        // 0010 0010
        // 1100 0000

        if self.cts_status_reg & EVNT == 0 {
            self.cts_status_reg &= !PAST_EVNT;
        }
    }

    pub fn sensor(&mut self) -> Option<i16> {
        // 1. Allocate measurement array on stack
        let mut meas_cnt = [0u16; N];

        // 2. Perform measurements (Equivalent to TI_CAPT_Custom)
        self.custom(&mut meas_cnt);

        const EVNT: u8 = 0x01;

        // 3. Check if any element triggered an event
        if (self.cts_status_reg & EVNT) == 0 {
            return None;
        }

        // 4. Find the strongest element (Equivalent to Dominant_Element)
        let index = self.dominant_element(&mut meas_cnt) as usize;

        self.sensor.process::<N>(index, &meas_cnt)
    }

    fn dominant_element(&mut self, delta_cnt: &mut [u16]) -> u8 {
        let mut dominant_element = 0;
        let mut percent_delta = 0;

        for (i, element) in self.elements.iter().enumerate() {
            if delta_cnt[i] >= element.threshold {
                if delta_cnt[i] > element.max_response {
                    delta_cnt[i] = element.max_response;
                }

                // delta_cnt[i] = 100 * (delta_cnt[i] - element.threshold)
                //     / (element.max_response - element.threshold);

                let diff = delta_cnt[i].saturating_sub(element.threshold);

                // Tell the compiler the result of 100 * diff is guaranteed to be <= u16::MAX
                if (diff as u32 * 100) > u16::MAX as u32 {
                    unsafe {
                        core::hint::unreachable_unchecked();
                    }
                }

                delta_cnt[i] = (100 * diff) / element.range;

                if delta_cnt[i] >= percent_delta {
                    percent_delta = delta_cnt[i];
                    dominant_element = i as u8;
                }
            } else {
                delta_cnt[i] = 0;
            }
        }

        return dominant_element;
    }
}
