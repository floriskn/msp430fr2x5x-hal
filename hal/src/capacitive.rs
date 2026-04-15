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
use core::{cell::Cell, marker::PhantomData};

// --- Traits ---

pub trait CaptivateIoTimer {}
pub trait CapacitiveCapable {
    const CHANNEL_SELECT: u16;
}

pub trait AnyCapacitiveElement {
    fn get_threshold(&self) -> u16;
    fn get_max_response(&self) -> u16;
    fn get_pin(&self) -> u16;
}

pub trait Gate<T: CapCmpTimer3<M> + CaptivateIoTimer, M: PinMap = DefaultMapping> {
    fn prepare(&self, timer: &T, accumulation_cycles: u16);
    fn capture(&self, timer: &T) -> u16;
    fn release(&self, timer: &T);
}

pub trait AnySensor {
    fn init(&mut self);
    fn update(&mut self, number_of_averages: u8);
    // Return a slice instead of a fixed-size array
    fn raw(&mut self, out: &mut [u16]);
    fn buttons(&mut self) -> Option<u8>;
    fn custom(&mut self, delta_cnt: &mut [u16]);
    fn dominant_element(&mut self, delta_cnt: &mut [u16]) -> u8;
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
            j += 1;
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

// Blanket implementation:
// "Anything that is a Gate and is RoCapable automatically implements RoGate"
// impl<I, T, M> RoGate<T, M> for I
// where
//     I: Gate<T, M> + RoCapable,
//     T: CapCmpTimer3<M> + CaptivateIoTimer,
//     M: PinMap,
// {
// }

// impl<I, T, M> FroGate<T, M> for I
// where
//     I: Gate<T, M> + FroCapable,
//     T: CapCmpTimer3<M> + CaptivateIoTimer,
//     M: PinMap,
// {
// }

// --- Elements & Sensors (Same as before) ---

pub struct CapacitiveElement<P: CapacitiveCapable> {
    pub pin: P,
    pub threshold: u16,
    pub max_response: u16,
}
impl<P: CapacitiveCapable> AnyCapacitiveElement for CapacitiveElement<P> {
    fn get_threshold(&self) -> u16 {
        self.threshold
    }
    fn get_max_response(&self) -> u16 {
        self.max_response
    }
    fn get_pin(&self) -> u16 {
        P::CHANNEL_SELECT
        // ((P::NUM as u16) << 1) | ((P::PORT() as u16) << 4)
        // let x = P::PORT;
        // let x1 = P::NUM;
        // 0
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
        timer.continuous();
        Self {
            timer,
            _pin_map: PhantomData,
        }
    }
}

pub struct CapacitiveSensor<'a, T, const N: usize, C, Method, M = DefaultMapping>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
{
    pub parts: &'a CapacitiveParts3<T, M>,
    pub elements: [&'a dyn AnyCapacitiveElement; N],

    // Use the combined trait here!
    pub gate: &'a dyn Gate<T, M>,

    pub base_count: [u16; N],
    capacitive: &'a C,
    accumulation_cycles: u16,
    cts_status_reg: u8,
    _method: PhantomData<Method>,
}

impl<'a, T, const N: usize, C, M> CapacitiveSensor<'a, T, N, C, RO, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
{
    pub fn new_ro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [&'a dyn AnyCapacitiveElement; N],
        gate: &'a dyn RoGate<T, M>, // Single trait object now
        capacitive: &'a C,
        accumulation_cycles: u16,
    ) -> Self {
        Self {
            parts,
            elements,
            gate,
            base_count: [0; N],
            capacitive,
            accumulation_cycles,
            cts_status_reg: 0x02 + 0x00 + 0x10,
            _method: PhantomData,
        }
    }
}

impl<'a, T, const N: usize, C, M> CapacitiveSensor<'a, T, N, C, FRO, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    C: Capacitive,
{
    pub fn new_fro(
        parts: &'a CapacitiveParts3<T, M>,
        elements: [&'a dyn AnyCapacitiveElement; N],
        gate: &'a dyn FroGate<T, M>, // Single trait object now
        capacitive: &'a C,
        accumulation_cycles: u16,
    ) -> Self {
        Self {
            parts,
            elements,
            gate,
            base_count: [0; N],
            capacitive,
            accumulation_cycles,
            cts_status_reg: 0x02 + 0x00 + 0x10,
            _method: PhantomData,
        }
    }
}

impl<'a, T, const N: usize, C, Method, M> AnySensor for CapacitiveSensor<'a, T, N, C, Method, M>
where
    T: CapCmpTimer3<M> + CaptivateIoTimer,
    M: PinMap,
    Method: MeasurementMethod,
    C: Capacitive,
{
    fn init(&mut self) {
        let mut out = [0; N];
        self.raw(&mut out);
        for i in 0..N {
            self.base_count[i] = out[i];
        }
    }

    fn update(&mut self, number_of_averages: u8) {
        // TODO: make uintit way cheaper in asm
        let mut out = [0; N];
        for _ in 0..number_of_averages {
            self.raw(&mut out);
            for i in 0..N {
                self.base_count[i] = (out[i] >> 1) + (self.base_count[i] >> 1);
            }
        }
    }

    fn raw(&mut self, out: &mut [u16]) {
        self.gate
            .prepare(&self.parts.timer, self.accumulation_cycles);

        let context_save_ctl = self.capacitive.captioxctl_rd();

        for (i, element) in self.elements.iter().enumerate() {
            self.capacitive.captioxctl_set(element.get_pin());
            self.capacitive.enable();

            out[i] = self.gate.capture(&self.parts.timer);
        }

        self.capacitive.captioxctl_set(context_save_ctl);
        self.gate.release(&self.parts.timer);
    }

    fn custom(&mut self, delta_cnt: &mut [u16]) {
        const EVNT: u8 = 0x01;
        const PAST_EVNT: u8 = 0x04;
        // ctsStatusReg &= ~ EVNT;

        self.cts_status_reg &= !EVNT;

        // let mut delta_cnt = [0u16; N];
        self.raw(delta_cnt);

        for (i, element) in self.elements.iter_mut().enumerate() {
            let mut temp_cnt = delta_cnt[i];
            if temp_cnt == 0 {
                continue;
            }

            let is_roi = Method::IS_ROI;
            let is_doi = (self.cts_status_reg & 0x02) != 0;

            // Interest in decrease vs increase logic
            // Logic: (DOI && RO) || (!DOI && !RO)
            if is_doi == is_roi {
            } else {
            }

            const TRADOI_VSLOW: u8 = 0xC0;
            const TRADOI_FAST: u8 = 0x00;
            const TRADOI_MED: u8 = 0x40;
            const TRADOI_SLOW: u8 = 0x80;

            const TRIDOI_VSLOW: u8 = 0x00;
            const TRIDOI_SLOW: u8 = 0x10;
            const TRIDOI_MED: u8 = 0x20;
            const TRIDOI_FAST: u8 = 0x30;

            if delta_cnt[i] == 0 {
                let mut remainder: u16 = 0;
                match self.cts_status_reg & TRADOI_VSLOW {
                    TRADOI_FAST => {
                        temp_cnt /= 2;
                        self.base_count[i] /= 2;
                    }
                    TRADOI_MED => {
                        temp_cnt /= 4;
                        self.base_count[i] = 3 * (self.base_count[i] / 4);
                    }
                    TRADOI_SLOW => {
                        remainder = (0x3F & self.base_count[i]) * 63;
                        remainder = (remainder + (0x3F & temp_cnt)) >> 6;
                        temp_cnt /= 64;
                        self.base_count[i] = 63 * (self.base_count[i] / 64);
                    }
                    _ => {
                        // VSLOW
                        remainder = (0x7F & self.base_count[i]) * 127;
                        remainder = (remainder + (0x7F & temp_cnt)) >> 7;
                        temp_cnt /= 128;
                        self.base_count[i] = 127 * (self.base_count[i] / 128);
                    }
                }
                self.base_count[i] += temp_cnt + remainder;

                // Adjust baseline by 1 to prevent "sticking"
                if is_roi {
                    self.base_count[i] += 1;
                } else {
                    self.base_count[i] = self.base_count[i].saturating_sub(1);
                }
            } else if delta_cnt[i] < element.get_threshold() && self.cts_status_reg & PAST_EVNT == 0
            {
                // Drift following (Slow tracking while not in a touch event)
                let mut remainder = 1;
                match self.cts_status_reg & TRIDOI_FAST {
                    TRIDOI_VSLOW => {
                        temp_cnt = 0;
                    }
                    TRIDOI_SLOW => {
                        remainder = 2;
                        temp_cnt = 0;
                    }
                    TRIDOI_MED => {
                        temp_cnt /= 4;
                        self.base_count[i] = 3 * (self.base_count[i] / 4);
                    }
                    _ => {
                        // fast
                        temp_cnt /= 2;
                        self.base_count[i] /= 2;
                    } // FAST
                }
                self.base_count[i] += temp_cnt;
                if is_roi {
                    self.base_count[i] = self.base_count[i].saturating_sub(remainder);
                } else {
                    self.base_count[i] += remainder;
                }
            } else if delta_cnt[i] >= element.get_threshold() {
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

    fn buttons(&mut self) -> Option<u8> {
        let mut delta_cnt = [0u16; N];

        self.custom(&mut delta_cnt);

        const EVNT: u8 = 0x01;
        if self.cts_status_reg & EVNT != 0 {
            Some(self.dominant_element(&mut delta_cnt))
        } else {
            None
        }
    }

    fn dominant_element(&mut self, delta_cnt: &mut [u16]) -> u8 {
        let mut dominant_element = 0;
        let mut percent_delta = 0;

        for (i, element) in self.elements.iter().enumerate() {
            if delta_cnt[i] >= element.get_threshold() {
                if delta_cnt[i] > element.get_max_response() {
                    delta_cnt[i] = element.get_max_response();
                }

                delta_cnt[i] = 100 * (delta_cnt[i] - element.get_threshold())
                    / (element.get_max_response() - element.get_threshold());

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

