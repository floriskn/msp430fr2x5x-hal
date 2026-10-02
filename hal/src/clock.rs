//! Clock system for configuration of MCLK, SMCLK, ACLK, and XT1.
//!
//! Once configuration is complete, `Aclk`, `Smclk`, and optionally `Xt1clk` clock objects
//! are returned. These objects are used to set the clock sources on other peripherals.
//!
//! Configuration of MCLK and SMCLK *must* occur, though SMCLK can be disabled. XT1
//! configuration is optional but, when enabled, provides a high-precision source for
//! system clocks or the FLL reference.
//!
//! DCO with FLL is supported on MCLK, at the frequencies of [`DcoclkFreqSel`] or at any frequency
//! from 1 MHz up to the device maximum ([`ClockConfig::mclk_dcoclk_hz`]). The FLL can be
//! referenced by either the internal REFO or XT1. The highest frequency of the device uses the
//! factory DCO trim; every other frequency is trimmed in software while the clocks are
//! configured, as the user's guide recommends, so the FLL locks reliably.
//!
//! XT1 runs in low-frequency mode, with a 32.768 kHz watch crystal or clock input, on every
//! device. Devices with high-frequency XT1 support also accept 1 MHz to 24 MHz crystals and
//! clock inputs (`HighFrequency`).
//!
//! When XT1 fails, the hardware fail-safe keeps the clocks sourced from XT1 running from a
//! fallback oscillator, and keeps them there until software clears the fault flags. See
//! [`Xt1clk::is_faulted`] and [`Xt1clk::clear_fault`].

use core::arch::asm;
use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;

pub use crate::_pac::cs::csctl5::{Divm as MclkDiv, Divs as SmclkDiv};
pub use crate::_pac::cs::csctl6::Xt1drive as Xt1Drive;
pub use crate::device_specific::clock::{Xt1Xin, Xt1Xout};
use crate::delay::{delay_cycles, SysDelay};
use crate::device_specific::clock::FLLREFDIV_1;
use crate::fram::{Fram, WaitStates};
use crate::_pac::{
    self,
    cs::{
        csctl1::Dcorsel,
        csctl3::{Fllrefdiv, Selref},
        csctl4::{Sela, Selms},
    },
};

#[cfg(feature = "xt1_high_frequency")]
use crate::_pac::cs::csctl6::{Diva, Xt1hffreq, Xts};

/// REFOCLK frequency
pub const REFOCLK_FREQ_HZ: u16 = 32768;
/// VLOCLK frequency
pub const VLOCLK_FREQ_HZ: u16 = 10000;
pub use crate::device_specific::MODCLK_FREQ_HZ;

/// MCLK frequency out of reset: DCOCLKDIV locked to 32 x REFOCLK. XT1 start-up timeouts are
/// timed against it, because they run before the new configuration is applied.
const RESET_MCLK_FREQ_HZ: u32 = 32 * REFOCLK_FREQ_HZ as u32;

/// Highest XT1 frequency in low-frequency mode. ACLK must not exceed this either
/// (SLAU445I 3.1).
const XT1_LF_MAX_HZ: u32 = 40_000;
/// Lowest XT1 frequency in high-frequency mode (SLAU445I Table 3-10)
#[cfg(feature = "xt1_high_frequency")]
const XT1_HF_MIN_HZ: u32 = 1_000_000;
/// Highest XT1 frequency in high-frequency mode. The band above 16 MHz exists on the enhanced
/// clock system only (SLAU445I Table 3-1).
#[cfg(all(feature = "xt1_high_frequency", feature = "enhanced_cs"))]
const XT1_HF_MAX_HZ: u32 = 24_000_000;
#[cfg(all(feature = "xt1_high_frequency", not(feature = "enhanced_cs")))]
const XT1_HF_MAX_HZ: u32 = 16_000_000;

/// Highest MCLK frequency that needs no FRAM wait states. Every further 8 MHz needs one more
/// wait state (device data sheets, recommended operating conditions).
const FRAM_NO_WAIT_MAX_HZ: u32 = 8_000_000;

/// Highest MCLK frequency (device data sheets, recommended operating conditions, fSYSTEM)
#[cfg(feature = "enhanced_cs")]
const MCLK_MAX_HZ: u32 = 24_000_000;
#[cfg(not(feature = "enhanced_cs"))]
const MCLK_MAX_HZ: u32 = 16_000_000;

/// Lowest DCOCLK target `mclk_dcoclk_hz` accepts, the nominal frequency of the lowest DCO range
const DCO_MIN_HZ: u32 = 1_000_000;
/// Nominal frequency of each DCO range, by DCORSEL (SLAU445I CSCTL1)
const DCO_RANGE_NOMINAL_HZ: [u32; 8] = [
    1_000_000, 2_000_000, 4_000_000, 8_000_000, 12_000_000, 16_000_000, 20_000_000, 24_000_000,
];
/// The targets above which `mclk_dcoclk_hz` moves to the next DCO range: the geometric means of
/// neighbouring nominal frequencies, so every target gets the range whose nominal frequency is
/// closest to it. Each range then reaches the targets it gets with its taps near the middle at
/// some trim setting (device data sheets, DCO frequency), so the software trim can center them.
const DCO_RANGE_BOUNDARY_HZ: [u32; 7] = [
    1_414_214, 2_828_427, 5_656_854, 9_797_959, 13_856_406, 17_888_544, 21_908_902,
];

// CSCTL0 and CSCTL1 fields used by the DCO software trim. They're accessed as raw bits because
// the PACs name them differently.
/// CSCTL0 DCO tap bits
const DCO_TAP_MASK: u16 = 0x1FF;
/// The middle of the DCO tap range, where the trim routine aims the locked tap
const DCO_TAP_MID: u16 = 256;
/// CSCTL1 DCOFTRIMEN bit
const DCOFTRIMEN: u16 = 1 << 7;
/// CSCTL1 DCOFTRIM field position
const DCOFTRIM_SHIFT: u16 = 4;
/// CSCTL1 DCOFTRIM field
const DCOFTRIM_MASK: u16 = 0b111 << DCOFTRIM_SHIFT;
/// Highest DCOFTRIM value
const DCOFTRIM_MAX: u16 = 7;
/// DCOFTRIM value the trim routine starts from, as in TI's reference routine
const DCOFTRIM_START: u16 = 3;
/// CSCTL1 DCORSEL field position
const DCORSEL_SHIFT: u16 = 1;

#[derive(Clone, Copy)]
enum MclkSel {
    Refoclk,
    Vloclk,
    Dcoclk(DcoTarget),
    Xt1clk,
}

impl MclkSel {
    #[inline(always)]
    fn selms(&self) -> Selms {
        match self {
            MclkSel::Vloclk => Selms::Vloclk,
            MclkSel::Refoclk => Selms::Refoclk,
            MclkSel::Dcoclk(_) => Selms::Dcoclkdiv,
            MclkSel::Xt1clk => Selms::Xt1clk,
        }
    }
}

#[derive(Clone, Copy)]
enum AclkSel {
    #[cfg(feature = "vloclk_source")]
    Vloclk,
    Refoclk,
    Xt1clk,
}

impl AclkSel {
    #[inline(always)]
    fn sela(self) -> Sela {
        match self {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => Sela::Vloclk,
            AclkSel::Refoclk => Sela::Refoclk,
            AclkSel::Xt1clk => Sela::Xt1clk,
        }
    }

    /// This selection, with XT1CLK replaced by REFOCLK
    #[inline(always)]
    fn without_xt1(self) -> Self {
        match self {
            AclkSel::Xt1clk => AclkSel::Refoclk,
            sel => sel,
        }
    }
}

/// Selectable DCOCLK frequencies. With REFO as FLL reference the DCO locks to the frequency in
/// brackets: the largest multiple of 32.768 kHz that doesn't exceed the target, so the clock
/// stays within the device limits (1 MHz keeps the reset default instead). Other frequencies
/// are available through [`ClockConfig::mclk_dcoclk_hz`].
#[derive(Clone, Copy)]
pub enum DcoclkFreqSel {
    /// 1 MHz (1.048576 MHz)
    _1MHz,
    /// 2 MHz (1.998848 MHz)
    _2MHz,
    /// 4 MHz (3.997696 MHz)
    _4MHz,
    /// 8 MHz (7.995392 MHz)
    _8MHz,
    /// 12 MHz (11.993088 MHz)
    _12MHz,
    /// 16 MHz (15.990784 MHz)
    _16MHz,
    #[cfg(feature = "enhanced_cs")]
    /// 20 MHz (19.988480 MHz)
    _20MHz,
    #[cfg(feature = "enhanced_cs")]
    /// 24 MHz (23.986176 MHz)
    _24MHz,
}

impl DcoclkFreqSel {
    #[inline(always)]
    fn dcorsel(self) -> Dcorsel {
        match self {
            DcoclkFreqSel::_1MHz => Dcorsel::Dcorsel0,
            DcoclkFreqSel::_2MHz => Dcorsel::Dcorsel1,
            DcoclkFreqSel::_4MHz => Dcorsel::Dcorsel2,
            DcoclkFreqSel::_8MHz => Dcorsel::Dcorsel3,
            DcoclkFreqSel::_12MHz => Dcorsel::Dcorsel4,
            DcoclkFreqSel::_16MHz => Dcorsel::Dcorsel5,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_20MHz => Dcorsel::Dcorsel6,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_24MHz => Dcorsel::Dcorsel7,
        }
    }

    /// FLL multiplier (FLLN + 1) with REFO as reference
    #[inline(always)]
    fn multiplier(self) -> u16 {
        match self {
            DcoclkFreqSel::_1MHz => 32,
            DcoclkFreqSel::_2MHz => 61,
            DcoclkFreqSel::_4MHz => 122,
            DcoclkFreqSel::_8MHz => 244,
            DcoclkFreqSel::_12MHz => 366,
            DcoclkFreqSel::_16MHz => 488,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_20MHz => 610,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_24MHz => 732,
        }
    }

    /// Whether this is the highest DCO range of the device. TI recommends the factory DCO trim
    /// there, and software trim for every other range (SLAU445I 3.2.11).
    #[inline(always)]
    fn factory_trimmed(self) -> bool {
        #[cfg(feature = "enhanced_cs")]
        let highest = matches!(self, DcoclkFreqSel::_24MHz);
        #[cfg(not(feature = "enhanced_cs"))]
        let highest = matches!(self, DcoclkFreqSel::_16MHz);
        highest
    }

    /// Numerical frequency, with REFO as FLL reference
    #[inline]
    pub fn freq(self) -> u32 {
        (self.multiplier() as u32) * (REFOCLK_FREQ_HZ as u32)
    }

    /// The highest frequency of the device
    #[cfg(feature = "enhanced_cs")]
    const HIGHEST: Self = DcoclkFreqSel::_24MHz;
    #[cfg(not(feature = "enhanced_cs"))]
    const HIGHEST: Self = DcoclkFreqSel::_16MHz;
}

/// What the FLL locks DCOCLKDIV to
#[derive(Clone, Copy)]
struct DcoTarget {
    /// The FLL locks to the largest multiple of its reference that doesn't exceed this
    freq: u32,
    /// DCO range
    range: Dcorsel,
    /// Lock with the factory DCO trim instead of trimming in software
    factory_trim: bool,
}

impl From<DcoclkFreqSel> for DcoTarget {
    #[inline(always)]
    fn from(sel: DcoclkFreqSel) -> Self {
        DcoTarget { freq: sel.freq(), range: sel.dcorsel(), factory_trim: sel.factory_trimmed() }
    }
}

impl DcoTarget {
    /// The target for `freq` Hz: the DCO range whose nominal frequency is closest to it, trimmed
    /// in software. Targets from the highest [`DcoclkFreqSel`] frequency up lock like it does,
    /// with the factory trim the user's guide recommends there (SLAU445I 3.2.11).
    #[inline(always)]
    fn from_hz(freq: u32) -> Self {
        let highest = DcoTarget::from(DcoclkFreqSel::HIGHEST);
        if freq >= highest.freq {
            return DcoTarget { freq, ..highest };
        }
        let range = match DCO_RANGE_BOUNDARY_HZ.iter().filter(|&&boundary| freq > boundary).count() {
            0 => Dcorsel::Dcorsel0,
            1 => Dcorsel::Dcorsel1,
            2 => Dcorsel::Dcorsel2,
            3 => Dcorsel::Dcorsel3,
            4 => Dcorsel::Dcorsel4,
            #[cfg(not(feature = "enhanced_cs"))]
            _ => Dcorsel::Dcorsel5,
            #[cfg(feature = "enhanced_cs")]
            5 => Dcorsel::Dcorsel5,
            #[cfg(feature = "enhanced_cs")]
            6 => Dcorsel::Dcorsel6,
            #[cfg(feature = "enhanced_cs")]
            _ => Dcorsel::Dcorsel7,
        };
        DcoTarget { freq, range, factory_trim: false }
    }

    /// Nominal frequency of the DCO range
    #[inline(always)]
    fn range_freq(self) -> u32 {
        DCO_RANGE_NOMINAL_HZ[self.range as usize]
    }
}

/// Typestate for `ClockConfig` that represents unconfigured clocks
pub struct NoClockDefined;
/// Typestate for `ClockConfig` that represents a configured MCLK
pub struct MclkDefined(MclkSel);
/// Typestate for `ClockConfig` that represents a configured SMCLK
pub struct SmclkDefined(SmclkDiv);
/// Typestate for `ClockConfig` that represents disabled SMCLK
pub struct SmclkDisabled;
/// Typestate for `ClockConfig` that represents a configured XT1CLK
pub struct Xt1Defined<MODE, RANGE>(Xt1Config<MODE, RANGE>);
/// Typestate for `ClockConfig` that represents disabled/unconfigured XT1CLK
pub struct Xt1Disabled;

/// Typestate marker for XT1 **crystal mode** (external crystal, oscillator enabled).
pub struct CrystalMode;

/// Typestate marker for XT1 **bypass mode** (external clock input, oscillator disabled).
pub struct BypassMode;

/// Typestate for XT1 in low-frequency mode, with a 32.768 kHz watch crystal or clock input.
/// Only this mode keeps running in LPM3 and LPM3.5, and only this mode can clock the RTC.
pub struct LowFrequency;

/// Typestate for XT1 in high-frequency mode, with a 1 MHz to 24 MHz crystal or clock input
/// (16 MHz without the enhanced clock system). XT1 only runs from active mode to LPM0 in this
/// mode, and cannot clock the RTC (device data sheet, XTCLK distribution).
#[cfg(feature = "xt1_high_frequency")]
pub struct HighFrequency;

mod sealed {
    pub trait SealedXt1Range {}

    impl SealedXt1Range for super::LowFrequency {}
    #[cfg(feature = "xt1_high_frequency")]
    impl SealedXt1Range for super::HighFrequency {}
}

/// XT1 frequency modes: [`LowFrequency`], and `HighFrequency` on devices that support it
pub trait Xt1Range: sealed::SealedXt1Range {
    #[doc(hidden)]
    const HIGH_FREQUENCY: bool;
}

impl Xt1Range for LowFrequency {
    const HIGH_FREQUENCY: bool = false;
}

#[cfg(feature = "xt1_high_frequency")]
impl Xt1Range for HighFrequency {
    const HIGH_FREQUENCY: bool = true;
}

/// Configuration object for the XT1 oscillator.
///
/// This struct defines how the XT1 clock source should be initialized when
/// applying the clock configuration via [`ClockConfig::xt1clk_on`].
///
/// XT1 can operate in two modes:
///
/// - **Crystal mode**: Uses an external crystal connected to both XIN and XOUT.
/// - **Bypass mode**: Uses an external clock signal fed into XIN only.
///
/// Each mode runs in low-frequency mode ([`LowFrequency`]), or in high-frequency mode
/// (`HighFrequency`) on devices that support it.
///
/// The configuration includes:
/// - The input frequency (used for timing calculations and routing decisions)
/// - Drive strength for the oscillator (relevant in crystal mode)
/// - Whether bypass mode is enabled
/// - Automatic Gain Control (AGC) behavior
pub struct Xt1Config<MODE, RANGE = LowFrequency> {
    frequency: u32,
    drive: Xt1Drive,
    agc: bool,
    start_counter: bool,
    bypass: bool,
    #[cfg(feature = "enhanced_cs")]
    fault_switch: bool,
    auto_off: bool,
    _mode: PhantomData<(MODE, RANGE)>,
}

#[cfg(feature = "enhanced_cs")]
impl<MODE, RANGE> Xt1Config<MODE, RANGE> {
    /// Disable the automatic ACLK fallback on XT1 fault.
    ///
    /// When disabled, ACLK does not switch to REFO if XT1 fails or stops, so it stops as well.
    /// MCLK and SMCLK are not affected by this setting and always fall back.
    pub fn disable_fault_switch(mut self) -> Self {
        self.fault_switch = false;
        self
    }
}

impl Xt1Config<CrystalMode> {
    /// Configure XT1 for a low-frequency watch crystal on XIN and XOUT.
    ///
    /// - `frequency`: Crystal frequency in Hz. The data sheets specify 32768 Hz; up to 40 kHz is
    ///   accepted.
    /// - `_xin`, `_xout`: Pins connected to the crystal.
    ///
    /// The start counter is enabled by default in crystal mode because
    /// crystals require a stabilization period before producing a valid clock.
    /// The counter ensures the oscillator is given sufficient startup time
    /// before being considered stable.
    ///
    /// Automatic gain control is enabled, as it is after reset; see [`Self::disable_agc`].
    ///
    /// # Panics
    ///
    /// If `frequency` is 0 or above 40 kHz. With a constant, valid frequency the check is
    /// optimised away.
    pub fn crystal<XinDir, XoutDir>(
        frequency: u32,
        _xin: Xt1Xin<XinDir>,
        _xout: Xt1Xout<XoutDir>,
    ) -> Self {
        assert!(
            frequency > 0 && frequency <= XT1_LF_MAX_HZ,
            "XT1 low-frequency mode supports up to 40 kHz"
        );
        Self::crystal_config(frequency)
    }
}

#[cfg(feature = "xt1_high_frequency")]
impl Xt1Config<CrystalMode, HighFrequency> {
    /// Configure XT1 for a high-frequency crystal or resonator on XIN and XOUT.
    ///
    /// - `frequency`: Crystal frequency in Hz, from 1 MHz up to 24 MHz (16 MHz without the
    ///   enhanced clock system).
    /// - `_xin`, `_xout`: Pins connected to the crystal.
    ///
    /// The start counter and automatic gain control are enabled, as for [`Xt1Config::crystal`].
    ///
    /// # Panics
    ///
    /// If `frequency` is outside the range above. With a constant, valid frequency the check is
    /// optimised away.
    pub fn crystal_hf<XinDir, XoutDir>(
        frequency: u32,
        _xin: Xt1Xin<XinDir>,
        _xout: Xt1Xout<XoutDir>,
    ) -> Self {
        assert!(
            (XT1_HF_MIN_HZ..=XT1_HF_MAX_HZ).contains(&frequency),
            "XT1 high-frequency frequency out of range"
        );
        Self::crystal_config(frequency)
    }
}

impl<RANGE> Xt1Config<CrystalMode, RANGE> {
    #[inline(always)]
    fn crystal_config(frequency: u32) -> Self {
        Self {
            frequency,
            drive: Xt1Drive::Xt1drive3,
            agc: true,
            start_counter: true,
            bypass: false,
            #[cfg(feature = "enhanced_cs")]
            fault_switch: true,
            auto_off: true,
            _mode: PhantomData,
        }
    }

    /// Disable Automatic Gain Control (AGC).
    ///
    /// AGC is on by default: once the crystal has started it lowers the oscillation
    /// amplitude, which reduces current consumption. Disabling it keeps the full amplitude,
    /// which can help in electrically noisy environments at the cost of a higher current.
    pub fn disable_agc(mut self) -> Self {
        self.agc = false;
        self
    }

    /// Set oscillator drive strength.
    ///
    /// Higher drive strength may be required for higher-frequency crystals
    /// or specific load conditions.
    ///
    /// Note that startup and stabilization always run at the highest drive
    /// strength, as required by the user's guide; the level requested here is
    /// applied once the oscillator is stable.
    pub fn with_drive(mut self, drive: Xt1Drive) -> Self {
        self.drive = drive;
        self
    }

    /// Disable the startup counter.
    ///
    /// This skips the oscillator stabilization wait period. Only disable this
    /// if startup timing is externally managed or guaranteed, as doing so may
    /// result in using an unstable clock.
    pub fn disable_start_counter(mut self) -> Self {
        self.start_counter = false;
        self
    }

    /// Disable automatic crystal power-down.
    ///
    /// Setting this to false ensures XT1 remains active even if it is not
    /// currently requested by a system clock (ACLK, MCLK, SMCLK) or the FLL.
    pub fn disable_auto_off(mut self) -> Self {
        self.auto_off = false;
        self
    }
}

impl Xt1Config<BypassMode> {
    /// Configure XT1 in low-frequency bypass mode, with an external clock signal on XIN.
    ///
    /// In this mode, a digital clock signal is fed directly into XIN and the
    /// internal crystal oscillator circuitry is bypassed. XOUT is not used and
    /// remains available as a GPIO.
    ///
    /// - `frequency`: Input clock frequency in Hz. The data sheets specify 32.768 kHz with a
    ///   40 % to 60 % duty cycle; up to 40 kHz is accepted.
    /// - `_xin`: Pin receiving the external clock.
    ///
    /// The start counter is disabled by default because an external clock
    /// source is assumed to already be stable and does not require oscillator
    /// startup time.
    ///
    /// # Panics
    ///
    /// If `frequency` is 0 or above 40 kHz. With a constant, valid frequency the check is
    /// optimised away.
    pub fn bypass<DIR>(frequency: u32, _xin: Xt1Xin<DIR>) -> Self {
        assert!(
            frequency > 0 && frequency <= XT1_LF_MAX_HZ,
            "XT1 low-frequency mode supports up to 40 kHz"
        );
        Self::bypass_config(frequency)
    }
}

#[cfg(feature = "xt1_high_frequency")]
impl Xt1Config<BypassMode, HighFrequency> {
    /// Configure XT1 in high-frequency bypass mode, with an external clock signal on XIN.
    ///
    /// - `frequency`: Input clock frequency in Hz, from 1 MHz up to 24 MHz (16 MHz without the
    ///   enhanced clock system), with a 40 % to 60 % duty cycle.
    /// - `_xin`: Pin receiving the external clock.
    ///
    /// # Panics
    ///
    /// If `frequency` is outside the range above. With a constant, valid frequency the check is
    /// optimised away.
    pub fn bypass_hf<DIR>(frequency: u32, _xin: Xt1Xin<DIR>) -> Self {
        assert!(
            (XT1_HF_MIN_HZ..=XT1_HF_MAX_HZ).contains(&frequency),
            "XT1 high-frequency frequency out of range"
        );
        Self::bypass_config(frequency)
    }
}

impl<RANGE> Xt1Config<BypassMode, RANGE> {
    #[inline(always)]
    fn bypass_config(frequency: u32) -> Self {
        Self {
            frequency,
            drive: Xt1Drive::Xt1drive0, // Ignored in bypass mode
            agc: true,                  // Not applicable in bypass mode, left at its reset value
            start_counter: false,       // External clock assumed stable
            bypass: true,
            #[cfg(feature = "enhanced_cs")]
            fault_switch: true,
            auto_off: true,
            _mode: PhantomData,
        }
    }

    /// Enable the startup counter.
    ///
    /// This is typically unnecessary in bypass mode, but may be useful if the
    /// external clock source has a delayed or uncertain startup behavior and
    /// additional stabilization time is required.
    pub fn enable_start_counter(mut self) -> Self {
        self.start_counter = true;
        self
    }
}

impl<MODE, RANGE: Xt1Range> Xt1Config<MODE, RANGE> {
    /// The XTS mode bit and XT1HFFREQ range for the configured frequency (SLAU445I Table 3-10):
    /// 1 to 4 MHz, above 4 to 6 MHz, above 6 to 16 MHz, above 16 to 24 MHz.
    #[cfg(feature = "xt1_high_frequency")]
    #[inline]
    fn mode_bits(&self) -> (Xts, Xt1hffreq) {
        if !RANGE::HIGH_FREQUENCY {
            (Xts::Xts0, Xt1hffreq::Xt1hffreq0)
        } else if self.frequency <= 4_000_000 {
            (Xts::Xts1, Xt1hffreq::Xt1hffreq0)
        } else if self.frequency <= 6_000_000 {
            (Xts::Xts1, Xt1hffreq::Xt1hffreq1)
        } else if self.frequency <= 16_000_000 {
            (Xts::Xts1, Xt1hffreq::Xt1hffreq2)
        } else {
            (Xts::Xts1, Xt1hffreq::Xt1hffreq3)
        }
    }

    /// The ACLK divider (DIVA) for XT1 in high-frequency mode, as the register setting and
    /// the division factor.
    ///
    /// ACLK must not exceed 40 kHz. Of the dividers that keep it there, pick the one landing
    /// closest to 32.768 kHz, which is what peripherals clocked from ACLK usually expect. In
    /// low-frequency mode the hardware bypasses DIVA, so the choice does not matter there.
    #[cfg(feature = "xt1_high_frequency")]
    fn aclk_divider(&self) -> (Diva, u32) {
        const DIVIDERS: &[(Diva, u32)] = &[
            (Diva::_1, 1),
            (Diva::_16, 16),
            (Diva::_32, 32),
            (Diva::_64, 64),
            (Diva::_128, 128),
            (Diva::_256, 256),
            (Diva::_384, 384),
            (Diva::_512, 512),
        ];
        // Dividers that only exist on the enhanced clock system (SLAU445I Table 3-1)
        #[cfg(feature = "enhanced_cs")]
        const ENHANCED_DIVIDERS: &[(Diva, u32)] = &[
            (Diva::_108, 108),
            (Diva::_338, 338),
            (Diva::_414, 414),
            (Diva::_640, 640),
            (Diva::_768, 768),
            (Diva::_1024, 1024),
        ];
        #[cfg(not(feature = "enhanced_cs"))]
        const ENHANCED_DIVIDERS: &[(Diva, u32)] = &[];

        let mut best = (Diva::_1, 1);
        let mut best_error = u32::MAX;
        for &(diva, div) in DIVIDERS.iter().chain(ENHANCED_DIVIDERS) {
            let aclk = self.frequency / div;
            let error = aclk.abs_diff(REFOCLK_FREQ_HZ as u32);
            if aclk <= XT1_LF_MAX_HZ && error < best_error {
                best = (diva, div);
                best_error = error;
            }
        }
        best
    }

    /// ACLK frequency when ACLK is sourced from XT1
    #[inline]
    fn aclk_freq(&self) -> u32 {
        #[cfg(feature = "xt1_high_frequency")]
        if RANGE::HIGH_FREQUENCY {
            return self.frequency / self.aclk_divider().1;
        }
        self.frequency
    }

    /// The FLL reference XT1 provides: its frequency after FLLREFDIV, and the FLLREFDIV setting
    #[inline]
    fn fll_reference(&self) -> (u32, Fllrefdiv) {
        #[cfg(feature = "xt1_high_frequency")]
        if RANGE::HIGH_FREQUENCY {
            return xt1_hf_fll_ref_divider(self.frequency);
        }
        // A 32 kHz XT1 is used undivided. On devices whose XT1 only supports 32 kHz,
        // "FLLREFDIV always reads and should be written as zero" (SLAU445I 3.3.4).
        (self.frequency, FLLREFDIV_1)
    }

    /// Bring up the XT1 oscillator and wait until it has stabilized, giving up
    /// after roughly `timeout_ms` milliseconds if a timeout is given. Returns
    /// whether XT1 started.
    ///
    /// Stabilization deliberately runs with settings that differ from the
    /// user's configuration; [`Self::finalize`] applies the requested values
    /// once the system clocks have been switched over:
    ///
    /// - Drive strength is forced to the maximum. Per SLAU445I 3.2.4, XT1
    ///   "starts with the highest drive settings for fast reliable startup"
    ///   and only "after startup, user software can reduce the drive strength".
    /// - XT1 is kept requested, so the oscillator runs and its fault detection
    ///   is active even though no system clock or FLL reference has selected
    ///   it yet. Clearing XT1AUTOOFF does this in crystal mode, but not in
    ///   bypass mode (SLAU445I Table 3-10), so ACLK is pointed at XT1 as well.
    ///   The fail-safe keeps ACLK on REFO until XT1 is stable, and ACLK's final
    ///   source is selected afterwards.
    ///
    /// On a timeout ACLK and XT1AUTOOFF are restored, so XT1 switches off again
    /// once nothing requests it.
    fn start(&self, periph: &_pac::Cs, timeout_ms: Option<u16>) -> bool {
        // The start fault counter must be configured before the oscillator
        // starts. When enabled, the hardware holds the fault condition (and
        // XT1OFFG) asserted until XT1 has run cleanly for 1024 cycles in
        // low-frequency mode, crystal or bypass, or 4096 cycles for a
        // high-frequency crystal, which is what turns the fault-polling loop
        // below into a stabilization wait (SLAU445I 3.2.13). The cycle counts
        // come from the device data sheets, and 1024 was measured on the
        // FR2476; the 8192 and 1024 in SLAU445I 3.2.13 don't match the hardware.
        periph.csctl7().modify(|_, w| w.enstfcnt1().bit(self.start_counter));

        periph.csctl6().modify(|_, w| {
            let w = w
                .xt1bypass().bit(self.bypass)
                .xt1agcoff().bit(!self.agc)
                .xt1autooff().clear_bit()
                .xt1drive().variant(Xt1Drive::Xt1drive3);

            // DIVA is set before ACLK is pointed at XT1 below, so ACLK never
            // exceeds 40 kHz.
            #[cfg(feature = "xt1_high_frequency")]
            let w = {
                let (xts, hf_range) = self.mode_bits();
                w.xts().variant(xts)
                    .xt1hffreq().variant(hf_range)
                    .diva().variant(self.aclk_divider().0)
            };
            // Devices without high-frequency support run XT1 in
            // low-frequency mode only.
            #[cfg(not(feature = "xt1_high_frequency"))]
            let w = w.xts().clear_bit();

            #[cfg(feature = "enhanced_cs")]
            let w = w.xt1faultoff().bit(!self.fault_switch);

            w
        });

        let csctl4 = periph.csctl4().read().bits();
        periph.csctl4().modify(|_, w| w.sela().variant(Sela::Xt1clk));

        // Oscillator fault flags are sticky: they stay latched even after the
        // fault condition disappears, and re-assert if cleared while the fault
        // persists (SLAU445I 3.2.13). Clearing them and checking whether they
        // return is therefore the canonical way to wait for the oscillator:
        // this loop only exits once XT1 runs fault-free (for the full start
        // counter period, if enabled). Polling once per millisecond lets the
        // timeout be counted.
        let mut delay = SysDelay::new(RESET_MCLK_FREQ_HZ);
        let mut elapsed_ms: u16 = 0;
        loop {
            clear_osc_faults();
            if !osc_fault_pending() {
                return true;
            }
            if let Some(timeout_ms) = timeout_ms {
                if elapsed_ms >= timeout_ms {
                    break;
                }
                elapsed_ms += 1;
            }
            delay.delay_ms(1);
        }

        // XT1 did not start in time: put ACLK back and let XT1 switch off again
        periph.csctl4().write(|w| unsafe { w.bits(csctl4) });
        periph.csctl6().modify(|_, w| w.xt1autooff().set_bit());
        clear_osc_faults();
        false
    }

    /// Apply the user-requested drive strength and auto-off behavior.
    ///
    /// This runs *after* the system clocks have been switched over, so XT1
    /// stays continuously powered (auto-off was held disabled by
    /// [`Self::start`]) from stabilization through selection. No restart can
    /// occur in between, which matters because a latched fault flag freezes
    /// the fail-safe REFO fallback in place until software clears it
    /// (SLAU445I 3.2.13).
    fn finalize(&self, periph: &_pac::Cs) {
        periph.csctl6().modify(|_, w| {
            w.xt1drive().variant(self.drive)
                .xt1autooff().bit(self.auto_off)
        });
    }
}

/// Pick the FLL reference divider for a high-frequency XT1 so that the divided reference lands
/// in the stable ~23 kHz to ~47 kHz range. Returns the divided reference frequency along with
/// the divider setting.
#[cfg(feature = "xt1_high_frequency")]
#[inline]
fn xt1_hf_fll_ref_divider(freq: u32) -> (u32, Fllrefdiv) {
    // Each cutoff is the "handover" point between hardware dividers: at
    // 1.5 MHz, /32 gives 46.8 kHz and /64 gives 23.4 kHz, and so on.
    if freq <= 1_500_000 {
        (freq / 32, Fllrefdiv::_32)
    } else if freq <= 3_000_000 {
        (freq / 64, Fllrefdiv::_64)
    } else if freq <= 6_000_000 {
        (freq / 128, Fllrefdiv::_128)
    } else if freq <= 12_000_000 {
        (freq / 256, Fllrefdiv::_256)
    } else if freq <= 18_000_000 {
        (freq / 512, Fllrefdiv::_512)
    } else {
        // The /640 and /768 dividers only exist on the enhanced clock system
        // (SLAU445I 3.3.4); without them /512 is the closest available.
        #[cfg(feature = "enhanced_cs")]
        let res = if freq <= 22_000_000 {
            (freq / 640, Fllrefdiv::Fllrefdiv6)
        } else {
            (freq / 768, Fllrefdiv::Fllrefdiv7)
        };
        #[cfg(not(feature = "enhanced_cs"))]
        let res = (freq / 512, Fllrefdiv::_512);
        res
    }
}

/// Clear the XT1 and DCO fault flags, then OFIFG. Flags whose fault condition
/// persists are set again by the hardware straight away (SLAU445I 3.2.13).
#[inline]
fn clear_osc_faults() {
    let cs = unsafe { &*_pac::Cs::ptr() };
    let sfr = unsafe { &*_pac::Sfr::ptr() };
    unsafe {
        cs.csctl7().clear_bits(|w|
            w.xt1offg().clear_bit()
              .dcoffg().clear_bit()
        );
        sfr.sfrifg1().clear_bits(|w| w.ofifg().clear_bit());
    }
}

/// Whether an oscillator fault (XT1 or DCO) is latched in OFIFG
#[inline]
fn osc_fault_pending() -> bool {
    let sfr = unsafe { &*_pac::Sfr::ptr() };
    sfr.sfrifg1().read().ofifg().bit_is_set()
}

/// Whether the FLL reports the DCO as too fast, too slow or out of range
#[inline]
fn fll_unlocked(cs: &_pac::Cs) -> bool {
    !cs.csctl7().read().fllunlock().is_fllunlock_0()
}

// Using Xt1State as a trait bound outside the HAL will never be useful, since we only
// configure the clocks once, so just keep it hidden (same treatment as `SmclkState`).
#[doc(hidden)]
pub trait Xt1State {
    /// XT1 frequency in Hz, or `None` when XT1 is not configured
    fn freq(&self) -> Option<u32>;
    /// ACLK frequency in Hz when ACLK is sourced from XT1, or `None` when XT1 is not configured
    fn aclk_freq(&self) -> Option<u32>;
    /// The FLL reference frequency and FLLREFDIV setting XT1 provides, or `None` when XT1 is
    /// not configured
    fn fll_reference(&self) -> Option<(u32, Fllrefdiv)>;
    /// Bring up and stabilize XT1, giving up after `timeout_ms` if given. Returns whether XT1
    /// is running (always `true` when XT1 is not configured)
    fn start(&self, periph: &_pac::Cs, timeout_ms: Option<u16>) -> bool;
    /// Apply post-stabilization XT1 settings (no-op when XT1 is not configured)
    fn finalize(&self, periph: &_pac::Cs);
}

impl<MODE, RANGE: Xt1Range> Xt1State for Xt1Defined<MODE, RANGE> {
    #[inline(always)]
    fn freq(&self) -> Option<u32> {
        Some(self.0.frequency)
    }

    #[inline(always)]
    fn aclk_freq(&self) -> Option<u32> {
        Some(self.0.aclk_freq())
    }

    #[inline(always)]
    fn fll_reference(&self) -> Option<(u32, Fllrefdiv)> {
        Some(self.0.fll_reference())
    }

    #[inline(always)]
    fn start(&self, periph: &_pac::Cs, timeout_ms: Option<u16>) -> bool {
        self.0.start(periph, timeout_ms)
    }

    #[inline(always)]
    fn finalize(&self, periph: &_pac::Cs) {
        self.0.finalize(periph);
    }
}

impl Xt1State for Xt1Disabled {
    #[inline(always)]
    fn freq(&self) -> Option<u32> {
        None
    }

    #[inline(always)]
    fn aclk_freq(&self) -> Option<u32> {
        None
    }

    #[inline(always)]
    fn fll_reference(&self) -> Option<(u32, Fllrefdiv)> {
        None
    }

    #[inline(always)]
    fn start(&self, _periph: &_pac::Cs, _timeout_ms: Option<u16>) -> bool {
        true
    }

    #[inline(always)]
    fn finalize(&self, _periph: &_pac::Cs) {}
}


// Using SmclkState as a trait bound outside the HAL will never be useful, since we only configure
// the clock once, so just keep it hidden
#[doc(hidden)]
pub trait SmclkState {
    fn div(&self) -> Option<SmclkDiv>;
}

impl SmclkState for SmclkDefined {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> { Some(self.0) }
}

impl SmclkState for SmclkDisabled {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> { None }
}

/// Builder object that configures system clocks
///
/// Can only commit configurations to hardware if both MCLK and SMCLK settings have been
/// configured. ACLK configurations are optional, with its default source being REFOCLK.
pub struct ClockConfig<MCLK, SMCLK, XT1CLK> {
    periph: _pac::Cs,
    mclk: MCLK,
    mclk_div: MclkDiv,
    aclk_sel: AclkSel,
    smclk: SMCLK,
    xt1clk: XT1CLK,
    fll_ref: Selref,
    #[cfg(feature = "enhanced_cs")]
    refo_low_power: bool,
    fll_unlock_reset: bool,
}

macro_rules! make_clkconf {
    ($conf:expr, $mclk:expr, $smclk:expr, $xt1clk:expr, $fll_ref: expr) => {
        ClockConfig {
            periph: $conf.periph,
            mclk: $mclk,
            mclk_div: $conf.mclk_div,
            aclk_sel: $conf.aclk_sel,
            smclk: $smclk,
            xt1clk: $xt1clk,
            fll_ref: $fll_ref,
            #[cfg(feature = "enhanced_cs")]
            refo_low_power: $conf.refo_low_power,
            fll_unlock_reset: $conf.fll_unlock_reset,
        }
    };
}

impl ClockConfig<NoClockDefined, NoClockDefined, Xt1Disabled> {
    /// Converts CS into a fresh, unconfigured clock builder object
    pub fn new(cs: _pac::Cs) -> Self {
        ClockConfig {
            periph: cs,
            smclk: NoClockDefined,
            mclk: NoClockDefined,
            xt1clk: Xt1Disabled,
            mclk_div: MclkDiv::_1,
            aclk_sel: AclkSel::Refoclk,
            fll_ref: Selref::Refoclk,
            #[cfg(feature = "enhanced_cs")]
            refo_low_power: false,
            fll_unlock_reset: false,
        }
    }
}

impl<MCLK, SMCLK, XT1CLK> ClockConfig<MCLK, SMCLK, XT1CLK> {
    /// Reset the device with a PUC if the FLL finds the DCO running too fast (FLLULPUC, SLAU445I
    /// 3.2.9), so MCLK can't run faster than the FRAM wait states allow.
    /// [`Pmm::take_reset_cause()`](crate::pmm::Pmm::take_reset_cause) then returns
    /// [`ResetCause::FllUnlock`](crate::pmm::ResetCause::FllUnlock).
    ///
    /// Only applies when MCLK runs from the DCO. It takes effect once the FLL has locked, at the end
    /// of `freeze()`.
    #[inline]
    pub fn reset_on_fll_unlock(mut self) -> Self {
        self.fll_unlock_reset = true;
        self
    }

    /// Select REFOCLK for ACLK
    #[inline]
    pub fn aclk_refoclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Refoclk;
        self
    }

    #[cfg(feature = "vloclk_source")]
    /// Select VLOCLK for ACLK
    #[inline]
    pub fn aclk_vloclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Vloclk;
        self
    }

    /// Select REFOCLK for MCLK and set the MCLK divider. Frequency is `32_768 / mclk_div` Hz.
    #[inline]
    pub fn mclk_refoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Refoclk), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Select VLOCLK for MCLK and set the MCLK divider. Frequency is `10_000 / mclk_div` Hz.
    #[inline]
    pub fn mclk_vloclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Vloclk), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Select DCOCLK for MCLK with FLL for stabilization. Frequency is `target_freq / mclk_div` Hz.
    /// See [`DcoclkFreqSel`] for the frequencies, and [`mclk_dcoclk_hz`](Self::mclk_dcoclk_hz) for
    /// any other.
    ///
    /// With XT1 as the FLL reference (see `fll_ref_xt1`) the FLL locks to the multiple of the
    /// XT1 frequency closest to `target_freq` without exceeding it, and the returned clock
    /// objects report that frequency.
    #[inline]
    pub fn mclk_dcoclk(
        self,
        target_freq: DcoclkFreqSel,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Dcoclk(target_freq.into())), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Select DCOCLK for MCLK with FLL for stabilization, at any frequency from 1 MHz to 24 MHz
    /// (16 MHz without the enhanced clock system). MCLK runs at the locked frequency /
    /// `mclk_div`.
    ///
    /// The FLL locks DCOCLKDIV to the largest multiple of its reference that doesn't exceed
    /// `target_hz`: with REFO a multiple of 32.768 kHz, so 5 MHz becomes 4.980736 MHz. The
    /// returned clock objects report the locked frequency. The DCO runs in the range whose
    /// nominal frequency is closest to the target, trimmed in software while the clocks are
    /// configured; from the highest [`DcoclkFreqSel`] frequency up it locks like that one, with
    /// the factory trim.
    ///
    /// # Panics
    ///
    /// If `target_hz` is outside the range above. With a constant, valid frequency the check is
    /// optimised away.
    #[inline]
    pub fn mclk_dcoclk_hz(
        self,
        target_hz: u32,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
        assert!(
            (DCO_MIN_HZ..=MCLK_MAX_HZ).contains(&target_hz),
            "DCO frequency out of range"
        );
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Dcoclk(DcoTarget::from_hz(target_hz))), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Enable SMCLK and set SMCLK divider, which divides the MCLK frequency
    #[inline]
    pub fn smclk_on(self, div: SmclkDiv) -> ClockConfig<MCLK, SmclkDefined, XT1CLK> {
        make_clkconf!(self, self.mclk, SmclkDefined(div), self.xt1clk, self.fll_ref)
    }

    /// Disable SMCLK
    #[inline]
    pub fn smclk_off(self) -> ClockConfig<MCLK, SmclkDisabled, XT1CLK> {
        make_clkconf!(self, self.mclk, SmclkDisabled, self.xt1clk, self.fll_ref)
    }

    /// Enable XT1 with specific hardware requirements. Calling this again replaces the
    /// previous XT1 configuration.
    #[inline]
    pub fn xt1clk_on<MODE, RANGE>(
        self,
        config: Xt1Config<MODE, RANGE>
    ) -> ClockConfig<MCLK, SMCLK, Xt1Defined<MODE, RANGE>> {
        make_clkconf!(self, self.mclk, self.smclk, Xt1Defined(config), self.fll_ref)
    }

    /// Run REFO in its low-power mode, which draws about 1 µA instead of 15 µA (FR235x data
    /// sheet). Only the enhanced clock system has this mode (SLAU445I Table 3-1).
    ///
    /// The mode is switched off again when entering LPM3.5 or LPM4.5, where it would draw extra
    /// current.
    #[cfg(feature = "enhanced_cs")]
    #[inline]
    pub fn refo_low_power(mut self) -> Self {
        self.refo_low_power = true;
        self
    }
}

impl<SMCLK, XT1CLK> ClockConfig<NoClockDefined, SMCLK, XT1CLK> {
    /// Disable XT1 (the default), undoing [`ClockConfig::xt1clk_on`]. If ACLK or the FLL
    /// reference were sourced from XT1, they fall back to REFOCLK.
    #[inline]
    pub fn xt1clk_off(self) -> ClockConfig<NoClockDefined, SMCLK, Xt1Disabled> {
        ClockConfig {
            aclk_sel: self.aclk_sel.without_xt1(),
            ..make_clkconf!(self, self.mclk, self.smclk, Xt1Disabled, Selref::Refoclk)
        }
    }
}

impl<SMCLK, XT1CLK> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
    /// Disable XT1 (the default), undoing [`ClockConfig::xt1clk_on`]. Every clock sourced
    /// from XT1 falls back to REFOCLK: MCLK (keeping its divider), ACLK and the FLL reference.
    ///
    /// Use this to fall back to the internal oscillators when `try_freeze` timed out.
    #[inline]
    pub fn xt1clk_off(self) -> ClockConfig<MclkDefined, SMCLK, Xt1Disabled> {
        let mclk = match self.mclk.0 {
            MclkSel::Xt1clk => MclkSel::Refoclk,
            sel => sel,
        };
        ClockConfig {
            aclk_sel: self.aclk_sel.without_xt1(),
            ..make_clkconf!(self, MclkDefined(mclk), self.smclk, Xt1Disabled, Selref::Refoclk)
        }
    }
}

impl<MCLK, SMCLK, MODE, RANGE> ClockConfig<MCLK, SMCLK, Xt1Defined<MODE, RANGE>> {
    /// Select XT1CLK for ACLK.
    ///
    /// ACLK must not exceed 40 kHz, so a high-frequency XT1 is divided down for ACLK, using
    /// the divider that lands closest to 32.768 kHz. [`Aclk`] reports the divided frequency.
    #[inline]
    pub fn aclk_xt1clk(mut self) -> Self {
        self.aclk_sel = AclkSel::Xt1clk;
        self
    }

    /// Select XT1CLK for MCLK and set the MCLK divider. Frequency is `xt1_freq / mclk_div` Hz.
    #[inline]
    pub fn mclk_xt1clk(
        self,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK, Xt1Defined<MODE, RANGE>> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Xt1clk), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Reference the FLL to XT1CLK instead of REFOCLK, when MCLK is sourced from the DCO.
    ///
    /// If a low-frequency XT1 fails, the FLL falls back to REFO. A high-frequency XT1 has no
    /// such fallback: the DCO drops to its lowest tap instead (SLAU445I 3.2.13).
    #[inline]
    pub fn fll_ref_xt1(mut self) -> Self  {
        self.fll_ref = Selref::Xt1clk;
        self
    }
}

#[inline(always)]
fn fll_off() {
    // 64 = 1 << 6, which is the 6th bit of SR
    unsafe { asm!("bis.b #64, SR", options(nomem, nostack)) };
}

#[inline(always)]
fn fll_on() {
    // 64 = 1 << 6, which is the 6th bit of SR
    unsafe { asm!("bic.b #64, SR", options(nomem, nostack)) };
}

/// FLL settings for a DCOCLKDIV target
struct FllSettings {
    selref: Selref,
    ref_div: Fllrefdiv,
    /// FLLN register value: DCOCLKDIV = (FLLN + 1) x reference / FLLREFDIV
    flln: u16,
    /// The frequency the FLL locks DCOCLKDIV to
    freq: u32,
}

/// Set the FRAM wait states MCLK needs at `mclk_freq`: one per 8 MHz above the first 8 MHz
/// (device data sheets, recommended operating conditions)
#[inline]
unsafe fn configure_fram(fram: &mut Fram, mclk_freq: u32) {
    let wait_states = match mclk_freq.saturating_sub(1) / FRAM_NO_WAIT_MAX_HZ {
        0 => WaitStates::Wait0,
        1 => WaitStates::Wait1,
        2 => WaitStates::Wait2,
        3 => WaitStates::Wait3,
        4 => WaitStates::Wait4,
        5 => WaitStates::Wait5,
        6 => WaitStates::Wait6,
        _ => WaitStates::Wait7,
    };
    fram.set_wait_states(wait_states);
}

impl<SMCLK: SmclkState, XT1CLK: Xt1State> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
    /// FLL settings that lock DCOCLKDIV as close to `target` as possible without exceeding it
    #[inline]
    fn fll_settings(&self, target: DcoTarget) -> FllSettings {
        // The FLL is referenced by XT1CLK only if XT1 has actually been
        // configured; in every other case it is referenced by REFOCLK.
        // The typestate API already guarantees `fll_ref` can only be
        // XT1CLK while XT1 is defined, but resolving the pair here keeps
        // the hardware configuration consistent by construction.
        let (selref, ref_freq, ref_div) = match (self.fll_ref, self.xt1clk.fll_reference()) {
            (Selref::Xt1clk, Some((ref_freq, ref_div))) => (Selref::Xt1clk, ref_freq, ref_div),
            _ => (Selref::Refoclk, REFOCLK_FREQ_HZ as u32, FLLREFDIV_1),
        };

        // FLLN is 10 bits wide, so the multiplier (FLLN + 1) only reaches 1024.
        // A reference too slow for the target (16 MHz from a 10 kHz XT1, say)
        // would overflow it, so clamp the multiplier and report the frequency
        // the FLL actually locks to.
        let multiplier = (target.freq / ref_freq).clamp(1, 1024);

        FllSettings {
            selref,
            ref_div,
            flln: (multiplier - 1) as u16,
            freq: multiplier * ref_freq,
        }
    }

    /// MCLK frequency, after the MCLK divider
    #[inline]
    fn mclk_freq(&self) -> u32 {
        let source_freq = match self.mclk.0 {
            MclkSel::Refoclk => REFOCLK_FREQ_HZ as u32,
            MclkSel::Vloclk => VLOCLK_FREQ_HZ as u32,
            MclkSel::Dcoclk(target) => self.fll_settings(target).freq,
            // `xt1clk_off` moves MCLK back to REFO, so XT1 is configured
            // whenever it is selected here
            MclkSel::Xt1clk => self.xt1clk.freq().unwrap_or(REFOCLK_FREQ_HZ as u32),
        };
        source_freq >> (self.mclk_div as u32)
    }

    /// The fastest MCLK may run while the clocks are configured. MCLK runs undivided from the
    /// DCO while the DCO is set up, before the MCLK divider takes effect. A DCO started by the
    /// factory trim procedure rises from its lowest tap and doesn't overshoot, but the software
    /// trim tries other trim settings, which run at up to about 2.2 times the nominal frequency
    /// of the range (device data sheets, DCO frequency). The bound is 2.25 times the nominal
    /// frequency or the target, whichever is higher.
    #[inline]
    fn mclk_freq_during_config(&self) -> u32 {
        let dco_freq = match self.mclk.0 {
            MclkSel::Dcoclk(target) if target.factory_trim => self.fll_settings(target).freq,
            MclkSel::Dcoclk(target) => target.freq.max(target.range_freq()) / 4 * 9,
            _ => 0,
        };
        dco_freq.max(self.mclk_freq())
    }

    /// ACLK frequency
    #[inline]
    fn aclk_freq(&self) -> u32 {
        match self.aclk_sel {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => VLOCLK_FREQ_HZ as u32,
            AclkSel::Refoclk => REFOCLK_FREQ_HZ as u32,
            // `xt1clk_off` moves ACLK back to REFO, so XT1 is configured
            // whenever it is selected here
            AclkSel::Xt1clk => self.xt1clk.aclk_freq().unwrap_or(REFOCLK_FREQ_HZ as u32),
        }
    }

    #[inline]
    fn configure_dco_fll(&self) {
        // If MCLK runs from the DCO, run the FLL configuration procedure of the user's guide: the
        // software trim procedure (SLAU445I 3.2.11.2), or the factory trim procedure (3.2.11.1) for
        // the highest frequency. The step numbers are those of the software trim procedure.
        if let MclkSel::Dcoclk(target) = self.mclk.0 {
            let fll = self.fll_settings(target);
            let cs = &self.periph;

            // 1. Disable the FLL
            fll_off();

            // 2. Select the reference clock
            cs.csctl3()
                .write(|w| w.selref().variant(fll.selref).fllrefdiv().variant(fll.ref_div));

            // 3. Set the DCO range, and for the software trim enable the trim, at its middle
            //    setting with modulation enabled, as TI's routine does. The DCO starts from its
            //    lowest tap, as the factory trim procedure requires.
            cs.csctl0().write(|w| unsafe { w.bits(0) });
            if target.factory_trim {
                cs.csctl1().write(|w| w.dcorsel().variant(target.range));
            } else {
                let dcorsel = target.range as u16;
                cs.csctl1().write(|w| unsafe {
                    w.bits(DCOFTRIMEN | DCOFTRIM_START << DCOFTRIM_SHIFT | dcorsel << DCORSEL_SHIFT)
                });
            }

            // 4. Set FLLN and FLLD for the target frequency
            cs.csctl2().write(|w| {
                unsafe { w.flln().bits(fll.flln) }
                    .flld()
                    ._1()
            });

            // 5. Three NOPs, for the settings to be applied
            msp430::asm::nop();
            msp430::asm::nop();
            msp430::asm::nop();

            // 6. Enable the FLL
            fll_on();

            if target.factory_trim {
                // Factory trim procedure, step 7: wait for lock
                while fll_unlocked(cs) {}
            } else {
                // Steps 7 to 15
                self.trim_dco(&fll);
            }
        }
    }

    /// Steps 7 to 15 of the DCO software trim procedure (SLAU445I 3.2.11.2), with the FLL
    /// running: find the DCOFTRIM setting whose locked DCO tap is closest to the middle of the tap
    /// range, so the FLL keeps lock over temperature, then lock with it.
    fn trim_dco(&self, fll: &FllSettings) {
        let cs = &self.periph;
        // Step 9 waits until the lock status (FLLUNLOCK) is valid for the new tap: at least 24
        // FLL reference clock cycles. Wait four times that, about the 3 ms TI's routine waits with
        // REFO. MCLK runs from the DCO meanwhile, which makes FLLN + 1 cycles per reference cycle
        // at the target frequency, and less than four times as many before it locks (device data
        // sheets, DCO frequency).
        let lock_status_wait_cycles = 4 * 24 * (fll.flln as u32 + 1);

        let mut best_csctl0 = 0;
        let mut best_csctl1 = 0;
        let mut best_delta = u16::MAX;
        let mut prev_tap: Option<u16> = None;
        loop {
            // 7. Set the DCO tap to the middle of its range
            cs.csctl0().write(|w| unsafe { w.bits(DCO_TAP_MID) });
            // 8. Clear DCOFFG, until it reads back clear as TI's routine does. Right after the FLL
            //    is enabled it takes several writes (measured on an MSP430FR2476), and a flag left
            //    set would end step 10 at once, recording a tap that hasn't settled.
            loop {
                unsafe { cs.csctl7().clear_bits(|w| w.dcoffg().clear_bit()) };
                if cs.csctl7().read().dcoffg().bit_is_clear() {
                    break;
                }
            }
            // 9. Wait for the lock status to be valid for the new tap
            delay_cycles(lock_status_wait_cycles);
            // 10. Wait for lock, or for the tap to run into either end of its range (DCOFFG)
            while fll_unlocked(cs) && cs.csctl7().read().dcoffg().bit_is_clear() {}

            // 11. Read the tap, and how far it is from the middle
            let csctl0 = cs.csctl0().read().bits();
            let csctl1 = cs.csctl1().read().bits();
            let tap = csctl0 & DCO_TAP_MASK;
            let delta = tap.abs_diff(DCO_TAP_MID);
            // 12. Record the registers if this tap is the closest to the middle so far
            if delta < best_delta {
                best_csctl0 = csctl0;
                best_csctl1 = csctl1;
                best_delta = delta;
            }

            // 13. A tap below the middle means the DCO runs fast with this trim, so lower the
            //     trim; a tap above it means it runs slow, so raise it.
            // 14. Repeat until the tap has crossed the middle between two adjacent trims, or the
            //     trim runs out of range.
            let below_mid = tap < DCO_TAP_MID;
            let crossed = prev_tap.is_some_and(|prev| (prev < DCO_TAP_MID) != below_mid);
            let trim = (csctl1 & DCOFTRIM_MASK) >> DCOFTRIM_SHIFT;
            let next_trim = if below_mid {
                trim.checked_sub(1)
            } else {
                Some(trim + 1).filter(|&trim| trim <= DCOFTRIM_MAX)
            };
            match next_trim {
                Some(next_trim) if !crossed => {
                    cs.csctl1().write(|w| unsafe {
                        w.bits(csctl1 & !DCOFTRIM_MASK | next_trim << DCOFTRIM_SHIFT)
                    });
                    prev_tap = Some(tap);
                }
                _ => break,
            }
        }

        // 15. Reload the recorded registers, and let the FLL lock with them
        cs.csctl0().write(|w| unsafe { w.bits(best_csctl0) });
        cs.csctl1().write(|w| unsafe { w.bits(best_csctl1) });
        while fll_unlocked(cs) {}
    }

    #[inline]
    fn configure_cs(&self) {
        // Configure clock selector and divisors
        self.periph.csctl4().write(|w| w
                .sela().variant(self.aclk_sel.sela())
                .selms().variant(self.mclk.0.selms()));

        self.periph.csctl5().write(|w| {
            let w = w.vloautooff().set_bit().divm().variant(self.mclk_div);
            match self.smclk.div() {
                Some(div) => w.divs().variant(div),
                None => w.smclkoff().set_bit(),
            }
        });
    }

    /// Switch REFO to its low-power mode, if requested
    #[cfg(feature = "enhanced_cs")]
    fn configure_refo(&self) {
        if !self.refo_low_power {
            return;
        }
        let cs = &self.periph;
        unsafe { cs.csctl3().set_bits(|w| w.refolp().set_bit()) };

        let fll_uses_refo = match self.mclk.0 {
            MclkSel::Dcoclk(target) => matches!(self.fll_settings(target).selref, Selref::Refoclk),
            _ => false,
        };
        let refo_used = matches!(self.mclk.0, MclkSel::Refoclk)
            || matches!(self.aclk_sel, AclkSel::Refoclk)
            || fll_uses_refo;
        // The low-power mode is only valid once REFOREADY is set (SLAU445I Table 3-7). REFO only
        // runs while something uses it, so there's only something to wait for in that case.
        if refo_used {
            while cs.csctl7().read().refoready().bit_is_clear() {}
        }
        // Let an FLL referenced to REFO lock to the low-power REFO
        if fll_uses_refo {
            while fll_unlocked(cs) {}
        }
    }

    /// Commit the configuration to hardware, in the order required by the
    /// user's guide:
    ///
    /// 1. FRAM wait states for the fastest MCLK during configuration (must be
    ///    set *before* MCLK exceeds 8 MHz)
    /// 2. XT1 bring-up and stabilization (must be stable *before* it can serve
    ///    as FLL reference or system clock source)
    /// 3. DCO and FLL configuration (waits for FLL lock)
    /// 4. Clock source selection and dividers
    /// 5. XT1 post-stabilization settings (user drive strength and auto-off,
    ///    applied only after the switch-over so the oscillator never restarts
    ///    in between)
    /// 6. FRAM wait states for the final MCLK
    ///
    /// Returns `false`, before switching any clock, if XT1 does not start
    /// within `xt1_timeout_ms`.
    #[inline]
    fn freeze_internal(&self, fram: &mut Fram, xt1_timeout_ms: Option<u16>) -> bool {
        unsafe { configure_fram(fram, self.mclk_freq_during_config()) };
        if !self.xt1clk.start(&self.periph, xt1_timeout_ms) {
            return false;
        }
        self.configure_dco_fll();
        self.configure_cs();
        #[cfg(feature = "enhanced_cs")]
        self.configure_refo();
        self.xt1clk.finalize(&self.periph);
        // Leave the oscillator fault flags clean, including a DCOFFG raised while trimming the
        // DCO. A single clear pass leaves a healthy system fault-free; if a genuine fault
        // remains the hardware simply re-asserts the flags for the user to observe. No loop
        // here: this must never hang post-configuration.
        clear_osc_faults();
        unsafe { configure_fram(fram, self.mclk_freq()) };
        if self.fll_unlock_reset && matches!(self.mclk.0, MclkSel::Dcoclk(_)) {
            let cs = &self.periph;
            // Clear a flag left from the DCO configuration first, or it resets the device right away
            unsafe { cs.csctl7().clear_bits(|w| w.fllulifg().clear_bit()) };
            unsafe { cs.csctl7().set_bits(|w| w.fllulpuc().set_bit()) };
        }
        true
    }
}

impl<MODE, RANGE: Xt1Range> ClockConfig<MclkDefined, SmclkDefined, Xt1Defined<MODE, RANGE>> {
    /// Apply clock configuration to hardware and return SMCLK, ACLK and XT1CLK clock objects.
    /// Also returns delay provider.
    ///
    /// Blocks until XT1 is stable, which is forever if it never starts (a missing crystal,
    /// say). `try_freeze` gives up after a timeout instead.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, Xt1clk<RANGE>, SysDelay) {
        // Without a timeout this only returns once XT1 is running
        self.freeze_internal(fram, None);
        self.clocks()
    }

    /// Like `freeze`, but give up if XT1 is not stable after roughly `timeout_ms` milliseconds.
    ///
    /// On a timeout no clock is switched and the configuration is handed back, so a fallback
    /// can be frozen instead, for example with `xt1clk_off()`.
    #[inline]
    pub fn try_freeze(
        self,
        fram: &mut Fram,
        timeout_ms: u16,
    ) -> Result<(Smclk, Aclk, Xt1clk<RANGE>, SysDelay), Self> {
        if self.freeze_internal(fram, Some(timeout_ms)) {
            Ok(self.clocks())
        } else {
            Err(self)
        }
    }

    #[inline]
    fn clocks(&self) -> (Smclk, Aclk, Xt1clk<RANGE>, SysDelay) {
        let mclk_freq = self.mclk_freq();
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_freq()),
            Xt1clk(self.xt1clk.0.frequency, PhantomData),
            SysDelay::new(mclk_freq),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDefined, Xt1Disabled> {
    /// Apply clock configuration to hardware and return SMCLK and ACLK clock objects.
    /// Also returns delay provider
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, SysDelay) {
        // Nothing to wait for without XT1
        self.freeze_internal(fram, None);
        let mclk_freq = self.mclk_freq();
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_freq()),
            SysDelay::new(mclk_freq),
        )
    }
}

impl<MODE, RANGE: Xt1Range> ClockConfig<MclkDefined, SmclkDisabled, Xt1Defined<MODE, RANGE>> {
    /// Apply clock configuration to hardware and return ACLK and XT1CLK clock objects, as SMCLK
    /// is disabled. Also returns delay provider.
    ///
    /// Blocks until XT1 is stable, which is forever if it never starts (a missing crystal,
    /// say). `try_freeze` gives up after a timeout instead.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, Xt1clk<RANGE>, SysDelay) {
        // Without a timeout this only returns once XT1 is running
        self.freeze_internal(fram, None);
        self.clocks()
    }

    /// Like `freeze`, but give up if XT1 is not stable after roughly `timeout_ms` milliseconds.
    ///
    /// On a timeout no clock is switched and the configuration is handed back, so a fallback
    /// can be frozen instead, for example with `xt1clk_off()`.
    #[inline]
    pub fn try_freeze(
        self,
        fram: &mut Fram,
        timeout_ms: u16,
    ) -> Result<(Aclk, Xt1clk<RANGE>, SysDelay), Self> {
        if self.freeze_internal(fram, Some(timeout_ms)) {
            Ok(self.clocks())
        } else {
            Err(self)
        }
    }

    #[inline]
    fn clocks(&self) -> (Aclk, Xt1clk<RANGE>, SysDelay) {
        (
            Aclk(self.aclk_freq()),
            Xt1clk(self.xt1clk.0.frequency, PhantomData),
            SysDelay::new(self.mclk_freq()),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDisabled, Xt1Disabled> {
    /// Apply clock configuration to hardware and return ACLK clock object, as SMCLK is disabled.
    /// Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, SysDelay) {
        // Nothing to wait for without XT1
        self.freeze_internal(fram, None);
        (Aclk(self.aclk_freq()), SysDelay::new(self.mclk_freq()))
    }
}

/// SMCLK clock object
pub struct Smclk(u32);
/// ACLK clock object
pub struct Aclk(u32);
/// XT1CLK clock object, for XT1 in low-frequency mode unless `RANGE` says otherwise
pub struct Xt1clk<RANGE = LowFrequency>(u32, PhantomData<RANGE>);

impl<RANGE> Xt1clk<RANGE> {
    /// Whether XT1 has faulted since the fault flags were last cleared.
    ///
    /// The flag is sticky: it stays set after the fault has gone. Until the flags are cleared,
    /// the clocks sourced from XT1 keep running from their fail-safe fallback (REFOCLK, or
    /// DCOCLKDIV for MCLK and SMCLK with a high-frequency XT1). Call [`Xt1clk::clear_fault`]
    /// first to sample the current state instead.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        let cs = unsafe { &*_pac::Cs::ptr() };
        cs.csctl7().read().xt1offg().bit_is_set()
    }

    /// Clear the oscillator fault flags (XT1OFFG, DCOFFG and OFIFG).
    ///
    /// If XT1 is healthy again, the clocks sourced from it switch back from their fail-safe
    /// fallback. If the fault persists, the hardware sets the flags again straight away. With
    /// the start counter enabled, XT1 has to run cleanly for 1024 cycles (4096 for a
    /// high-frequency crystal) after a fault before the flag stays clear; without it the flag
    /// stays clear as soon as the fault is gone.
    #[inline]
    pub fn clear_fault(&mut self) {
        clear_osc_faults();
    }

    /// Request an interrupt when an oscillator fault occurs, on top of the fail-safe switch.
    ///
    /// The interrupt is the user NMI: write an `UNMI` interrupt handler that calls
    /// [`take_fault_interrupt`]. It covers every oscillator fault, so a DCO fault requests it as
    /// well. Being non-maskable, it is requested even while interrupts are disabled, so the
    /// handler must not share data with the rest of the program through a critical section; use
    /// atomics instead, such as those of the `msp430-atomic` crate. A fault that is still flagged
    /// requests it as soon as it is enabled, so clear the fault first.
    #[inline]
    pub fn enable_fault_interrupt(&mut self) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };
        unsafe { sfr.sfrie1().set_bits(|w| w.ofie().set_bit()) };
    }

    /// Stop requesting an interrupt when an oscillator fault occurs
    #[inline]
    pub fn disable_fault_interrupt(&mut self) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };
        unsafe { sfr.sfrie1().clear_bits(|w| w.ofie().clear_bit()) };
    }
}

/// For the `UNMI` interrupt handler: whether an oscillator fault requested the interrupt, see
/// [`Xt1clk::enable_fault_interrupt`].
///
/// If so, this also disables the fault interrupt. The fault flags stay set until they're
/// cleared, and they can only be cleared once the fault is gone, so the interrupt would
/// otherwise be requested again straight away. Once [`Xt1clk::clear_fault`] shows that the
/// fault is gone, [`Xt1clk::enable_fault_interrupt`] turns it back on.
#[inline]
pub fn take_fault_interrupt() -> bool {
    let sfr = unsafe { &*_pac::Sfr::ptr() };
    let requested = sfr.sfrie1().read().ofie().bit_is_set() && osc_fault_pending();
    if requested {
        unsafe { sfr.sfrie1().clear_bits(|w| w.ofie().clear_bit()) };
    }
    requested
}

/// Trait for configured clock objects
pub trait Clock {
    /// Returning a 32-bit frequency may seem suspect, since we're on a 16-bit system, but it is
    /// required as SMCLK can go up to 24 MHz. Clock frequencies are usually for initialization
    /// tasks such as computing baud rates, which should be optimized away, avoiding the extra cost
    /// of 32-bit computations.
    fn freq(&self) -> u32;
}

impl Clock for Smclk {
    #[inline]
    fn freq(&self) -> u32 { self.0 }
}

impl Clock for Aclk {
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}

impl<RANGE> Clock for Xt1clk<RANGE> {
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}
