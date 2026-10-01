//! Clock system for configuration of MCLK, SMCLK, ACLK, and XT1.
//!
//! Once configuration is complete, `Aclk`, `Smclk`, and optionally `Xt1clk` clock objects
//! are returned. These objects are used to set the clock sources on other peripherals.
//!
//! Configuration of MCLK and SMCLK *must* occur, though SMCLK can be disabled. XT1
//! configuration is optional but, when enabled, provides a high-precision source for
//! system clocks or the FLL reference.
//!
//! DCO with FLL is supported on MCLK for select frequencies. The FLL can be
//! referenced by either the internal REFO or the external XT1 crystal. Supporting
//! arbitrary frequencies on the DCO requires complex calibration routines not
//! supported by the HAL.

use core::arch::asm;
use core::marker::PhantomData;

pub use crate::_pac::cs::csctl5::{Divm as MclkDiv, Divs as SmclkDiv};
use crate::_pac::{
    self,
    cs::{
        csctl1::Dcorsel,
        csctl4::{Sela, Selms},
    },
};
use crate::delay::SysDelay;
use crate::fram::{Fram, WaitStates};
use crate::_pac::{
    self,
    cs::{
        csctl1::Dcorsel,
        csctl4::{Sela, Selms},
        csctl3::{Selref, Fllrefdiv},
        csctl6::Xt1drive,
    },
};
pub use crate::_pac::cs::csctl5::{Divm as MclkDiv, Divs as SmclkDiv};

#[cfg(feature = "xt1_high_frequency")]
use crate::_pac::cs::csctl6::{Xt1hffreq, Xts};

/// REFOCLK frequency
pub const REFOCLK_FREQ_HZ: u16 = 32768;
/// VLOCLK frequency
pub const VLOCLK_FREQ_HZ: u16 = 10000;
pub use crate::device_specific::MODCLK_FREQ_HZ;

// The "divide by 1" FLLREFDIV encoding. The FR2433 PAC names the `Fllrefdiv`
// variants differently from the other PACs, so give it a common name here.
#[cfg(not(feature = "msp430fr2433"))]
const FLLREFDIV_1: Fllrefdiv = Fllrefdiv::_1;
#[cfg(feature = "msp430fr2433")]
const FLLREFDIV_1: Fllrefdiv = Fllrefdiv::Fllrefdiv0;

enum MclkSel {
    Refoclk,
    Vloclk,
    Dcoclk(DcoclkFreqSel),
    Xt1clk(u32),
}

impl MclkSel {
    #[inline]
    fn freq(&self) -> u32 {
        match self {
            MclkSel::Vloclk => VLOCLK_FREQ_HZ as u32,
            MclkSel::Refoclk => REFOCLK_FREQ_HZ as u32,
            MclkSel::Dcoclk(sel) => sel.freq(),
            MclkSel::Xt1clk(freq) => *freq,
        }
    }

    #[inline(always)]
    fn selms(&self) -> Selms {
        match self {
            MclkSel::Vloclk => Selms::Vloclk,
            MclkSel::Refoclk => Selms::Refoclk,
            MclkSel::Dcoclk(_) => Selms::Dcoclkdiv,
            MclkSel::Xt1clk(_) => Selms::Xt1clk,
        }
    }
}

#[derive(Clone, Copy)]
enum AclkSel {
    #[cfg(feature = "vloclk_source")]
    Vloclk,
    Refoclk,
    Xt1clk(u32),
}

impl AclkSel {
    #[inline(always)]
    fn sela(self) -> Sela {
        match self {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => Sela::Vloclk,
            AclkSel::Refoclk => Sela::Refoclk,
            AclkSel::Xt1clk(_) => Sela::Xt1clk,
        }
    }

    #[inline(always)]
    fn freq(self) -> u32 {
        match self {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => VLOCLK_FREQ_HZ as u32,
            AclkSel::Refoclk => REFOCLK_FREQ_HZ as u32,
            AclkSel::Xt1clk(freq) => freq,
        }
    }
}

/// Selectable DCOCLK frequencies when using factory trim settings.
/// Actual frequencies may be slightly higher.
#[derive(Clone, Copy)]
pub enum DcoclkFreqSel {
    /// 1 MHz
    _1MHz,
    /// 2 MHz
    _2MHz,
    /// 4 MHz
    _4MHz,
    /// 8 MHz
    _8MHz,
    /// 12 MHz
    _12MHz,
    /// 16 MHz
    _16MHz,
    #[cfg(feature = "enhanced_cs")]
    /// 20 MHz
    _20MHz,
    #[cfg(feature = "enhanced_cs")]
    /// 24 MHz
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

    #[inline(always)]
    fn multiplier(self) -> u16 {
        match self {
            DcoclkFreqSel::_1MHz => 32,
            DcoclkFreqSel::_2MHz => 61,
            DcoclkFreqSel::_4MHz => 122,
            DcoclkFreqSel::_8MHz => 245,
            DcoclkFreqSel::_12MHz => 366,
            DcoclkFreqSel::_16MHz => 490,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_20MHz => 610,
            #[cfg(feature = "enhanced_cs")]
            DcoclkFreqSel::_24MHz => 732,
        }
    }

    /// Numerical frequency
    #[inline]
    pub fn freq(self) -> u32 {
        (self.multiplier() as u32) * (REFOCLK_FREQ_HZ as u32)
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
pub struct Xt1Defined<MODE>(Xt1Config<MODE>);
/// Typestate for `ClockConfig` that represents disabled/unconfigured XT1CLK
pub struct Xt1Disabled;


/// Valid XT1 XIN (input) pin
pub trait Xt1XinPin {}

/// Valid XT1 XOUT (output) pin
pub trait Xt1XoutPin {}


/// Typestate marker for XT1 **crystal mode** (external crystal, oscillator enabled).
pub struct CrystalMode;

/// Typestate marker for XT1 **bypass mode** (external clock input, oscillator disabled).
pub struct BypassMode;

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
/// The configuration includes:
/// - The input frequency (used for timing calculations and routing decisions)
/// - Drive strength for the oscillator (relevant in crystal mode)
/// - Whether bypass mode is enabled
/// - Automatic Gain Control (AGC) behavior
pub struct Xt1Config<MODE> {
    frequency: u32,
    drive: Xt1drive,
    agc: bool,
    start_counter: bool,
    bypass: bool,
    #[cfg(feature = "enhanced_cs")]
    fault_switch: bool,
    auto_off: bool,
    _mode: PhantomData<MODE>,
}

#[cfg(feature = "enhanced_cs")]
impl<MODE> Xt1Config<MODE> {
    /// Disable the automatic clock fallback on XT1 fault.
    ///
    /// When disabled, the system will not automatically switch to a fallback
    /// oscillator (like REFO) if the XT1 crystal fails or stops.
    pub fn disable_fault_switch(mut self) -> Self {
        self.fault_switch = false;
        self
    }
}

impl Xt1Config<CrystalMode> {
    /// Configure XT1 as a crystal oscillator.
    ///
    /// This mode expects a crystal connected to XIN/XOUT and enables
    /// internal oscillator circuitry.
    ///
    /// - `frequency`: Target crystal frequency in Hz. Devices without
    ///   high-frequency XT1 support only accept low-frequency watch crystals
    ///   (32768 Hz typical, 40 kHz max). Devices with high-frequency support
    ///   additionally accept crystals from 1 MHz up to 24 MHz.
    /// - `_xin`, `_xout`: Pins connected to the crystal.
    ///
    /// The start counter is enabled by default in crystal mode because
    /// crystals require a stabilization period before producing a valid clock.
    /// The counter ensures the oscillator is given sufficient startup time
    /// before being considered stable.
    pub fn crystal<XIN, XOUT>(frequency: u32, _xin: XIN, _xout: XOUT) -> Self
    where
        XIN: Xt1XinPin,
        XOUT: Xt1XoutPin,
    {
        Self {
            frequency,
            drive: Xt1drive::Xt1drive3,
            agc: false,
            start_counter: true,
            bypass: false,
            #[cfg(feature = "enhanced_cs")]
            fault_switch: true,
            auto_off: true,
            _mode: PhantomData,
        }
    }

    /// Enable Automatic Gain Control (AGC).
    ///
    /// AGC is only applicable in crystal mode and helps stabilize oscillation
    /// amplitude across varying conditions.
    pub fn enable_agc(mut self) -> Self {
        self.agc = true;
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
    pub fn with_drive(mut self, drive: Xt1drive) -> Self {
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
    /// Configure XT1 in bypass mode using an external clock source.
    ///
    /// In this mode, a digital clock signal is fed directly into XIN and the
    /// internal crystal oscillator circuitry is bypassed.
    ///
    /// - `frequency`: Input clock frequency in Hz. Devices without
    ///   high-frequency XT1 support only accept low-frequency inputs
    ///   (40 kHz max). Devices with high-frequency support additionally
    ///   accept inputs from 1 MHz up to 24 MHz.
    /// - `_xin`: Pin receiving the external clock.
    ///
    /// The start counter is disabled by default because an external clock
    /// source is assumed to already be stable and does not require oscillator
    /// startup time.
    pub fn bypass<XIN>(frequency: u32, _xin: XIN) -> Self
    where
        XIN: Xt1XinPin,
    {
        Self {
            frequency,
            drive: Xt1drive::Xt1drive0, // Ignored in bypass mode
            agc: false,                 // Not applicable in bypass mode
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

impl<MODE> Xt1Config<MODE> {
    /// Map the configured frequency onto the XTS mode bit and XT1HFFREQ range.
    ///
    /// Ranges per SLAU445I Table 3-10: 1 to 4 MHz, above 4 to 6 MHz, above 6 to
    /// 16 MHz, above 16 to 24 MHz (the last range exists on the enhanced clock
    /// system only, which is the only system with high-frequency XT1 support
    /// covered by this HAL).
    #[cfg(feature = "xt1_high_frequency")]
    #[inline]
    fn mode_bits(&self) -> (Xts, Xt1hffreq) {
        if self.frequency <= 40_000 {
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

    /// Bring up the XT1 oscillator and block until it has stabilized.
    ///
    /// Stabilization deliberately runs with settings that differ from the
    /// user's configuration; [`Self::finalize`] applies the requested values
    /// once the system clocks have been switched over:
    ///
    /// - Drive strength is forced to the maximum. Per SLAU445I 3.2.4, XT1
    ///   "starts with the highest drive settings for fast reliable startup"
    ///   and only "after startup, user software can reduce the drive strength".
    /// - XT1AUTOOFF is cleared so the oscillator runs unconditionally. This
    ///   guarantees the fault-polling loop below actually exercises the
    ///   crystal even though no system clock or FLL reference has requested
    ///   XT1 yet, and keeps it running until the switch-over.
    fn start(&self, periph: &_pac::Cs) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };

        // The start fault counter must be configured before the oscillator
        // starts. When enabled, the hardware holds the fault condition (and
        // XT1OFFG) asserted until XT1 has oscillated cleanly for 8192 (LF) or
        // 1024 (HF) cycles, which is what turns the fault-polling loop below
        // into a stabilization wait (SLAU445I 3.2.13).
        periph.csctl7().modify(|_, w| w.enstfcnt1().bit(self.start_counter));

        periph.csctl6().modify(|_, w| {
            let w = w
                .xt1bypass().bit(self.bypass)
                .xt1agcoff().bit(!self.agc)
                .xt1autooff().clear_bit()
                .xt1drive().variant(Xt1drive::Xt1drive3);

            #[cfg(feature = "xt1_high_frequency")]
            let w = {
                let (xts, hf_range) = self.mode_bits();
                w.xts().variant(xts).xt1hffreq().variant(hf_range)
            };
            // Devices without high-frequency support run XT1 in
            // low-frequency mode only.
            #[cfg(not(feature = "xt1_high_frequency"))]
            let w = w.xts().clear_bit();

            #[cfg(feature = "enhanced_cs")]
            let w = w.xt1faultoff().bit(!self.fault_switch);

            w
        });

        // Oscillator fault flags are sticky: they stay latched even after the
        // fault condition disappears, and re-assert if cleared while the fault
        // persists (SLAU445I 3.2.13). Clearing them and checking whether they
        // return is therefore the canonical way to wait for the oscillator:
        // this loop only exits once XT1 runs fault-free (for the full start
        // counter period, if enabled).
        loop {
            unsafe {
                periph.csctl7().clear_bits(|w|
                    w.xt1offg().clear_bit()
                      .dcoffg().clear_bit()
                );
                sfr.sfrifg1().clear_bits(|w| w.ofifg().clear_bit());
            }

            if sfr.sfrifg1().read().ofifg().bit_is_clear() {
                break;
            }
        }
    }

    /// Apply the user-requested drive strength and auto-off behavior, then
    /// leave the oscillator fault flags clean.
    ///
    /// This runs *after* the system clocks have been switched over, so XT1
    /// stays continuously powered (auto-off was held disabled by
    /// [`Self::start`]) from stabilization through selection. No restart can
    /// occur in between, which matters because a latched fault flag freezes
    /// the fail-safe REFO fallback in place until software clears it
    /// (SLAU445I 3.2.13).
    fn finalize(&self, periph: &_pac::Cs) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };

        periph.csctl6().modify(|_, w| {
            w.xt1drive().variant(self.drive)
                .xt1autooff().bit(self.auto_off)
        });

        // A single clear pass leaves a healthy system fault-free; if a genuine
        // fault remains the hardware simply re-asserts the flags for the user
        // to observe. No loop here: this must never hang post-configuration.
        unsafe {
            periph.csctl7().clear_bits(|w|
                w.xt1offg().clear_bit()
                  .dcoffg().clear_bit()
            );
            sfr.sfrifg1().clear_bits(|w| w.ofifg().clear_bit());
        }
    }
}

// Using Xt1State as a trait bound outside the HAL will never be useful, since we only
// configure the clocks once, so just keep it hidden (same treatment as `SmclkState`).
#[doc(hidden)]
pub trait Xt1State {
    /// XT1 frequency in Hz, or `None` when XT1 is not configured
    fn freq(&self) -> Option<u32>;
    /// Bring up and stabilize XT1 (no-op when XT1 is not configured)
    fn start(&self, periph: &_pac::Cs);
    /// Apply post-stabilization XT1 settings (no-op when XT1 is not configured)
    fn finalize(&self, periph: &_pac::Cs);
}

impl<MODE> Xt1State for Xt1Defined<MODE> {
    #[inline(always)]
    fn freq(&self) -> Option<u32> {
        Some(self.0.frequency)
    }

    #[inline(always)]
    fn start(&self, periph: &_pac::Cs) {
        self.0.start(periph);
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
    fn start(&self, _periph: &_pac::Cs) {}

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
    fll_ref: Selref
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
        }
    }
}

impl<MCLK, SMCLK, XT1CLK> ClockConfig<MCLK, SMCLK, XT1CLK> {
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
    /// This setting selects the default factory trim for DCO trimming and performs no extra
    /// calibration, so only a select few frequency targets can be selected.
    #[inline]
    pub fn mclk_dcoclk(
        self,
        target_freq: DcoclkFreqSel,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Dcoclk(target_freq)), self.smclk, self.xt1clk, self.fll_ref)
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

    /// Enable XT1 with specific hardware requirements
    #[inline]
    pub fn xt1clk_on<MODE>(
        self,
        config: Xt1Config<MODE>
    ) -> ClockConfig<MCLK, SMCLK, Xt1Defined<MODE>> {
        make_clkconf!(self, self.mclk, self.smclk, Xt1Defined(config), self.fll_ref)
    }

    /// Explicitly disable XT1 to save power
    #[inline]
    pub fn xt1clk_off(self) -> ClockConfig<MCLK, SMCLK, Xt1Disabled> {
        make_clkconf!(self, self.mclk, self.smclk, Xt1Disabled, Selref::Refoclk)
    }
}

impl<MCLK, SMCLK, MODE> ClockConfig<MCLK, SMCLK, Xt1Defined<MODE>> {
    /// Select XT1CLK for ACLK
    #[inline]
    pub fn aclk_xt1clk(mut self) -> Self {
        self.aclk_sel = AclkSel::Xt1clk(self.xt1clk.0.frequency);
        self
    }

    /// Select XT1CLK for MCLK
    #[inline]
    pub fn mclk_xt1clk(
        self,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK, Xt1Defined<MODE>> {
        let freq = self.xt1clk.0.frequency;
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Xt1clk(freq)), self.smclk, self.xt1clk, self.fll_ref)
        }
    }

    /// Select XT1CLK for fll
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

/// Pick the FLL reference divider for an XT1-referenced FLL so that the
/// divided reference lands in the stable ~23 kHz to ~47 kHz range. Returns the
/// divided reference frequency along with the divider setting.
#[cfg(feature = "xt1_high_frequency")]
#[inline]
fn xt1_fll_ref_divider(freq: u32) -> (u32, Fllrefdiv) {
    // Each cutoff is the "handover" point between hardware dividers: at
    // 1.5 MHz, /32 gives 46.8 kHz and /64 gives 23.4 kHz, and so on.
    if freq <= 40_000 {
        // Low-frequency crystal: the reference is used undivided
        (freq, FLLREFDIV_1)
    } else if freq <= 1_500_000 {
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

/// On devices without high-frequency XT1 support there is nothing to divide:
/// "If XT1 supports only a 32-kHz clock, FLLREFDIV always reads and should be
/// written as zero" (SLAU445I 3.3.4).
#[cfg(not(feature = "xt1_high_frequency"))]
#[inline]
fn xt1_fll_ref_divider(freq: u32) -> (u32, Fllrefdiv) {
    (freq, FLLREFDIV_1)
}

impl<SMCLK: SmclkState, XT1CLK: Xt1State> ClockConfig<MclkDefined, SMCLK, XT1CLK> {
    #[inline]
    fn configure_dco_fll(&self) {
        // Run FLL configuration procedure from the user's guide if we are using DCO
        if let MclkSel::Dcoclk(target_freq) = self.mclk.0 {
            // The FLL is referenced by XT1CLK only if XT1 has actually been
            // configured; in every other case it is referenced by REFOCLK.
            // The typestate API already guarantees `fll_ref` can only be
            // XT1CLK while XT1 is defined, but resolving the pair here keeps
            // the hardware configuration consistent by construction.
            let (selref, ref_freq, ref_div) = match (self.fll_ref, self.xt1clk.freq()) {
                (Selref::Xt1clk, Some(freq)) => {
                    let (ref_freq, ref_div) = xt1_fll_ref_divider(freq);
                    (Selref::Xt1clk, ref_freq, ref_div)
                }
                _ => (Selref::Refoclk, REFOCLK_FREQ_HZ as u32, FLLREFDIV_1),
            };

            fll_off();

            self.periph.csctl3()
                .write(|w| w.selref().variant(selref).fllrefdiv().variant(ref_div));
            self.periph.csctl0().write(|w| unsafe { w.bits(0) });
            self.periph
                .csctl1()
                .write(|w| w.dcorsel().variant(target_freq.dcorsel()));

            // Use the divided reference frequency to get a precise multiplier
            let multiplier = (target_freq.freq() / ref_freq) as u16;

            self.periph.csctl2().write(|w| {
                unsafe { w.flln().bits(multiplier - 1) }
                    .flld()
                    ._1()
            });

            msp430::asm::nop();
            msp430::asm::nop();
            msp430::asm::nop();

            fll_on();

            while !self.periph.csctl7().read().fllunlock().is_fllunlock_0() {}
        }
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

    #[inline]
    unsafe fn configure_fram(fram: &mut Fram, mclk_freq: u32) {
        if mclk_freq > 16_000_000 {
            fram.set_wait_states(WaitStates::Wait2);
        } else if mclk_freq > 8_000_000 {
            fram.set_wait_states(WaitStates::Wait1);
        } else {
            fram.set_wait_states(WaitStates::Wait0);
        }
    }

    /// Commit the configuration to hardware, in the order required by the
    /// user's guide, and return the resulting MCLK frequency:
    ///
    /// 1. FRAM wait states (must be set *before* MCLK exceeds 8 MHz)
    /// 2. XT1 bring-up and stabilization (must be stable *before* it can serve
    ///    as FLL reference or system clock source)
    /// 3. DCO and FLL configuration (waits for FLL lock)
    /// 4. Clock source selection and dividers
    /// 5. XT1 post-stabilization settings (user drive strength and auto-off,
    ///    applied only after the switch-over so the oscillator never restarts
    ///    in between)
    #[inline]
    fn freeze_internal(&self, fram: &mut Fram) -> u32 {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        self.xt1clk.start(&self.periph);
        self.configure_dco_fll();
        self.configure_cs();
        self.xt1clk.finalize(&self.periph);
        mclk_freq
    }
}

impl<MODE> ClockConfig<MclkDefined, SmclkDefined, Xt1Defined<MODE>> {
    /// Apply clock configuration to hardware and return SMCLK, ACLK and XT1CLK clock objects.
    /// Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, Xt1clk, SysDelay) {
        let mclk_freq = self.freeze_internal(fram);
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_sel.freq()),
            Xt1clk(self.xt1clk.0.frequency),
            SysDelay::new(mclk_freq),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDefined, Xt1Disabled> {
    /// Apply clock configuration to hardware and return SMCLK and ACLK clock objects.
    /// Also returns delay provider
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, SysDelay) {
        let mclk_freq = self.freeze_internal(fram);
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_sel.freq()),
            SysDelay::new(mclk_freq),
        )
    }
}

impl<MODE> ClockConfig<MclkDefined, SmclkDisabled, Xt1Defined<MODE>> {
    /// Apply clock configuration to hardware and return ACLK and XT1CLK clock objects, as SMCLK
    /// is disabled. Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, Xt1clk, SysDelay) {
        let mclk_freq = self.freeze_internal(fram);
        (
            Aclk(self.aclk_sel.freq()),
            Xt1clk(self.xt1clk.0.frequency),
            SysDelay::new(mclk_freq),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDisabled, Xt1Disabled> {
    /// Apply clock configuration to hardware and return ACLK clock object, as SMCLK is disabled.
    /// Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, SysDelay) {
        let mclk_freq = self.freeze_internal(fram);
        (Aclk(self.aclk_sel.freq()), SysDelay::new(mclk_freq))
    }
}

/// SMCLK clock object
pub struct Smclk(u32);
/// ACLK clock object
pub struct Aclk(u32);
/// XT1CLK clock object
pub struct Xt1clk(u32);

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

impl Clock for Xt1clk {
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}
