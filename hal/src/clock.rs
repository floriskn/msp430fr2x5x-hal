//! Clock system for configuration of MCLK, SMCLK, and ACLK.
//!
//! Once configuration is complete, `Aclk` and `Smclk` clock objects are returned. The clock
//! objects are used to set the clock sources on other peripherals.
//! Configuration of MCLK and SMCLK *must* occur, though SMCLK can be disabled. In that case, only
//! `Aclk` is returned.
//!
//! DCO with FLL is supported on MCLK for select frequencies. Supporting arbitrary frequencies on
//! the DCO requires complex calibration routines not supported by the HAL.

use core::arch::asm;
use core::marker::PhantomData;

use crate::delay::SysDelay;
use crate::fram::{Fram, WaitStates};
use crate::_pac::{self, cs::{csctl1::Dcorsel, csctl4::{Sela, Selms}}};
pub use crate::_pac::cs::csctl5::{Divm as MclkDiv, Divs as SmclkDiv};

/// REFOCLK frequency
pub const REFOCLK_FREQ_HZ: u16 = 32768;
/// VLOCLK frequency
pub const VLOCLK_FREQ_HZ: u16 = 10000;
pub use crate::device_specific::MODCLK_FREQ_HZ;

// Make PAC CLOCK peripherals available as a re-export
#[cfg(feature = "xt1clk_source")]
pub(crate) use crate::device_specific::clock::*;

enum MclkSel {
    Refoclk,
    Vloclk,
    Dcoclk(DcoclkFreqSel),
}

impl MclkSel {
    #[inline]
    fn freq(&self) -> u32 {
        match self {
            MclkSel::Vloclk => VLOCLK_FREQ_HZ as u32,
            MclkSel::Refoclk => REFOCLK_FREQ_HZ as u32,
            MclkSel::Dcoclk(sel) => sel.freq(),
        }
    }

    #[inline(always)]
    fn selms(&self) -> Selms {
        match self {
            MclkSel::Vloclk => Selms::Vloclk,
            MclkSel::Refoclk => Selms::Refoclk,
            MclkSel::Dcoclk(_) => Selms::Dcoclkdiv,
        }
    }
}

#[derive(Clone, Copy)]
enum AclkSel {
    #[cfg(feature = "vloclk_source")]
    Vloclk,
    Refoclk,
    #[cfg(feature = "xt1clk_source")]
    Xt1clk(u16),
}

impl AclkSel {
    #[inline(always)]
    fn sela(self) -> Sela {
        match self {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => Sela::Vloclk,
            AclkSel::Refoclk => Sela::Refoclk,
            #[cfg(feature = "xt1clk_source")]
            AclkSel::Xt1clk(_) => Sela::Xt1clk,
        }
    }

    #[inline(always)]
    fn freq(self) -> u16 {
        match self {
            #[cfg(feature = "vloclk_source")]
            AclkSel::Vloclk => VLOCLK_FREQ_HZ,
            AclkSel::Refoclk => REFOCLK_FREQ_HZ,            
            #[cfg(feature = "xt1clk_source")]
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
    #[cfg(feature = "2x5x")]
    /// 20 MHz
    _20MHz,
    #[cfg(feature = "2x5x")]
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
            #[cfg(feature = "2x5x")]
            DcoclkFreqSel::_20MHz => Dcorsel::Dcorsel6,
            #[cfg(feature = "2x5x")]
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
            #[cfg(feature = "2x5x")]
            DcoclkFreqSel::_20MHz => 610,
            #[cfg(feature = "2x5x")]
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

// Using SmclkState as a trait bound outside the HAL will never be useful, since we only configure
// the clock once, so just keep it hidden
#[doc(hidden)]
pub trait SmclkState {
    fn div(&self) -> Option<SmclkDiv>;
}

impl SmclkState for SmclkDefined {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        Some(self.0)
    }
}

impl SmclkState for SmclkDisabled {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        None
    }
}

/// Associates an external crystal oscillator with its respective input and output pins.
pub trait Xt1Oscillator {
    /// The crystal's output pin (typically XOUT).
    type XoutPin;
    /// The crystal's input pin (typically XIN).
    type XinPin;
}

/// Builder object that configures system clocks
///
/// Can only commit configurations to hardware if both MCLK and SMCLK settings have been
/// configured. ACLK configurations are optional, with its default source being REFOCLK.
pub struct ClockConfig<MCLK, SMCLK, OSCILLATOR> {
    periph: _pac::Cs,
    mclk: MCLK,
    mclk_div: MclkDiv,
    aclk_sel: AclkSel,
    smclk: SMCLK,
    _osc: PhantomData<OSCILLATOR>,
}

macro_rules! make_clkconf {
    ($conf:expr, $mclk:expr, $smclk:expr) => {
        ClockConfig {
            periph: $conf.periph,
            mclk: $mclk,
            mclk_div: $conf.mclk_div,
            aclk_sel: $conf.aclk_sel,
            smclk: $smclk,
            _osc: PhantomData,
        }
    };
}

#[cfg(feature = "xt1clk_source")]
/// Target oscillator type for XT1 clock configurations.
pub type DefaultOsc = Xt1clk;

#[cfg(not(feature = "xt1clk_source"))]
/// Placeholder type when external crystal sources are disabled.
pub type DefaultOsc = NoClockDefined;

#[cfg(not(feature = "xt1clk_source"))]
impl Xt1Oscillator for NoClockDefined {
    type XoutPin = ();
    type XinPin  = ();
}

impl ClockConfig<NoClockDefined, NoClockDefined, DefaultOsc> {
    /// Converts CS into a fresh, unconfigured clock builder object
    pub fn new(cs: _pac::Cs) -> Self {
        ClockConfig {
            periph: cs,
            smclk: NoClockDefined,
            mclk: NoClockDefined,
            mclk_div: MclkDiv::_1,
            aclk_sel: AclkSel::Refoclk,
            _osc: PhantomData,
        }
    }
}

impl<MCLK, SMCLK, OSCILLATOR> ClockConfig<MCLK, SMCLK, OSCILLATOR>
where 
    OSCILLATOR: Xt1Oscillator 
{
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

    #[cfg(feature = "xt1clk_source")]
    /// Select XT1CLK for ACLK
    #[inline]
    pub fn aclk_xt1clk<T: Into<OSCILLATOR::XoutPin>, R: Into<OSCILLATOR::XinPin>>(mut self, freq: u16, _xout: T, _xin: R) -> Self {
        self.aclk_sel = AclkSel::Xt1clk(freq);
        self
    }

    /// Select REFOCLK for MCLK and set the MCLK divider. Frequency is `32_768 / mclk_div` Hz.
    #[inline]
    pub fn mclk_refoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK, OSCILLATOR> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Refoclk), self.smclk)
        }
    }

    /// Select VLOCLK for MCLK and set the MCLK divider. Frequency is `10_000 / mclk_div` Hz.
    #[inline]
    pub fn mclk_vloclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK, OSCILLATOR> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Vloclk), self.smclk)
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
    ) -> ClockConfig<MclkDefined, SMCLK, OSCILLATOR> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Dcoclk(target_freq)), self.smclk)
        }
    }

    /// Enable SMCLK and set SMCLK divider, which divides the MCLK frequency
    #[inline]
    pub fn smclk_on(self, div: SmclkDiv) -> ClockConfig<MCLK, SmclkDefined, OSCILLATOR> {
        make_clkconf!(self, self.mclk, SmclkDefined(div))
    }

    /// Disable SMCLK
    #[inline]
    pub fn smclk_off(self) -> ClockConfig<MCLK, SmclkDisabled, OSCILLATOR> {
        make_clkconf!(self, self.mclk, SmclkDisabled)
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

impl<SMCLK: SmclkState, OSCILLATOR: Xt1Oscillator> ClockConfig<MclkDefined, SMCLK, OSCILLATOR> {
    #[inline]
    fn configure_dco_fll(&self) {
        // Run FLL configuration procedure from the user's guide if we are using DCO
        if let MclkSel::Dcoclk(target_freq) = self.mclk.0 {
            fll_off();

            let is_xt1 = self.aclk_sel.sela() == Sela::Xt1clk;

            self.periph.csctl3().write(|w| {
                if is_xt1 { w.selref().xt1clk() } else { w.selref().refoclk() }
            });
            
            if is_xt1 {
                self.stabilize_xt1();
            }

            self.periph.csctl0().write(|w| unsafe { w.bits(0) });
            self.periph
                .csctl1()
                .write(|w| w.dcorsel().variant(target_freq.dcorsel()));
            self.periph.csctl2().write(|w| {
                unsafe { w.flln().bits(target_freq.multiplier() - 1) }
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
        self.periph.csctl4().write(|w| {
            w.sela()
                .variant(self.aclk_sel.sela())
                .selms()
                .variant(self.mclk.0.selms())
        });

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

    #[inline(always)]
    fn stabilize_xt1(&self) {
        let sfr = unsafe { &*_pac::Sfr::ptr() };
        loop {
            unsafe {
                // self.periph.csctl7().read();
                self.periph.csctl7().clear_bits(|w| 
                    w.xt1offg().clear_bit()
                      .dcoffg().clear_bit()
                );
                // sfr.sfrifg1().clear_bits(|w| w.ofifg().clear_bit());
            }

            // Poll global fault flag
            if sfr.sfrifg1().read().ofifg().is_ofifg_0() {
                break;
            }
        }
    }
}

impl<OSCILLATOR: Xt1Oscillator> ClockConfig<MclkDefined, SmclkDefined, OSCILLATOR> {
    /// Apply clock configuration to hardware and return SMCLK and ACLK clock objects.
    /// Also returns delay provider
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, SysDelay) {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        self.configure_dco_fll();
        self.configure_cs();
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_sel.freq()),
            SysDelay::new(mclk_freq),
        )
    }
}

impl<OSCILLATOR: Xt1Oscillator> ClockConfig<MclkDefined, SmclkDisabled, OSCILLATOR> {
    /// Apply clock configuration to hardware and return ACLK clock object, as SMCLK is disabled.
    /// Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, SysDelay) {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        self.configure_dco_fll();
        self.configure_cs();
        (Aclk(self.aclk_sel.freq()), SysDelay::new(mclk_freq))
    }
}

/// SMCLK clock object
pub struct Smclk(u32);
/// ACLK clock object
pub struct Aclk(u16);

/// Trait for configured clock objects
pub trait Clock {
    /// Type of the returned frequency value
    type Freq;

    /// Frequency of the clock
    fn freq(&self) -> Self::Freq;
}

impl Clock for Smclk {
    type Freq = u32;

    /// Returning a 32-bit frequency may seem suspect, since we're on a 16-bit system, but it is
    /// required as SMCLK can go up to 24 MHz. Clock frequencies are usually for initialization
    /// tasks such as computing baud rates, which should be optimized away, avoiding the extra cost
    /// of 32-bit computations.
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}

impl Clock for Aclk {
    type Freq = u16;

    #[inline]
    fn freq(&self) -> u16 {
        self.0
    }
}

/// Manages the GPIO multiplexing state for the XT1 crystal.
pub trait Xt1ClockState {
    /// Checks if the pins are currently in XT1 mode.
    fn is_active() -> bool;

    /// Resets the associated port selection registers.
    fn reset_ports();

    /// Clears only the XT1-specific bits from the registers.
    fn clear_bits();
}

macro_rules! impl_xt1_clk {
    (
        $target_struct:ident,
        $port_a:ident, $pin_a:ident, $alt_a:ident,
        $port_b:ident, $pin_b:ident, $alt_b:ident,
    ) => {
        impl $crate::clock::Xt1Oscillator for $target_struct {
            type XoutPin = Pin<$port_a, $pin_a, $alt_a<Input<Floating>>>;
            type XinPin  = Pin<$port_b, $pin_b, $alt_b<Input<Floating>>>;
        }

        impl $crate::clock::Xt1ClockState for $target_struct {
            fn is_active() -> bool {
                let pa = unsafe { $port_a::steal() };
                const MASK_A: u8 = $pin_a::SET_MASK;
                const MASK_B: u8 = $pin_b::SET_MASK;

                macro_rules! check_logic {
                    ($p:expr, $mask:expr, $alt:ident) => {
                        match $alt::<()>::MODE {
                            Alternate::Alternate1 => ($p.pxsel1_rd() & $mask == 0) && ($p.pxsel0_rd() & $mask == $mask),
                            Alternate::Alternate2 => ($p.pxsel1_rd() & $mask == $mask) && ($p.pxsel0_rd() & $mask == 0),
                            Alternate::Alternate3 => ($p.pxsel1_rd() & $mask == $mask) && ($p.pxsel0_rd() & $mask == $mask),
                        }
                    };
                }

                macro_rules! dispatch_port_logic {
                    ($port_a, $port_a, $alt_a, $alt_a) => {
                        {
                            let combined_mask = MASK_A | MASK_B;
                            check_logic!(pa, combined_mask, $alt_a)
                        }
                    };
                    ($port_a, $port_b, $alt_a, $alt_b) => {
                        {
                            let pb = unsafe { $p_b::steal() };
                            check_logic!(pa, MASK_A, $alt_a) && check_logic!(pb, MASK_B, $alt_b)
                        }
                    };
                }

                dispatch_port_logic!($port_a, $port_b, $alt_a, $alt_b)
            }

            /// Resets the selection registers for the crystal ports
            fn reset_ports() {
                let pa = unsafe { $port_a::steal() };
                pa.pxsel0_reset();
                pa.pxsel1_reset();

                macro_rules! dispatch_port_logic {
                    ($port_a, $port_a) => { {} };
                    ($port_a, $port_b) => {
                        {
                            let pb = unsafe { $p_b::steal() };
                            pb.pxsel0_reset();
                            pb.pxsel1_reset();
                        }
                    };
                }

                dispatch_port_logic!($port_a, $port_b);
            }

            /// Clears the specific pin masks from the selection registers
            fn clear_bits() {
                let pa = unsafe { $port_a::steal() };
                const MASK_A: u8 = $pin_a::SET_MASK;
                const MASK_B: u8 = $pin_b::SET_MASK;

                macro_rules! dispatch_port_logic {
                    ($port_a, $port_a) => {
                        {
                            let combined = MASK_A | MASK_B;
                            pa.pxsel0_clear(combined);
                            pa.pxsel1_clear(combined);
                        }
                    };
                    ($port_a, $port_b) => { 
                        {
                            pa.pxsel0_clear(MASK_A);
                            pa.pxsel1_clear(MASK_A);
                            let pb = unsafe { $p_b::steal() };
                            pb.pxsel0_clear(MASK_B);
                            pb.pxsel1_clear(MASK_B);
                        }
                    };
                }

                dispatch_port_logic!($port_a, $port_b);
            }
        }
    };
}
pub(crate) use impl_xt1_clk;