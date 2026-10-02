//! Power management module
//!
//! Besides the internal voltage reference and temperature sensor, [`Pmm`] reports why the device
//! reset ([`Pmm::take_reset_cause()`]), triggers software resets and controls the high-side supply
//! voltage supervisor (SVSH).

use core::marker::PhantomData;

use crate::{_pac, info_mem::InfoMemory, lpm::SvsState};

/// PMM type
pub struct Pmm(_pac::Pmm);

/// Struct indicating that the internal voltage reference has been enabled and configured.
/// This can be passed to the ADC to read the reference voltage.
#[derive(Debug)]
pub struct InternalVRef(ReferenceVoltage);
impl InternalVRef {
    /// Get the requested internal reference voltage
    pub fn voltage(&self) -> ReferenceVoltage { self.0 }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
/// A list of possible internal reference voltages
pub enum ReferenceVoltage {
    /// 1.5V
    _1V5 = 0b00,

    #[cfg(feature = "enhanced_ref")]
    /// 2.0V
    _2V0 = 0b01,

    #[cfg(feature = "enhanced_ref")]
    /// 2.5V
    _2V5 = 0b10,
}

/// Token indicating that the internal temperature sensor has been enabled.
/// This can be passed to the ADC to read the temperature sensor voltage.
#[derive(Debug)]
pub struct InternalTempSensor<'a>(PhantomData<&'a InternalVRef>);

/// A reason for a reset, in priority order (SYSRSTIV, data sheets: System Module Interrupt Vector
/// Registers). A brownout reset (BOR) resets the most, then a power-on reset (POR), then a power-up
/// clear (PUC) (SLAU445I 1.2).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetCause {
    /// Power-up, or the supply dropped below the brownout level (BOR)
    Brownout,
    /// A low level on the RST/NMI pin (BOR)
    ResetPin,
    /// [`Pmm::software_bor()`] (BOR)
    SoftwareBor,
    /// A wake-up from LPM3.5 or LPM4.5 (BOR)
    Lpmx5WakeUp,
    /// A security violation (BOR)
    SecurityViolation,
    /// The supply dropped below the high-side SVS level (BOR)
    Svsh,
    /// [`Pmm::software_por()`] (POR)
    SoftwarePor,
    /// The watchdog timed out (PUC)
    WatchdogTimeout,
    /// A write to the watchdog without its password (PUC)
    WatchdogPassword,
    /// A write to the FRAM controller without its password (PUC)
    FramPassword,
    /// The FRAM detected a bit error it couldn't correct, see
    /// [`Fram::set_uncorrectable_bit_error_action()`](crate::fram::Fram::set_uncorrectable_bit_error_action) (PUC)
    FramBitError,
    /// The CPU fetched an instruction from the peripheral area (PUC)
    PeripheralAreaFetch,
    /// A write to the PMM without its password (PUC)
    PmmPassword,
    /// The DCO ran too fast for the FLL, see
    /// [`ClockConfig::reset_on_fll_unlock()`](crate::clock::ClockConfig::reset_on_fll_unlock) (PUC)
    FllUnlock,
    /// A value the data sheets list as reserved
    Reserved(u16),
}

impl Pmm {
    /// Clears the LOCKLPM5 bit, so the I/O pins take on their configured state, and returns a
    /// `Pmm` (and an `InfoMemory`).
    pub fn new(pmm: _pac::Pmm, sys: _pac::Sys) -> (Pmm, InfoMemory) {
        let mut pmm = Pmm(pmm);
        pmm.unlock_lpm5();
        (pmm, InfoMemory::new(sys))
    }

    /// Like [`Pmm::new`], but leaves the LOCKLPM5 bit set. Use this after a wake-up from
    /// LPM3.5, and call [`Pmm::unlock_lpm5`] once the GPIO pins and clocks are configured.
    ///
    /// After a wake-up from LPMx.5 the I/O pins, and XT1 if it clocked the RTC, keep the
    /// configuration they had while asleep until LOCKLPM5 is cleared, while their registers
    /// start out reset. Configuring the pins, and XT1 through
    /// [`ClockConfig`](crate::clock::ClockConfig), before clearing LOCKLPM5 lets them carry on
    /// without a glitch (SLAU445I 1.4.3.3). With [`Pmm::new`], clearing LOCKLPM5 first would
    /// stop XT1, and the RTC with it, until XT1 is reconfigured.
    ///
    /// After a cold start the locked pins are held in their power-on state, so XT1 cannot
    /// start before LOCKLPM5 is cleared. Use [`Pmm::new`] then.
    pub fn new_locked(pmm: _pac::Pmm, sys: _pac::Sys) -> (Pmm, InfoMemory) {
        (Pmm(pmm), InfoMemory::new(sys))
    }

    /// Clears the LOCKLPM5 bit, so the I/O pins take on their configured state. Only needed
    /// after [`Pmm::new_locked`]; [`Pmm::new`] already does this.
    pub fn unlock_lpm5(&mut self) {
        self.0.pm5ctl0().write(|w| w.locklpm5().clear_bit());
    }

    /// Returns the highest-priority reason for a reset that hasn't been read yet and clears it
    /// (SYSRSTIV), or `None` once all have been read.
    ///
    /// The reasons accumulate until they are read, so a reset can have several, for example a
    /// brownout at power-up and a later watchdog timeout. Call this until it returns `None` to see
    /// them all. Reading a wake-up from LPMx.5 also clears PMMLPM5IFG.
    ///
    /// A debugger can start the program without a reset, after flashing it for example, and then
    /// there may be no reason at all.
    pub fn take_reset_cause(&mut self) -> Option<ResetCause> {
        let sys = unsafe { &*_pac::Sys::ptr() };
        match sys.sysrstiv().read().bits() {
            0x00 => None,
            0x02 => Some(ResetCause::Brownout),
            0x04 => Some(ResetCause::ResetPin),
            0x06 => Some(ResetCause::SoftwareBor),
            0x08 => Some(ResetCause::Lpmx5WakeUp),
            0x0A => Some(ResetCause::SecurityViolation),
            0x0E => Some(ResetCause::Svsh),
            0x14 => Some(ResetCause::SoftwarePor),
            0x16 => Some(ResetCause::WatchdogTimeout),
            0x18 => Some(ResetCause::WatchdogPassword),
            0x1A => Some(ResetCause::FramPassword),
            0x1C => Some(ResetCause::FramBitError),
            0x1E => Some(ResetCause::PeripheralAreaFetch),
            0x20 => Some(ResetCause::PmmPassword),
            0x24 => Some(ResetCause::FllUnlock),
            other => Some(ResetCause::Reserved(other)),
        }
    }

    /// Reset the device with a brownout reset (BOR), the reset of a power-up (PMMSWBOR).
    /// [`Pmm::take_reset_cause()`] then returns [`ResetCause::SoftwareBor`].
    pub fn software_bor(&mut self) -> ! {
        self.0.pmmctl0().modify(|_, w| w.pmmpw().password().pmmswbor().set_bit());
        loop {
            msp430::asm::nop();
        }
    }

    /// Reset the device with a power-on reset (POR), which resets less than a brownout reset
    /// (PMMSWPOR). [`Pmm::take_reset_cause()`] then returns [`ResetCause::SoftwarePor`].
    pub fn software_por(&mut self) -> ! {
        self.0.pmmctl0().modify(|_, w| w.pmmpw().password().pmmswpor().set_bit());
        loop {
            msp430::asm::nop();
        }
    }

    /// Whether the high-side supply voltage supervisor (SVSH) stays on in LPM2, LPM3 and LPM4, as
    /// after reset (PMMCTL0.SVSHE). It is always on in active mode, LPM0 and LPM1. Turning it off
    /// saves power in the low-power modes, but a supply drop there then resets the device only once
    /// it reaches the brownout level. For LPM3.5 and LPM4.5, `enter_lpm3_5()` and `enter_lpm4_5()`
    /// set this.
    pub fn set_svsh(&mut self, svs: SvsState) {
        self.unlocked(|pmm| pmm.pmmctl0().modify(|_, w| w.pmmpw().password().svshe().bit(svs == SvsState::Enabled)));
    }

    /// Run `f` with write access to the PMM registers, and lock them again afterwards.
    ///
    /// Writing a PMM register other than PMMCTL0 while they are locked causes a PUC, and so does a
    /// word write of a wrong password, so they are locked again with a byte write to the upper
    /// byte of PMMCTL0 (SLAU445I 2.3). Interrupts are disabled meanwhile, so an interrupt handler
    /// can't lock them halfway.
    #[inline]
    fn unlocked<R>(&mut self, f: impl FnOnce(&_pac::Pmm) -> R) -> R {
        critical_section::with(|_| {
            self.0.pmmctl0().modify(|_, w| w.pmmpw().password());
            let ret = f(&self.0);
            let pmmctl0_h = (self.0.pmmctl0().as_ptr() as *mut u8).wrapping_add(1);
            unsafe { pmmctl0_h.write_volatile(0) };
            ret
        })
    }

    /// Configures the internal voltage reference to the specified voltage and enables it.
    /// Returns a token signifying that the voltage reference has been enabled, unless it was *already* enabled.
    ///
    /// Waits until the reference has settled (REFGENRDY), as the user's guide recommends.
    pub fn enable_internal_reference(&mut self, vref: ReferenceVoltage) -> Option<InternalVRef> {
        if self.0.pmmctl2().read().intrefen().bit() {
            return None;
        }
        self.unlocked(|pmm| pmm.pmmctl2().modify(|_, w| unsafe { w
            .refvsel().bits(vref as u8)
            .intrefen().set_bit()
        }));
        while self.0.pmmctl2().read().refgenrdy().bit_is_clear() {}
        Some(InternalVRef(vref))
    }

    /// Disables the internal reference voltage
    pub fn disable_internal_reference(&mut self, _vref: InternalVRef) {
        self.unlocked(|pmm| unsafe { pmm.pmmctl2().clear_bits(|w| w.intrefen().clear_bit()) });
    }

    /// Enables the internal temperature sensor.
    /// Returns a token signifying that the temp sensor has been enabled, unless it was *already* enabled.
    pub fn enable_internal_temp_sensor<'a>(
        &mut self,
        _vref: &'a InternalVRef,
    ) -> Option<InternalTempSensor<'a>> {
        match self.0.pmmctl2().read().tsensoren().bit() {
            true  => None,
            false => {
                self.unlocked(|pmm| unsafe { pmm.pmmctl2().set_bits(|w| w.tsensoren().set_bit()) });
                Some(InternalTempSensor(PhantomData))
            }
        }
    }

    /// Disables the internal temperature sensor
    pub fn disable_internal_temp_sensor(&mut self, _tsense: InternalTempSensor) {
        self.unlocked(|pmm| unsafe { pmm.pmmctl2().clear_bits(|w| w.tsensoren().clear_bit()) });
    }
}
