//! Power management module

use core::marker::PhantomData;

use crate::{_pac, info_mem::InfoMemory};

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

    /// Configures the internal voltage reference to the specified voltage and enables it.
    /// Returns a token signifying that the voltage reference has been enabled, unless it was *already* enabled.
    pub fn enable_internal_reference(&mut self, vref: ReferenceVoltage) -> Option<InternalVRef> {
        let pmmctl2 = self.0.pmmctl2().read();
        match pmmctl2.intrefen().bit() {
            true => None,
            false => {
                // Unlock PMM registers
                self.0.pmmctl0().modify(|_, w| w.pmmpw().password());

                self.0.pmmctl2().write(|w| unsafe{ w
                    .bits(pmmctl2.bits()) 
                    .refvsel().bits(vref as u8)
                    .intrefen().set_bit()
                });
                Some(InternalVRef(vref))
            }
        }
    }

    /// Disables the internal reference voltage
    pub fn disable_internal_reference(&mut self, _vref: InternalVRef) {
        unsafe {
            self.0.pmmctl2().clear_bits(|w| w.intrefen().clear_bit());
        }
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
                unsafe { self.0.pmmctl2().set_bits(|w| w.tsensoren().set_bit()) };
                Some(InternalTempSensor(PhantomData))
            }
        }
    }

    /// Disables the internal temperature sensor
    pub fn disable_internal_temp_sensor(&mut self, _tsense: InternalTempSensor) {
        unsafe { self.0.pmmctl2().clear_bits(|w| w.tsensoren().clear_bit()) };
    }
}
