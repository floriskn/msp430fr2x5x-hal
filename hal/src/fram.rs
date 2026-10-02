//! FRAM controller
//!
//! Besides the wait states, [`Fram`] configures what happens when the FRAM detects a bit error
//! (SLAU445I 6.6). Its error correction fixes single-bit errors, and the controller can report those
//! and the errors it can't correct.

use crate::_pac;

/// FRAM controller
pub struct Fram {
    fram: _pac::Frctl,
}

impl Fram {
    /// Turn FRCTL into `Fram`
    pub fn new(fram: _pac::Frctl) -> Self { Fram { fram } }
}

const PASSWORD: u8 = 0xA5;
#[cfg(feature = "frwpoa")]
const SYSCFG0_PASSWORD: u8 = 0xA5;

/// FRAM wait states
pub enum WaitStates {
    /// No wait
    Wait0,
    /// Wait 1 cycle
    Wait1,
    /// Wait 2 cycles
    Wait2,
    /// Wait 3 cycles
    Wait3,
    /// Wait 4 cycles
    Wait4,
    /// Wait 5 cycles
    Wait5,
    /// Wait 6 cycles
    Wait6,
    /// Wait 7 cycles
    Wait7,
}

/// What the FRAM controller does when it detects a bit error it can't correct (GCCTL0.UBDRSTEN, UBDIE)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UncorrectableBitError {
    /// Nothing, as after reset
    Ignore,
    /// Reset the device with a PUC. [`Pmm::take_reset_cause()`](crate::pmm::Pmm::take_reset_cause)
    /// then returns [`ResetCause::FramBitError`](crate::pmm::ResetCause::FramBitError).
    Reset,
    /// Request the `SYSNMI` interrupt. [`take_system_nmi()`](crate::sys::take_system_nmi) then
    /// returns [`SystemNmi::FramUncorrectableBitError`](crate::sys::SystemNmi::FramUncorrectableBitError).
    Interrupt,
}

impl Fram {
    /// Run `f` with write access to the FRAM controller registers, and lock them again afterwards.
    ///
    /// The password in FRCTL0 unlocks them, and a byte write of a wrong password to the upper byte
    /// of FRCTL0 locks them again (SLAU445I 6.10). Writing them while locked causes a PUC.
    #[inline]
    fn unlocked<R>(&mut self, f: impl FnOnce(&_pac::Frctl) -> R) -> R {
        critical_section::with(|_| {
            self.fram.frctl0().modify(|_, w| unsafe { w.frctlpw().bits(PASSWORD) });
            let ret = f(&self.fram);
            let frctl0_h = (self.fram.frctl0().as_ptr() as *mut u8).wrapping_add(1);
            unsafe { frctl0_h.write_volatile(0) };
            ret
        })
    }

    /// Set number of FRAM wait states. Could cause issues reading instructions from FRAM if
    /// incorrect.
    /// # Safety
    /// Should wait 1 cycle if MCLK > 8MHz and 2 cycles if MCLK > 16MHz.
    #[inline]
    pub unsafe fn set_wait_states(&mut self, wait: WaitStates) {
        self.unlocked(|fram| fram.frctl0().write(|w| unsafe { w
            .frctlpw().bits(PASSWORD)
            .nwaits().bits(wait as u8) }));
    }

    /// Select what happens when the FRAM detects a bit error it can't correct.
    #[inline]
    pub fn set_uncorrectable_bit_error_action(&mut self, action: UncorrectableBitError) {
        self.unlocked(|fram| {
            // UBDRSTEN and UBDIE must not be set at the same time, so clear both first
            fram.gcctl0().modify(|_, w| w.ubdrsten().clear_bit().ubdie().clear_bit());
            match action {
                UncorrectableBitError::Ignore => (),
                UncorrectableBitError::Reset => { fram.gcctl0().modify(|_, w| w.ubdrsten().set_bit()); }
                UncorrectableBitError::Interrupt => { fram.gcctl0().modify(|_, w| w.ubdie().set_bit()); }
            }
        });
    }

    /// Request the `SYSNMI` interrupt when the FRAM detects and corrects a bit error (CBDIE).
    /// [`take_system_nmi()`](crate::sys::take_system_nmi) then returns
    /// [`SystemNmi::FramCorrectableBitError`](crate::sys::SystemNmi::FramCorrectableBitError).
    #[inline]
    pub fn enable_correctable_bit_error_interrupts(&mut self) {
        self.unlocked(|fram| fram.gcctl0().modify(|_, w| w.cbdie().set_bit()));
    }

    /// Stop requesting the `SYSNMI` interrupt for corrected bit errors (CBDIE).
    #[inline]
    pub fn disable_correctable_bit_error_interrupts(&mut self) {
        self.unlocked(|fram| fram.gcctl0().modify(|_, w| w.cbdie().clear_bit()));
    }

    /// Leave the first `kib` KiB of program FRAM writable, while the program FRAM write protection
    /// protects the rest (SYSCFG0.FRWPOA). 0, as after reset, protects all of it.
    ///
    /// Use this for data that changes often, in place of the information memory. By default the
    /// linker puts the program at the start of program FRAM, so first reserve this space for data in
    /// `memory.x`, or the program becomes writable instead.
    ///
    /// Only available on the MSP430FR2x5x and the MSP430FR2522 (SLAU445I 1.16, SYSCFG0).
    ///
    /// # Panics
    ///
    /// If `kib` is above 63.
    #[cfg(feature = "frwpoa")]
    #[inline]
    pub fn set_writable_program_fram(&mut self, kib: u8) {
        assert!(kib <= 63, "at most 63 KiB of program FRAM can be left writable");
        let sys = unsafe { &*_pac::Sys::ptr() };
        critical_section::with(|_| {
            sys.syscfg0().modify(|_, w| unsafe { w
                .frwppw().bits(SYSCFG0_PASSWORD)
                .frwpoa().bits(kib)
            })
        });
    }
}
