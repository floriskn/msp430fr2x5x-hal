//! Low Power Mode (LPM) control
//!
//! The MSP430FR2x5x series supports several low power modes, namely LPM0, LPM3, LPM4, as well as
//! LPM3.5 and LPM4.5 (SLASEC4D Table 6-1, p. 61).
//! # LPM0
//! LPM0 turns off the CPU, while the rest of the system continues unimpeded. Entering LPM0 has no
//! special requirements (SLAU445I Table 1-2, p. 39).
//!
//! # LPM3
//! LPM3 turns off most high frequency clocks (FLL and DCO subsystems, MODCLK, etc.), most notably
//! SMCLK (SLASEC4D Table 6-1, p. 61).
//! Since most peripherals can be clocked by a low frequency clock source, this allows many peripherals
//! to continue operating at a reduced speed.
//!
//! LPM3 will only be entered if no peripherals have been configured to use SMCLK, otherwise LPM0 will
//! be entered instead (SLAU445I Table 1-3, p. 39).
//!
//! GPIO pins will maintain the value they had when LPM3 was entered (SLAU445I 1.4, p. 36).
//!
//! # LPM4
//! LPM4 turns off all clock sources (SLAU445I Table 1-2, p. 39). The RTC or Watchdog peripherals can
//! request very low power oscillators (VLOCLK or XTCLK) if needed, though this will increase power
//! consumption and is not considered 'true' LPM4 (SLASEC4D 6.2, p. 62: "XT1CLK and VLOCLK can be
//! active during LPM4 if requested by low-frequency peripherals"; SLAU445I 12.2.5, p. 364). Some
//! analog peripherals (eCOMP, SAC) continue to function in LPM4 (SLASEC4D Table 6-1, p. 62).
//! Wake-up events are limited to GPIO or RTC interrupts (SLASEC4D Table 6-1, p. 61 to p. 62).
//!
//! LPM4 will only be entered if no peripherals have been configured to use SMCLK or ACLK. If any
//! peripherals have requested SMCLK then LPM0 will be entered, otherwise if any peripherals have
//! requested ACLK then LPM3 will be entered (SLAU445I Table 1-3, p. 39).
//!
//! GPIO pins will maintain the value they had when LPM4 was entered (SLAU445I 1.4, p. 36).
//!
//! # LPM3.5
//! LPM3.5 is an extension of LPM4 that also disables the RAM and analog peripherals. Unlike in LPM4,
//! the very low power oscillators (VLOCLK or XTCLK) are expected to be used in LPM3.5 (SLASEC4D
//! Table 6-1, p. 61 to p. 62).
//! Because the RAM is unpowered, all internal state is lost and when the MCU wakes up execution will
//! restart from the beginning of the program (SLAU445I 1.4.3, p. 40). The 32-byte region of Backup
//! Memory is powered through LPM3.5, which can be used to maintain some state between iterations
//! (SLASEC4D Table 6-1, p. 62).
//!
//! Unlike with LPM3 and 4, a peripheral requesting a high-speed clock source like SMCLK will not stop
//! LPM3.5 from being entered (SLAU445I Table 3-2, p. 109).
//!
//! During LPM3.5 GPIO pins will maintain the value they had when LPM3.5 was entered but the register
//! contents are lost, so after a wake-up the GPIO pins will take on their reset values when LOCKLPM5
//! is cleared (SLAU445I 8.3.3, p. 318).
//!
//! # LPM4.5
//! LPM4.5 is an extension of LPM3.5 that also disables the very low power oscillators, backup memory,
//! and RTC (SLASEC4D Table 6-1, p. 61 to p. 62). Barely anything is powered in this mode. The only
//! methods to wake from LPM4.5 are a GPIO interrupt, the reset pin, or a power cycle (SLAU445I 1.4,
//! p. 37).
//! Like LPM3.5, all internal state is lost and program execution restarts from the beginning when a
//! wake-up occurs. Unlike LPM3.5, the backup memory is unpowered, so can't be used to store state.
//! The non-volatile Information Memory (`INFO_MEM_SIZE` bytes) can however be used to store data
//! while the MCU is in active mode, and can be read back after a wake-up event.
//!
//! Unlike with LPM3 and 4, a peripheral requesting a clock source like SMCLK or ACLK will not stop
//! LPM4.5 from being entered (SLAU445I Table 3-2, p. 109; the HAL clears ACLKREQEN, as SLAU445I
//! 3.2.12.1, p. 109 requires).
//!
//! During LPM4.5 GPIO pins will maintain the value they had when LPM4.5 was entered but the register
//! contents are lost, so after a wake-up the GPIO pins will take on their reset values when LOCKLPM5
//! is cleared (SLAU445I 8.3.3, p. 318).

use crate::_pac;
use core::any::TypeId;
use core::arch::asm;

use crate::{
    clock::{Xt1Xin, Xt1Xout},
    device_specific::lpm::reset_all_pin_functions,
    gpio::{AlternatePin, PortNum},
    rtc::{Rtc, RtcLpm3_5ClockSrc},
    watchdog::{WatchdogSelect, Wdt},
};

/// Whether the high-side supply voltage supervisor (SVSH) stays on in the low-power modes (SVSHE,
/// SLAU445I Table 2-2, p. 91)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SvsState {
    /// SVSH is off in LPM2, LPM3, LPM4, LPM3.5 and LPM4.5, which saves power. It stays on in active
    /// mode, LPM0 and LPM1 (SLAU445I Table 2-2, p. 91; lower power: SLAU445I 2.2.4, p. 87).
    Disabled = 0,
    /// SVSH is always on (SLAU445I Table 2-2, p. 91)
    Enabled = 1,
}

#[allow(non_upper_case_globals)]
impl SvsState {
    /// The same as [`SvsState::Disabled`], under the name most PACs use
    pub const Svshe0: SvsState = SvsState::Disabled;
    /// The same as [`SvsState::Enabled`], under the name most PACs use
    pub const Svshe1: SvsState = SvsState::Enabled;
}

// Status register (SLAU445I Figure 4-9, p. 130):
// SCG1 SCG0 OSC_OFF CPU_OFF GIE N Z C
// 7    6    5       4       3   2 1 0
// The low-power modes set these bits as in SLAU445I Table 1-2, p. 39.
const SCG1:    u8 = 1 << 7;
const SCG0:    u8 = 1 << 6;
const OSC_OFF: u8 = 1 << 5;
const CPU_OFF: u8 = 1 << 4;
const GIE:     u8 = 1 << 3;

/// For each set bit in the bitmask, set the corresponding bit in the status register (SLAU445I
/// Figure 4-9, p. 130; BIS changes SR bits: SLAU445I 4.3.3, p. 130).
#[inline(always)]
fn set_sr_bits<const MASK: u8>() {
    unsafe { asm!("bis.b #{mask}, SR", mask = const MASK, options(nomem, nostack)) };
}

/// Enter a low-power mode by setting the status register bits in `MASK`. Unlike `set_sr_bits` it
/// is a compiler barrier, so memory accesses aren't moved across the sleep. With GIE in the mask,
/// interrupts are enabled in the same instruction that starts the sleep, followed by the NOP TI
/// recommends after enabling interrupts (SLAU445I 1.3.4.1, p. 34: "It is recommended to always
/// insert at least one instruction between EINT and DINT").
#[inline(always)]
fn sleep<const MASK: u8>() {
    unsafe { asm!("bis.b #{mask}, SR", "nop", mask = const MASK, options(nostack)) };
}

/// Enter Low Power Mode 0 (LPM0).
///
/// In LPM0 the CPU and MCLK are disabled (SLAU445I Table 1-2, p. 39).
///
/// Power draw in LPM0: Approx 40 uA / MHz (MSP430FR2355: SLASEC4D Table 6-1, p. 61).
///
/// Only an interrupt wakes the CPU (SLAU445I 1.4, p. 36), so interrupts must be enabled already.
/// To check a condition and then sleep without missing an interrupt that arrives in between, check
/// it with interrupts disabled and use [`enter_lpm0_with_interrupts`] instead.
#[inline(always)]
pub fn enter_lpm0() {
    const LPM0: u8 = CPU_OFF;
    sleep::<LPM0>();
}

/// Enable interrupts and enter Low Power Mode 0 (LPM0) in one instruction, as the user's guide
/// does (`BIS #GIE+CPUOFF,SR`, SLAU445I 1.4.2, p. 40). An interrupt that is already pending is
/// taken right after (SLAU445I 1.3.4, p. 33), so it can't be missed between checking a condition
/// and going to sleep.
#[inline(always)]
pub fn enter_lpm0_with_interrupts() {
    const LPM0: u8 = CPU_OFF | GIE;
    sleep::<LPM0>();
}

/// Request Low Power Mode 3 (LPM3).
///
/// LPM3 can only be reached if no peripherals have been configured to use SMCLK.
/// If any peripherals are configured to use SMCLK then LPM0 will be entered instead (SLAU445I
/// Table 1-3, p. 39).
///
/// In LPM3 the CPU, FLL, and all clocks (except ACLK) are disabled (SLAU445I Table 1-2, p. 39).
///
/// Power draw in LPM3: Approx 1.4 uA (MSP430FR2355 with the RTC counter on XT1: SLASEC4D
/// Table 6-1, p. 61).
///
/// Interrupts must be enabled already; see [`request_lpm3_with_interrupts`].
///
/// Errata: on the MSP430FR2355, MSP430FR2433 and MSP430FR2522 the transition to LPM3 or LPM4 can
/// lock up the device under some conditions (SLAZ695J CS13, PMM32; SLAZ664S CS13, PMM32; SLAZ705H
/// CS13, PMM32).
#[inline(always)]
pub fn request_lpm3() {
    const LPM3: u8 = SCG1 | SCG0 | CPU_OFF;
    sleep::<LPM3>();
}

/// Enable interrupts and request Low Power Mode 3 (LPM3) in one instruction, like
/// [`enter_lpm0_with_interrupts`].
///
/// Errata: see [`request_lpm3`] (SLAZ695J CS13, PMM32; SLAZ664S CS13, PMM32; SLAZ705H CS13,
/// PMM32).
#[inline(always)]
pub fn request_lpm3_with_interrupts() {
    const LPM3: u8 = SCG1 | SCG0 | CPU_OFF | GIE;
    sleep::<LPM3>();
}

/// Request Low Power Mode 4 (LPM4).
///
/// LPM4 can only be reached if no peripherals have been configured to use SMCLK or ACLK.
///
/// If any peripherals use SMCLK then LPM0 will be entered.
/// If no peripherals use SMCLK but at least one uses ACLK then LPM3 will be entered (SLAU445I
/// Table 1-3, p. 39).
///
/// In LPM4 the CPU, FLL, and all clocks (except optionally the very low power oscillators VLOCLK or
/// XTCLK) are disabled (SLAU445I Table 1-2, p. 39; SLASEC4D 6.2, p. 62).
///
/// Power draw in LPM4: Approx 820 nA (MSP430FR2355 without SVS: SLASEC4D Table 6-1, p. 61).
///
/// Interrupts must be enabled already; see [`request_lpm4_with_interrupts`].
///
/// Errata: see [`request_lpm3`] (SLAZ695J CS13, PMM32; SLAZ664S CS13, PMM32; SLAZ705H CS13,
/// PMM32).
#[inline(always)]
pub fn request_lpm4() {
    const LPM4: u8 = SCG1 | SCG0 | OSC_OFF | CPU_OFF;
    sleep::<LPM4>();
}

/// Enable interrupts and request Low Power Mode 4 (LPM4) in one instruction, like
/// [`enter_lpm0_with_interrupts`].
///
/// Errata: see [`request_lpm3`] (SLAZ695J CS13, PMM32; SLAZ664S CS13, PMM32; SLAZ705H CS13,
/// PMM32).
#[inline(always)]
pub fn request_lpm4_with_interrupts() {
    const LPM4: u8 = SCG1 | SCG0 | OSC_OFF | CPU_OFF | GIE;
    sleep::<LPM4>();
}

/// Enter Low Power Mode 3.5 (LPM3.5).
///
/// In LPM3.5 everything except the backup memory, the RTC and its clock (VLOCLK or XT1) are
/// disabled (SLASEC4D Table 6-1, p. 61 to p. 62; SLAU445I 15.2.2, p. 417). The only enabled
/// interrupts are from the RTC, I/O pins, an LF crystal fault, the RST pin, or a power cycle
/// (SLAU445I 1.4, p. 37).
///
/// I/O pins have their state latched while in LPM3.5, but the IO register values are reset on
/// wake-up (SLAU445I 8.3.3, p. 318).
/// If XT1 clocks the RTC, use [`Pmm::new_locked`](crate::pmm::Pmm::new_locked) after the wake-up
/// to keep it running (SLAU445I 1.4.3.3, p. 42).
///
/// **Waking up from LPM3.5 requires a full system reset** (SLAU445I 1.4.3.2, p. 42: "Any exit
/// from LPMx.5 causes a BOR").
///
/// Power draw in LPM3.5: Approx 620 nA (MSP430FR2355 with the RTC counter on XT1: SLASEC4D
/// Table 6-1, p. 61).
#[inline(always)]
pub fn enter_lpm3_5<MODE: WatchdogSelect, SRC: RtcLpm3_5ClockSrc>(
    wdt: Wdt<MODE>,
    _rtc: Rtc<SRC>,
    svs: SvsState,
) -> ! {
    lpm3_5(wdt, svs);
}

/// Enter LPM3.5 without providing a correctly configured RTC (because it has already been configured in a prior iteration).
/// # Safety
/// If the RTC was not correctly configured previously then the system will not enter LPM3.5
/// (SLAU445I 1.4.3.1, p. 41: "The device enters LPM4.5 if none of the modules that are connected to
/// the RTC LDO are enabled").
#[inline(always)]
pub unsafe fn enter_lpm3_5_unchecked<MODE: WatchdogSelect>(wdt: Wdt<MODE>, svs: SvsState) -> ! {
    lpm3_5(wdt, svs)
}

fn lpm3_5<MODE: WatchdogSelect>(wdt: Wdt<MODE>, svs: SvsState) -> ! {
    // Every pin returns to GPIO, except the XT1 pins while XT1 is in use, so it can keep
    // clocking the RTC (SLAU445I 1.4.3.1, p. 41, step 2)
    reset_all_pin_functions(KeepXt1Pins::in_use());
    enter_lpmx_5(wdt, svs)
}

/// Enter Low Power Mode 4.5 (LPM4.5).
///
/// In LPM4.5 *everything* is disabled (SLAU445I Table 1-2, p. 39). The only available interrupt
/// sources are from I/O pins, the RST pin, or a power cycle (SLAU445I 1.4, p. 37).
///
/// I/O pins have their state latched while in LPM4.5, but the register values are reset on
/// wake-up (SLAU445I 8.3.3, p. 318).
///
/// **Waking up from LPM4.5 requires a full system reset** (SLAU445I 1.4.3.2, p. 42: "Any exit
/// from LPMx.5 causes a BOR").
///
/// Power draw in LPM4.5: Approx 42 nA (MSP430FR2355 without SVS: SLASEC4D Table 6-1, p. 61).
#[inline]
pub fn enter_lpm4_5<MODE: WatchdogSelect>(wdt: Wdt<MODE>, rtc_reg: _pac::Rtc, svs: SvsState) -> ! {
    // Disable RTC (RTCSS = 00b, no clock: SLAU445I Table 15-2, p. 420), so the device enters
    // LPM4.5 rather than LPM3.5 (SLAU445I 1.4.3.1, p. 41)
    unsafe { rtc_reg.rtcctl().clear_bits(|w| w.rtcss().disabled()) };

    // LPM4.5 stops every oscillator, so every pin returns to GPIO, the XT1 pins included
    // (SLAU445I 1.4.3.1, p. 41, step 2)
    reset_all_pin_functions(KeepXt1Pins::NONE);
    enter_lpmx_5(wdt, svs)
}

/// The XT1 pins to leave in their XT1 function when entering LPMx.5 (SLAU445I 1.4.3.1, p. 41,
/// step 2)
#[derive(Clone, Copy)]
pub(crate) struct KeepXt1Pins {
    xin: bool,
    xout: bool,
}

impl KeepXt1Pins {
    const NONE: Self = Self { xin: false, xout: false };

    /// The XT1 pins currently in their XT1 function. XT1 is in use when XIN is selected for it;
    /// XOUT only belongs to XT1 in crystal mode and may be a GPIO in bypass mode
    /// (SLAU445I 3.2.4, p. 103).
    fn in_use() -> Self {
        let xin = Xt1Xin::<()>::function_matches_type();
        Self { xin, xout: xin && Xt1Xout::<()>::function_matches_type() }
    }

    /// Bit mask of the kept pins on `PORT`
    fn mask_on<PORT: PortNum + 'static>(self) -> u8 {
        fn pin_mask<PIN: AlternatePin, PORT: 'static>(keep: bool) -> u8
        where
            PIN::Port: 'static,
        {
            if keep && TypeId::of::<PIN::Port>() == TypeId::of::<PORT>() { PIN::MASK } else { 0 }
        }
        pin_mask::<Xt1Xin<()>, PORT>(self.xin) | pin_mask::<Xt1Xout<()>, PORT>(self.xout)
    }
}

/// Return every pin of `PORT` to GPIO (PxSEL0 and PxSEL1 cleared), except the XT1 pins in `keep`
/// (SLAU445I 1.4.3.1, p. 41, step 2)
pub(crate) fn reset_pin_functions<PORT: PortNum + 'static>(keep: KeepXt1Pins) {
    let keep = keep.mask_on::<PORT>();
    let port = unsafe { PORT::steal() };
    // Clearing leaves only the bits in the mask set
    port.pxsel0_clear(keep);
    port.pxsel1_clear(keep);
}

/// Define `reset_all_pin_functions()` in a device's `lpm` module, from the list of the device's
/// ports
macro_rules! reset_all_pin_functions_impl {
    ($($port:ident),+ $(,)?) => {
        /// Return every pin of the device to GPIO before LPMx.5, except the XT1 pins in `keep`
        /// (SLAU445I 1.4.3.1, p. 41, step 2)
        pub(crate) fn reset_all_pin_functions(keep: $crate::lpm::KeepXt1Pins) {
            $($crate::lpm::reset_pin_functions::<$crate::gpio::$port>(keep);)+
        }
    };
}
pub(crate) use reset_all_pin_functions_impl;

/// Configuration common to LPM3.5 and 4.5, following the entry steps of SLAU445I 1.4.3.1, p. 41
fn enter_lpmx_5<MODE: WatchdogSelect>(mut wdt: Wdt<MODE>, svs: SvsState) -> ! {
    // Take peripherals. Execution won't return from this fn.
    let regs = unsafe { _pac::Peripherals::steal() };

    // Pause WDT (SLAU445I 1.4.3.1, p. 41, step 7: with the WDT in watchdog mode "the device does
    // not enter LPMx.5")
    wdt.pause();

    // A module that still requests ACLK keeps the device out of LPMx.5 (SLAU445I 3.2.12.1, p. 109;
    // ACLKREQEN: SLAU445I Table 3-12, p. 123)
    unsafe { regs.cs.csctl8().clear_bits(|w| w.aclkreqen().clear_bit()) };
    // The low-power REFO mode must be switched off before LPMx.5, or it draws extra current
    // (SLAU445I Table 3-7, p. 116)
    #[cfg(feature = "enhanced_cs")]
    unsafe { regs.cs.csctl3().clear_bits(|w| w.refolp().clear_bit()) };

    // Clear GIE (SLAU445I 1.4.3.1, p. 41, step 8)
    let interrupts_were_enabled = msp430::register::sr::read().gie();
    msp430::interrupt::disable();

    // Write PMM password to get PMM control regs
    // Set PMMREGOFF
    // (SLAU445I 1.4.3.1, p. 41, steps 9a to 9c; PMMPW, SVSHE and PMMREGOFF: SLAU445I Table 2-2,
    // p. 91)
    const PASSWORD: u8 = 0xA5;
    regs.pmm.pmmctl0().write(|w| unsafe { w
        .pmmpw().bits(PASSWORD)
        .svshe().bit(svs == SvsState::Enabled)
        .pmmregoff().set_bit()
    });

    // Write incorrect password to PMM to lock
    // Only write to the upper byte of PMMCTL0
    // (SLAU445I 1.4.3.1, p. 41, step 9d; a word write with a wrong password causes a PUC:
    // SLAU445I 2.3, p. 90)
    let pmmctl0_h = (regs.pmm.pmmctl0().as_ptr() as *mut u8).wrapping_add(1);
    unsafe { pmmctl0_h.write_volatile(0) };

    // Enter LPMx.5 with CPUOFF, OSCOFF, SCG0 and SCG1 (SLAU445I 1.4.3.1, p. 41, step 10). If
    // interrupts were enabled, GIE is set again in the same instruction, as SLAU445I 8.3.3, p. 318
    // recommends ("TI also recommends setting GIE = 1 before entry into LPMx.5"), although step 8
    // of SLAU445I 1.4.3.1, p. 41 clears it.
    if interrupts_were_enabled {
        const LPMX_5: u8 = SCG1 | SCG0 | OSC_OFF | CPU_OFF | GIE;
        set_sr_bits::<LPMX_5>();
    } else {
        const LPMX_5: u8 = SCG1 | SCG0 | OSC_OFF | CPU_OFF;
        set_sr_bits::<LPMX_5>();
    }

    // LPMx.5 achieved.

    // This part won't actually run, but just to appease compiler about '!'
    #[allow(clippy::empty_loop)]
    loop {}
}
