//! System control: the RST/NMI pin, the vacant memory access interrupt, the JTAG mailbox and the
//! non-maskable interrupts (NMIs)
//!
//! Begin with [`SysParts::new()`], which splits the special function registers (SFR) into these
//! functions. The reset cause and software resets are on [`Pmm`](crate::pmm::Pmm), FRAM bit error
//! handling on [`Fram`](crate::fram::Fram).
//!
//! # Non-maskable interrupts
//!
//! The general interrupt enable (GIE) doesn't mask NMIs: once a source is enabled, it interrupts even
//! with interrupts disabled. Two vectors take them (SLAU445I 1.3.1, data sheets: System Module
//! Interrupt Vector Registers):
//!
//! | Vector   | Sources                                                    | In the handler, call                  |
//! |:--------:|:----------------------------------------------------------:|:-------------------------------------:|
//! | `UNMI`   | An edge on the RST/NMI pin in NMI mode, oscillator faults  | [`take_nmi_pin_interrupt()`], [`clock::take_fault_interrupt()`](crate::clock::take_fault_interrupt) |
//! | `SYSNMI` | Vacant memory access, the JTAG mailbox, FRAM bit errors    | [`take_system_nmi()`]                 |
//!
//! While one NMI of a vector is handled, further NMIs of that vector wait until it returns. A handler
//! must clear the flag it handles, or the NMI repeats as soon as it returns.

use crate::_pac;
use core::{convert::Infallible, marker::PhantomData};

const SYSFLTE: u16 = 1 << 4; // Missing from most PACs

/// The system control functions of the special function registers (SFR)
pub struct SysParts {
    /// The RST/NMI pin, in reset mode as after a brownout reset
    pub rst_nmi_pin: RstNmiPin<ResetMode>,
    /// The vacant memory access interrupt
    pub vacant_memory: VacantMemory,
    /// The JTAG mailbox, in 16-bit mode as after reset
    pub jtag_mailbox: JtagMailbox<Mode16>,
}

impl SysParts {
    /// Split the special function registers into their system control functions. The RST/NMI pin and
    /// the JTAG mailbox keep their current configuration.
    #[inline]
    pub fn new(_sfr: _pac::Sfr) -> Self {
        SysParts {
            rst_nmi_pin: RstNmiPin(PhantomData),
            vacant_memory: VacantMemory(()),
            jtag_mailbox: JtagMailbox(PhantomData),
        }
    }
}

#[inline(always)]
fn sfr() -> &'static _pac::sfr::RegisterBlock { unsafe { &*_pac::Sfr::ptr() } }

#[inline(always)]
fn sys() -> &'static _pac::sys::RegisterBlock { unsafe { &*_pac::Sys::ptr() } }

/// Typestate for the RST/NMI pin in reset mode: a low level resets the device
pub struct ResetMode;
/// Typestate for the RST/NMI pin in NMI mode: an edge requests the user NMI
pub struct NmiMode;

/// The resistor on the RST/NMI pin (SFRRPCR.SYSRSTRE, SYSRSTUP)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RstPull {
    /// Pull-up, as after reset. The user's guide requires this or an external resistor if the pin is unused.
    Up,
    /// Pull-down
    Down,
    /// No resistor
    None,
}

/// The edge of the RST/NMI pin that requests the NMI (SFRRPCR.SYSNMIIES)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum NmiEdge {
    /// Rising edge
    Rising,
    /// Falling edge
    Falling,
}

/// The RST/NMI pin (SFRRPCR, SLAU445I 1.7). It is also the Spy-Bi-Wire data pin (SBWTDIO).
pub struct RstNmiPin<MODE>(PhantomData<MODE>);

impl<MODE> RstNmiPin<MODE> {
    /// Select the resistor on the pin.
    #[inline]
    pub fn set_pull(&mut self, pull: RstPull) {
        sfr().sfrrpcr().modify(|r, w| unsafe { w.bits(with_pull(r.bits(), pull)) });
    }

    /// Use the pin as reset input, as after a brownout reset. A low level then resets the device,
    /// right away if the pin is low.
    #[inline]
    pub fn into_reset(self, pull: RstPull) -> RstNmiPin<ResetMode> {
        disable_nmi_pin_interrupt();
        sfr().sfrrpcr().modify(|r, w| unsafe { w.bits(with_pull(r.bits(), pull) & !0b01) });
        RstNmiPin(PhantomData)
    }

    /// Use the pin as NMI input: `edge` sets the NMI flag, which requests the `UNMI` interrupt once
    /// enabled with [`RstNmiPin::enable_interrupts()`]. The pin no longer resets the device.
    #[inline]
    pub fn into_nmi(self, edge: NmiEdge, pull: RstPull) -> RstNmiPin<NmiMode> {
        let enabled = nmi_pin_interrupt_enabled();
        disable_nmi_pin_interrupt();
        let rpcr = sfr().sfrrpcr();
        let ies = match edge {
            NmiEdge::Rising => 0,
            NmiEdge::Falling => 0b10,
        };
        // Changing SYSNMIIES in NMI mode can set the flag, so set it before switching to NMI mode
        // (SFRRPCR register description)
        rpcr.modify(|r, w| unsafe { w.bits(with_pull(r.bits(), pull) & !0b10 | ies) });
        rpcr.modify(|r, w| unsafe { w.bits(r.bits() | 0b01) });
        clear_nmi_pin_flag();
        if enabled {
            enable_nmi_pin_interrupt();
        }
        RstNmiPin(PhantomData)
    }
}

impl RstNmiPin<ResetMode> {
    /// Enable or disable the digital filter that suppresses short pulses on the pin, so they don't
    /// reset the device (SFRRPCR.SYSFLTE). It is enabled after reset. See the data sheet for the
    /// shortest pulse that resets the device.
    #[inline]
    pub fn set_filter(&mut self, enabled: bool) {
        sfr().sfrrpcr().modify(|r, w| unsafe {
            w.bits(if enabled { r.bits() | SYSFLTE } else { r.bits() & !SYSFLTE })
        });
    }
}

impl RstNmiPin<NmiMode> {
    /// Change the edge that requests the NMI. This can set the NMI flag, so this clears it.
    #[inline]
    pub fn set_edge(&mut self, edge: NmiEdge) {
        let enabled = nmi_pin_interrupt_enabled();
        disable_nmi_pin_interrupt();
        sfr().sfrrpcr().modify(|r, w| unsafe {
            w.bits(match edge {
                NmiEdge::Rising => r.bits() & !0b10,
                NmiEdge::Falling => r.bits() | 0b10,
            })
        });
        clear_nmi_pin_flag();
        if enabled {
            enable_nmi_pin_interrupt();
        }
    }

    /// Request the `UNMI` interrupt on the selected edge (NMIIE). Clear the flag with
    /// [`take_nmi_pin_interrupt()`] in the handler.
    #[inline]
    pub fn enable_interrupts(&mut self) { enable_nmi_pin_interrupt(); }

    /// Stop requesting the `UNMI` interrupt (NMIIE).
    #[inline]
    pub fn disable_interrupts(&mut self) { disable_nmi_pin_interrupt(); }
}

#[inline(always)]
fn with_pull(rpcr: u16, pull: RstPull) -> u16 {
    // SYSRSTRE is bit 3, SYSRSTUP bit 2
    let rpcr = rpcr & !0b1100;
    match pull {
        RstPull::Up => rpcr | 0b1100,
        RstPull::Down => rpcr | 0b1000,
        RstPull::None => rpcr,
    }
}

#[inline(always)]
fn nmi_pin_interrupt_enabled() -> bool { sfr().sfrie1().read().nmiie().bit_is_set() }

#[inline(always)]
fn enable_nmi_pin_interrupt() { unsafe { sfr().sfrie1().set_bits(|w| w.nmiie().set_bit()) } }

#[inline(always)]
fn disable_nmi_pin_interrupt() { unsafe { sfr().sfrie1().clear_bits(|w| w.nmiie().clear_bit()) } }

#[inline(always)]
fn clear_nmi_pin_flag() { unsafe { sfr().sfrifg1().clear_bits(|w| w.nmiifg().clear_bit()) } }

/// Returns `true`, and clears the flag (NMIIFG), if an edge on the RST/NMI pin requested the user NMI.
/// Call this in the `UNMI` handler, where an oscillator fault can request it too.
#[inline]
pub fn take_nmi_pin_interrupt() -> bool {
    let sfr = sfr();
    let requested = sfr.sfrie1().read().nmiie().bit_is_set() && sfr.sfrifg1().read().nmiifg().bit_is_set();
    if requested {
        clear_nmi_pin_flag();
    }
    requested
}

/// The vacant memory access interrupt (SFRIE1.VMAIE)
///
/// Vacant memory is address space with nothing behind it. Reads return 3FFFh, and executing from it
/// runs `JMP $`, which hangs the CPU (SLAU445I 1.9.2). This catches such accesses, for example
/// through a corrupted pointer, with the `SYSNMI` interrupt.
pub struct VacantMemory(());

impl VacantMemory {
    /// Request the `SYSNMI` interrupt when the CPU accesses vacant memory. [`take_system_nmi()`]
    /// then returns [`SystemNmi::VacantMemoryAccess`].
    #[inline]
    pub fn enable_interrupts(&mut self) {
        unsafe { sfr().sfrifg1().clear_bits(|w| w.vmaifg().clear_bit()) };
        unsafe { sfr().sfrie1().set_bits(|w| w.vmaie().set_bit()) };
    }

    /// Stop requesting the `SYSNMI` interrupt on vacant memory accesses.
    #[inline]
    pub fn disable_interrupts(&mut self) {
        unsafe { sfr().sfrie1().clear_bits(|w| w.vmaie().clear_bit()) };
    }
}

/// Typestate for 16-bit JTAG mailbox transfers, through SYSJMBI0 and SYSJMBO0
pub struct Mode16;
/// Typestate for 32-bit JTAG mailbox transfers, through SYSJMBI0-1 and SYSJMBO0-1
pub struct Mode32;

/// The JTAG mailbox, which exchanges messages with a debugger through the JTAG or Spy-Bi-Wire
/// connection (SLAU445I 1.10). The debugger needs to support it.
pub struct JtagMailbox<MODE>(PhantomData<MODE>);

impl<MODE> JtagMailbox<MODE> {
    /// Request the `SYSNMI` interrupt when a message from the debugger arrives (JMBINIE).
    /// [`take_system_nmi()`] then returns [`SystemNmi::JtagMailboxIn`].
    #[inline]
    pub fn enable_rx_interrupts(&mut self) { unsafe { sfr().sfrie1().set_bits(|w| w.jmbinie().set_bit()) } }

    /// Stop requesting the `SYSNMI` interrupt for messages from the debugger (JMBINIE).
    #[inline]
    pub fn disable_rx_interrupts(&mut self) { unsafe { sfr().sfrie1().clear_bits(|w| w.jmbinie().clear_bit()) } }

    /// Request the `SYSNMI` interrupt when the debugger has read the outgoing message, so the next one
    /// can be written (JMBOUTIE). [`take_system_nmi()`] then returns [`SystemNmi::JtagMailboxOut`].
    /// The mailbox starts out empty, so this requests the interrupt right away until a message is written.
    #[inline]
    pub fn enable_tx_interrupts(&mut self) { unsafe { sfr().sfrie1().set_bits(|w| w.jmboutie().set_bit()) } }

    /// Stop requesting the `SYSNMI` interrupt for outgoing messages (JMBOUTIE).
    #[inline]
    pub fn disable_tx_interrupts(&mut self) { unsafe { sfr().sfrie1().clear_bits(|w| w.jmboutie().clear_bit()) } }
}

impl JtagMailbox<Mode16> {
    /// Switch to 32-bit transfers (JMBMODE). Read and write any partial message first, the user's
    /// guide warns that it can be lost otherwise.
    #[inline]
    pub fn into_32bit(self) -> JtagMailbox<Mode32> {
        unsafe { sys().sysjmbc().set_bits(|w| w.jmbmode().set_bit()) };
        JtagMailbox(PhantomData)
    }

    /// Send `msg` to the debugger. Returns `WouldBlock` until the debugger has read the previous
    /// message (JMBOUT0FG).
    #[inline]
    pub fn write(&mut self, msg: u16) -> nb::Result<(), Infallible> {
        let sys = sys();
        if sys.sysjmbc().read().jmbout0fg().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        sys.sysjmbo0().write(|w| unsafe { w.bits(msg) });
        Ok(())
    }

    /// A message from the debugger, or `WouldBlock` if none has arrived (JMBIN0FG). Reading it
    /// clears the flag.
    #[inline]
    pub fn read(&mut self) -> nb::Result<u16, Infallible> {
        let sys = sys();
        if sys.sysjmbc().read().jmbin0fg().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(sys.sysjmbi0().read().bits())
    }
}

impl JtagMailbox<Mode32> {
    /// Switch to 16-bit transfers (JMBMODE). Read and write any partial message first, the user's
    /// guide warns that it can be lost otherwise.
    #[inline]
    pub fn into_16bit(self) -> JtagMailbox<Mode16> {
        unsafe { sys().sysjmbc().clear_bits(|w| w.jmbmode().clear_bit()) };
        JtagMailbox(PhantomData)
    }

    /// Send `msg` to the debugger, the low half through SYSJMBO0 and the high half through SYSJMBO1.
    /// Returns `WouldBlock` until the debugger has read the previous message (JMBOUT0FG, JMBOUT1FG).
    #[inline]
    pub fn write(&mut self, msg: u32) -> nb::Result<(), Infallible> {
        let sys = sys();
        let jmbc = sys.sysjmbc().read();
        if jmbc.jmbout0fg().bit_is_clear() || jmbc.jmbout1fg().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        sys.sysjmbo0().write(|w| unsafe { w.bits(msg as u16) });
        sys.sysjmbo1().write(|w| unsafe { w.bits((msg >> 16) as u16) });
        Ok(())
    }

    /// A message from the debugger, or `WouldBlock` if none has arrived (JMBIN0FG, JMBIN1FG). The
    /// low half comes from SYSJMBI0 and the high half from SYSJMBI1. Reading it clears the flags.
    #[inline]
    pub fn read(&mut self) -> nb::Result<u32, Infallible> {
        let sys = sys();
        let jmbc = sys.sysjmbc().read();
        if jmbc.jmbin0fg().bit_is_clear() || jmbc.jmbin1fg().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        let low = sys.sysjmbi0().read().bits() as u32;
        let high = sys.sysjmbi1().read().bits() as u32;
        Ok(high << 16 | low)
    }
}

/// The sources of the system NMI, in priority order (SYSSNIV, data sheets: System Module Interrupt
/// Vector Registers)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SystemNmi {
    /// SVS low-power reset entry
    SvsLowPowerResetEntry,
    /// The FRAM detected a bit error it couldn't correct, see
    /// [`Fram::set_uncorrectable_bit_error_action()`](crate::fram::Fram::set_uncorrectable_bit_error_action)
    FramUncorrectableBitError,
    /// The CPU accessed vacant memory, see [`VacantMemory`]
    VacantMemoryAccess,
    /// A message from the debugger arrived in the JTAG mailbox
    JtagMailboxIn,
    /// The debugger read the outgoing JTAG mailbox message
    JtagMailboxOut,
    /// The FRAM detected and corrected a bit error, see
    /// [`Fram::enable_correctable_bit_error_interrupts()`](crate::fram::Fram::enable_correctable_bit_error_interrupts)
    FramCorrectableBitError,
    /// A value the data sheets list as reserved
    Reserved(u16),
}

/// Returns the highest-priority pending system NMI source and clears its flag (SYSSNIV), or `None`
/// if none is pending. Call this in the `SYSNMI` handler, until it returns `None` if several
/// sources are enabled.
#[inline]
pub fn take_system_nmi() -> Option<SystemNmi> {
    match sys().syssniv().read().bits() {
        0x00 => None,
        0x02 => Some(SystemNmi::SvsLowPowerResetEntry),
        0x04 => Some(SystemNmi::FramUncorrectableBitError),
        0x12 => Some(SystemNmi::VacantMemoryAccess),
        0x14 => Some(SystemNmi::JtagMailboxIn),
        0x16 => Some(SystemNmi::JtagMailboxOut),
        0x18 => Some(SystemNmi::FramCorrectableBitError),
        other => Some(SystemNmi::Reserved(other)),
    }
}
