//! Infrared modulation
//!
//! The system module (SYS) combines two timer outputs and a data signal into a modulated signal on eUSCI_A0's
//! TXD pin, to drive an infrared LED (SLAU445I 1.12, SYSCFG1; data sheets: timer signal connections):
//!
//! | Device       | First input | Second input | Output (UCA0TXD)         |
//! |:------------:|:-----------:|:------------:|:------------------------:|
//! | MSP430FR2x5x | TB0 CCR2    | TB1 CCR2     | `P1.7`                   |
//! | MSP430FR247x | TA0 CCR2    | TA1 CCR2     | `P1.4`, default mapping  |
//! | MSP430FR2433 | TA0 CCR2    | TA1 CCR2     | `P1.4`                   |
//! | MSP430FR25x2 | TA0 CCR2    | TA1 CCR2     | `P2.0`, remapped mapping |
//!
//! Only that one TXD pin carries the modulated signal, so eUSCI_A0 has to use its pin mapping, [`IrMapping`]. On
//! the MSP430FR247x the output doesn't follow eUSCI_A0 to its remapped pins (measured on an MSP430FR2476). The
//! MSP430FR25x2 output is the one its data sheet shows, not tested on hardware.
//!
//! Set both timers up for PWM and turn their CCR2 outputs into modulator inputs with
//! [`PwmUninit::into_ir_input`](crate::pwm::PwmUninit::into_ir_input). The data comes from software
//! ([`IrModulator::with_software_data`]), or from eUSCI_A0, whose UART then sends its characters through the
//! modulator ([`IrModulator::with_uart_data`]).
//!
//! In ASK mode the first input is the carrier and the second the envelope; in FSK mode they are the two
//! frequencies (user's guide). Measured on an MSP430FR2476:
//!
//! - ASK: the output is the carrier while the envelope and the data differ (`carrier & (envelope ^ data)`). With
//!   the envelope running at the bit rate, that sends the data Manchester coded, as RC-5 does.
//! - FSK: the output is the second input while the data is 1, and the first input while it is 0.
//!
//! The output inverts with the `inverted` argument (IRPSEL).

use crate::{_pac, serial::{SerialUsci, Tx}};
use core::marker::PhantomData;

pub use crate::device_specific::ir::{IrMapping, IrUsci};

/// Marker trait for the timers whose CCR2 output can feed the modulator
pub trait IrInputTimer {}
/// Marker trait for the timer whose CCR2 output is the first modulator input: TB0 on the MSP430FR2x5x, TA0 on the
/// other devices
pub trait IrFirstTimer: IrInputTimer {}
/// Marker trait for the timer whose CCR2 output is the second modulator input: TB1 on the MSP430FR2x5x, TA1 on the
/// other devices
pub trait IrSecondTimer: IrInputTimer {}

/// Marker trait for the pin that carries the modulated signal: eUSCI_A0's TXD pin in its eUSCI function, in
/// the pin mapping [`IrMapping`]
pub trait IrOutputPin {}

/// A timer's CCR2 output feeding the modulator, see [`PwmUninit::into_ir_input`](crate::pwm::PwmUninit::into_ir_input)
pub struct IrInput<T>(pub(crate) PhantomData<T>);

/// How the modulator combines its inputs (IRMSEL)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum IrMode {
    /// Amplitude-shift keying: the first input is the carrier, the second the envelope
    Ask,
    /// Frequency-shift keying: the two inputs are the two frequencies
    Fsk,
}

/// Typestate for a modulator whose data is set from software
pub struct SoftwareData;
/// Typestate for a modulator whose data comes from eUSCI_A0
pub struct UartData;

// SYSCFG1 bits (SLAU445I SYSCFG1). Other bits of SYSCFG1 belong to other functions on some devices.
const IREN: u16 = 1 << 0;
const IRPSEL: u16 = 1 << 1;
const IRMSEL: u16 = 1 << 2;
const IRDSSEL: u16 = 1 << 3;
const IRDATA: u16 = 1 << 4;

#[inline(always)]
fn sys() -> &'static _pac::sys::RegisterBlock { unsafe { &*_pac::Sys::ptr() } }

/// The infrared modulator. Disable it with [`IrModulator::disable`].
pub struct IrModulator<DATA>(PhantomData<DATA>);

#[inline(always)]
fn enable(mode: IrMode, inverted: bool, software_data: bool) {
    let mut bits = IREN;
    if mode == IrMode::Fsk {
        bits |= IRMSEL;
    }
    if inverted {
        bits |= IRPSEL;
    }
    if software_data {
        bits |= IRDSSEL;
    }
    let sys = sys();
    unsafe { sys.syscfg1().clear_bits(|w| w.bits(!(IREN | IRPSEL | IRMSEL | IRDSSEL | IRDATA))) };
    unsafe { sys.syscfg1().set_bits(|w| w.bits(bits)) };
}

impl IrModulator<SoftwareData> {
    /// Enable the modulator with its data set from software, see [`IrModulator::set_data`]. The output inverts
    /// with `inverted` (IRPSEL). `pin` is eUSCI_A0's TXD pin, whose function the modulator takes over; this also
    /// selects eUSCI_A0's pin mapping [`IrMapping`].
    #[inline]
    pub fn with_software_data<F, S, P>(
        _first: &IrInput<F>,
        _second: &IrInput<S>,
        mode: IrMode,
        inverted: bool,
        _pin: P,
    ) -> Self
    where
        F: IrFirstTimer,
        S: IrSecondTimer,
        P: IrOutputPin,
    {
        <IrUsci as SerialUsci<IrMapping>>::configure_pin_mapping();
        enable(mode, inverted, true);
        IrModulator(PhantomData)
    }

    /// Set the data bit (IRDATA).
    #[inline]
    pub fn set_data(&mut self, high: bool) {
        let sys = sys();
        if high {
            unsafe { sys.syscfg1().set_bits(|w| w.bits(IRDATA)) };
        } else {
            unsafe { sys.syscfg1().clear_bits(|w| w.bits(!IRDATA)) };
        }
    }
}

impl IrModulator<UartData> {
    /// Enable the modulator with the characters eUSCI_A0's UART sends as data. The output inverts with
    /// `inverted` (IRPSEL). The UART has to use the pin mapping [`IrMapping`], whose TXD pin carries the output.
    #[inline]
    pub fn with_uart_data<F, S>(
        _first: &IrInput<F>,
        _second: &IrInput<S>,
        mode: IrMode,
        inverted: bool,
        _tx: &Tx<IrUsci, IrMapping>,
    ) -> Self
    where
        F: IrFirstTimer,
        S: IrSecondTimer,
    {
        enable(mode, inverted, false);
        IrModulator(PhantomData)
    }
}

impl<DATA> IrModulator<DATA> {
    /// Disable the modulator, so the pin carries the eUSCI_A0 signal again (IREN).
    #[inline]
    pub fn disable(self) {
        unsafe { sys().syscfg1().clear_bits(|w| w.bits(!(IREN | IRDSSEL | IRDATA))) };
    }
}
