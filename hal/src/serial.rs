//! Serial UART
//!
//! The eUSCI_A peripherals can be used as serial UARTs. Pins used (pins with `RemappedMapping` in brackets):
//!
//! | Device       | eUSCI | TXD             | RXD             | External clock  |
//! |:-------------|:-----:|:---------------:|:---------------:|:---------------:|
//! | MSP430FR2x5x | A0    | `P1.7`          | `P1.6`          | `P1.5`          |
//! | MSP430FR2x5x | A1    | `P4.3`          | `P4.2`          | `P4.1`          |
//! | MSP430FR2433 | A0    | `P1.4`          | `P1.5`          | `P1.6`          |
//! | MSP430FR2433 | A1    | `P2.6`          | `P2.5`          | `P2.4`          |
//! | MSP430FR247x | A0    | `P1.4` (`P5.2`) | `P1.5` (`P5.1`) | `P1.6` (`P5.0`) |
//! | MSP430FR247x | A1    | `P2.6`          | `P2.5`          | `P2.4`          |
//! | MSP430FR25x2 | A0    | `P1.4` (`P2.0`) | `P1.5` (`P2.1`) | `P1.6`          |
//!
//! On the MSP430FR2433 the PAC exposes each eUSCI once per mode, for example `usci_a0_uart_mode` and
//! `usci_a0_spi_mode`. Both are the same hardware, so only use one of them for each eUSCI.
//!
//! On the MSP430FR2x5x, eUSCI_A1's pins in alternate function 2 (`to_alternate2()`) invert the polarity of TXD and
//! RXD, and a rising edge then starts a character (data sheet, eUSCI_A1 UART polarity configurations).
//!
//! Begin configuration by calling [`SerialConfig::new()`]. After configuration, [`Rx`] and/or [`Tx`] structs are produced by
//! providing the corresponding GPIO pins.
//!
//! The [`Tx`] and [`Rx`] structs are used to send and receive bytes via serial. They implement both [`embedded-io`](embedded_io)'s
//! serial traits (which are buffer-based, blocking), and the single-byte-based non-blocking [`embedded-hal-nb`](embedded_hal_nb::serial) version.
//!
//! As the MSP430 has only a single byte buffer, `embedded-io`'s buffer-based traits can be a bit unwieldy -
//! [`emb_io::Write::write`](embedded_io::Write::write) and [`emb_io::Read::read`](embedded_io::Read::read) will
//! always only send or recieve a single byte, despite taking slices as inputs.
//!
//! For reading or writing single bytes it is recommended to use `embedded-hal-nb`'s
//! [`emb_hal_nb::Read::read`](embedded_hal_nb::serial::Read::read) and
//! [`emb_hal_nb::Write::write`](embedded_hal_nb::serial::Write::write) ([`nb::block`] can be used to make them blocking).
//!
//! For writing multiple bytes, embedded_io's [`Write::write_all`](embedded_io::Write::write_all) and
//! [`Read::read_exact`](embedded_io::Read::read_exact) methods are useful.
//!
//! Besides plain UART, [`SerialConfig::mode`] selects the multiprocessor formats, which mark address characters
//! ([`Tx::send_address`], [`Rx::set_dormant`]), and automatic baud-rate detection from a LIN break and synch
//! field. [`SerialConfig::irda`] adds IrDA encoding and decoding, and [`SerialConfig::deglitch`] sets how short a
//! pulse on RXD is ignored.
//!

#[cfg(feature = "eusci_aclk")]
use crate::clock::Aclk;
use crate::clock::{Clock, Smclk};
use crate::hw_traits::eusci::{
    EUsciUart, UartUcxStatw, UcaCtlw0, Ucssel, UCADDR_UCIDLE, UCDORM, UCSTTIFG, UCTXADDR, UCTXBRK,
    UCTXCPTIFG,
};
use crate::pin_mapping::*;
use core::convert::Infallible;
use core::fmt::Display;
use core::marker::PhantomData;
use core::num::NonZeroU32;

/// Bit order of transmit and receive
#[derive(Clone, Copy)]
pub enum BitOrder {
    /// LSB first (typically the default)
    LsbFirst,
    /// MSB first
    MsbFirst,
}

impl BitOrder {
    #[inline(always)]
    fn to_bool(self) -> bool {
        match self {
            BitOrder::LsbFirst => false,
            BitOrder::MsbFirst => true,
        }
    }
}

/// Number of bits per transaction
#[derive(Clone, Copy)]
pub enum BitCount {
    /// 8 bits
    EightBits,
    /// 7 bits
    SevenBits,
}

impl BitCount {
    #[inline(always)]
    fn to_bool(self) -> bool {
        match self {
            BitCount::EightBits => false,
            BitCount::SevenBits => true,
        }
    }
}

/// Number of stop bits at end of each byte
#[derive(Clone, Copy)]
pub enum StopBits {
    /// 1 stop bit
    OneStopBit,
    /// 2 stop bits
    TwoStopBits,
}

impl StopBits {
    #[inline(always)]
    fn to_bool(self) -> bool {
        match self {
            StopBits::OneStopBit => false,
            StopBits::TwoStopBits => true,
        }
    }
}

/// Parity bit for error checking
#[derive(Clone, Copy)]
pub enum Parity {
    /// No parity
    NoParity,
    /// Odd parity
    OddParity,
    /// Even parity
    EvenParity,
}

impl Parity {
    #[inline(always)]
    fn ucpen(self) -> bool {
        match self {
            Parity::NoParity => false,
            _ => true,
        }
    }

    #[inline(always)]
    fn ucpar(self) -> bool {
        match self {
            Parity::OddParity => false,
            Parity::EvenParity => true,
            _ => false,
        }
    }
}

/// Loopback settings
#[derive(Clone, Copy)]
pub enum Loopback {
    /// No loopback
    NoLoop,
    /// Tx feeds into Rx
    Loopback,
}

impl Loopback {
    #[inline(always)]
    fn to_bool(self) -> bool {
        match self {
            Loopback::NoLoop => false,
            Loopback::Loopback => true,
        }
    }
}

/// How short a pulse on RXD the receiver ignores (UCGLIT)
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub enum UartDeglitch {
    /// About 2 ns
    _2ns = 0,
    /// About 50 ns
    _50ns = 1,
    /// About 100 ns
    _100ns = 2,
    /// About 200 ns, as after reset
    #[default]
    _200ns = 3,
}

/// The length of the break delimiter sent before the synch field in automatic baud-rate mode (UCDELIM)
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub enum BreakDelimiter {
    /// 1 bit time
    #[default]
    _1Bit = 0,
    /// 2 bit times
    _2Bits = 1,
    /// 3 bit times
    _3Bits = 2,
    /// 4 bit times
    _4Bits = 3,
}

/// The UART mode (UCMODE, user's guide 22.3.3 and 22.3.4)
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub enum UartMode {
    /// Plain UART
    #[default]
    Uart,
    /// Idle-line multiprocessor format: the first character after an idle line of 10 or more bits is an
    /// address
    IdleLineMultiprocessor,
    /// Address-bit multiprocessor format: each character has an extra bit that marks addresses
    AddressBitMultiprocessor,
    /// Automatic baud-rate detection, as in LIN: a received break and synch field (0x55) set the baud rate.
    /// For LIN, use 8 data bits, LSB first, no parity and one stop bit. The receiver measures with the
    /// transmitter's baud-rate generator, so it can't measure a break and synch field it sends itself, in
    /// loopback for example.
    AutoBaud {
        /// The length of the break delimiter [`Tx::send_break`] sends
        delimiter: BreakDelimiter,
    },
}

impl UartMode {
    #[inline(always)]
    fn ucmode(self) -> u8 {
        match self {
            UartMode::Uart => 0b00,
            UartMode::IdleLineMultiprocessor => 0b01,
            UartMode::AddressBitMultiprocessor => 0b10,
            UartMode::AutoBaud { .. } => 0b11,
        }
    }
}

/// The clock the IrDA transmit pulse length is counted in (UCIRTXCLK)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum IrdaClock {
    /// The baud-rate clock, BRCLK
    Brclk,
    /// 16 times the baud rate (BITCLK16). This needs oversampling, which the baud-rate calculation uses when
    /// the clock is at least 16 times the baud rate; otherwise BRCLK is used.
    BitClk16,
}

/// IrDA encoding and decoding (UCAxIRCTL, user's guide 22.3.5)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct IrdaConfig {
    /// Transmit pulse length: (tx_pulse + 1) / (2 * pulse clock), with `tx_pulse` from 0 to 63 (UCIRTXPL)
    pub tx_pulse: u8,
    /// The clock the transmit pulse length is counted in
    pub pulse_clock: IrdaClock,
    /// Ignore received pulses shorter than (filter + 4) / (2 * pulse clock), with the filter from 0 to 63, or
    /// `None` to accept all (UCIRRXFE, UCIRRXFL)
    pub rx_filter: Option<u8>,
    /// The transceiver gives a low pulse for light, instead of a high pulse (UCIRRXPL)
    pub rx_inverted: bool,
}

impl IrdaConfig {
    /// The standard IrDA pulse of 3/16 of a bit time, from 6 half periods of BITCLK16
    pub const fn standard() -> Self {
        IrdaConfig { tx_pulse: 5, pulse_clock: IrdaClock::BitClk16, rx_filter: None, rx_inverted: false }
    }

    #[inline(always)]
    fn irctl(&self) -> u16 {
        let filter = match self.rx_filter {
            Some(len) => (len.min(63) as u16) << 10 | 1 << 8,
            None => 0,
        };
        filter
            | (self.rx_inverted as u16) << 9
            | (self.tx_pulse.min(63) as u16) << 2
            | ((self.pulse_clock == IrdaClock::BitClk16) as u16) << 1
            | 1
    }
}

/// The highest-priority pending UART interrupt among the enabled ones, as read from UCAxIV by
/// `interrupt_source()`
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartVector {
    /// No interrupt pending
    None,
    /// A character was received (UCRXIFG). Reading it clears the flag.
    RxBufFull,
    /// The Tx buffer is empty (UCTXIFG). Writing to it clears the flag.
    TxBufEmpty,
    /// A start bit was received (UCSTTIFG)
    StartBit,
    /// A character was sent completely (UCTXCPTIFG)
    TxComplete,
}

#[inline(always)]
fn read_uart_iv<USCI: EUsciUart>(usci: &USCI) -> UartVector {
    match usci.iv_rd() {
        0x02 => UartVector::RxBufFull,
        0x04 => UartVector::TxBufEmpty,
        0x06 => UartVector::StartBit,
        0x08 => UartVector::TxComplete,
        _ => UartVector::None,
    }
}

/// Marks a USCI type that can be used as a serial UART
pub trait SerialUsci<M: PinMap = DefaultMapping>: EUsciUart {
    /// Pin used for serial UCLK
    type ClockPin;
    /// Pin used for Tx
    type TxPin;
    /// Pin used for Rx
    type RxPin;

    /// Additional configuration
    #[inline(always)]
    fn configure_pin_mapping() {}
}

// The pin's alternate function defaults to Alternate1
macro_rules! impl_serial_pin {
    ($struct_name: ident, $port: ty, $pin: ty) => {
        impl_serial_pin!($struct_name, $port, $pin, Alternate1);
    };
    ($struct_name: ident, $port: ty, $pin: ty, $alt: ident) => {
        impl<DIR> From<Pin<$port, $pin, $alt<DIR>>> for $struct_name {
            #[inline(always)]
            fn from(_val: Pin<$port, $pin, $alt<DIR>>) -> Self { $struct_name }
        }
    };
}
pub(crate) use impl_serial_pin;

/// Typestate for a serial interface with an unspecified clock source
pub struct NoClockSet {
    baudrate: NonZeroU32,
}

/// Typestate for a serial interface with a specified clock source
pub struct ClockSet {
    baud_config: BaudConfig,
    clksel: Ucssel,
}

/// Builder object for configuring a serial UART
///
/// Once the clock source has been selected, the builder can be converted into pins that can
/// transmit or received bytes via a serial connection.
pub struct SerialConfig<USCI, S, M: PinMap = DefaultMapping>
where USCI: SerialUsci<M>
{
    usci: USCI,
    order: BitOrder,
    cnt: BitCount,
    stopbits: StopBits,
    parity: Parity,
    loopback: Loopback,
    mode: UartMode,
    deglitch: UartDeglitch,
    irda: Option<IrdaConfig>,
    break_interrupts: bool,
    state: S,
    _map: PhantomData<M>,
}

macro_rules! serial_config {
    ($conf:expr, $state:expr) => {
        SerialConfig::<_, _, _> {
            usci: $conf.usci,
            order: $conf.order,
            cnt: $conf.cnt,
            stopbits: $conf.stopbits,
            parity: $conf.parity,
            loopback: $conf.loopback,
            mode: $conf.mode,
            deglitch: $conf.deglitch,
            irda: $conf.irda,
            break_interrupts: $conf.break_interrupts,
            state: $state,
            _map: core::marker::PhantomData,
        }
    };
}

impl<USCI, S, M> SerialConfig<USCI, S, M>
where
    USCI: SerialUsci<M>,
    M: PinMap,
{
    /// Select the UART mode: plain UART, a multiprocessor format or automatic baud-rate detection.
    #[inline]
    pub fn mode(mut self, mode: UartMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set how short a pulse on RXD the receiver ignores. After reset that's about 200 ns.
    #[inline]
    pub fn deglitch(mut self, deglitch: UartDeglitch) -> Self {
        self.deglitch = deglitch;
        self
    }

    /// Encode transmitted and decode received bits as IrDA pulses, for an infrared transceiver.
    #[inline]
    pub fn irda(mut self, irda: IrdaConfig) -> Self {
        self.irda = Some(irda);
        self
    }

    /// Report received breaks: a break then reads as [`RecvError::Break`] (UCBRKIE). In automatic baud-rate
    /// mode, the break and synch field are reported that way.
    #[inline]
    pub fn break_interrupts(mut self) -> Self {
        self.break_interrupts = true;
        self
    }
}

impl<USCI, M> SerialConfig<USCI, NoClockSet, M>
where
    USCI: SerialUsci<M>,
    M: PinMap,
{
    /// Create a new serial configuration using a EUSCI peripheral
    #[inline]
    pub fn new(
        usci: USCI,
        order: BitOrder,
        cnt: BitCount,
        stopbits: StopBits,
        parity: Parity,
        loopback: Loopback,
        baudrate: u32,
    ) -> Self {
        const ONE: NonZeroU32 = NonZeroU32::new(1).unwrap();
        SerialConfig {
            order,
            cnt,
            stopbits,
            parity,
            loopback,
            usci,
            mode: UartMode::Uart,
            deglitch: UartDeglitch::_200ns,
            irda: None,
            break_interrupts: false,
            state: NoClockSet { baudrate: NonZeroU32::new(baudrate).unwrap_or(ONE) },
            _map: PhantomData,
        }
    }

    /// Configure serial UART to use external UCLK, passing in the appropriately configured pin
    /// used as the clock signal as well as the frequency of the clock.
    ///
    /// # Panics
    ///
    /// If the baud rate is above a third of the clock frequency, the most the eUSCI supports (user's guide).
    #[inline(always)]
    pub fn use_uclk<P: Into<USCI::ClockPin>>(
        self,
        _clk_pin: P,
        freq: u32,
    ) -> SerialConfig<USCI, ClockSet, M> {
        serial_config!(
            self,
            ClockSet {
                baud_config: calculate_baud_config(freq, self.state.baudrate),
                clksel: Ucssel::Uclk,
            }
        )
    }

    #[cfg(feature = "eusci_aclk")]
    /// Configure serial UART to use ACLK.
    ///
    /// # Panics
    ///
    /// If the baud rate is above a third of the clock frequency, the most the eUSCI supports (user's guide).
    #[inline(always)]
    pub fn use_aclk(self, aclk: &Aclk) -> SerialConfig<USCI, ClockSet, M> {
        serial_config!(
            self,
            ClockSet {
                baud_config: calculate_baud_config(aclk.freq() as u32, self.state.baudrate),
                clksel: Ucssel::DeviceSpecific,
            }
        )
    }

    #[cfg(feature = "eusci_modclk")]
    /// Configure serial UART to use MODCLK.
    ///
    /// # Panics
    ///
    /// If the baud rate is above a third of the clock frequency, the most the eUSCI supports (user's guide).
    #[inline(always)]
    pub fn use_modclk(self) -> SerialConfig<USCI, ClockSet, M> {
        serial_config!(
            self,
            ClockSet {
                baud_config: calculate_baud_config(
                    crate::device_specific::MODCLK_FREQ_HZ,
                    self.state.baudrate
                ),
                clksel: Ucssel::DeviceSpecific,
            }
        )
    }

    /// Configure serial UART to use SMCLK.
    ///
    /// # Panics
    ///
    /// If the baud rate is above a third of the clock frequency, the most the eUSCI supports (user's guide).
    #[inline(always)]
    pub fn use_smclk(self, smclk: &Smclk) -> SerialConfig<USCI, ClockSet, M> {
        serial_config!(
            self,
            ClockSet {
                baud_config: calculate_baud_config(smclk.freq(), self.state.baudrate),
                clksel: Ucssel::Smclk,
            }
        )
    }
}

struct BaudConfig {
    br: u16,
    brs: u8,
    brf: u8,
    ucos16: bool,
}

#[inline]
fn calculate_baud_config(clk_freq: u32, bps: NonZeroU32) -> BaudConfig {
    // In low-frequency mode the baud rate can be at most a third of the clock (user's guide,
    // low-frequency baud-rate generation)
    assert!(clk_freq / bps.get() >= 3, "baud rate above a third of the UART clock");
    // Ensure n stays within the 16 bit boundary
    let n = (clk_freq / bps).min(0xFFFF);

    let brs = lookup_brs(clk_freq, bps);

    if (n >= 16) && (bps.get() < u32::MAX / 16) {
        //  div = bps * 16
        const SIXTEEN: NonZeroU32 = NonZeroU32::new(16).unwrap();
        let div = bps.saturating_mul(SIXTEEN);

        // n / 16, but more precise
        let br = (clk_freq / div) as u16;

        // same as n % 16, but more precise
        let brf = ((clk_freq % div) / bps) as u8;
        BaudConfig { ucos16: true, br, brf, brs }
    } else {
        BaudConfig { ucos16: false, br: n as u16, brf: 0, brs }
    }
}

#[inline(always)]
fn lookup_brs(clk_freq: u32, bps: NonZeroU32) -> u8 {
    // bps is between [1, 5_000_000] (datasheet max)
    // clk_freq is between [0, 24_000_000] (datasheet max)

    // modulo = clk_freq % bps => modulo is between [0, 4_999_999]
    let modulo = clk_freq % bps;

    // fraction = modulo * 10_000 / (bps), so within [0, ((bps-1) * 10_000) / bps].
    // To prove upper bound we note `(bps-1)/bps` is largest when bps == 5_000_000:
    // (4_999_999 * 10_000) / 5_000_000 = 49_999_990_000 (watch out for overflow!) / 5_000_000 = 9999.99... truncated to 9_999 because integer division
    // So fraction is within [0, 9999]
    let fraction_as_ten_thousandths = if modulo < u32::MAX / 10_000 {
        // Most accurate
        ((modulo * 10_000) / bps) as u16
    } else {
        // Avoid overflow if modulo is large. Assume modulo < 5_000_000 from datasheet max
        (((modulo * 500) / bps) * 20) as u16
    };

    // See Table 22-4 from MSP430FR4xx and MSP430FR2xx family user's guide (Rev. I)
    match fraction_as_ten_thousandths {
        0..529     => 0x00,
        529..715   => 0x01,
        715..835   => 0x02,
        835..1001  => 0x04,
        1001..1252 => 0x08,
        1252..1430 => 0x10,
        1430..1670 => 0x20,
        1670..2147 => 0x11,
        2147..2224 => 0x21,
        2224..2503 => 0x22,
        2503..3000 => 0x44,
        3000..3335 => 0x25,
        3335..3575 => 0x49,
        3575..3753 => 0x4A,
        3753..4003 => 0x52,
        4003..4286 => 0x92,
        4286..4378 => 0x53,
        4378..5002 => 0x55,
        5002..5715 => 0xAA,
        5715..6003 => 0x6B,
        6003..6254 => 0xAD,
        6254..6432 => 0xB5,
        6432..6667 => 0xB6,
        6667..7001 => 0xD6,
        7001..7147 => 0xB7,
        7147..7503 => 0xBB,
        7503..7861 => 0xDD,
        7861..8004 => 0xED,
        8004..8333 => 0xEE,
        8333..8464 => 0xBF,
        8464..8572 => 0xDF,
        8572..8751 => 0xEF,
        8751..9004 => 0xF7,
        9004..9170 => 0xFB,
        9170..9288 => 0xFD,
        9288..     => 0xFE,
    }
}

impl<USCI, M> SerialConfig<USCI, ClockSet, M>
where
    USCI: SerialUsci<M>,
    M: PinMap,
{
    #[inline]
    fn config_hw(self) {
        let ClockSet { baud_config, clksel } = self.state;
        let usci = self.usci;

        USCI::configure_pin_mapping();

        usci.ctl0_reset();
        usci.brw_settings(baud_config.br);
        usci.mctlw_settings(baud_config.ucos16, baud_config.brs, baud_config.brf);
        usci.loopback(self.loopback.to_bool());
        usci.ctl1_wr(self.deglitch as u16);
        usci.abctl_wr(match self.mode {
            UartMode::AutoBaud { delimiter } => (delimiter as u16) << 4 | 1,
            _ => 0,
        });
        usci.irctl_wr(self.irda.map_or(0, |irda| irda.irctl()));
        usci.ctl0_settings(UcaCtlw0 {
            ucpen: self.parity.ucpen(),
            ucpar: self.parity.ucpar(),
            ucmsb: self.order.to_bool(),
            uc7bit: self.cnt.to_bool(),
            ucspb: self.stopbits.to_bool(),
            ucmode: self.mode.ucmode(),
            ucssel: clksel,
            // We want erroneous bytes to trigger RXIFG so all errors can be caught
            ucrxeie: true,
            ucbrkie: self.break_interrupts,
        });
        // Everything is configured while UCSWRST is set, then the eUSCI is released (user's guide,
        // eUSCI_A initialization)
        usci.ctl0_clear_rst();
    }

    /// Perform hardware configuration and split into Tx and Rx pins from appropriate GPIOs
    #[inline]
    pub fn split<T: Into<USCI::TxPin>, R: Into<USCI::RxPin>>(
        self,
        _tx: T,
        _rx: R,
    ) -> (Tx<USCI, M>, Rx<USCI, M>) {
        self.config_hw();
        (Tx(PhantomData, PhantomData), Rx(PhantomData, PhantomData))
    }

    /// Perform hardware configuration and create Tx pin from appropriate GPIO
    #[inline]
    pub fn tx_only<T: Into<USCI::TxPin>>(self, _tx: T) -> Tx<USCI, M> {
        self.config_hw();
        Tx(PhantomData, PhantomData)
    }

    /// Perform hardware configuration and create Rx pin from appropriate GPIO
    #[inline]
    pub fn rx_only<R: Into<USCI::RxPin>>(self, _rx: R) -> Rx<USCI, M> {
        self.config_hw();
        Rx(PhantomData, PhantomData)
    }
}

/// Serial transmitter pin
pub struct Tx<USCI, M = DefaultMapping>(PhantomData<USCI>, PhantomData<M>)
where
    USCI: SerialUsci<M>,
    M: PinMap;

impl<USCI, M> Tx<USCI, M>
where
    USCI: SerialUsci<M>,
    M: PinMap,
{
    /// Enable Tx interrupts, which fire when ready to send.
    #[inline(always)]
    pub fn enable_tx_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.txie_set();
    }

    /// Disable Tx interrupts
    #[inline(always)]
    pub fn disable_tx_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.txie_clear();
    }

    /// Enable interrupts when a character has been sent completely (UCTXCPTIE). Due to erratum
    /// USCI42 the flag is set after each character, even while the next one waits in the Tx buffer.
    #[inline(always)]
    pub fn enable_tx_complete_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.ie_set_bits(UCTXCPTIFG);
    }

    /// Disable interrupts when a character has been sent completely (UCTXCPTIE)
    #[inline(always)]
    pub fn disable_tx_complete_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.ie_clr_bits(UCTXCPTIFG);
    }

    /// The highest-priority pending interrupt of this eUSCI among the enabled ones (UCAxIV), shared with
    /// [`Rx::interrupt_source`]. Reading it clears the start-bit and transmit-complete flags.
    #[inline(always)]
    pub fn interrupt_source(&mut self) -> UartVector {
        let usci = unsafe { USCI::steal() };
        read_uart_iv(&usci)
    }

    /// Send an address character, in the multiprocessor modes (UCTXADDR): preceded by an idle line in the
    /// idle-line format, with the address bit set in the address-bit format. Returns `WouldBlock` until the Tx
    /// buffer is free.
    #[inline]
    pub fn send_address(&mut self, address: u8) -> nb::Result<(), Infallible> {
        let usci = unsafe { USCI::steal() };
        if !usci.txifg_rd() {
            return Err(nb::Error::WouldBlock);
        }
        usci.ctl0_set_bits(UCTXADDR);
        usci.tx_wr(address);
        Ok(())
    }

    /// Send a break: all bits low for a character time, or in automatic baud-rate mode a 13-bit break, the
    /// break delimiter and the synch field 0x55, as LIN needs (UCTXBRK). Returns `WouldBlock` until the Tx
    /// buffer is free.
    #[inline]
    pub fn send_break(&mut self) -> nb::Result<(), Infallible> {
        let usci = unsafe { USCI::steal() };
        if !usci.txifg_rd() {
            return Err(nb::Error::WouldBlock);
        }
        let auto_baud = usci.ctl0_rd() >> 9 & 0b11 == 0b11;
        usci.ctl0_set_bits(UCTXBRK);
        usci.tx_wr(if auto_baud { 0x55 } else { 0x00 });
        Ok(())
    }

    // Internal flush function: done once the Tx buffer is empty and the last character has left the
    // shift register. UCBUSY also covers a character being received, so this can wait for that too.
    // (UCTXCPTIFG can't be used: erratum USCI42 sets it after each character.)
    #[inline]
    fn flush(&mut self) -> nb::Result<(), Infallible> {
        let usci = unsafe { USCI::steal() };
        if usci.txifg_rd() && !usci.statw_rd().ucbusy() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline(always)]
    /// Writes a byte into the Tx buffer with no checks for validity
    /// # Safety
    /// May clobber unsent data still in the buffer
    pub unsafe fn write_no_check(&mut self, data: u8) {
        let usci = unsafe { USCI::steal() };
        usci.tx_wr(data);
    }

    // Internal send function
    #[inline]
    fn send(&mut self, data: u8) -> nb::Result<(), Infallible> {
        let usci = unsafe { USCI::steal() };
        if usci.txifg_rd() {
            usci.tx_wr(data);
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// Serial receiver pin
pub struct Rx<USCI, M = DefaultMapping>(PhantomData<USCI>, PhantomData<M>)
where
    USCI: SerialUsci<M>,
    M: PinMap;

impl<USCI, M> Rx<USCI, M>
where
    USCI: SerialUsci<M>,
    M: PinMap,
{
    /// Enable Rx interrupts, which fire when ready to read
    #[inline(always)]
    pub fn enable_rx_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.rxie_set();
    }

    /// Disable Rx interrupts
    #[inline(always)]
    pub fn disable_rx_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.rxie_clear();
    }

    /// Enable interrupts when a start bit is received (UCSTTIE), for example to wake up from a low-power
    /// mode in time for the character
    #[inline(always)]
    pub fn enable_start_bit_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.ifg_clr_bits(UCSTTIFG);
        usci.ie_set_bits(UCSTTIFG);
    }

    /// Disable interrupts when a start bit is received (UCSTTIE)
    #[inline(always)]
    pub fn disable_start_bit_interrupts(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.ie_clr_bits(UCSTTIFG);
    }

    /// The highest-priority pending interrupt of this eUSCI among the enabled ones (UCAxIV), shared with
    /// [`Tx::interrupt_source`]. Reading it clears the start-bit and transmit-complete flags.
    #[inline(always)]
    pub fn interrupt_source(&mut self) -> UartVector {
        let usci = unsafe { USCI::steal() };
        read_uart_iv(&usci)
    }

    /// In the multiprocessor and automatic baud-rate modes, receive only address characters, or the character
    /// after a break and synch field, while dormant (UCDORM). After receiving one that's for this device, leave
    /// the dormant state to receive the data that follows.
    #[inline(always)]
    pub fn set_dormant(&mut self, dormant: bool) {
        let usci = unsafe { USCI::steal() };
        if dormant {
            usci.ctl0_set_bits(UCDORM);
        } else {
            usci.ctl0_clr_bits(UCDORM);
        }
    }

    /// Like reading a character, but also returns whether it's an address character, in the multiprocessor
    /// modes (UCADDR, UCIDLE).
    #[inline]
    pub fn read_with_address_flag(&mut self) -> nb::Result<(u8, bool), RecvError> {
        let usci = unsafe { USCI::steal() };
        // The flag is cleared when the character is read, so read it first
        let address = usci.statw_bits() & UCADDR_UCIDLE != 0;
        self.recv().map(|data| (data, address))
    }

    /// In automatic baud-rate mode: whether a break was longer than 22 bit times (UCBTOE), and whether a synch
    /// field was too long to measure (UCSTOE).
    #[inline]
    pub fn auto_baud_errors(&self) -> (bool, bool) {
        let usci = unsafe { USCI::steal() };
        let abctl = usci.abctl_rd();
        (abctl & 1 << 2 != 0, abctl & 1 << 3 != 0)
    }

    /// Reads raw value from Rx buffer with no checks for validity
    /// # Safety
    /// May read duplicate data
    #[inline(always)]
    pub unsafe fn read_no_check(&mut self) -> u8 {
        let usci = unsafe { USCI::steal() };
        usci.rx_rd()
    }

    // Internal recieve function
    fn recv(&mut self) -> nb::Result<u8, RecvError> {
        let usci = unsafe { USCI::steal() };

        if usci.rxifg_rd() {
            let statw = usci.statw_rd();
            let data = usci.rx_rd();

            if statw.ucbrk() {
                Err(nb::Error::Other(RecvError::Break))
            } else if statw.ucfe() {
                Err(nb::Error::Other(RecvError::Framing))
            } else if statw.ucpe() {
                Err(nb::Error::Other(RecvError::Parity))
            } else if statw.ucoe() {
                Err(nb::Error::Other(RecvError::Overrun(data)))
            } else {
                Ok(data)
            }
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// Serial receive errors
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum RecvError {
    /// A break was received: all bits low, see [`SerialConfig::break_interrupts`]
    Break,
    /// Framing error
    Framing,
    /// Parity error
    Parity,
    /// Buffer overrun error. Contains the most recently read byte, which is still valid.
    Overrun(u8),
}
// embedded-io requires this
impl Display for RecvError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}
impl core::error::Error for RecvError {}

mod emb_io {
    use super::*;
    use embedded_io::{Error, ErrorType, Read, ReadReady, Write, WriteReady};
    use nb::block;

    impl<USCI, M> ErrorType for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = RecvError;
    }
    impl Error for RecvError {
        fn kind(&self) -> embedded_io::ErrorKind {
            match self {
                RecvError::Break        => embedded_io::ErrorKind::Other,
                RecvError::Framing      => embedded_io::ErrorKind::Other,
                RecvError::Parity       => embedded_io::ErrorKind::Other,
                RecvError::Overrun(_)   => embedded_io::ErrorKind::Other,
            }
        }
    }
    impl<USCI, M> Read for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        #[inline]
        /// Read one byte into the specified buffer, then returns the number of bytes sent (1).
        /// If a byte isn't currently available to read, this function blocks until one is available.
        ///
        /// If `buf` is length zero, `write` returns `Ok(0)` without blocking.
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if buf.is_empty() { return Ok(0) }
            buf[0] = block!(self.recv())?;
            Ok(1)
        }
    }
    impl<USCI, M> ReadReady for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            let usci = unsafe { USCI::steal() };
            Ok(usci.rxifg_rd())
        }
    }

    impl<USCI, M> ErrorType for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = Infallible;
    }

    impl<USCI, M> Write for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        /// Due to errata USCI42, UCTXCPTIFG will fire every time a byte is done transmitting,
        /// even if there's still more buffered. Thus, the implementation uses UCTXIFG instead. When
        /// `flush()` completes, the Tx buffer will be empty but the FIFO may still be sending.
        ///
        /// As the error type is `Infallible`, this can be safely unwrapped.
        #[inline]
        fn flush(&mut self) -> Result<(), Self::Error> { block!(self.flush()) }

        #[inline]
        /// This function sends only **THE FIRST** byte in the buffer, blocking until the writer is ready to accept, then returns `Ok(1)`.
        /// If you want to send the entire buffer use `write_all()` instead.
        ///
        /// If `buf` is length zero, `write` returns `Ok(0)` without blocking.
        ///
        /// As the error type is `Infallible`, this can be safely unwrapped.
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            if buf.is_empty() { return Ok(0) }
            block!(self.send(buf[0]))?;
            Ok(1)
        }
        // The default version of this impl panics if .write() returns Ok(0) when given a non-empty buffer. Our impl never does this, so remove it.
        /// Write an entire buffer into this writer.
        ///
        /// This function calls `write()` in a loop until exactly `buf.len()` bytes have
        /// been written, blocking if needed.
        ///
        /// If you are using [`WriteReady`] to avoid blocking, you should not use this function.
        /// `WriteReady::write_ready()` returning true only guarantees the first call to `write()` will
        /// not block, so this function may still block in subsequent calls.
        fn write_all(&mut self, mut buf: &[u8]) -> Result<(), Self::Error> {
            while !buf.is_empty() {
                let Ok(n) = self.write(buf);
                buf = &buf[n..];
            }
            Ok(())
        }
    }
    impl<USCI, M> WriteReady for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        /// Whether the writer is ready for immediate writing. If this returns `true`, the next call to [`Write::write`] will not block.
        ///
        /// As the error type is `Infallible`, this can be safely unwrapped.
        fn write_ready(&mut self) -> Result<bool, Self::Error> {
            let usci = unsafe { USCI::steal() };
            Ok(usci.txifg_rd())
        }
    }
}

mod ehal_nb1 {
    use super::*;
    use embedded_hal_nb::serial::{Error, ErrorKind, ErrorType, Read, Write};

    impl Error for RecvError {
        fn kind(&self) -> ErrorKind {
            match self {
                RecvError::Break        => ErrorKind::FrameFormat,
                RecvError::Framing      => ErrorKind::FrameFormat,
                RecvError::Parity       => ErrorKind::Parity,
                RecvError::Overrun(_)   => ErrorKind::Overrun,
            }
        }
    }
    impl<USCI, M> ErrorType for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = RecvError;
    }

    impl<USCI, M> Read<u8> for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        #[inline]
        /// Check if Rx interrupt flag is set. If so, try reading the received byte and clear the flag.
        /// Otherwise return `WouldBlock`. May return errors caused by data corruption or
        /// buffer overruns.
        fn read(&mut self) -> nb::Result<u8, Self::Error> { self.recv() }
    }

    impl<USCI, M> ErrorType for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = Infallible;
    }

    impl<USCI, M> Write<u8> for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        /// Due to errata USCI42, UCTXCPTIFG will fire every time a byte is done transmitting,
        /// even if there's still more buffered. Thus, the implementation uses UCTXIFG instead. When
        /// `flush()` completes, the Tx buffer will be empty but the FIFO may still be sending.
        #[inline]
        fn flush(&mut self) -> nb::Result<(), Self::Error> { self.flush() }

        #[inline]
        /// Check if Tx interrupt flag is set. If so, write a byte into the Tx buffer. Otherwise return `WouldBlock`
        fn write(&mut self, data: u8) -> nb::Result<(), Self::Error> { self.send(data) }
    }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use super::*;
    use embedded_hal_02::serial::{Read, Write};

    impl<USCI, M> Read<u8> for Rx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = RecvError;

        #[inline]
        /// Check if Rx interrupt flag is set. If so, try reading the received byte and clear the flag.
        /// Otherwise return `WouldBlock`. May return errors caused by data corruption or
        /// buffer overruns.
        fn read(&mut self) -> nb::Result<u8, Self::Error> { self.recv() }
    }

    impl<USCI, M> Write<u8> for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {
        type Error = void::Void;

        /// Due to errata USCI42, UCTXCPTIFG will fire every time a byte is done transmitting,
        /// even if there's still more buffered. Thus, the implementation uses UCTXIFG instead. When
        /// `flush()` completes, the Tx buffer will be empty but the FIFO may still be sending.
        #[inline]
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.flush().map_err(|_| nb::Error::WouldBlock)
        }

        #[inline]
        /// Check if Tx interrupt flag is set. If so, write a byte into the Tx buffer. Otherwise return `WouldBlock`
        fn write(&mut self, data: u8) -> nb::Result<(), Self::Error> {
            self.send(data).map_err(|_| nb::Error::WouldBlock)
        }
    }

    impl<USCI, M> embedded_hal_02::blocking::serial::write::Default<u8> for Tx<USCI, M>
    where
        USCI: SerialUsci<M>,
        M: PinMap,
    {}
}
