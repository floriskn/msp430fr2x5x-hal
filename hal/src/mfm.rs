//! Manchester Function Module (MFM)
//!
//! Only on the MSP430FR2x5x. The MFM encodes and decodes Manchester-coded packets of 256 data bits (512 bits on the
//! wire, the first data bit 1) on P5.0 (MFM.RX) and P5.1 (MFM.TX), selected by their alternate function 2 (SLAU445I
//! chapter 25, data sheet 6.10.14). It works as the SPI master of eUSCI_B1, which must be a 4-wire SPI slave:
//! received data arrives in eUSCI_B1's Rx buffer byte by byte, and the data to send is read from its Tx buffer.
//!
//! - It works in active mode, LPM0 and LPM1.
//! - It oversamples 8 times, so the bit rate is SMCLK / 8: 500 kbit/s needs a 4 MHz SMCLK.
//! - A rising edge of TB2's CCR0 output starts sending a packet, so write the first byte before it. TB2's capture
//!   input CCI0B is the MFM complete event, for timing.
//! - Each received byte has to be read within about 16 bit times (64 SMCLK cycles at 4 MHz), so use eUSCI_B1's
//!   receive interrupt.
//!
//! The user's guide doesn't say which SPI clock mode and STE polarity the MFM uses, so this module leaves them to
//! [`SpiConfig`](crate::spi::SpiConfig). It is not tested on hardware yet.

use crate::{
    gpio::{Alternate2, Pin, Pin0, Pin1, P5},
    pac::EUsciB1,
    pin_mapping::{DefaultMapping, PinMap},
    spi::{SpiErr, SpiSlave, SpiUsci},
};
use core::convert::Infallible;

/// The Manchester Function Module, with eUSCI_B1 as its SPI slave
pub struct Mfm<M: PinMap = DefaultMapping>
where
    EUsciB1: SpiUsci<M>,
{
    spi: SpiSlave<EUsciB1, M>,
}

impl<M: PinMap> Mfm<M>
where
    EUsciB1: SpiUsci<M>,
{
    /// Enable the MFM by putting its pins in their MFM function. `spi` is eUSCI_B1 as a 4-wire SPI slave, from
    /// [`SpiConfig::mfm_slave`](crate::spi::SpiConfig::mfm_slave).
    #[inline]
    pub fn new<RXDIR, TXDIR>(
        spi: SpiSlave<EUsciB1, M>,
        _rx: Pin<P5, Pin0, Alternate2<RXDIR>>,
        _tx: Pin<P5, Pin1, Alternate2<TXDIR>>,
    ) -> Self {
        Mfm { spi }
    }

    /// A received byte, or `WouldBlock` if none has arrived.
    #[inline]
    pub fn read(&mut self) -> nb::Result<u8, SpiErr> { self.spi.read() }

    /// Queue a byte to send, or `WouldBlock` if the Tx buffer is full.
    #[inline]
    pub fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> { self.spi.write(byte) }

    /// eUSCI_B1, to enable its interrupts or read its interrupt source.
    #[inline]
    pub fn spi(&mut self) -> &mut SpiSlave<EUsciB1, M> { &mut self.spi }
}
