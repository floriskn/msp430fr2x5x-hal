//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master whose eUSCI drives the slave's enable signal on STE, with 7-bit characters.
//!
//! eUSCI_B0 sends 55h and AAh at 100 kHz, about eight times a second. STE goes low while the characters
//! go out, as the enable signal of a single slave. With 7-bit characters the top bit of each byte isn't
//! sent, so AAh goes out as 2Ah. With MOSI looped back to MISO, the master reads back what it sent: 55h
//! and 2Ah. eUSCI_A0 prints that on P1.4, to a USB-to-UART adapter: there's no LaunchPad for the
//! MSP430FR25x2.
//! (STE as the slave enable, UCSTEM = 1: SLAU445I 23.3.3.2, p. 608. 7-bit characters, UC7BIT: SLAU445I
//! 23.3.2, p. 607. eUSCI_B0's SPI pins and UCA0TXD: SLASEE4C Table 6-11, p. 53.)
//!
//! How to test (a jumper wire, a 3.3-V USB-to-UART adapter, the scope):
//! 1. Connect MOSI, P1.2, to MISO, P1.3, with the jumper wire.
//! 2. Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 3. Flash this example. Expected: `sent 55 AA, read back 55 2A`. Without the wire, MISO floats and the
//!    read-back values mean nothing.
//! 4. Scope, ground on GND, 20 µs/div, trigger on CH3 falling: CH1 on SCLK, P1.1, CH2 on MOSI, P1.2, CH3 on
//!    STE, P1.0. STE is low for the whole transfer, with 7 clock pulses for each character, 14 in all. MOSI
//!    shows 1010101 and then 0101010, the bits of 55h and 2Ah, MSB first.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, spi::MODE_0};
use embedded_hal_nb::spi::FullDuplex;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    spi::{Spi, SpiConfig, StePolarity},
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

const SENT: [u8; 2] = [0x55, 0xAA];

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, so the CPU keeps up with the SPI, and ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The console: eUSCI_A0's TXD on P1.4, P1SELx = 01 in the default mapping, USCIARMP = 0, 8N1 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58; SLAU445I Table 22-8, p. 593)
    let mut console = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p1.pin4.to_alternate1());

    // eUSCI_B0 as SPI master: SCLK on P1.1, MOSI on P1.2, MISO on P1.3 and STE on P1.0, with P1SELx = 01
    // in the default mapping, USCIBRMP = 0 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). MODE_0
    // captures data on the first clock edge with the clock idle low (UCCKPH = 1, UCCKPL = 0), `true` sends
    // the MSB first (UCMSB = 1) (SLAU445I Table 23-12, p. 620). SMCLK / 80 is 100 kHz (fBitClock = fBRCLK /
    // UCBRx: SLAU445I 23.3.6, p. 609). STE is low while the slave is enabled (UCMODEx = 10b: SLAU445I
    // Table 23-12, p. 620).
    let mut spi: Spi<_, DefaultMapping> = SpiConfig::new(periph.e_usci_b0, MODE_0, true)
        .seven_bit_characters()
        .to_master_using_smclk(&smclk, 80)
        .single_slave_bus(
            p1.pin3.to_alternate1(),
            p1.pin2.to_alternate1(),
            p1.pin1.to_alternate1(),
            p1.pin0.to_alternate1(),
            StePolarity::EnabledWhenLow,
        );

    loop {
        // The second character waits in the Tx buffer while the first goes out, so they follow each other
        // without a gap and STE stays low ("The UCxTXBUF data is moved to the transmit (TX) shift register
        // when the TX shift register is empty": SLAU445I 23.3.3, p. 607)
        block!(spi.write(SENT[0])).ok();
        block!(spi.write(SENT[1])).ok();
        let first = block!(spi.read()).unwrap_or(0xEE);
        let second = block!(spi.read()).unwrap_or(0xEE);
        writeln!(console, "sent {:02X} {:02X}, read back {:02X} {:02X}\r", SENT[0], SENT[1], first, second).ok();
        delay.delay_ms(100);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
