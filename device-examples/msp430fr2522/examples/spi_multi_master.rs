//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master that shares the bus with other masters, which take it through the STE pin.
//!
//! eUSCI_B0 sends 16 bytes at 2 kHz, which takes 64 ms, over and over, and checks what comes back: with MOSI
//! looped back to MISO it reads back what it sent. STE has its pullup on, and this master may use the bus
//! while STE is high. Pulling STE low, as another master would, takes the bus from it: a transfer that is
//! under way stops and returns `SpiErr::BusConflict`, and the next one waits until STE is high again.
//! eUSCI_A0 reports each transfer on P1.4, to a USB-to-UART adapter: there's no LaunchPad for the
//! MSP430FR25x2.
//! (4-pin master mode with UCSTEM = 0: SLAU445I 23.3.3.1, p. 608; the STE levels: SLAU445I Table 23-1,
//! p. 606. eUSCI_B0's SPI pins and UCA0TXD: SLASEE4C Table 6-11, p. 53.)
//!
//! How to test (two jumper wires, a 3.3-V USB-to-UART adapter, the scope):
//! 1. Connect MOSI, P1.2, to MISO, P1.3, with a jumper wire.
//! 2. Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 3. Flash this example. Expected: `transfer 1: read back what was sent`, then transfer 2 and so on, about
//!    seven a second.
//! 4. Connect STE, P1.0, to GND with the other jumper wire. Expected: `another master has the bus, waiting`,
//!    and no more transfers. If the wire went on while a transfer was under way, `transfer N: another master
//!    took the bus, aborted` comes first.
//! 5. Remove the wire. Expected: the transfers go on. An aborted transfer is repeated, with its number.
//! 6. Scope, ground on GND, 50 ms/div: CH1 on SCLK, P1.1, CH2 on STE, P1.0. The bursts of SCLK stop while
//!    STE is low, also in the middle of a transfer.
#![no_main]
#![no_std]

use embedded_hal::{
    delay::DelayNs,
    spi::{SpiBus, MODE_0},
};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    spi::{Spi, SpiConfig, SpiErr, StePolarity},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The bytes of each transfer
const LEN: usize = 16;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
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

    // eUSCI_B0 as SPI master: SCLK on P1.1, MOSI on P1.2 and MISO on P1.3, and STE on P1.0, with
    // P1SELx = 01 in the default mapping, USCIBRMP = 0 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15,
    // p. 58). STE has its pullup (P1REN = 1, P1OUT = 1: SLAU445I Table 8-1, p. 313), which keeps it high
    // while nothing pulls it low. MODE_0 captures data on the first clock edge with the clock idle low
    // (UCCKPH = 1, UCCKPL = 0), `true` sends the MSB first (UCMSB = 1) (SLAU445I Table 23-12, p. 620).
    // SMCLK / 4000 is 2 kHz (fBitClock = fBRCLK / UCBRx: SLAU445I 23.3.6, p. 609). This master may use the
    // bus while STE is high (UCMODEx = 10b: SLAU445I Table 23-1, p. 606).
    let mut spi: Spi<_, DefaultMapping> = SpiConfig::new(periph.e_usci_b0, MODE_0, true)
        .to_master_using_smclk(&smclk, 4000)
        .multi_master_bus(
            p1.pin3.to_alternate1(),
            p1.pin2.to_alternate1(),
            p1.pin1.to_alternate1(),
            p1.pin0.pullup().to_alternate1(),
            StePolarity::EnabledWhenHigh,
        );

    let mut number: u16 = 0;
    let mut sent = [0u8; LEN];
    let mut aborted = false;
    loop {
        if !spi.bus_available() {
            writeln!(console, "another master has the bus, waiting\r").ok();
            while !spi.bus_available() {}
        }
        // A new transfer, or the aborted one again
        if !aborted {
            number = number.wrapping_add(1);
            for (i, byte) in sent.iter_mut().enumerate() {
                *byte = (number as u8).wrapping_add(i as u8);
            }
        }
        let mut read = [0u8; LEN];
        aborted = false;
        match spi.transfer(&mut read, &sent) {
            Ok(()) if read == sent => writeln!(console, "transfer {}: read back what was sent\r", number).ok(),
            Ok(()) => writeln!(console, "transfer {}: read back other bytes, is P1.2 wired to P1.3?\r", number).ok(),
            Err(SpiErr::BusConflict) => {
                aborted = true;
                writeln!(console, "transfer {}: another master took the bus, aborted\r", number).ok()
            }
            Err(SpiErr::Overrun(_)) => writeln!(console, "transfer {}: overrun\r", number).ok(),
        };
        delay.delay_ms(30);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
