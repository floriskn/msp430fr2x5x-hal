//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master that shares the bus with other masters, which take it through the STE pin.
//!
//! eUSCI_A1 sends 16 bytes at 2 kHz, which takes 64 ms, over and over, and checks what comes back: with MOSI
//! looped back to MISO it reads back what it sent. STE has its pullup on, and this master may use the bus
//! while STE is high. Pulling STE low, as another master would, takes the bus from it: a transfer that is
//! under way stops and returns `SpiErr::BusConflict`, and the next one waits until STE is high again. The
//! backchannel UART reports each transfer.
//! (4-pin master mode with UCSTEM = 0: SLAU445I 23.3.3.1, p. 608; the STE levels: SLAU445I Table 23-1,
//! p. 606. eUSCI_A1's pins: SLASE59F Table 6-10, p. 49. Header pins: SLAU739 Figure 18, p. 23.)
//!
//! How to test (a jumper, a jumper wire, the scope):
//! 1. Put a jumper across J2 pins 14 and 15, which loops MOSI (P2.6) back to MISO (P2.5). A jumper wire
//!    works too.
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9). Expected: `transfer 1: read back what was sent`, then
//!    transfer 2 and so on, about seven a second.
//! 3. Connect STE, P3.1 (J2 pin 13), to GND (J2 pin 20) with the jumper wire. Expected: `another master
//!    has the bus, waiting`, and no more transfers. If the wire went on while a transfer was under way,
//!    `transfer N: another master took the bus, aborted` comes first.
//! 4. Remove the wire. Expected: the transfers go on. An aborted transfer is repeated, with its number.
//! 5. Scope, ground on GND (J3 pin 22), 50 ms/div: CH1 on SCLK, P2.4 (J1 pin 7), CH2 on STE, P3.1. The
//!    bursts of SCLK stop while STE is low, also in the middle of a transfer.
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
    pmm::Pmm,
    serial::*,
    spi::{SpiConfig, SpiErr, StePolarity},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The bytes of each transfer
const LEN: usize = 16;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SELx = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
    // Table 6-17, p. 55; SLAU445I Table 22-8, p. 593)
    let mut console = SerialConfig::new(
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

    // eUSCI_A1 as SPI master: SCLK on P2.4, MOSI on P2.6 and MISO on P2.5, with P2SELx = 01 (SLASE59F
    // Table 6-19, p. 58), and STE on P3.1, with P3SELx = 01 and its pullup (P3REN = 1, P3OUT = 1: SLASE59F
    // Table 6-20, p. 59; SLAU445I Table 8-1, p. 313), which keeps STE high while nothing pulls it low. MODE_0
    // captures data on the first clock edge with the clock idle low (UCCKPH = 1, UCCKPL = 0), `true` sends
    // the MSB first (UCMSB = 1) (SLAU445I Table 23-3, p. 613). SMCLK / 4000 is 2 kHz (fBitClock = fBRCLK /
    // UCBRx: SLAU445I 23.3.6, p. 609). This master may use the bus while STE is high (UCMODEx = 10b:
    // SLAU445I Table 23-1, p. 606).
    let mut spi = SpiConfig::new(periph.e_usci_a1, MODE_0, true)
        .to_master_using_smclk(&smclk, 4000)
        .multi_master_bus(
            p2.pin5.to_alternate1(),
            p2.pin6.to_alternate1(),
            p2.pin4.to_alternate1(),
            p3.pin1.pullup().to_alternate1(),
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
            Ok(()) => writeln!(console, "transfer {}: read back other bytes, is the J2 jumper on?\r", number).ok(),
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
