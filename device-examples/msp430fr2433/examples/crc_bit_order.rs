//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The CRC module's two input bit orders and its bit-reversed result, checked against the signatures the
//! user's guide gives: once a second the backchannel UART prints the CRC-16-CCITT signature of "123456789",
//! fed in four ways, next to the expected values, with `pass` or `FAIL`.
//!
//! The `_msb` methods write CRCDI, which takes each byte as it is, and the `_lsb` methods write CRCDIRB,
//! which reverses the bits of each byte first. Both go in a byte at a time, and a word at a time with the
//! lower byte first. `result()` reads the signature from CRCINIRES, and `result_reversed()` from CRCRESR,
//! which holds it with its bits in reverse order. Every run starts from the seed FFFFh.
//! (The data, the seed and the signatures: SLAU445I Example 11-2, p. 356. CRCDI, CRCDIRB and the byte order
//! of words: SLAU445I 11.3.1, p. 354. CRCRESR: SLAU445I Table 11-5, p. 359. The polynomial, x^16 + x^12 +
//! x^5 + 1: SLASE59F 6.10.6, p. 49.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 2. Expected, once a second:
//!    `CRCDI, bytes: 89F6h 6F91h, expected 89F6h 6F91h: pass`
//!    `CRCDI, words: 89F6h 6F91h, expected 89F6h 6F91h: pass`
//!    `CRCDIRB, bytes: 29B1h 8D94h, expected 29B1h 8D94h: pass`
//!    `CRCDIRB, words: 29B1h 8D94h, expected 29B1h 8D94h: pass`
//!    A line that ends in `FAIL` names the register and the access that gave a wrong signature.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    crc::Crc,
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The seed of SLAU445I Example 11-2, p. 356
const SEED: u16 = 0xFFFF;
/// The data of the example, "123456789": as bytes, and as words of two characters each, the first one in
/// the lower byte, which goes in first (SLAU445I 11.3.1, p. 354). The ninth character then goes in as a byte.
const BYTES: &[u8] = b"123456789";
const WORDS: [u16; 4] = [0x3231, 0x3433, 0x3635, 0x3837];

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
    // Table 6-17, p. 55; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::new(
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

    // Writing CRCINIRES sets the seed (SLAU445I Table 11-4, p. 359)
    let mut crc = Crc::new(periph.crc, SEED);

    loop {
        // Through CRCDI: a byte at a time to CRCDI_L, then a word at a time to CRCDI
        crc.reset(SEED);
        crc.add_bytes_msb(BYTES);
        report(&mut tx, "CRCDI, bytes", &mut crc, 0x89F6, 0x6F91);
        crc.reset(SEED);
        crc.add_words_msb(&WORDS);
        crc.add_byte_msb(b'9');
        report(&mut tx, "CRCDI, words", &mut crc, 0x89F6, 0x6F91);

        // The same through CRCDIRB, which reverses the bits of each byte
        crc.reset(SEED);
        crc.add_bytes_lsb(BYTES);
        report(&mut tx, "CRCDIRB, bytes", &mut crc, 0x29B1, 0x8D94);
        crc.reset(SEED);
        crc.add_words_lsb(&WORDS);
        crc.add_byte_lsb(b'9');
        report(&mut tx, "CRCDIRB, words", &mut crc, 0x29B1, 0x8D94);

        writeln!(tx, "\r").ok();
        delay.delay_ms(1000);
    }
}

/// Print the signature, from CRCINIRES and from CRCRESR, next to the values SLAU445I Example 11-2, p. 356
/// gives for them
fn report(tx: &mut impl Write, name: &str, crc: &mut Crc, expected: u16, expected_reversed: u16) {
    let (result, reversed) = (crc.result(), crc.result_reversed());
    let verdict = if (result, reversed) == (expected, expected_reversed) { "pass" } else { "FAIL" };
    writeln!(
        tx,
        "{}: {:04X}h {:04X}h, expected {:04X}h {:04X}h: {}\r",
        name, result, reversed, expected, expected_reversed, verdict
    )
    .ok();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
