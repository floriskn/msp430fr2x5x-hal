//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The CRC module's two input bit orders and its bit-reversed result, checked against the signatures the
//! user's guide gives: the CRC-16-CCITT signature of "123456789" is fed in four ways and compared with the
//! expected values, and an LED on P1.0 lights if all of them match, one on P1.1 if any doesn't.
//!
//! The `_msb` methods write CRCDI, which takes each byte as it is, and the `_lsb` methods write CRCDIRB,
//! which reverses the bits of each byte first. Both go in a byte at a time, and a word at a time with the
//! lower byte first. `result()` reads the signature from CRCINIRES, and `result_reversed()` from CRCRESR,
//! which holds it with its bits in reverse order. Every run starts from the seed FFFFh.
//! (The data, the seed and the signatures: SLAU445I Example 11-2, p. 356. CRCDI, CRCDIRB and the byte order
//! of words: SLAU445I 11.3.1, p. 354. CRCRESR: SLAU445I Table 11-5, p. 359. The polynomial, x^16 + x^12 +
//! x^5 + 1: SLASEE4C 6.10.6, p. 53. No board document covers the LEDs: there is none for the MSP430FR25x2.
//! P1.0 and P1.1 are GPIO outputs, P1SELx = 00 and P1DIR = 1: SLASEE4C Table 6-15, p. 58.)
//!
//! How to test (two LEDs and two resistors):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another one from P1.1 to
//!    GND.
//! 2. Flash this example.
//! 3. Expected: the LED on P1.0 lights, as all eight values match. If the one on P1.1 lights instead, a
//!    signature is wrong: it was fed through CRCDI with `add_bytes_msb()`, or with `add_words_msb()` and
//!    `add_byte_msb()`, or the same through CRCDIRB with the `_lsb` methods.
#![no_main]
#![no_std]

use embedded_hal::digital::OutputPin;
use msp430_rt::entry;
use msp430_hal::{crc::Crc, gpio::Batch, pmm::Pmm, watchdog::Wdt};
use panic_msp430 as _;

/// The seed of SLAU445I Example 11-2, p. 356
const SEED: u16 = 0xFFFF;
/// The data of the example, "123456789": as bytes, and as words of two characters each, the first one in
/// the lower byte, which goes in first (SLAU445I 11.3.1, p. 354). The ninth character then goes in as a byte.
const BYTES: &[u8] = b"123456789";
const WORDS: [u16; 4] = [0x3231, 0x3433, 0x3635, 0x3837];

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut pass_led = p1.pin0.to_output_low();
    let mut fail_led = p1.pin1.to_output_low();

    // Writing CRCINIRES sets the seed (SLAU445I Table 11-4, p. 359)
    let mut crc = Crc::new(periph.crc, SEED);

    // Through CRCDI: a byte at a time to CRCDI_L, then a word at a time to CRCDI
    crc.add_bytes_msb(BYTES);
    let mut all_match = matches(&mut crc, 0x89F6, 0x6F91);
    crc.reset(SEED);
    crc.add_words_msb(&WORDS);
    crc.add_byte_msb(b'9');
    all_match &= matches(&mut crc, 0x89F6, 0x6F91);

    // The same through CRCDIRB, which reverses the bits of each byte
    crc.reset(SEED);
    crc.add_bytes_lsb(BYTES);
    all_match &= matches(&mut crc, 0x29B1, 0x8D94);
    crc.reset(SEED);
    crc.add_words_lsb(&WORDS);
    crc.add_byte_lsb(b'9');
    all_match &= matches(&mut crc, 0x29B1, 0x8D94);

    pass_led.set_state(all_match.into()).ok();
    fail_led.set_state((!all_match).into()).ok();

    loop {
        msp430::asm::nop();
    }
}

/// Whether the signature, from CRCINIRES and from CRCRESR, has the values SLAU445I Example 11-2, p. 356 gives
fn matches(crc: &mut Crc, expected: u16, expected_reversed: u16) -> bool {
    crc.result() == expected && crc.result_reversed() == expected_reversed
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
