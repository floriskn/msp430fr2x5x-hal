//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The UART address-bit multiprocessor format: one transmitter addresses several receivers on a shared
//! line, and each receiver only wakes up for the characters sent to its own address.
//!
//! eUSCI_A0 plays both parts, in loopback mode: it sends a message to address 12h (this receiver) and
//! one to address 34h (another receiver). The receiver sleeps in the dormant state, where it only takes
//! address characters. After its own address it leaves that state and takes the data that follows;
//! after another address it goes back to sleep, so it never sees the data for address 34h. eUSCI_A0 is the
//! device's only UART, so an LED on P1.0 shows the result instead of a terminal: it toggles each time the
//! receiver got `Hello`, and nothing else.
//! (The address-bit format: SLAU445I 22.3.3.2, p. 579. UCDORM: SLAU445I Table 22-8, p. 594. Loopback,
//! UCLISTEN: SLAU445I 22.4.5, p. 596. "One eUSCI_A supports UART, IrDA, and SPI": SLASEE4C 1.1, p. 1.
//! UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document covers the LED: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, and optionally the scope):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example.
//! 3. Expected: the LED toggles about once a second, on for 1 s and off for 1 s. It would stop if the
//!    receiver also took `World`, the message for the other address.
//! 4. Optional, with the scope, ground on GND: eUSCI_A0's TXD, P1.4, still carries what it sends. Each
//!    character has 9 bits between start and stop bit: 8 data bits and the address bit, which is 1 for the
//!    addresses and 0 for the data.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// This receiver's address
const MY_ADDRESS: u8 = 0x12;
/// The other receiver's address
const OTHER_ADDRESS: u8 = 0x34;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let mut led = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0 in the address-bit multiprocessor format (UCMODEx = 10b: SLAU445I Table 22-8, p. 593), in
    // loopback mode: its transmitter feeds its receiver. Its pins are P1.4 (TXD) and P1.5 (RXD), with
    // P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15,
    // p. 58).
    let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::Loopback,
        9600,
    )
    .mode(UartMode::AddressBitMultiprocessor)
    .use_smclk(&smclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    // Wait for an address (UCDORM = 1: "Only characters that are preceded by an idle-line or with address
    // bit set" are received: SLAU445I Table 22-8, p. 594)
    rx.set_dormant(true);

    loop {
        // The data characters the receiver takes this round, and how many
        let mut got = [0u8; 10];
        let mut len = 0;
        for (address, message) in [(MY_ADDRESS, b"Hello"), (OTHER_ADDRESS, b"World")] {
            // UCTXADDR marks the next character as an address: its address bit is 1 (SLAU445I 22.3.3.2,
            // p. 579)
            block!(tx.send_address(address)).ok();
            receive(&mut rx, &mut delay);
            for &byte in message {
                tx.write_all(&[byte]).ok();
                if let Some(data) = receive(&mut rx, &mut delay) {
                    got[len] = data;
                    len += 1;
                }
            }
        }
        if &got[..len] == b"Hello" {
            led.toggle().ok();
        }
        delay.delay_ms(1000);
    }
}

/// Give the receiver its turn after a character was sent: an address wakes it up or puts it to sleep, and
/// a data character, which only arrives while it's awake, is returned
fn receive<USCI: SerialUsci>(rx: &mut Rx<USCI>, delay: &mut impl DelayNs) -> Option<u8> {
    // A character of 11 bits takes 1.15 ms at 9600 baud. In the dormant state a data character never
    // arrives, so this waits a fixed time instead of blocking.
    delay.delay_ms(2);
    // UCADDR shows whether the character had its address bit set (SLAU445I Table 22-12, p. 596)
    let Ok((byte, is_address)) = rx.read_with_address_flag() else { return None };
    if is_address {
        // "user software can validate the address and must reset UCDORM to continue receiving data"
        // (SLAU445I 22.3.3.2, p. 579)
        rx.set_dormant(byte != MY_ADDRESS);
        None
    } else {
        Some(byte)
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
