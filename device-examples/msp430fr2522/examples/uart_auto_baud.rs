//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Automatic baud-rate detection, as in LIN: a receiver whose clock is off measures the sender's baud rate
//! from a break and synch field, and then receives the data correctly.
//!
//! The receiver measures with its own transmitter's baud-rate generator, so eUSCI_A0, the device's only
//! UART, can't measure what it sends itself: the sender is a second MSP430FR2522, running this example with
//! `SENDER` set to true. Once a second it sends at 9600 baud, on P1.4: a break, the synch field 55h, then
//! `OK`, and its LED on P1.0 toggles. The receiver, running it with `SENDER` false, receives on P1.5, set up
//! for 11 000 baud, about 15 % too fast. It measures the synch field, takes on the sender's baud rate, and
//! receives `OK`. Its LEDs show the result: the one on P1.0 flashes for `OK`, the one on P2.2 for anything
//! else.
//! (Automatic baud-rate detection, and "The transmit baud-rate generator is used for the measurement":
//! SLAU445I 22.3.4, p. 580. Sending the break and synch field: SLAU445I 22.3.4.1, p. 581. "One eUSCI_A
//! supports UART, IrDA, and SPI": SLASEE4C 1.1, p. 1. UCA0TXD is P1.4 and UCA0RXD P1.5: SLASEE4C
//! Table 6-11, p. 53. No board document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! The receiver detects a break as 11 to 21 of its own bit times ("A break is detected when 11 or more
//! continuous zeros (spaces) are received. If the length of the break exceeds 21 bit times, the break
//! timeout error flag UCBTOE is set": SLAU445I 22.3.4, p. 580). The 13-bit break at 9600 baud is 15 bit
//! times at 11 000 baud, so the receiver's baud rate may be off by about -15 % to +60 %.
//!
//! How to test (two MSP430FR2522, three LEDs and resistors, two jumper wires, and optionally the scope):
//! 1. Power both MSP430FR2522 from 3.3 V. On the sender, connect an LED with a series resistor (about 1 kΩ)
//!    from P1.0 to GND; on the receiver, one from P1.0 to GND and one from P2.2 to GND.
//! 2. Connect the sender's P1.4 to the receiver's P1.5, and the sender's GND to the receiver's GND.
//! 3. Set `SENDER` to true and flash this example to the sender. Set it back to false and flash it to the
//!    receiver.
//! 4. Expected: the sender's LED toggles once a second, and each time the receiver's LED on P1.0 flashes.
//! 5. Set `AUTO_BAUD = false` and flash the receiver again: it is now a plain UART at 11 000 baud, and its
//!    LED on P2.2 flashes instead, as it can't read 9600 baud.
//! 6. With the scope on the sender's P1.4, ground on GND, 1 ms/div: the line low for 1.4 ms (the break),
//!    a short high (the delimiter), then the synch field 55h, whose bits alternate, and `OK`.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_hal_nb::serial::Read;
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

/// Which board this is: the sender, or the receiver
const SENDER: bool = false;
/// Measure the baud rate from the break and synch field, or receive as a plain UART
const AUTO_BAUD: bool = true;
/// The baud rate the sender uses
const SENDER_BAUD: u32 = 9600;
/// The baud rate the receiver starts with
const RECEIVER_BAUD: u32 = 11_000;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // The LEDs on P1.0 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let mut led = p1.pin0.to_output_low();
    let mut led_other = p2.pin2.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, so the CPU keeps up with the receiver, and ACLK from
    // REFO (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    if SENDER {
        // The sender: eUSCI_A0's TXD, P1.4, with P1SELx = 01 in the default mapping, USCIARMP = 0
        // (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). In automatic baud-rate mode
        // (UCMODEx = 11b), a break comes with a delimiter and the synch field (SLAU445I 22.3.4.1, p. 581);
        // 8 data bits, LSB first, no parity and one stop bit, as LIN needs (SLAU445I 22.3.4, p. 580).
        let mut sender = SerialConfig::<_, _, DefaultMapping>::new(
            periph.e_usci_a0,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            Parity::NoParity,
            Loopback::NoLoop,
            SENDER_BAUD,
        )
        .mode(UartMode::AutoBaud { delimiter: BreakDelimiter::_1Bit })
        .use_smclk(&smclk)
        .tx_only(p1.pin4.to_alternate1());

        loop {
            block!(sender.send_break()).ok();
            sender.write_all(b"OK").ok();
            led.toggle().ok();
            delay.delay_ms(1000);
        }
    } else {
        // The receiver: eUSCI_A0's RXD, P1.5, with P1SELx = 01 in the default mapping (SLASEE4C Table 6-11,
        // p. 53; SLASEE4C Table 6-15, p. 58). UCBRKIE makes the break and synch field readable, as
        // `RecvError::Break` (SLAU445I 22.3.4, p. 580: "If the UCBRKIE bit is set, reception of the
        // break/synch sets the UCRXIFG").
        let mode = if AUTO_BAUD { UartMode::AutoBaud { delimiter: BreakDelimiter::_1Bit } } else { UartMode::Uart };
        let mut receiver = SerialConfig::<_, _, DefaultMapping>::new(
            periph.e_usci_a0,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            Parity::NoParity,
            Loopback::NoLoop,
            RECEIVER_BAUD,
        )
        .mode(mode)
        .break_interrupts()
        .use_smclk(&smclk)
        .rx_only(p1.pin5.to_alternate1());

        loop {
            // While dormant, the receiver waits for a break and synch field (SLAU445I 22.3.4, p. 580: "If
            // UCDORM remains set, only the character after the next reception of a break/synch field is
            // received")
            if AUTO_BAUD {
                receiver.set_dormant(true);
            }
            // The sender's next break, up to a second away
            wait_for_break(&mut receiver);

            // Then the characters: "user software must reset UCDORM to continue receiving data" (SLAU445I
            // 22.3.4, p. 580)
            receiver.set_dormant(false);
            let mut received = [0u8; 2];
            let mut all = true;
            for byte in received.iter_mut() {
                match read_within(&mut receiver, &mut delay) {
                    Some(Ok(value)) => *byte = value,
                    _ => all = false,
                }
            }
            // Characters a plain UART at the wrong baud rate reads from the synch field may be left over
            while receiver.read().is_ok() {}
            // UCBTOE and UCSTOE, the break and synch timeout errors (SLAU445I Table 22-15, p. 598)
            let (break_too_long, synch_too_long) = receiver.auto_baud_errors();
            let ok = all && &received == b"OK" && !break_too_long && !synch_too_long;

            let led: &mut dyn OutputPin<Error = _> = if ok { &mut led } else { &mut led_other };
            led.set_high().ok();
            delay.delay_ms(100);
            led.set_low().ok();
        }
    }
}

/// Wait until the receiver reports a break
fn wait_for_break<USCI: SerialUsci>(receiver: &mut RxOnly<USCI>) {
    loop {
        // A plain UART at the wrong baud rate may read other things first
        if let Err(nb::Error::Other(RecvError::Break)) = receiver.read() {
            return;
        }
    }
}

/// The next character or error from the receiver, or `None` if nothing comes within 10 ms
fn read_within<USCI: SerialUsci>(receiver: &mut RxOnly<USCI>, delay: &mut impl DelayNs) -> Option<Result<u8, RecvError>> {
    for _ in 0..1000 {
        match receiver.read() {
            Ok(byte) => return Some(Ok(byte)),
            Err(nb::Error::Other(error)) => return Some(Err(error)),
            Err(nb::Error::WouldBlock) => delay.delay_us(10),
        }
    }
    None
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
