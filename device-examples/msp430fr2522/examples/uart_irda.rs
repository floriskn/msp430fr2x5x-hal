//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! IrDA encoding: eUSCI_A0 sends each 0 bit as a short pulse, 3/16 of a bit time, as an infrared
//! transceiver needs, and decodes such pulses when receiving. In loopback mode it receives what it sends,
//! and an LED on P1.0 toggles each time `IrDA` came back whole.
//!
//! eUSCI_A0 is the device's only UART, so the LED shows the result instead of a terminal.
//! (IrDA encoding and decoding: SLAU445I 22.3.5, p. 581. Loopback, UCLISTEN: SLAU445I 22.4.5, p. 596.
//! "One eUSCI_A supports UART, IrDA, and SPI": SLASEE4C 1.1, p. 1. UCA0TXD is P1.4: SLASEE4C Table 6-11,
//! p. 53. No board document covers the LED: there is none for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, and optionally the scope):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Flash this example. Expected: the LED blinks, toggling about five times a second. It stops when a
//!    character comes back wrong.
//! 3. Probe eUSCI_A0's TXD, P1.4, ground clip on GND. Trigger on the rising edge, 200 µs/div. Instead of a
//!    normal UART signal, which is high when idle, the line is low, with short high pulses of about 20 µs:
//!    one for each 0 bit, 104 µs apart at 9600 baud. The start bit is a 0, so each character begins with a
//!    pulse. (SLAU445I Figure 22-7, p. 581.)
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
use panic_msp430 as _;

const MESSAGE: &[u8; 4] = b"IrDA";

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

    // eUSCI_A0 with IrDA encoding: pulses of 3/16 of a bit time, counted in BITCLK16 (UCIRTXPLx = 5,
    // UCIRTXCLK = 1: SLAU445I 22.3.5.1, p. 581), in loopback mode. 1 MHz is more than 16 × 9600 baud, so
    // the baud-rate generator oversamples, which BITCLK16 needs (SLAU445I Table 22-16, p. 599). Its pins
    // are P1.4 (TXD) and P1.5 (RXD), with P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58).
    let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::Loopback,
        9600,
    )
    .irda(IrdaConfig::standard())
    .use_smclk(&smclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    loop {
        let mut received = [0u8; 4];
        for (&byte, slot) in MESSAGE.iter().zip(received.iter_mut()) {
            tx.write_all(&[byte]).ok();
            // Each character arrives back before the next is sent
            *slot = nb::block!(rx.read()).unwrap_or(0);
        }
        if received == *MESSAGE {
            led.toggle().ok();
        }
        delay.delay_ms(200);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
