//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! eUSCI_A1's inverted UART, and how the same pins give it. P4.3 and P4.2 each have three functions,
//! chosen by their two P4SEL bits (SLASEC4D Table 6-66, p. 102):
//! - `00`: a GPIO pin;
//! - `01` (`to_alternate1()`): eUSCI_A1's TXD (P4.3) or RXD (P4.2), the usual UART: the line idles high,
//!   and a character starts with a falling edge;
//! - `10` (`to_alternate2()`): the same TXD or RXD, with the signal inverted between eUSCI_A1 and the pin:
//!   the line idles low, and a character starts with a rising edge ("When PSEL = 10b, the inverted UART mode
//!   is enabled to transmit and receive data in inverted polarity", SLASEC4D 6.10.8, p. 73).
//!
//! eUSCI_A1 itself is the same in both; only the pins' function differs, and `split()` takes the pins in
//! either. `POLARITY` below chooses the functions:
//! - `Normal`: TXD and RXD in function 1. The bytes come back.
//! - `Inverted`: TXD and RXD in function 2. The bytes come back too, as the line is inverted twice.
//! - `Mixed`: TXD in function 1, RXD in function 2. The receiver sees the line upside down, so the bytes
//!   don't come back.
//!
//! Ten times a second the example sends a byte through a wire from TXD to RXD and reads it back: LED2 lights
//! while the bytes come back, LED1 while they don't.
//!
//! P4.3 and P4.2 are also the backchannel UART's pins, which go to the debug probe through the TXD and RXD
//! jumpers of J101; the test takes those jumpers off.
//! (SLASEC4D Table 6-15, p. 73 names P4.4 for RXD, but UCA1RXD is P4.2 in the pin function table, which the
//! HAL follows. The backchannel UART is on P4.3 (TXD) and P4.2 (RXD), LED1 on P1.0 is red and LED2 on P6.6
//! green: SLAU680 Figure 18, p. 26.)
//!
//! How to test (a jumper wire, a multimeter, and optionally the scope):
//! 1. Take the TXD and RXD jumpers of J101 off, and connect J101 pins 6 and 8 with a jumper wire: the TXD
//!    and RXD pins on the MSP430 side, away from the USB connector. The RXD jumper must be off, or the wire
//!    ties P4.3 to the debug probe's TXD output on pin 7. (J101 pin 6 is BCLUART_TXD, pin 8 BCLUART_RXD
//!    and pin 7 EZFET_UARTTXD: SLAU680 Figure 17, p. 25; BCLUART_TXD and BCLUART_RXD are P4.3 and P4.2:
//!    SLAU680 Figure 18, p. 26. Probe and target sides of J101: SLAU680 Figure 6, p. 10; board layout:
//!    SLAU680 Figure 1, p. 1.)
//! 2. Flash the example as it is, with `POLARITY` `Inverted`. Expected: LED2 lights. The multimeter between
//!    J101 pin 6 (P4.3) and GND (J3 pin 22) shows about 0 V: the inverted line idles low. The bytes take
//!    about 1 % of the time, so the meter shows the idle level.
//! 3. Set `POLARITY` to `Normal` and flash again. Expected: LED2 lights, and the meter shows about 3.3 V.
//! 4. Set `POLARITY` to `Mixed` and flash again. Expected: LED1 lights. If LED2 lights, the two pins aren't
//!    inverted separately: please report it.
//! 5. Pull the wire off: LED1 lights whatever `POLARITY` is. Put it back.
//! 6. Optional, with `Inverted`: scope on J101 pin 6, ground clip on GND (J3 pin 22), 200 µs/div, trigger on
//!    a rising edge at 1.5 V in normal mode. The line is low between characters, and each character starts
//!    with a high start bit, 104 µs long at 9600 baud, and ends with a low stop bit.
//! 7. Put the TXD and RXD jumpers back for the examples that use the backchannel UART.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use embedded_hal_nb::serial::{Read, Write};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The pin functions to test, see the steps above
const POLARITY: Polarity = Polarity::Inverted;

/// TXD's and RXD's pin functions
#[allow(dead_code)]
enum Polarity {
    /// TXD and RXD in function 1, P4SELx = 01: the usual UART (SLASEC4D Table 6-66, p. 102)
    Normal,
    /// TXD and RXD in function 2, P4SELx = 10: both inverted (SLASEC4D 6.10.8, p. 73)
    Inverted,
    /// TXD in function 1, RXD in function 2: the receiver sees the line upside down
    Mixed,
}

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let p6 = Batch::new(periph.p6).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p6.pin6.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A1 at 9600 baud, 8N1, from SMCLK (SLAU445I Table 22-8, p. 593), on P4.3 (TXD) and P4.2 (RXD)
    // in the functions `POLARITY` chooses
    let config = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk);
    let (mut tx, mut rx) = match POLARITY {
        Polarity::Normal => config.split(p4.pin3.to_alternate1(), p4.pin2.to_alternate1()),
        Polarity::Inverted => config.split(p4.pin3.to_alternate2(), p4.pin2.to_alternate2()),
        Polarity::Mixed => config.split(p4.pin3.to_alternate1(), p4.pin2.to_alternate2()),
    };

    let mut byte: u8 = 0;
    loop {
        block!(tx.write(byte)).ok();
        // A character takes about 1 ms at 9600 baud
        delay.delay_ms(5);
        let came_back = matches!(rx.read(), Ok(received) if received == byte);
        led2.set_state(came_back.into()).ok();
        led1.set_state((!came_back).into()).ok();

        byte = byte.wrapping_add(1);
        delay.delay_ms(95);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
