//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! eUSCI_A1's inverted UART: in alternate function 2 its TXD and RXD pins, P4.3 and P4.2, have the
//! opposite polarity, so the line idles low and a character starts with a rising edge. Ten times a second
//! the example sends a byte and reads it back through a wire from TXD to RXD: LED2 lights while each byte
//! comes back unchanged, LED1 when one doesn't.
//!
//! P4.3 and P4.2 are also the backchannel UART's pins, which go to the debug probe through the TXD and RXD
//! jumpers of J101; the test takes those jumpers off.
//! (Inverted UART mode, P4SELx = 10b: SLASEC4D 6.10.8, p. 73; SLASEC4D Table 6-15, p. 73; SLASEC4D
//! Table 6-66, p. 102. Table 6-15 names P4.4 for RXD, but UCA1RXD is P4.2 in the pin function table, which
//! the HAL follows. The backchannel UART is on P4.3 (TXD) and P4.2 (RXD), LED1 on P1.0 is red and LED2 on
//! P6.6 green: SLAU680 Figure 18, p. 26.)
//!
//! How to test (a jumper wire, and optionally the scope):
//! 1. Take the TXD and RXD jumpers of J101 off, and connect J101 pins 6 and 8 with a jumper wire: the TXD
//!    and RXD pins on the MSP430 side, away from the USB connector. The RXD jumper must be off, or the wire
//!    ties P4.3 to the debug probe's TXD output on pin 7. (J101 pin 6 is BCLUART_TXD, pin 8 BCLUART_RXD
//!    and pin 7 EZFET_UARTTXD: SLAU680 Figure 17, p. 25; BCLUART_TXD and BCLUART_RXD are P4.3 and P4.2:
//!    SLAU680 Figure 18, p. 26. Probe and target sides of J101: SLAU680 Figure 6, p. 10; board layout:
//!    SLAU680 Figure 1, p. 1.)
//! 2. Flash this example. Expected: LED2 lights, and LED1 stays off.
//! 3. Pull the wire off: LED2 goes out and LED1 lights. Put it back: LED2 lights again.
//! 4. Scope on J101 pin 6, ground clip on GND (J3 pin 22), 200 µs/div, trigger on a rising edge at 1.5 V in
//!    normal mode: the line is low between characters, and each character starts with a high start bit,
//!    104 µs long at 9600 baud, and ends with a low stop bit.
//! 5. Put the TXD and RXD jumpers back for the examples that use the backchannel UART.
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

    // eUSCI_A1 at 9600 baud, 8N1, with the inverted TXD on P4.3 and the inverted RXD on P4.2: P4SELx = 10,
    // alternate function 2 (SLASEC4D Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
    let (mut tx, mut rx) = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .split(p4.pin3.to_alternate2(), p4.pin2.to_alternate2());

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
