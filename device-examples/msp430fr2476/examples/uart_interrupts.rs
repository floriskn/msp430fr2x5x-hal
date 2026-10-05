//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An echo on the backchannel UART that runs in its interrupt: each character you type comes back, while
//! the main loop only blinks LED1.
//!
//! The receive interrupt (UCRXIFG) keeps the character and turns the transmit interrupt on. The transmit
//! interrupt (UCTXIFG), which is requested while the transmit buffer is empty, sends the character back,
//! and turns itself off when there's nothing left to send. Both share eUSCI_A0's one interrupt vector.
//! (UCTXIFG and UCRXIFG: SLAU445I 22.3.15.1 and 22.3.15.2, p. 590 to p. 591; one vector: SLAU445I
//! 22.3.15, p. 590. The backchannel UART is eUSCI_A0, on P1.4 (TXD) and P1.5 (RXD): SLAU802 2.2.4, p. 9;
//! SLAU802 Figure 16, p. 22. LED1 on P1.0 is green: SLAU802 Figure 19, p. 25.)
//!
//! How to test:
//! 1. Flash this example, with the TXD and RXD jumpers of J101 on, and open the COM port of "MSP
//!    Application UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 2. Expected: LED1 blinks, 0.5 s on and 0.5 s off.
//! 3. Type something: each character comes back, so the terminal shows what you type (with its local
//!    echo off), and LED1 keeps blinking.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use embedded_hal_nb::serial::{Read, Write};
use msp430::interrupt::{enable as enable_interrupts, Mutex};
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
use msp430fr247x::{interrupt, EUsciA0};
use panic_msp430 as _;

static SERIAL: Mutex<RefCell<Option<(Tx<EUsciA0>, Rx<EUsciA0>)>>> = Mutex::new(RefCell::new(None));
/// The last character received, until it's sent back
static RECEIVED: Mutex<Cell<Option<u8>>> = Mutex::new(Cell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4 and RXD on P1.5, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9;
    // SLASEO7C Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
    let (tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    // UCRXIE (SLAU445I Table 22-17, p. 600); the interrupt turns UCTXIE on when it has a character to send.
    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33).
    rx.enable_rx_interrupts();
    with(|cs| SERIAL.borrow_ref_mut(cs).replace((tx, rx)));
    unsafe { enable_interrupts() };

    loop {
        led1.toggle().ok();
        delay.delay_ms(500);
    }
}

// The eUSCI_A0 vector (FFE0h: SLASEO7C Table 9-2, p. 46). Reading UCA0RXBUF clears UCRXIFG, and writing
// UCA0TXBUF clears UCTXIFG (SLAU445I 22.3.15.1 and 22.3.15.2, p. 590 to p. 591). If the other flag is set
// too, the interrupt comes again right after this one (SLAU445I 22.3.15.4, p. 591).
#[interrupt]
fn EUSCI_A0() {
    with(|cs| {
        if let Some((tx, rx)) = SERIAL.borrow_ref_mut(cs).as_mut() {
            let received = RECEIVED.borrow(cs);
            match rx.interrupt_source() {
                UartVector::RxBufFull => {
                    // A character with an error is dropped
                    if let Ok(byte) = rx.read() {
                        received.set(Some(byte));
                        tx.enable_tx_interrupts();
                    }
                }
                UartVector::TxBufEmpty => match received.take() {
                    Some(byte) => {
                        tx.write(byte).ok();
                    }
                    None => tx.disable_tx_interrupts(),
                },
                _ => {}
            }
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
