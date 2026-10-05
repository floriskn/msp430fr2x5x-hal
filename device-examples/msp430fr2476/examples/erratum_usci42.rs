//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum USCI42, and of `flush()`, which works around it: the UART sets its transmit complete
//! flag UCTXCPTIFG after every character, not only once the last one has gone out. The backchannel UART
//! sends a test line again and again in loopback mode, so its own receiver gets every character too, and
//! counts them. When `flush()` returns, the receiver must have the whole line. Every 32 lines LED1 toggles
//! and the UART prints how often the wait ended early, which must stay at 0.
//!
//! The erratum: "UCTXCPTIFG flag is triggered at the last stop bit of every UART byte transmission,
//! independently of an empty buffer, when transmitting multiple byte sequences via UART", with no
//! workaround (SLAZ726B USCI42, p. 8). In the user's guide "UCTXCPTIFG is set when the entire byte in the
//! internal shift register is shifted out and UCAxTXBUF is empty" (SLAU445I Table 22-18, p. 601), so a
//! flush that waits for it would return while the last characters are still being sent. The HAL's
//! `flush()` waits for UCTXIFG and for UCBUSY to clear instead (see the HAL's `serial` module).
//!
//! In loopback mode the transmitter's output feeds the receiver and still drives TXD, so the terminal shows
//! the lines (UCLISTEN: SLAU445I Figure 22-1, p. 576). UCBUSY also covers receiving, "transmitting or
//! receiving" (SLAU445I Table 22-12, p. 596), so when `flush()` returns, the receive interrupt has counted
//! the last character.
//! (eUSCI_A0 is the backchannel UART, TXD on P1.4 and RXD on P1.5 with P1SEL = 01: SLAU802 2.2.4, p. 9;
//! SLASEO7C Table 9-23, p. 65. LED1 on P1.0 is green: SLAU802 Figure 19, p. 25.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 2. Expected: lines of `USCI42 test line: 0123456789`, and after every 32 of them `lines: 32, early: 0`,
//!    `lines: 64, early: 0` and so on, with LED1 toggling at each count. It counts about once a second.
//! 3. Pass: `early` stays at 0. Fail: it goes up. A minute is enough, as the erratum doesn't depend on
//!    timing.
//! 4. To see the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the loop then waits for
//!    UCTXCPTIFG instead of calling `flush()`, and every line ends early: `lines: 32, early: 32`.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::digital::*;
use embedded_hal_nb::serial::Read;
use embedded_io::Write;
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

/// `false` waits for UCTXCPTIFG instead of calling `flush()`, to see the erratum
const USE_HAL_WORKAROUND: bool = true;

/// The test line, sent back to back
const LINE: &[u8] = b"USCI42 test line: 0123456789\r\n";

static RX: Mutex<RefCell<Option<Rx<EUsciA0>>>> = Mutex::new(RefCell::new(None));
/// Characters received since the line began
static RECEIVED: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range, and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART in loopback mode, 8N1 (UCLISTEN: SLAU445I Table 22-12, p. 596; SLAU445I
    // Table 22-8, p. 593)
    let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::Loopback,
        9600,
    )
    .use_smclk(&smclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    // UCRXIE: an interrupt for each character received (SLAU445I Table 22-17, p. 600). Set GIE, which masks
    // every maskable interrupt while clear (SLAU445I 1.3.3, p. 33).
    rx.enable_rx_interrupts();
    with(|cs| RX.borrow_ref_mut(cs).replace(rx));
    unsafe { enable_interrupts() };

    let usci = unsafe { &*EUsciA0::ptr() };
    let mut lines: u32 = 0;
    let mut early: u32 = 0;
    loop {
        with(|cs| RECEIVED.borrow(cs).set(0));
        if USE_HAL_WORKAROUND {
            tx.write_all(LINE).ok();
            tx.flush().ok();
        } else {
            // Clear UCTXCPTIFG, send the line, and wait for UCTXCPTIFG (SLAU445I Table 22-18, p. 601)
            unsafe { usci.uca0ifg().clear_bits(|w| w.uctxcptifg().clear_bit()) };
            tx.write_all(LINE).ok();
            while usci.uca0ifg().read().uctxcptifg().bit_is_clear() {}
        }
        if with(|cs| RECEIVED.borrow(cs).get()) != LINE.len() as u16 {
            early += 1;
        }
        // Let the line finish before the next one starts, whatever the wait above did
        tx.flush().ok();

        lines += 1;
        if lines % 32 == 0 {
            led1.toggle().ok();
            writeln!(tx, "lines: {}, early: {}\r", lines, early).ok();
            tx.flush().ok();
        }
    }
}

// The eUSCI_A0 vector (FFE0h: SLASEO7C Table 9-2, p. 46). Reading the character clears UCRXIFG (SLAU445I
// 22.3.15.2, p. 591).
#[interrupt]
fn EUSCI_A0() {
    with(|cs| {
        if let Some(rx) = RX.borrow_ref_mut(cs).as_mut() {
            if !matches!(rx.read(), Err(nb::Error::WouldBlock)) {
                let received = RECEIVED.borrow(cs);
                received.set(received.get() + 1);
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
