//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! UART breaks and the start-bit and transmit-complete interrupts, on eUSCI_A0 in loopback mode.
//!
//! Once a second eUSCI_A0 sends `Hi` and then a break: a character time with all bits low. Its receiver,
//! set up to report breaks, reads 'H', 'i' and then `RecvError::Break`. Meanwhile its interrupt counts
//! the start bits it receives and the characters it finishes sending, with `interrupt_source()`. eUSCI_A0
//! is the device's only UART, so LEDs show the results instead of a terminal: the one on P1.0 toggles when
//! the receiver read 'H', 'i' and the break, and the one on P2.2 when the interrupt counted three start
//! bits and three characters sent.
//! (Breaks: SLAU445I 22.3.3.2.1, p. 579. UCSTTIFG and UCTXCPTIFG: SLAU445I Table 22-6, p. 591. Loopback,
//! UCLISTEN: SLAU445I 22.4.5, p. 596. "One eUSCI_A supports UART, IrDA, and SPI": SLASEE4C 1.1, p. 1.
//! UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document covers the LEDs: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (two LEDs and resistors, and optionally the scope):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another from P2.2 to GND.
//! 2. Flash this example.
//! 3. Expected: both LEDs toggle together once a second, on for 1 s and off for 1 s.
//! 4. With the scope on eUSCI_A0's TXD, P1.4, ground on GND: two characters, then the line low for about
//!    1 ms, the break.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
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
use msp430fr25x2::{interrupt, EUsciA0};
use nb::block;
use panic_msp430 as _;

static RX: Mutex<RefCell<Option<Rx<EUsciA0>>>> = Mutex::new(RefCell::new(None));
/// Start bits received and characters sent since the last check
static COUNTS: Mutex<Cell<(u16, u16)>> = Mutex::new(Cell::new((0, 0)));

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
    let mut led_received = p1.pin0.to_output_low();
    let mut led_counts = p2.pin2.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0 in loopback mode, reporting breaks (UCBRKIE: SLAU445I Table 22-8, p. 593), and ignoring
    // pulses on RXD shorter than about 50 ns (UCGLITx = 01b: SLAU445I Table 22-9, p. 594). Its pins are P1.4
    // (TXD) and P1.5 (RXD), with P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C Table 6-11,
    // p. 53; SLASEE4C Table 6-15, p. 58).
    let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::Loopback,
        9600,
    )
    .break_interrupts()
    .deglitch(UartDeglitch::_50ns)
    .use_smclk(&smclk)
    .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

    // UCSTTIE and UCTXCPTIE (SLAU445I Table 22-17, p. 600). Set GIE, which masks every maskable interrupt
    // while clear (SLAU445I 1.3.3, p. 33).
    rx.enable_start_bit_interrupts();
    tx.enable_tx_complete_interrupts();
    with(|cs| RX.borrow_ref_mut(cs).replace(rx));
    unsafe { enable_interrupts() };

    loop {
        let mut received = [Ok(0); 3];
        for (&byte, slot) in b"Hi".iter().zip(received.iter_mut()) {
            tx.write_all(&[byte]).ok();
            *slot = read();
        }
        // All bits low for a character time (UCTXBRK: SLAU445I 22.3.3.2.1, p. 579)
        block!(tx.send_break()).ok();
        received[2] = read();

        // UCTXCPTIFG is only set once the whole character, stop bit included, has been shifted out
        // (SLAU445I Table 22-6, p. 591), which may be after the receiver took it in: wait, then count
        delay.delay_ms(1000);
        let counts = with(|cs| COUNTS.borrow(cs).replace((0, 0)));
        if matches!(received, [Ok(b'H'), Ok(b'i'), Err(RecvError::Break)]) {
            led_received.toggle().ok();
        }
        // Three start bits received and three characters sent, the break included
        if counts == (3, 3) {
            led_counts.toggle().ok();
        }
    }
}

/// Wait for the next character, or error, from eUSCI_A0's receiver
fn read() -> Result<u8, RecvError> {
    loop {
        let result = with(|cs| RX.borrow_ref_mut(cs).as_mut().unwrap().read());
        match result {
            Err(nb::Error::WouldBlock) => continue,
            Err(nb::Error::Other(error)) => return Err(error),
            Ok(byte) => return Ok(byte),
        }
    }
}

// The eUSCI_A0 vector (FFECh: SLASEE4C Table 6-2, p. 46). `interrupt_source()` clears UCSTTIFG and
// UCTXCPTIFG, as a UCA0IV read does (SLAU445I 22.3.15.4, p. 591).
#[interrupt]
fn EUSCI_A0() {
    with(|cs| {
        if let Some(rx) = RX.borrow_ref_mut(cs).as_mut() {
            let counts = COUNTS.borrow(cs);
            loop {
                let (start_bits, sent) = counts.get();
                match rx.interrupt_source() {
                    UartVector::StartBit => counts.set((start_bits + 1, sent)),
                    UartVector::TxComplete => counts.set((start_bits, sent + 1)),
                    _ => break,
                }
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
