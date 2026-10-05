//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A frequency counter with two TA0 capture inputs: CCR1 on P1.1, read in TA0's interrupt through TA0IV
//! with `InterruptCapture::interrupt_capture()`, and CCR2 on P1.2, polled. Twice a second the backchannel
//! UART prints the frequency each one measures.
//!
//! TA0 counts SMCLK at about 8 MHz in continuous mode, and each input captures the count at every rising
//! edge, so a signal's period is the difference between two captures. The clocks come from REFO, which is
//! accurate to ±3.5 % (SLASE59F Table 5-7, p. 25), so the result is too. TA0's CCR0 has no input pin on
//! this device, so CCR1 uses the vector that TA0's CCR1, CCR2 and overflow share. P1.1 is also LED2's pin,
//! so LED2 glows while the signal is on.
//! (TA0.CCI1A is P1.1 and TA0.CCI2A is P1.2: SLASE59F Table 6-11, p. 50. "The CCR0 registers on
//! Timer0_A3 and Timer1_A3 are not externally connected": SLASE59F 6.10.8, p. 50. Captures: SLAU445I
//! 13.2.4.1, p. 374. TA0IV: SLAU445I 13.2.6.2, p. 380. LED2 on P1.1: SLAU739 Figure 18, p. 23.)
//!
//! How to test (function generator):
//! 1. Generator: square wave, 1 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load High-Z. Check the
//!    levels on the scope before connecting: a negative or >3.6 V signal can damage the pins.
//! 2. Connect it to P1.1 (J2 pin 19) and P1.2 (J1 pin 10), with the BNC T-piece and two leads, and its
//!    ground to GND (J2 pin 20). (Header pins: SLAU739 Figure 18, p. 23.)
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 4. Expected: `CCR1 (P1.1): 1000.0 Hz, CCR2 (P1.2): 1000.0 Hz`, give or take the clock's tolerance.
//!    Try 200 Hz to 20 kHz: below 123 Hz a period no longer fits in the 16-bit counter, and far above
//!    20 kHz the interrupt can't keep up with the edges.
//! 5. Disconnect one input: its value shows `no signal`.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    capture::{CapTrigger, Capture, CaptureParts3, CapturePin, CaptureVector, TBxIV, TimerConfig, CCR1, CCR2},
    clock::{Clock, ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use msp430fr2433::{interrupt, Ta0};
use panic_msp430 as _;

static CAPTURE1: Mutex<RefCell<Option<Capture<Ta0, CCR1>>>> = Mutex::new(RefCell::new(None));
static VECTOR: Mutex<RefCell<Option<TBxIV<Ta0>>>> = Mutex::new(RefCell::new(None));
/// The last capture of CCR1, and the period between the last two (0 until there are two)
static LAST1: Mutex<Cell<Option<u16>>> = Mutex::new(Cell::new(None));
static PERIOD1: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);
    let smclk_hz = smclk.freq();

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SELx = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
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

    // TA0 counts SMCLK (TASSEL = 10b: SLASE59F Table 6-7, p. 46) in continuous mode. Input A of CCR1 is
    // P1.1 and of CCR2 P1.2, both with P1SELx = 10 and P1DIR = 0 (SLASE59F Table 6-11, p. 50; SLASE59F
    // Table 6-17, p. 55). Both capture on rising edges (CM = 01b: SLAU445I Table 13-6, p. 386).
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::smclk(&smclk))
        .config_cap1_input_A(p1.pin1.to_alternate2())
        .config_cap1_trigger(CapTrigger::RisingEdge)
        .config_cap2_input_A(p1.pin2.to_alternate2())
        .config_cap2_trigger(CapTrigger::RisingEdge)
        .commit();
    let mut capture1 = captures.cap1;
    let mut capture2 = captures.cap2;

    // CCIE requests the interrupt for each capture of CCR1 (SLAU445I Table 13-6, p. 386); CCR2's stays
    // off, so TA0IV never reports it. Set GIE, which masks every maskable interrupt while clear (SLAU445I
    // 1.3.3, p. 33).
    capture1.enable_interrupts();
    with(|cs| {
        CAPTURE1.borrow_ref_mut(cs).replace(capture1);
        VECTOR.borrow_ref_mut(cs).replace(captures.tbxiv);
    });
    unsafe { enable_interrupts() };

    loop {
        delay.delay_ms(500);

        let period1 = with(|cs| PERIOD1.borrow(cs).replace(0));
        let period2 = poll_period(&mut capture2);

        write!(tx, "CCR1 (P1.1): ").ok();
        print_frequency(&mut tx, smclk_hz, period1);
        write!(tx, ", CCR2 (P1.2): ").ok();
        print_frequency(&mut tx, smclk_hz, period2);
        writeln!(tx, "\r").ok();
    }
}

/// The period between two captures of CCR2, in SMCLK cycles, or 0 if one doesn't come within 20 000 tries
fn poll_period(capture: &mut Capture<Ta0, CCR2>) -> u16 {
    // Throw away a capture from before
    let _ = capture.capture();
    let mut captures = [0u16; 2];
    for value in captures.iter_mut() {
        let mut tries = 0u16;
        *value = loop {
            match capture.capture() {
                Ok(count) => break count,
                // A second capture before the first was read: start again from this one
                Err(nb::Error::Other(over)) => break over.0,
                Err(nb::Error::WouldBlock) => {
                    tries += 1;
                    if tries == 20_000 {
                        return 0;
                    }
                }
            }
        };
    }
    captures[1].wrapping_sub(captures[0])
}

/// Print `smclk_hz / period` with one decimal, or `no signal` for a period of 0
fn print_frequency(tx: &mut impl Write, smclk_hz: u32, period: u16) {
    if period == 0 {
        write!(tx, "no signal").ok();
    } else {
        let decihertz = smclk_hz * 10 / period as u32;
        write!(tx, "{}.{} Hz", decihertz / 10, decihertz % 10).ok();
    }
}

// The TA0 vector of CCR1, CCR2 and the timer overflow (FFF6h: SLASE59F Table 6-2, p. 41). Reading TA0IV
// gives the highest pending source and clears its flag (SLAU445I 13.2.6.2, p. 380; SLAU445I Table 13-8,
// p. 388), so the capture is read with `interrupt_capture()`.
#[interrupt]
fn TIMER0_A1() {
    with(|cs| {
        let mut vector = VECTOR.borrow_ref_mut(cs);
        let mut capture = CAPTURE1.borrow_ref_mut(cs);
        let (Some(vector), Some(capture)) = (vector.as_mut(), capture.as_mut()) else { return; };
        if let CaptureVector::Capture1(cap) = vector.interrupt_vector() {
            let (count, valid) = match cap.interrupt_capture(capture) {
                Ok(count) => (count, true),
                // An edge was missed, so this period would be wrong
                Err(over) => (over.0, false),
            };
            if let (Some(last), true) = (LAST1.borrow(cs).get(), valid) {
                PERIOD1.borrow(cs).set(count.wrapping_sub(last));
            }
            LAST1.borrow(cs).set(Some(count));
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
