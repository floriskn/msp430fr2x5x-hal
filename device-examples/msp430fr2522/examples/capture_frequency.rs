//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A frequency counter with two TA1 capture inputs: CCR0, read in CCR0's own interrupt with
//! `Capture::interrupt_capture()`, and CCR1 on P2.2, polled. Twice a second eUSCI_A0 prints the frequency
//! each one measures.
//!
//! TA1 counts SMCLK at about 8 MHz in continuous mode, and each input captures the count at every rising
//! edge, so a signal's period is the difference between two captures. CCR0 has no pin on this device: its
//! input B is the output of TA0's CCR0, which TA0 toggles every 32 ACLK cycles, a 512 Hz square wave. SMCLK
//! and ACLK both come from REFO, so CCR0 measures 512 Hz however far REFO is off, a check of the
//! measurement itself. CCR1 measures the function generator, as accurately as REFO runs (±3.5 %: SLASEE4C
//! Table 5-7, p. 27).
//! (TA1.CCI1A is P2.2, and TA1's CCR0 input B is TA0's CCR0 output: SLASEE4C Figure 6-2, p. 54. "The CCR0
//! registers on both Timer0_A3 and Timer1_A3 are not externally connected": SLASEE4C 6.10.8, p. 54.
//! Captures: SLAU445I 13.2.4.1, p. 374. CCR0's interrupt clears its own flag: SLAU445I 13.2.6.1, p. 380.
//! Toggle mode: SLAU445I Table 13-2, p. 376. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document
//! covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, the scope, and a 3.3-V USB-to-UART adapter):
//! 1. Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Generator: square wave, 1 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load High-Z. Check the
//!    levels on the scope before connecting: a negative or >3.6 V signal can damage the pin.
//! 3. Connect it to P2.2, its ground to GND.
//! 4. Flash this example.
//! 5. Expected: `CCR0 (TA0 output): 512.0 Hz, CCR1 (P2.2): 1000.0 Hz`, give or take 0.1 Hz for CCR0 and
//!    the clock's tolerance for CCR1. Try 200 Hz to 20 kHz: below 123 Hz a period no longer fits in the
//!    16-bit counter.
//! 6. Disconnect the generator: CCR1's value shows `no signal`.
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
    capture::{CapTrigger, Capture, CaptureParts3, CapturePin, TimerConfig, CCR0, CCR1},
    clock::{Clock, ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    pwm::PwmParts3,
    serial::*,
    watchdog::Wdt,
};
use msp430fr25x2::{interrupt, Ta1};
use panic_msp430 as _;

static CAPTURE0: Mutex<RefCell<Option<Capture<Ta1, CCR0>>>> = Mutex::new(RefCell::new(None));
/// The last capture of CCR0, and the period between the last two (0 until there are two)
static LAST0: Mutex<Cell<Option<u16>>> = Mutex::new(Cell::new(None));
static PERIOD0: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);
    let smclk_hz = smclk.freq();

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0, 8N1
    // (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::<_, _, DefaultMapping>::new(
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

    // TA0 counts ACLK (TASSEL = 01b: SLAU445I Table 13-4, p. 384) up to 31, and its CCR0 output toggles at
    // the end of each count: a period of 64 ACLK cycles (output mode toggle, "The output period is double
    // the timer period": SLAU445I Table 13-2, p. 376)
    let _ta0 = PwmParts3::new(periph.ta0, TimerConfig::aclk(&aclk), 31);

    // TA1 counts SMCLK (TASSEL = 10b: SLAU445I Table 13-4, p. 384) in continuous mode. CCR0 takes TA0's CCR0
    // output on its input B (CCIS = 01b), and CCR1 P2.2 on its input A (P2SELx = 01: SLASEE4C Table 6-16,
    // p. 60). Both capture on rising edges (CM = 01b: SLAU445I Table 13-6, p. 386).
    let captures = CaptureParts3::config(periph.ta1, TimerConfig::smclk(&smclk))
        .config_cap0_input_B()
        .config_cap0_trigger(CapTrigger::RisingEdge)
        .config_cap1_input_A(p2.pin2.to_alternate1())
        .config_cap1_trigger(CapTrigger::RisingEdge)
        .commit();
    let mut capture0 = captures.cap0;
    let mut capture1 = captures.cap1;

    // CCIE requests CCR0's interrupt for each capture (SLAU445I Table 13-6, p. 386). Set GIE, which masks
    // every maskable interrupt while clear (SLAU445I 1.3.3, p. 33).
    capture0.enable_interrupts();
    with(|cs| CAPTURE0.borrow_ref_mut(cs).replace(capture0));
    unsafe { enable_interrupts() };

    loop {
        delay.delay_ms(500);

        let period0 = with(|cs| PERIOD0.borrow(cs).replace(0));
        let period1 = poll_period(&mut capture1);

        print(&mut tx, "CCR0 (TA0 output): ");
        print_frequency(&mut tx, smclk_hz, period0);
        print(&mut tx, ", CCR1 (P2.2): ");
        print_frequency(&mut tx, smclk_hz, period1);
        print(&mut tx, "\r\n");
    }
}

/// The period between two captures of CCR1, in SMCLK cycles, or 0 if one doesn't come within 20 000 tries
fn poll_period(capture: &mut Capture<Ta1, CCR1>) -> u16 {
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
        print(tx, "no signal");
    } else {
        let decihertz = smclk_hz * 10 / period as u32;
        print_num(tx, decihertz / 10);
        print(tx, ".");
        print_num(tx, decihertz % 10);
        print(tx, " Hz");
    }
}

// Numbers are printed by hand: the formatting code of `write!` takes several KB, and this device has 7.25 KB
// of program FRAM (SLASEE4C Table 6-19, p. 62).

fn print(tx: &mut impl Write, text: &str) {
    tx.write_all(text.as_bytes()).ok();
}

/// Print `value` in decimal
fn print_num(tx: &mut impl Write, value: u32) {
    let mut digits = [0u8; 10];
    let mut pos = digits.len();
    let mut rest = value;
    loop {
        pos -= 1;
        digits[pos] = b'0' + (rest % 10) as u8;
        rest /= 10;
        if rest == 0 {
            break;
        }
    }
    tx.write_all(&digits[pos..]).ok();
}

// The TA1 CCR0 vector, CCIFG0 (FFF4h: SLASEE4C Table 6-2, p. 46). Servicing it clears CCIFG0 (SLAU445I
// 13.2.6.1, p. 380), so the capture is read with `interrupt_capture()`.
#[interrupt]
fn TIMER1_A0() {
    with(|cs| {
        if let Some(capture) = CAPTURE0.borrow_ref_mut(cs).as_mut() {
            let (count, valid) = match capture.interrupt_capture() {
                Ok(count) => (count, true),
                // An edge was missed, so this period would be wrong
                Err(over) => (over.0, false),
            };
            if let (Some(last), true) = (LAST0.borrow(cs).get(), valid) {
                PERIOD0.borrow(cs).set(count.wrapping_sub(last));
            }
            LAST0.borrow(cs).set(Some(count));
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
