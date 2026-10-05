//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Timer captures, polled: TA0 captures each press of button S2, which reaches P1.2 through a jumper wire,
//! and the backchannel UART prints the time since the press before, in REFO cycles.
//!
//! P1.2 is TA0.CCI2A, input A of TA0's CCR2. S2's pin, P2.7, has no timer function, so a jumper wire
//! connects it to P1.2: S2 pulls both low when pressed, and P2.7's internal pullup pulls them high again.
//! CCR2 captures TA0's count at each falling edge. TA0 counts ACLK from REFO, 32768 Hz, so a second is
//! 32768 counts, 0x8000; the count wraps around after 65536 counts, 2 s. Each capture also turns LED1 on.
//! (TA0.CCI2A on P1.2: SLASE59F Table 6-11, p. 50; SLASE59F Table 6-17, p. 55. P2.7 is a GPIO only:
//! SLASE59F Table 6-19, p. 58. Captures: SLAU445I 13.2.4.1, p. 374. REFO: SLASE59F Table 5-7, p. 25. S2
//! on P2.7, with no pull-up on the board, and LED1 on P1.0, red: SLAU739 Figure 18, p. 23.)
//!
//! How to test (a jumper wire):
//! 1. Connect P2.7 (J1 pin 8) to P1.2 (J1 pin 10) with a jumper wire. (Header pins: SLAU739 Figure 18,
//!    p. 23.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 3. Press S2 about once a second. Expected: a line per press with the REFO cycles since the press
//!    before, in hex: about `0x8000` when the presses are a second apart. The first press counts from the
//!    start, and presses more than 2 s apart wrap around. LED1 is on after the first press.
//! 4. S2 isn't debounced, so a press or a release can print an extra line with a small value, or `!`
//!    when a second edge came before the first was read.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_hal_nb::serial::Write;
use msp430_rt::entry;
use msp430_hal::{
    capture::{CapTrigger, CaptureParts3, OverCapture, TimerConfig},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // P1.0 drives LED1 (SLAU739 Figure 18, p. 23), switched on at the first capture
    let mut p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    // S2 on P2.7 with the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313),
    // which also pulls P1.2 high through the jumper wire
    let _p2 = Batch::new(periph.p2)
        .config_pin7(|p| p.pullup())
        .split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0 TXD is P1.4 with P1SELx = 01 (SLASE59F Table 6-17, p. 55)
    // (LSB first, 8 data bits, one stop bit, no parity: UCMSB, UC7BIT, UCSPB, UCPEN in SLAU445I
    // Table 22-8, p. 593; SMCLK is UCSSEL = 10b: SLASE59F Table 6-7, p. 46)
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

    // TA0 counts ACLK (TASSEL = 01b: SLASE59F Table 6-7, p. 46). Its CCR2 input A (CCI2A) is P1.2, with
    // P1SELx = 10 and P1DIR = 0 (SLASE59F Table 6-11, p. 50; SLASE59F Table 6-17, p. 55). A capture on the
    // falling edge (CM = 10b, CCIS = 00b: SLAU445I Table 13-6, p. 386) copies TA0R into TA0CCR2 and sets
    // CCIFG (SLAU445I 13.2.4.1, p. 374); a second capture before the first is read sets COV (SLAU445I
    // 13.2.4.1, p. 375), reported as `OverCapture`.
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::aclk(&aclk))
        .config_cap2_input_A(p1.pin2.to_alternate2())
        .config_cap2_trigger(CapTrigger::FallingEdge)
        .commit();
    let mut capture = captures.cap2;

    let mut last_cap = 0;
    loop {
        match block!(capture.capture()) {
            Ok(cap) => {
                let diff = cap.wrapping_sub(last_cap);
                last_cap = cap;
                p1.pin0.set_high().unwrap();
                print_num(&mut tx, diff);
            }
            Err(OverCapture(_)) => {
                p1.pin0.set_high().unwrap();
                write(&mut tx, '!');
                write(&mut tx, '\r');
                write(&mut tx, '\n');
            }
        }
    }
}

fn print_num<U: SerialUsci>(tx: &mut Tx<U>, num: u16) {
    write(tx, '0');
    write(tx, 'x');
    print_hex(tx, num >> 12);
    print_hex(tx, (num >> 8) & 0xF);
    print_hex(tx, (num >> 4) & 0xF);
    print_hex(tx, num & 0xF);
    write(tx, '\r');
    write(tx, '\n');
}

fn print_hex<U: SerialUsci>(tx: &mut Tx<U>, h: u16) {
    let c = match h {
        0 => '0',
        1 => '1',
        2 => '2',
        3 => '3',
        4 => '4',
        5 => '5',
        6 => '6',
        7 => '7',
        8 => '8',
        9 => '9',
        10 => 'a',
        11 => 'b',
        12 => 'c',
        13 => 'd',
        14 => 'e',
        15 => 'f',
        _ => '?',
    };
    write(tx, c);
}

fn write<U: SerialUsci>(tx: &mut Tx<U>, ch: char) {
    nb::block!(tx.write(ch as u8)).unwrap();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
