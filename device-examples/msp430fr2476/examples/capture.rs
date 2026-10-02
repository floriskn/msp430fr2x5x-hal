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

// Connect push button input to P1.1. When button is pressed, putty should print the # of cycles
// since the last press. Sometimes we get 2 consecutive readings due to lack of debouncing.
// P1.1 is J3 pin 28 (SLAU802 Figure 10, p. 13); the output of the TMP235 temperature sensor is wired to
// it too (SLAU802 2.2.5.1, p. 10). The text goes out on eUSCI_A1's TXD, P2.6 (J1 pin 4), which is not
// the backchannel UART: that is eUSCI_A0 on this LaunchPad (SLAU802 2.2.4, p. 9). For PuTTY, pull the
// TXD jumper off J101 and wire J1 pin 4 to the jumper's eZ-FET side, the pin nearer the USB connector
// (SLAU802 Table 2, p. 8; SLAU802 Figure 16, p. 22; SLAU802 Figure 1, p. 1).
#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // P1.0 drives LED1 (SLAU802 Figure 19, p. 25), switched on at the first capture
    let mut p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);

    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut fram);

    // eUSCI_A1 TXD is P2.6 with P2SEL = 01 (SLASEO7C Table 9-24, p. 66; SLASEO7C Table 9-11, p. 54)
    let mut tx = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p2.pin6.to_alternate1());

    // TA0 counts ACLK, here from the VLO. Its CCR1 input A (CCI1A) is P1.1 with P1SEL = 10 and P1DIR = 0
    // (SLASEO7C Table 9-12, p. 55; SLASEO7C Table 9-23, p. 65).
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::aclk(&aclk))
        .config_cap1_input_A(p1.pin1.to_alternate2())
        .config_cap1_trigger(CapTrigger::FallingEdge)
        .commit();
    let mut capture = captures.cap1;

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
