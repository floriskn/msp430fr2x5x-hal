//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The device descriptors (TLV): which device this is, and the calibration values measured in the
//! factory. The device checks the descriptors against their CRC and prints them on eUSCI_A0. An LED on P1.0
//! turns on if the CRC matches. It doesn't print the hardware revision: the data sheet lists it, but "This
//! device does not support reading the hardware revision from memory" (SLAZ705H 5.3, p. 4).
//! (The descriptors: SLASEE4C Table 6-18, p. 61 to p. 62. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. P1.0
//! is a GPIO output, P1SELx = 00 and P1DIR = 1: SLASEE4C Table 6-15, p. 58. No board document covers the
//! parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, and a 3.3-V USB-to-UART adapter):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND. Connect the adapter: its RX to
//!    P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Flash this example.
//! 3. Reset the device, with RST/NMI low for a moment, to print the descriptors again.
//!
//! Expected: `CRC: matches`, and the LED lights. The device ID is 8310 on an MSP430FR2522, 831C on an
//! MSP430FR2512 (SLASEE4C Table 6-17, p. 61). The other values are measured per chip ("Per unit" in
//! SLASEE4C Table 6-18, p. 61 to p. 62), so each device prints its own. The reference factor and the ADC
//! gain factor are close to 32768 (8000h), which means no correction (SLAU445I 1.13.3, p. 59 and SLAU445I
//! 1.13.3.2, p. 60: results are multiplied by the factor and divided by 2^15). How the CRC is computed was
//! found on an MSP430FR2476 (see `tlv::crc_matches`): if this device prints `DOES NOT MATCH`, report it.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    crc::Crc,
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::{Pmm, ReferenceVoltage},
    serial::*,
    tlv::{self, TempSensorCalibration},
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58; USCIA0RMP: SLAU445I Table 1-32, p. 83). 8N1: LSB
    // first, 8 data bits, one stop bit, no parity (UCMSB, UC7BIT, UCSPB, UCPEN: SLAU445I Table 22-8,
    // p. 593).
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

    // The CRC module checks the descriptors: the CRC-CCITT of 1A04h to 1AF5h, stored at 1A02h
    // (SLASEE4C Table 6-18 note 1, p. 61; SLASEE4C Table 6-18, p. 61)
    let mut crc = Crc::new(periph.crc, 0xFFFF);
    let crc_ok = tlv::crc_matches(&mut crc);

    // The information block, without the hardware revision, and the die record (SLASEE4C Table 6-18, p. 61)
    let die = tlv::die_record();
    print(&mut tx, "\r\nDevice descriptors (TLV)\r\nDevice ID:         ");
    print_hex(&mut tx, tlv::device_id() as u32, 4);
    print(&mut tx, "\r\nFirmware revision: ");
    print_hex(&mut tx, tlv::firmware_revision() as u32, 2);
    print(&mut tx, "\r\nLot wafer ID:      ");
    print_hex(&mut tx, die.lot_wafer_id, 8);
    print(&mut tx, "\r\nDie position:      X ");
    print_num(&mut tx, die.x_position as i32);
    print(&mut tx, ", Y ");
    print_num(&mut tx, die.y_position as i32);
    print(&mut tx, "\r\nTest result:       ");
    print_hex(&mut tx, die.test_result as u32, 4);

    // The ADC calibration, with the one reference level of this device, 1.5 V (SLASEE4C Table 6-18, p. 61;
    // how to use it: SLAU445I 1.13.3.2, p. 60 and SLAU445I 1.13.3.3, p. 60)
    print(&mut tx, "\r\nADC gain factor:   ");
    print_hex(&mut tx, tlv::adc_gain_factor() as u32, 4);
    print(&mut tx, "\r\nADC offset:        ");
    print_num(&mut tx, tlv::adc_offset() as i32);
    let temp = TempSensorCalibration::new(ReferenceVoltage::V1_5);
    print(&mut tx, "\r\n1.5 V reference: factor ");
    print_hex(&mut tx, tlv::reference_factor(ReferenceVoltage::V1_5) as u32, 4);
    print(&mut tx, ", temperature sensor ");
    print_num(&mut tx, temp.count_30c() as i32);
    print(&mut tx, " at 30 C and ");
    print_num(&mut tx, temp.count_high() as i32);
    print(&mut tx, " at ");
    print_num(&mut tx, temp.high_celsius() as i32);
    print(&mut tx, " C");

    // The DCO calibration (SLASEE4C Table 6-18, p. 62; SLAU445I 1.13.3.4, p. 60)
    print(&mut tx, "\r\nDCO tap, 16 MHz:   ");
    print_hex(&mut tx, tlv::dco_tap_16mhz() as u32, 4);
    print(&mut tx, if crc_ok { "\r\nCRC: matches\r\n" } else { "\r\nCRC: DOES NOT MATCH\r\n" });
    tx.flush().ok();

    led.set_state(crc_ok.into()).ok();

    loop {
        msp430::asm::nop();
    }
}

// Numbers are printed by hand: the formatting code of `write!` takes several KB, and this device has 7.25 KB
// of program FRAM (SLASEE4C Table 6-19, p. 62).

fn print(tx: &mut impl Write, text: &str) {
    tx.write_all(text.as_bytes()).ok();
}

/// Print `value` in decimal, with a minus sign if it's negative
fn print_num(tx: &mut impl Write, value: i32) {
    if value < 0 {
        print(tx, "-");
    }
    let mut digits = [0u8; 10];
    let mut pos = digits.len();
    let mut rest = value.unsigned_abs();
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

/// Print the low `digits` hexadecimal digits of `value`
fn print_hex(tx: &mut impl Write, value: u32, digits: u32) {
    for digit in (0..digits).rev() {
        let nibble = (value >> (4 * digit)) as u8 & 0xF;
        let c = if nibble < 10 { b'0' + nibble } else { b'A' + nibble - 10 };
        tx.write_all(&[c]).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
