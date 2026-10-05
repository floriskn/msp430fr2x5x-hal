//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The device descriptors (TLV): which device this is, and the calibration values measured in the
//! factory. The board checks the descriptors against their CRC and prints them on the backchannel UART.
//! LED1 turns on if the CRC matches.
//! (The descriptors: SLASEC4D Table 6-70, p. 107 to p. 108. LED1 on P1.0 is red: SLAU680 Figure 18,
//! p. 26.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on (SLAU680 Table 2, p. 10).
//! 2. In the Windows Device Manager, find the COM port of "MSP Application UART1", and open it at
//!    9600 baud in a serial terminal such as PuTTY (connection type Serial) (SLAU680 2.2.4, p. 11).
//! 3. Press the reset button S3 to print the descriptors again.
//!
//! Expected: `CRC: matches`, device ID 830C, the MSP430FR2355's (SLASEC4D Table 6-69, p. 107), and hardware
//! revision 20 on revision B of the die (SLAZ695J 5.3, p. 5). The other values are measured per chip
//! ("Per unit" in SLASEC4D Table 6-70, p. 107 to p. 108), so each board prints its own. Reference factors
//! and the ADC gain factor are close to 32768 (8000h), which means no correction (SLAU445I 1.13.3, p. 59 and
//! SLAU445I 1.13.3.2, p. 60: results are multiplied by the factor and divided by 2^15).
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
    pmm::{Pmm, ReferenceVoltage},
    serial::*,
    tlv::{self, TempSensorCalibration},
    watchdog::Wdt,
};
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
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART is eUSCI_A1's TXD on P4.3, P4SELx = 01 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102). 8N1: LSB first, 8 data bits, one stop bit, no parity (UCMSB, UC7BIT, UCSPB,
    // UCPEN: SLAU445I Table 22-8, p. 593).
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
    .tx_only(p4.pin3.to_alternate1());

    // The CRC module checks the descriptors: the CRC-CCITT of 1A04h to 1AF7h, stored at 1A02h
    // (SLASEC4D Table 6-70 note 1, p. 107; SLASEC4D Table 6-70, p. 107)
    let mut crc = Crc::new(periph.crc, 0xFFFF);
    let crc_ok = tlv::crc_matches(&mut crc);

    // The information block and the die record (SLASEC4D Table 6-70, p. 107)
    let die = tlv::die_record();
    writeln!(tx, "\r\nDevice descriptors (TLV)\r").ok();
    writeln!(tx, "Device ID:         {:04X}\r", tlv::device_id()).ok();
    writeln!(tx, "Hardware revision: {:02X}\r", tlv::hardware_revision()).ok();
    writeln!(tx, "Firmware revision: {:02X}\r", tlv::firmware_revision()).ok();
    writeln!(tx, "Lot wafer ID:      {:08X}\r", die.lot_wafer_id).ok();
    writeln!(tx, "Die position:      X {}, Y {}\r", die.x_position, die.y_position).ok();
    writeln!(tx, "Test result:       {:04X}\r", die.test_result).ok();

    // The ADC calibration (SLASEC4D Table 6-70, p. 108; how to use it: SLAU445I 1.13.3.2, p. 60 and
    // SLAU445I 1.13.3.3, p. 60)
    writeln!(tx, "ADC gain factor:   {:04X}\r", tlv::adc_gain_factor()).ok();
    writeln!(tx, "ADC offset:        {}\r", tlv::adc_offset()).ok();
    for (vref, name) in [
        (ReferenceVoltage::V1_5, "1.5"),
        (ReferenceVoltage::V2_0, "2.0"),
        (ReferenceVoltage::V2_5, "2.5"),
    ] {
        let temp = TempSensorCalibration::new(vref);
        writeln!(
            tx,
            "{} V reference: factor {:04X}, temperature sensor {} at 30 C and {} at {} C\r",
            name,
            tlv::reference_factor(vref),
            temp.count_30c(),
            temp.count_high(),
            temp.high_celsius(),
        )
        .ok();
    }

    // The DCO calibration, for 16 MHz and 24 MHz (SLASEC4D Table 6-70, p. 108; SLAU445I 1.13.3.4, p. 60)
    writeln!(tx, "DCO tap, 16 MHz:   {:04X}\r", tlv::dco_tap_16mhz()).ok();
    writeln!(tx, "DCO tap, 24 MHz:   {:04X}\r", tlv::dco_tap_24mhz()).ok();
    writeln!(tx, "CRC: {}\r", if crc_ok { "matches" } else { "DOES NOT MATCH" }).ok();
    tx.flush().ok();

    led1.set_state(crc_ok.into()).ok();

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
