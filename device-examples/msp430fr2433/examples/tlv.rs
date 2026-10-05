//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The device descriptors (TLV): which device this is, and the calibration values measured in the
//! factory. The board checks the descriptors against their CRC and prints them on the backchannel UART.
//! LED1 turns on if the CRC matches.
//! (The descriptors: SLASE59F Table 6-22, p. 60 to p. 61. LED1 on P1.0 is red: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on (SLAU739 Table 2, p. 8).
//! 2. In the Windows Device Manager, find the COM port of "MSP Application UART1", and open it at
//!    9600 baud in a serial terminal such as PuTTY (connection type Serial) (SLAU739 2.2.4, p. 9).
//! 3. Press the reset button S3 to print the descriptors again.
//!
//! Expected: `CRC: matches`, device ID 8240 (SLASE59F Table 6-21, p. 60), and hardware revision 11 on
//! revisions B and C of the die (SLAZ664S 5.3, p. 4). The other values are
//! measured per chip ("Per unit" in SLASE59F Table 6-22, p. 60 to p. 61), so each board prints its own.
//! This device has only the 1.5 V reference, so there is one reference factor and one pair of temperature
//! sensor counts, at 30 C and 85 C (SLASE59F Table 6-22, p. 60 to p. 61). The reference factor and the ADC
//! gain factor are close to 32768 (8000h), which means no correction (SLAU445I 1.13.3, p. 59 and SLAU445I
//! 1.13.3.2, p. 60: results are multiplied by the factor and divided by 2^15).
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
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART is eUSCI_A0's TXD on P1.4, P1SELx = 01 (SLAU739 2.2.4, p. 9; SLASE59F
    // Table 6-17, p. 55). 8N1: LSB first, 8 data bits, one stop bit, no parity (UCMSB, UC7BIT, UCSPB,
    // UCPEN: SLAU445I Table 22-8, p. 593).
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

    // The CRC module checks the descriptors: the CRC-CCITT of 1A04h to 1AF5h, stored at 1A02h
    // (SLASE59F Table 6-22 note 1, p. 60; SLASE59F Table 6-22, p. 60)
    let mut crc = Crc::new(periph.crc, 0xFFFF);
    let crc_ok = tlv::crc_matches(&mut crc);

    // The information block and the die record (SLASE59F Table 6-22, p. 60)
    let die = tlv::die_record();
    writeln!(tx, "\r\nDevice descriptors (TLV)\r").ok();
    writeln!(tx, "Device ID:         {:04X}\r", tlv::device_id()).ok();
    writeln!(tx, "Hardware revision: {:02X}\r", tlv::hardware_revision()).ok();
    writeln!(tx, "Firmware revision: {:02X}\r", tlv::firmware_revision()).ok();
    writeln!(tx, "Lot wafer ID:      {:08X}\r", die.lot_wafer_id).ok();
    writeln!(tx, "Die position:      X {}, Y {}\r", die.x_position, die.y_position).ok();
    writeln!(tx, "Test result:       {:04X}\r", die.test_result).ok();

    // The ADC calibration (SLASE59F Table 6-22, p. 60; how to use it: SLAU445I 1.13.3.2, p. 60 and
    // SLAU445I 1.13.3.3, p. 60)
    writeln!(tx, "ADC gain factor:   {:04X}\r", tlv::adc_gain_factor()).ok();
    writeln!(tx, "ADC offset:        {}\r", tlv::adc_offset()).ok();
    // The 1.5 V reference is the only one (SLASE59F 6.10.1, p. 45), and the temperature sensor's
    // calibration was measured against it (SLASE59F Table 6-22, p. 60)
    let temp = TempSensorCalibration::new(ReferenceVoltage::V1_5);
    writeln!(
        tx,
        "1.5 V reference: factor {:04X}, temperature sensor {} at 30 C and {} at {} C\r",
        tlv::reference_factor(ReferenceVoltage::V1_5),
        temp.count_30c(),
        temp.count_high(),
        temp.high_celsius(),
    )
    .ok();

    // The DCO calibration (SLASE59F Table 6-22, p. 61; SLAU445I 1.13.3.4, p. 60)
    writeln!(tx, "DCO tap, 16 MHz:   {:04X}\r", tlv::dco_tap_16mhz()).ok();
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
