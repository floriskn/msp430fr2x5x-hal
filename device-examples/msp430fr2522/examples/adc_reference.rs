//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC references: the same input measured against three references, printed on eUSCI_A0 once a second.
//! - AVCC, the 3.3 V supply, as after reset
//! - the internal 1.5 V reference, the only internal level of this device
//! - an external reference on VeREF+ (P1.0), here from the function generator's second channel
//!
//! An input voltage converts to `count = 1024 × input / reference`, at most 1023 (10-bit results:
//! SLAU445I 21.2.1, p. 541), and each line shows the count and the voltage worked out from the
//! reference. All three should give the input voltage, each as accurately as its reference: the
//! internal one within ±3 % (the ADC's gain error with it: SLASEE4C Table 5-22, p. 39), AVCC as the 3.3 V
//! supply, and VeREF+ as the generator.
//! (ADCSREFx: SLAU445I Table 21-8, p. 567. The internal reference is 1.5 V: SLASEE4C 6.10.1, p. 48.
//! A0/Veref+ is P1.0 and A1 is P1.1: SLASEE4C Table 6-13, p. 55. UCA0TXD is P1.4: SLASEE4C Table 6-11,
//! p. 53. No board document covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator with two channels, the multimeter, and a 3.3-V USB-to-UART adapter):
//! 1. Power the MSP430FR2522 from 3.3 V, as the code assumes. Connect the adapter: its RX to P1.4
//!    (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Generator channel 1: the DC waveform, Offset 1.000 V, output load High-Z. Connect it to P1.1.
//! 3. Generator channel 2: DC, Offset 2.000 V, output load High-Z. Connect it to P1.0, the VeREF+ input.
//!    Connect both grounds to GND. Check both voltages with the multimeter before connecting: neither may
//!    go above 3.3 V (the analog input range: SLASEE4C Table 5-20, p. 38).
//! 4. Flash this example.
//! 5. Expected: `AVCC: 310 = 1000 mV, internal 1.5 V: 682 = 1000 mV, VeREF+ 2.0 V: 512 = 1000 mV`, give
//!    or take a few counts. Change channel 1 between 0 V and 2 V and compare with the multimeter on P1.1.
//!    Above 1.5 V the internal reading stops at 1023, and above 2.0 V the VeREF+ one does.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, NegativeReference, PositiveReference, Predivider, Resolution, SampleTime, SamplingRate},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::{Pmm, ReferenceVoltage},
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The supply voltage, AVCC, that the MSP430FR2522 runs from
const AVCC_MV: u16 = 3300;
/// The internal reference
const VREF_MV: u16 = 1500;
/// The voltage generator channel 2 puts on VeREF+
const VEREF_MV: u16 = 2000;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

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

    // P1.1 is input A1 and P1.0 is VeREF+, each enabled by its ADCPCTLx bit in SYSCFG2 (SLASEE4C
    // Table 6-15, p. 58; SLASEE4C Table 6-13, p. 55)
    let mut input = p1.pin1.to_adc_mode();
    let veref_plus = p1.pin0.to_adc_mode();
    // INTREFEN = 1 turns the internal reference on (SLAU445I Table 2-4, p. 93 to p. 94)
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V1_5).unwrap();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES =
    // 01b: SLAU445I Table 21-5, p. 565) and 256 ADCCLK cycles of sampling (ADCSHTx = 1000b: SLAU445I
    // Table 21-3, p. 561): at least 44 µs at MODCLK's 5.8 MHz maximum (SLASEE4C Table 5-9, p. 28), longer
    // than the 30 µs the internal reference may take to settle (SLAU445I 21.2.3.1, p. 542)
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles256,
    )
    .use_modclk()
    .configure(periph.adc);
    let mut adc = adc.with_reference(PositiveReference::Avcc, NegativeReference::Avss);

    loop {
        // ADCSREFx = 000b: VR+ = AVCC
        adc = adc.with_reference(PositiveReference::Avcc, NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        print(&mut tx, "AVCC: ");
        print_reading(&mut tx, count, adc.count_to_mv(count, AVCC_MV));

        // ADCSREFx = 001b: VR+ = VREF, the internal reference
        adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        print(&mut tx, ", internal 1.5 V: ");
        print_reading(&mut tx, count, adc.count_to_mv(count, VREF_MV));

        // ADCSREFx = 010b: VR+ = VeREF+, through the reference buffer
        adc = adc.with_reference(PositiveReference::ExternalBuffered(&veref_plus), NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        print(&mut tx, ", VeREF+ 2.0 V: ");
        print_reading(&mut tx, count, adc.count_to_mv(count, VEREF_MV));
        print(&mut tx, "\r\n");

        delay.delay_ms(1000);
    }
}

// Numbers are printed by hand: the formatting code of `write!` takes several KB, and this device has 7.25 KB
// of program FRAM (SLASEE4C Table 6-19, p. 62).

/// Print `count = mv mV`
fn print_reading(tx: &mut impl Write, count: u16, mv: u16) {
    print_num(tx, count as u32);
    print(tx, " = ");
    print_num(tx, mv as u32);
    print(tx, " mV");
}

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

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
