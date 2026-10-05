//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC references: the same input measured against three references, which the backchannel UART prints
//! once a second.
//! - AVCC, the 3.3 V supply, as after reset
//! - the internal 1.5 V reference, the only one this device has
//! - an external reference on VeREF+ (P1.0), here from the function generator's second channel
//!
//! An input voltage converts to `count = 1024 × input / reference`, at most 1023 (10-bit results:
//! SLAU445I 21.2.1, p. 541), so each line shows the count and the voltage worked out from the reference.
//! All three should give the input voltage, each as accurately as its reference allows: with the internal
//! one the gain error is up to ±3 % (SLASE59F Table 5-22, p. 36), and AVCC is only as accurate as the
//! LaunchPad's 3.3 V supply.
//! (ADCSREFx: SLAU445I Table 21-8, p. 567. The 1.5 V reference: SLASE59F 6.10.1, p. 45. VeREF+ is P1.0
//! and A2 is P1.2: SLASE59F Table 6-15, p. 53. LED1 is on P1.0 too, through jumper J10: SLAU739
//! Figure 18, p. 23.)
//!
//! How to test (function generator with two channels, and the multimeter):
//! 1. Take the J10 jumper off, so LED1 draws no current from the reference on P1.0.
//! 2. Generator channel 1: the DC waveform, Offset 1.000 V, output load High-Z. Connect it to P1.2
//!    (J1 pin 10).
//! 3. Generator channel 2: DC, Offset 2.000 V, output load High-Z. Connect it to P1.0 (J1 pin 2), the
//!    VeREF+ input. Connect both grounds to GND (J2 pin 20 and J3 pin 22). Check both voltages with the
//!    multimeter before connecting: neither may go above 3.3 V.
//! 4. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 5. Expected: `AVCC: 310 = 1000 mV, internal 1.5 V: 682 = 1000 mV, VeREF+ 2.0 V: 512 = 1000 mV`, give or
//!    take a few counts. Change channel 1 between 0 V and 1.5 V and compare with the multimeter on P1.2.
//!    Above 1.5 V the internal reading stops at 1023, and above 2.0 V the VeREF+ one does.
//! 6. Put the J10 jumper back afterwards.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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
    pmm::{Pmm, ReferenceVoltage},
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The supply voltage, AVCC (SLAU739 2.3.1, p. 10: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;
/// The internal reference
const VREF_MV: u16 = 1500;
/// The voltage generator channel 2 puts on VeREF+
const VEREF_MV: u16 = 2000;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

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

    // P1.2 is input A2 and P1.0 is VeREF+, each with its ADCPCTLx bit set (SLASE59F Table 6-17, p. 55)
    let mut input = p1.pin2.to_adc_mode();
    let veref_plus = p1.pin0.to_adc_mode();
    // REFVSEL = 00b selects 1.5 V, and INTREFEN = 1 turns it on (SLAU445I Table 2-4, p. 93 to p. 94)
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V1_5).unwrap();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES =
    // 01b: SLAU445I Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I
    // Table 21-3, p. 561)
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc);
    let mut adc = adc.with_reference(PositiveReference::Avcc, NegativeReference::Avss);

    loop {
        // ADCSREFx = 000b: VR+ = AVCC
        adc = adc.with_reference(PositiveReference::Avcc, NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        write!(tx, "AVCC: {} = {} mV, ", count, adc.count_to_mv(count, AVCC_MV)).ok();

        // ADCSREFx = 001b: VR+ = VREF, the internal reference
        adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        write!(tx, "internal 1.5 V: {} = {} mV, ", count, adc.count_to_mv(count, VREF_MV)).ok();

        // ADCSREFx = 010b: VR+ = VeREF+, through the reference buffer
        adc = adc.with_reference(PositiveReference::ExternalBuffered(&veref_plus), NegativeReference::Avss);
        let count = block!(adc.read_count(&mut input)).unwrap();
        writeln!(tx, "VeREF+ 2.0 V: {} = {} mV\r", count, adc.count_to_mv(count, VEREF_MV)).ok();

        delay.delay_ms(1000);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
