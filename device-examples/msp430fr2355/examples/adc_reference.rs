//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC references: the same input measured against three references, which the backchannel UART prints
//! once a second.
//! - AVCC, the 3.3 V supply, as after reset
//! - the internal 2.5 V reference
//! - an external reference on VeREF+ (P1.0), here from the function generator's second channel
//!
//! An input voltage converts to `count = 4095 × input / reference` (12-bit results, SLAU445I 21.2.1,
//! p. 541), so each line shows the count and the voltage worked out from the reference. All three
//! should give the input voltage; the internal reference is the most accurate (2.5 V ±1.5 %: SLASEC4D
//! Table 5-10, p. 41), while AVCC is only as accurate as the LaunchPad's 3.3 V supply.
//! (ADCSREFx: SLAU445I Table 21-8, p. 567. VeREF+ is P1.0 and A1 is P1.1: SLASEC4D Table 6-21, p. 77.
//! P1.0 isn't on the header: it drives LED1 through jumper J10, SLAU680 Figure 18, p. 26.)
//!
//! How to test (function generator with two channels, a jumper wire, and the multimeter):
//! 1. Generator channel 1: the DC waveform, Offset 1.000 V, output load High-Z. Connect it to P1.1
//!    (J3 pin 28).
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 3. Remove the J10 jumper. To find which of its two pins is P1.0, touch each in turn with a jumper wire
//!    from 3.3 V (J1 pin 1): the one that lights LED1 leads to the LED, the other is P1.0, the VeREF+ input.
//! 4. Generator channel 2: DC, Offset 2.000 V, output load High-Z. Connect it to that P1.0 pin of J10.
//!    Connect both grounds to GND (J3 pin 22). Check both voltages with the multimeter before connecting:
//!    neither may go above 3.3 V.
//! 5. Expected: `AVCC: 1241 = 1000 mV, internal 2.5 V: 1638 = 1000 mV, VeREF+ 2.0 V: 2048 = 1000 mV`,
//!    give or take a few counts. Change channel 1 between 0 V and 2 V and compare with the multimeter on
//!    P1.1. Above 2.0 V the VeREF+ reading stops at 4095, and above 2.5 V the internal one does.
//! 6. Put the J10 jumper back afterwards, for LED1.
//! (Header pins: SLAU680 Figure 10, p. 15.)
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

/// The supply voltage, AVCC (SLAU680 2.3.1, p. 12: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;
/// The internal reference
const VREF_MV: u16 = 2500;
/// The voltage generator channel 2 puts on VeREF+
const VEREF_MV: u16 = 2000;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SELx = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
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

    // P1.1 is input A1 and P1.0 is VeREF+, each with P1SELx = 11 (SLASEC4D Table 6-63, p. 96)
    let mut input = p1.pin1.to_alternate3();
    let veref_plus = p1.pin0.to_alternate3();
    // REFVSEL = 10b selects 2.5 V, and INTREFEN = 1 turns it on (SLAU445I Table 2-4, p. 93 to p. 94)
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V2_5).unwrap();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES =
    // 10b: SLAU445I Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I
    // Table 21-3, p. 561)
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
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
        write!(tx, "internal 2.5 V: {} = {} mV, ", count, adc.count_to_mv(count, VREF_MV)).ok();

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
