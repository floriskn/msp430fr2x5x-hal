//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The 1.2 V reference on the VREF+ pin, P1.7. The board measures the pin with its ADC against the
//! internal 2.5 V reference, and the backchannel UART prints the result once a second. A multimeter
//! checks it independently.
//! (VREF+ is P1.7 with P1SELx = 11, and ADC channel A7 measures it: SLASEC4D 6.10.1, p. 67; SLASEC4D
//! Table 6-63, p. 96; SLASEC4D Table 6-21, p. 77. Its voltage: SLASEC4D Table 5-10, p. 41.)
//!
//! The data sheet gives only the typical voltage, 1.20 V, and no limits, so the example prints what it
//! measures instead of judging it.
//!
//! How to test (multimeter):
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 2. Expected, once a second: `VREF+ (P1.7): 1200 mV`, give or take a few tens of millivolts.
//! 3. Set the multimeter to DC volts. Put the black probe on GND (J3 pin 22) and the red probe on P1.7
//!    (J1 pin 4): it reads what the terminal shows, within about 20 mV, as the internal reference is
//!    accurate to ±1.5 % (SLASEC4D Table 5-10, p. 41).
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

/// The internal reference the ADC measures against
const VREF_MV: u16 = 2500;

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

    // EXTREFEN buffers the 1.2 V reference onto VREF+ (SLASEC4D 6.10.1, p. 67; SLAU445I Table 2-4, p. 94)
    let mut vref_out = pmm.enable_vref_output(p1.pin7.to_alternate3());

    // The ADC measures the pin, which is input A7 (SLASEC4D Table 6-21, p. 77), against the internal 2.5 V
    // reference, more precise than the supply (2.5 V ±1.5 %: SLASEC4D Table 5-10, p. 41). MODCLK clocks it
    // (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES = 10b: SLAU445I
    // Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I Table 21-3, p. 561).
    let vref = pmm.enable_internal_reference(ReferenceVoltage::V2_5).unwrap();
    let adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc);
    // ADCSREFx = 001b: VR+ = VREF and VR- = AVSS (SLAU445I Table 21-8, p. 567)
    let mut adc = adc.with_reference(PositiveReference::Internal(&vref), NegativeReference::Avss);

    loop {
        let count = block!(adc.read_count(&mut vref_out)).unwrap();
        writeln!(tx, "VREF+ (P1.7): {} mV\r", adc.count_to_mv(count, VREF_MV)).ok();
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
