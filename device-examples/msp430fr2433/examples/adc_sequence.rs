//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An ADC sequence: one start converts A2, A1 and A0 in turn, with results in the signed format. The
//! backchannel UART prints the three results once a second, raw and in millivolts.
//!
//! The inputs on this LaunchPad:
//! - A2, P1.2 (J1 pin 10): the function generator, for example 1.000 V
//! - A1, P1.1 (J2 pin 19): a jumper wire to GND (J2 pin 20). This is LED2's pin.
//! - A0, P1.0 (J1 pin 2): a jumper wire to 3.3 V (J1 pin 1). This is LED1's pin, so LED1 lights.
//! (A0 to A2: SLASE59F Table 6-15, p. 53. LED1 is P1.0, LED2 is P1.1, and the header pins: SLAU739
//! Figure 18, p. 23.)
//!
//! In the signed format the 10-bit result is a two's complement number in the top 10 bits: the bottom of
//! the range, 0 V, reads as -32768 (8000h) and the reference, 3.3 V here, as 32704 (7FC0h). (SLAU445I
//! Table 21-5, p. 565, ADCDF = 1: "Signed binary (2s complement), left aligned".) `Adc::count_to_mv`
//! converts these results too.
//!
//! How to test (function generator, two jumper wires, multimeter):
//! 1. Generator: the DC waveform, Offset 1.000 V, output load High-Z, on P1.2 (J1 pin 10), its ground on
//!    GND (J3 pin 22). Check the voltage with the multimeter first: at most 3.3 V.
//! 2. Connect P1.1 (J2 pin 19) to GND (J2 pin 20) and P1.0 (J1 pin 2) to 3.3 V (J1 pin 1) with jumper
//!    wires.
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 4. Expected, once a second: `A2: -12928 = 1000 mV, A1: -32768 = 0 mV, A0: 32704 = 3300 mV`, give or
//!    take a few millivolts.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    adc::{
        AdcConfig, AdcInterruptFlags, ClockDivider, ConversionConfig, ConversionMode, DataFormat, Predivider,
        Resolution, SampleMode, SampleTime, SamplingRate, TriggerSource,
    },
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The ADC's reference, AVCC (SLAU739 2.3.1, p. 10: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
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

    // A sequence converts every channel from the selected one down to A0, so all three pins are analog
    // inputs, each with its ADCPCTLx bit set (SLASE59F Table 6-17, p. 55)
    let mut a2 = p1.pin2.to_adc_mode();
    let _a1 = p1.pin1.to_adc_mode();
    let _a0 = p1.pin0.to_adc_mode();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES =
    // 01b) in the signed format (ADCDF = 1: SLAU445I Table 21-5, p. 565). 512 ADCCLK cycles of sampling
    // (ADCSHTx = 1010b: SLAU445I Table 21-3, p. 561), at least 88 µs at MODCLK's highest 5.8 MHz
    // (SLASE59F Table 5-9, p. 26), give the CPU time to read each result before the next one.
    let mut config = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles512,
    );
    config.data_format = DataFormat::Signed;
    let mut adc = config.use_modclk().configure(periph.adc);

    // Sequence-of-channels mode (ADCCONSEQx = 01b), started once by software (ADCSC). ADCMSC = 1 converts
    // the rest of the sequence without another start (SLAU445I 21.2.7.2, p. 549; SLAU445I 21.2.7.5, p. 555).
    let sequence = ConversionConfig {
        mode: ConversionMode::Sequence,
        trigger: TriggerSource::Software,
        sample_mode: SampleMode::RisingEdge,
        back_to_back: true,
    };

    loop {
        adc.start(&mut a2, sequence);
        let mut results = [0u16; 3];
        for result in results.iter_mut() {
            *result = block!(adc.result()).unwrap();
        }
        // A result that came before the one before it was read sets ADCOVIFG (SLAU445I Table 21-14, p. 571)
        if adc.interrupt_flags().contains(AdcInterruptFlags::Overflow) {
            adc.clear_interrupt_flags(AdcInterruptFlags::Overflow);
            writeln!(tx, "missed a result\r").ok();
        }

        for (channel, result) in (0..3).rev().zip(results) {
            write!(tx, "A{}: {} = {} mV", channel, result as i16, adc.count_to_mv(result, AVCC_MV)).ok();
            write!(tx, "{}", if channel == 0 { "\r\n" } else { ", " }).ok();
        }
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
