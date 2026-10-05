//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An ADC sequence: one start converts A3, A2, A1 and A0 in turn, with results in the signed format.
//! eUSCI_A0 prints the four results once a second, raw and in millivolts.
//!
//! The inputs:
//! - A3, P1.3: the function generator's channel 1, for example 1.000 V
//! - A2, P1.2: a jumper wire to 3.3 V
//! - A1, P1.1: the function generator's channel 2, for example 2.000 V
//! - A0, P1.0: a jumper wire to GND
//! (A0 to A3: SLASEE4C Table 6-13, p. 55. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document
//! covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! In the signed format the 10-bit result is a two's complement number in the top 10 bits: the bottom of
//! the range, 0 V, reads as -32768 (8000h) and the reference, 3.3 V here, as 32704 (7FC0h), in steps of
//! 64. (SLAU445I Table 21-5, p. 565, ADCDF = 1: "Signed binary (2s complement), left aligned", with these
//! values.) `Adc::count_to_mv` converts these results too.
//!
//! How to test (function generator with two channels, two jumper wires, the multimeter, and a 3.3-V
//! USB-to-UART adapter):
//! 1. Power the MSP430FR2522 from 3.3 V, as the code assumes. Connect the adapter: its RX to P1.4
//!    (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Generator channel 1: the DC waveform, Offset 1.000 V, output load High-Z, on P1.3. Channel 2: DC,
//!    Offset 2.000 V, output load High-Z, on P1.1. Both grounds on GND. Check the voltages with the
//!    multimeter first: at most 3.3 V.
//! 3. Connect P1.2 to 3.3 V and P1.0 to GND with jumper wires.
//! 4. Flash this example.
//! 5. Expected, once a second: `A3: -12928 = 1000 mV, A2: 32704 = 3300 mV, A1: 6912 = 2000 mV, A0: -32768
//!    = 0 mV`, give or take a step or two (a step is about 3 mV).
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
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The ADC's reference, AVCC: the 3.3 V the MSP430FR2522 runs from
const AVCC_MV: u16 = 3300;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

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

    // A sequence converts every channel from the selected one down to A0, so all four pins are analog
    // inputs, each enabled by its ADCPCTLx bit in SYSCFG2 (SLASEE4C Table 6-15, p. 58)
    let mut a3 = p1.pin3.to_adc_mode();
    let _a2 = p1.pin2.to_adc_mode();
    let _a1 = p1.pin1.to_adc_mode();
    let _a0 = p1.pin0.to_adc_mode();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES =
    // 01b) in the signed format (ADCDF = 1: SLAU445I Table 21-5, p. 565). 512 ADCCLK cycles of sampling
    // (ADCSHTx = 1010b: SLAU445I Table 21-3, p. 561), at least 88 µs at MODCLK's 5.8 MHz maximum (SLASEE4C
    // Table 5-9, p. 28), give the CPU time to read each result.
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
        adc.start(&mut a3, sequence);
        let mut results = [0u16; 4];
        for result in results.iter_mut() {
            *result = block!(adc.result()).unwrap();
        }
        // A result that came before the one before it was read sets ADCOVIFG (SLAU445I Table 21-14, p. 571)
        if adc.interrupt_flags().contains(AdcInterruptFlags::Overflow) {
            adc.clear_interrupt_flags(AdcInterruptFlags::Overflow);
            print(&mut tx, "missed a result\r\n");
        }

        for (channel, result) in (0..4).rev().zip(results) {
            print(&mut tx, "A");
            print_num(&mut tx, channel);
            print(&mut tx, ": ");
            // The result as the signed number it is
            let signed = result as i16;
            if signed < 0 {
                print(&mut tx, "-");
            }
            print_num(&mut tx, signed.unsigned_abs() as u32);
            print(&mut tx, " = ");
            print_num(&mut tx, adc.count_to_mv(result, AVCC_MV) as u32);
            print(&mut tx, if channel == 0 { " mV\r\n" } else { " mV, " });
        }
        delay.delay_ms(1000);
    }
}

// Numbers are printed by hand: the formatting code of `write!` takes several KB, and this device has 7.25 KB
// of program FRAM (SLASEE4C Table 6-19, p. 62).

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
