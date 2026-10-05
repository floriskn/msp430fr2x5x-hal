//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC conversions started by a timer: TA1 starts a conversion of P1.1 every millisecond, so the ADC
//! samples at exactly 1 kHz without the CPU starting each conversion. Every second eUSCI_A0 prints the
//! lowest, highest and average of the 1000 samples.
//! (The CCR1 output of TA1, TA1.1B, is the ADC's timer trigger, ADCSHSx = 10b: SLASEE4C Table 6-14, p. 56;
//! SLASEE4C Figure 6-2, p. 54. Repeat-single-channel mode: SLAU445I 21.2.7.3, p. 551. A1 is P1.1:
//! SLASEE4C Table 6-13, p. 55. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document covers the
//! parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, the scope, and a 3.3-V USB-to-UART adapter):
//! 1. Power the MSP430FR2522 from 3.3 V, as the code assumes. Connect the adapter: its RX to P1.4
//!    (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Generator: sine wave, 10 Hz, amplitude 2 Vpp, offset 1.5 V, output load High-Z, so it swings from
//!    0.5 V to 2.5 V. Check it on the scope first: it must stay between 0 V and 3.3 V.
//! 3. Connect it to P1.1, its ground to GND.
//! 4. Flash this example.
//! 5. Expected, once a second: `1000 samples: min 500 mV, max 2500 mV, average 1500 mV`, give or take
//!    the accuracy of the 3.3 V supply, which is the ADC's reference here. Change the amplitude or offset
//!    and watch the numbers follow.
#![no_main]
#![no_std]

use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, ConversionConfig, ConversionMode, Predivider, Resolution, SampleMode, SampleTime, SamplingRate, TriggerSource},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The ADC's reference, AVCC: the 3.3 V the MSP430FR2522 runs from
const AVCC_MV: u16 = 3300;
/// Samples per report: one second at 1 kHz
const SAMPLES: u16 = 1000;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = DCOCLKDIV in the 8 MHz range, 244 × 32.768 kHz, and SMCLK = MCLK / 8, 999.4 kHz, for the
    // timer: the 1 MHz range, 32 × 32.768 kHz, is 5 % faster (SLAU445I 3.2.5, p. 104). ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
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

    // TA1 counts SMCLK in up mode, 1000 counts per period: 1 ms (TASSEL = 10b: SLAU445I Table 13-4,
    // p. 384; SLAU445I 13.2.3.1, p. 371). Its CCR1 output goes high at the start of each period, the
    // rising edge that starts a conversion, and stays high for 10 counts.
    let ta1 = PwmParts3::new(periph.ta1, TimerConfig::smclk(&smclk), 999);
    let _trigger = ta1.pwm1.into_adc_trigger(10);

    // P1.1 is input A1, enabled by ADCPCTL1 in SYSCFG2 (SLASEE4C Table 6-15, p. 58). MODCLK clocks the ADC
    // (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 10-bit results (ADCRES = 01b: SLAU445I
    // Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I Table 21-3, p. 561):
    // at least 16 / 5.8 MHz = 2.76 µs (SLASEE4C Table 5-9, p. 28), more than the 2.0 µs tSample at 3 V for a
    // 1-kΩ source (SLASEE4C Table 5-21, p. 38).
    let mut input = p1.pin1.to_adc_mode();
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc);

    // Repeat-single-channel mode (ADCCONSEQx = 10b), each conversion started by a rising edge of the timer
    // output (ADCSHSx = 10b, ADCSHP = 1: SLAU445I Table 21-4, p. 563)
    let config = ConversionConfig {
        mode: ConversionMode::RepeatSingle,
        trigger: TriggerSource::Timer,
        sample_mode: SampleMode::RisingEdge,
        back_to_back: false,
    };

    loop {
        // Start afresh after each report, as results that came in while printing weren't read
        adc.start(&mut input, config);
        let (mut min, mut max, mut sum) = (u16::MAX, 0, 0u32);
        for _ in 0..SAMPLES {
            let count = block!(adc.result()).unwrap();
            min = min.min(count);
            max = max.max(count);
            sum += count as u32;
        }
        adc.stop();

        let average = (sum / SAMPLES as u32) as u16;
        print_num(&mut tx, SAMPLES as u32);
        print(&mut tx, " samples: min ");
        print_num(&mut tx, adc.count_to_mv(min, AVCC_MV) as u32);
        print(&mut tx, " mV, max ");
        print_num(&mut tx, adc.count_to_mv(max, AVCC_MV) as u32);
        print(&mut tx, " mV, average ");
        print_num(&mut tx, adc.count_to_mv(average, AVCC_MV) as u32);
        print(&mut tx, " mV\r\n");
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
