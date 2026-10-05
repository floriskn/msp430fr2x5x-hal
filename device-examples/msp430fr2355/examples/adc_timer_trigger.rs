//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC conversions started by a timer: TB1 starts a conversion of P1.1 every millisecond, so the ADC
//! samples at exactly 1 kHz without the CPU starting each conversion. Every second the backchannel UART
//! prints the lowest, highest and average of the 1000 samples.
//! (The CCR1 output of TB1, TB1.1B, is the ADC's timer trigger, ADCSHSx = 10b: SLASEC4D Table 6-22, p. 77;
//! SLASEC4D Table 6-17, p. 74. Repeat-single-channel mode: SLAU445I 21.2.7.3, p. 551. A1 is P1.1:
//! SLASEC4D Table 6-21, p. 77.)
//!
//! How to test (function generator):
//! 1. Generator: sine wave, 10 Hz, amplitude 2 Vpp, offset 1.5 V, output load High-Z, so it swings from
//!    0.5 V to 2.5 V. Check it on the scope first: it must stay between 0 V and 3.3 V.
//! 2. Connect it to P1.1 (J3 pin 28), its ground to GND (J3 pin 22). (Header pins: SLAU680 Figure 10,
//!    p. 15.)
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 4. Expected, once a second: `1000 samples: min 500 mV, max 2500 mV, average 1500 mV`, give or take
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
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The ADC's reference, AVCC (SLAU680 2.3.1, p. 12: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;
/// Samples per report: one second at 1 kHz
const SAMPLES: u16 = 1000;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);

    // MCLK = DCOCLKDIV in the 8 MHz range, 244 × 32.768 kHz, and SMCLK = MCLK / 8, 999.4 kHz, for the
    // timer: the 1 MHz range, 32 × 32.768 kHz, is 5 % faster (SLAU445I 3.2.5, p. 104). ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
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

    // TB1 counts SMCLK in up mode, 1000 counts per period: 1 ms (TBSSEL = 10b: SLAU445I Table 14-6,
    // p. 409; SLAU445I 14.2.3.1, p. 394). Its CCR1 output goes high at the start of each period, the
    // rising edge that starts a conversion, and stays high for 10 counts.
    let tb1 = PwmParts3::new(periph.tb1, TimerConfig::smclk(&smclk), 999);
    let _trigger = tb1.pwm1.into_adc_trigger(10);

    // P1.1 is input A1 with P1SELx = 11 (SLASEC4D Table 6-63, p. 96). MODCLK clocks the ADC (ADCSSELx =
    // 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES = 10b: SLAU445I Table 21-5, p. 565)
    // and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I Table 21-3, p. 561).
    let mut input = p1.pin1.to_alternate3();
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
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
        writeln!(
            tx,
            "{} samples: min {} mV, max {} mV, average {} mV\r",
            SAMPLES,
            adc.count_to_mv(min, AVCC_MV),
            adc.count_to_mv(max, AVCC_MV),
            adc.count_to_mv(average, AVCC_MV),
        )
        .ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
