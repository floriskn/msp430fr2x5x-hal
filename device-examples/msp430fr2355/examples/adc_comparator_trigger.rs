//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! ADC conversions started by the comparator: eCOMP0 compares P1.1 with its 1.2 V reference, and each
//! time the input rises through 1.2 V, its output starts a conversion of P1.4. With the same signal on
//! both pins, the ADC samples it just as it crosses 1.2 V, so every result is close to 1.2 V. Once a
//! second the backchannel UART prints how many conversions there were and their average.
//! (eCOMP0's output is the ADC's comparator trigger, ADCSHSx = 11b: SLASEC4D Table 6-22, p. 77. COMP0.1
//! is P1.1: SLASEC4D Table 6-23, p. 78. A4 is P1.4: SLASEC4D Table 6-21, p. 77. Header pins: SLAU680
//! Figure 10, p. 15.)
//!
//! How to test (function generator):
//! 1. Generator: sine wave, 10 Hz, amplitude 2 Vpp, offset 1.5 V (0.5 V to 2.5 V), output load High-Z.
//!    Check it on the scope first: it must stay between 0 V and 3.3 V.
//! 2. Connect it to both P1.1 (J3 pin 28) and P1.4 (J3 pin 23), with the BNC T-piece and two leads, its
//!    ground to GND (J3 pin 22).
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 4. Expected: `10 conversions, average 1200 mV` once a second, give or take a few tens of millivolts:
//!    the 1.2 V reference is typical, and the input moves on a little during the sampling. The count can
//!    also be 11: each one covers a little more than a second, the 1000 waits of 1 ms plus the time the
//!    loop and the printing take. Change the frequency and the count follows. Change the offset between
//!    1.0 V and 2.0 V: the average stays near 1.2 V. At an offset of 2.3 V the sine stays above 1.2 V
//!    (1.3 V to 3.3 V), and the count drops to 0. Never let the signal go below 0 V or above 3.3 V.
//! (The low-power 1.2 V reference, VeCOMP,LP: 1.20 V typical: SLASEC4D Table 5-10, p. 41.)
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    adc::{AdcConfig, ClockDivider, ConversionConfig, ConversionMode, Predivider, Resolution, SampleMode, SampleTime, SamplingRate, TriggerSource},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    ecomp::{ECompConfig, FilterStrength, Hysteresis, NegativeInput, OutputPolarity, PositiveInput, PowerMode},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The ADC's reference, AVCC (SLAU680 2.3.1, p. 12: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
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

    // V+ is COMP0.1 on P1.1, with P1SELx = 11 (SLASEC4D Table 6-63, p. 96), and V- the low-power 1.2 V
    // reference, so the output rises as P1.1 rises through 1.2 V (SLAU445I 18.2.1, p. 505). High-speed
    // mode (CPMSEL = 0), with 20 mV of hysteresis against noise (CPHSEL = 10b): SLAU445I Table 18-3,
    // p. 510.
    let (_dac_config, comparator_config) = ECompConfig::begin(periph.e_comp0);
    let _comparator = comparator_config
        .configure(
            PositiveInput::COMPx_1(p1.pin1.to_alternate3()),
            NegativeInput::_1V2,
            OutputPolarity::Noninverted,
            PowerMode::HighSpeed,
            Hysteresis::_20mV,
            FilterStrength::Off,
        )
        .no_output_pin();

    // P1.4 is input A4 with P1SELx = 11 (SLASEC4D Table 6-63, p. 96). MODCLK clocks the ADC (ADCSSELx =
    // 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES = 10b: SLAU445I Table 21-5, p. 565)
    // and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I Table 21-3, p. 561), about 4 µs.
    let mut input = p1.pin4.to_alternate3();
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc);

    // Repeat-single-channel mode (ADCCONSEQx = 10b), each conversion started by a rising edge of the
    // comparator's output (ADCSHSx = 11b, ADCSHP = 1: SLAU445I Table 21-4, p. 563)
    adc.start(
        &mut input,
        ConversionConfig {
            mode: ConversionMode::RepeatSingle,
            trigger: TriggerSource::Comparator,
            sample_mode: SampleMode::RisingEdge,
            back_to_back: false,
        },
    );

    loop {
        // Collect the results of about one second, in steps of 1 ms
        let (mut conversions, mut sum) = (0u16, 0u32);
        for _ in 0..1000 {
            if let Ok(count) = adc.result() {
                conversions += 1;
                sum += count as u32;
            }
            delay.delay_ms(1);
        }

        if conversions == 0 {
            writeln!(tx, "0 conversions\r").ok();
        } else {
            let average = (sum / conversions as u32) as u16;
            writeln!(tx, "{} conversions, average {} mV\r", conversions, adc.count_to_mv(average, AVCC_MV)).ok();
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
