//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! How to turn an ADC count into a voltage, compared with the input measured by a multimeter. The ADC
//! converts input A8 (P4.3) `SAMPLES` times with AVCC as the reference, and once a second the backchannel
//! UART prints the average count and the voltage it stands for by three formulas:
//! - `2^n`: count × reference / 4096, the inverse of the user's guide's conversion formula, which
//!   `Adc::count_to_mv` uses (SLAU445I 21.2.1, p. 541);
//! - `2^n+1/2`: half a step more, the middle of the count's range if the converter truncates;
//! - `2^n-1`: count × reference / 4095, the form of the data sheets' DVCC equation (SLASEC4D 6.10.1,
//!   p. 67; with 1023 for 10-bit results: SLASEO7C 9.10.1, p. 49).
//!
//! The averaged count has a resolution of a hundredth of a step, so the differences show: a step is
//! 3.3 V / 4096 = 0.8 mV, `2^n` and `2^n-1` are count/4095 of a step apart, almost nothing near 0 V and a
//! whole step near full scale, and `2^n+1/2` is half a step above `2^n` everywhere. The ADC itself may be
//! off by more: an offset error of up to ±4 mV and a gain error of up to ±9 LSB in 12-bit mode (SLASEO7C
//! 8.12.8.3, p. 41). A gain error scales every reading, as the choice between 4096 and 4095 does, and an
//! offset error shifts every reading, as the half step does, so one chip can't show which formula the
//! hardware follows. What it shows is how large the differences are next to that chip's own errors.
//! (A8 is P4.3 with P4SEL = 11: SLASEO7C Table 9-26, p. 68. AVCC as VR+, ADCSREFx = 000b: SLAU445I
//! Table 21-8, p. 567. The supply: SLAU802 2.3.1, p. 10.)
//!
//! How to test (function generator, multimeter):
//! 1. Measure the 3.3 V supply with the multimeter, between 3.3 V (J1 pin 1) and GND (J3 pin 22), and set
//!    `REFERENCE_UV` below to the reading, in microvolts.
//! 2. Set the generator to DC, 1.000 V, with its load set to HiZ. Connect it to P4.3 (J3 pin 24) and GND
//!    (J3 pin 22), and measure P4.3 with the multimeter too.
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 4. Expected, once a second, a line such as `count 1241.21: 2^n 999.998 mV, 2^n+1/2 1000.401 mV,
//!    2^n-1 1000.242 mV`, with your own numbers. Compare the three voltages with the multimeter.
//! 5. Repeat at, for example, 0.050 V, 1.650 V and 3.250 V, and note the differences. From two of these
//!    points the chip's own gain and offset follow: the counts rise by (difference of the two voltages) ×
//!    4096 / reference ideally, and any difference from that is the gain error, in the same steps.
//! (Header pins: SLAU802 Figure 10, p. 13.)
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
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// The supply voltage, AVCC, as the multimeter reads it, in microvolts
const REFERENCE_UV: u64 = 3_300_000;
/// Conversions per average
const SAMPLES: u64 = 1024;
/// The steps of a 12-bit result: 4096 (SLAU445I 21.2.1, p. 541)
const STEPS: u64 = 4096;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

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

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9; SLASEO7C
    // Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
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

    // P4.3 is input A8, with P4SEL = 11 (SLASEO7C Table 9-26, p. 68)
    let mut input = p4.pin3.to_alternate3();

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES =
    // 10b: SLAU445I Table 21-5, p. 565) and 16 ADCCLK cycles of sampling (ADCSHTx = 0010b: SLAU445I
    // Table 21-3, p. 561), against AVCC (ADCSREFx = 000b: SLAU445I Table 21-8, p. 567)
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
        SamplingRate::Max200ksps,
        SampleTime::Cycles16,
    )
    .use_modclk()
    .configure(periph.adc)
    .with_reference(PositiveReference::Avcc, NegativeReference::Avss);

    loop {
        let mut sum: u64 = 0;
        for _ in 0..SAMPLES {
            sum += block!(adc.read_count(&mut input)).unwrap() as u64;
        }
        // The average count in hundredths of a step, and the voltage in microvolts by each formula, worked
        // out from the sum so that the average loses nothing to rounding
        let centi = sum * 100 / SAMPLES;
        let two_n = sum * REFERENCE_UV / (SAMPLES * STEPS);
        let two_n_half = (2 * sum + SAMPLES) * REFERENCE_UV / (2 * SAMPLES * STEPS);
        let two_n_minus_1 = sum * REFERENCE_UV / (SAMPLES * (STEPS - 1));
        writeln!(
            tx,
            "count {}.{:02}: 2^n {}.{:03} mV, 2^n+1/2 {}.{:03} mV, 2^n-1 {}.{:03} mV\r",
            centi / 100,
            centi % 100,
            two_n / 1000,
            two_n % 1000,
            two_n_half / 1000,
            two_n_half % 1000,
            two_n_minus_1 / 1000,
            two_n_minus_1 % 1000,
        )
        .ok();

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
