//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The ADC's two internal supply channels: once a second the ADC converts channel 14, DVSS, and channel 15,
//! DVCC, and the backchannel UART prints both results, and how many times `adc_is_busy()` found the second
//! conversion still running.
//!
//! The reference is AVCC, as after reset, so the two channels are the ends of the ADC's range: DVSS converts
//! to 0 and DVCC to the full-scale count, 4095 for 12-bit results. A long sample time, 1024 ADCCLK cycles,
//! keeps each conversion going long enough for the loop that polls ADCBUSY to find it set many times.
//! (Channels 14 and 15: SLASEC4D Table 6-21, p. 77. DVCC and DVSS supply the analog modules too: SLASEC4D
//! 1.4, p. 4. The range: SLAU445I 21.2.1, p. 541. AVCC as the reference after reset: SLAU445I Table 21-8,
//! p. 567. ADCBUSY: SLAU445I Table 21-4, p. 564.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 2. Expected, once a second, for example: `DVSS (channel 14): 0 = 0 mV, DVCC (channel 15): 4095 = 3299 mV,
//!    busy for 30 polls`. The full-scale count converts to one step below the reference, the voltage at
//!    which the conversion formula reaches it (`Adc::count_to_mv`; SLAU445I 21.2.1, p. 541). The counts
//!    can be off by a little, within the ADC's offset and gain errors
//!    (SLASEC4D Table 5-22, p. 52). The number of polls depends on the clocks and the build, but is never 0:
//!    that would mean `adc_is_busy()` missed the conversion.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    adc::{
        adc_ch14_vss, adc_ch15_vcc, AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate,
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

/// The ADC's reference, AVCC (SLAU680 2.3.1, p. 12: the LaunchPad supplies 3.3 V)
const AVCC_MV: u16 = 3300;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p4 = Batch::new(periph.p4).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SEL = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
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

    // MODCLK clocks the ADC (ADCSSELx = 00b: SLAU445I Table 21-4, p. 564), with 12-bit results (ADCRES =
    // 10b: SLAU445I Table 21-5, p. 565) and 1024 ADCCLK cycles of sampling (ADCSHTx = 1100b: SLAU445I
    // Table 21-3, p. 561)
    let mut adc = AdcConfig::new(
        ClockDivider::_1,
        Predivider::_1,
        Resolution::Bits12,
        SamplingRate::Max200ksps,
        SampleTime::Cycles1024,
    )
    .use_modclk()
    .configure(periph.adc);

    // Channels 14 and 15 have no pins, so these stand for them (ADCINCHx = 1110b and 1111b: SLASEC4D
    // Table 6-21, p. 77)
    let mut dvss = adc_ch14_vss();
    let mut dvcc = adc_ch15_vcc();

    loop {
        let dvss_count = block!(adc.read_count(&mut dvss)).unwrap();

        // The first `read_count()` starts the conversion and returns `WouldBlock`. Count the reads of ADCBUSY
        // that find it still running, then fetch the result.
        adc.read_count(&mut dvcc).ok();
        let mut polls: u16 = 0;
        while adc.adc_is_busy() {
            polls += 1;
        }
        let dvcc_count = block!(adc.read_count(&mut dvcc)).unwrap();

        writeln!(
            tx,
            "DVSS (channel 14): {} = {} mV, DVCC (channel 15): {} = {} mV, busy for {} polls\r",
            dvss_count,
            adc.count_to_mv(dvss_count, AVCC_MV),
            dvcc_count,
            adc.count_to_mv(dvcc_count, AVCC_MV),
            polls
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
