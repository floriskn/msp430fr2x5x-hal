//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The ADC window comparator, with conversions started by the RTC and handled in the ADC interrupt.
//!
//! The RTC starts a conversion of P1.1 ten times a second. The window comparator sorts each result: below
//! 1.0 V, from 1.0 V to 2.0 V, or above 2.0 V, and the ADC interrupt keeps the window of the latest one. An
//! LED on P1.0 is on while the voltage is inside the window, and eUSCI_A0 prints the latest voltage and its
//! window twice a second.
//! (RTC overflows trigger the ADC, ADCSHSx = 01b: SLASEE4C 6.10.11, p. 55; SLASEE4C Table 6-14, p. 56.
//! Window comparator: SLAU445I 21.2.7.7, p. 555. ADC interrupts: SLAU445I 21.2.7.10, p. 558. A1 is P1.1:
//! SLASEE4C Table 6-13, p. 55. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. P1.0 is a GPIO output,
//! P1SELx = 00 and P1DIR = 1: SLASEE4C Table 6-15, p. 58. No board document covers the parts to connect:
//! there is none for the MSP430FR25x2.)
//!
//! How to test (function generator and the multimeter, or a jumper wire; an LED and a resistor; a 3.3-V
//! USB-to-UART adapter):
//! 1. Power the MSP430FR2522 from 3.3 V, as the code assumes, and connect an LED with a series resistor
//!    (about 1 kΩ) from P1.0 to GND. Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open
//!    its COM port at 9600 baud.
//! 2. Generator: the DC waveform, Offset 1.500 V, output load High-Z. Connect it to P1.1, its ground to
//!    GND. Check the voltage with the multimeter first: at most 3.3 V.
//! 3. Flash this example. The LED is on, and the terminal shows `1500 mV: InsideWindow`, give or take a few
//!    millivolts.
//! 4. Set the offset to 0.5 V: the LED goes off, and the terminal shows `BelowWindow`. Set it to 2.5 V:
//!    `AboveWindow`.
//! 5. A slow sine wave shows all three in turn: 0.2 Hz, 3 Vpp, offset 1.5 V (0 V to 3 V).
//!
//! Without the generator, a jumper wire from P1.1 to GND (`BelowWindow`) or to 3.3 V (`AboveWindow`) works
//! too.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    adc::{
        Adc, AdcConfig, AdcInterruptFlags, AdcVector, ClockDivider, ConversionConfig, ConversionMode, Predivider,
        Resolution, SampleMode, SampleTime, SamplingRate, TriggerSource,
    },
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    rtc::{Rtc, RtcDiv},
    serial::*,
    watchdog::Wdt,
};
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// The ADC's reference, AVCC: the 3.3 V the MSP430FR2522 runs from
const AVCC_MV: u16 = 3300;
/// The window, in counts of the 10-bit result: 1024 × voltage / 3.3 V (SLAU445I 21.2.1, p. 541)
const LOW: u16 = 310; // 1.0 V
const HIGH: u16 = 620; // 2.0 V

static ADC: Mutex<RefCell<Option<Adc>>> = Mutex::new(RefCell::new(None));
/// The window the last result was in
static LAST: Mutex<Cell<AdcVector>> = Mutex::new(Cell::new(AdcVector::None));

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
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

    // ADCLO and ADCHI, in the unsigned format of the results (SLAU445I 21.2.7.7, p. 555). Each result sets
    // one of ADCLOIFG, ADCINIFG and ADCHIIFG, and ADCIE requests the interrupt for them (SLAU445I
    // Table 21-13, p. 570).
    adc.set_window(LOW, HIGH);
    adc.enable_interrupts(AdcInterruptFlags::BelowWindow | AdcInterruptFlags::InsideWindow | AdcInterruptFlags::AboveWindow);
    // Repeat-single-channel mode (ADCCONSEQx = 10b), each conversion started by an RTC overflow (ADCSHSx =
    // 01b: SLAU445I Table 21-4, p. 563)
    adc.start(
        &mut input,
        ConversionConfig {
            mode: ConversionMode::RepeatSingle,
            trigger: TriggerSource::Rtc,
            sample_mode: SampleMode::RisingEdge,
            back_to_back: false,
        },
    );
    with(|cs| ADC.borrow_ref_mut(cs).replace(adc));

    // The RTC counts ACLK, from REFO at 32.768 kHz, and overflows every 3277 counts: 10 times a second
    // (RTCSS = 01b with RTCCKSEL = 1 selects ACLK: SLASEE4C Table 6-12, p. 55; SLAU445I Table 1-31, p. 82;
    // RTCMOD: SLAU445I 15.2.1, p. 417)
    let mut rtc = Rtc::new(periph.rtc).use_aclk(&aclk);
    rtc.set_clk_div(RtcDiv::_1);
    rtc.start(3276);

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    loop {
        let (last, count) = with(|cs| {
            let count = ADC.borrow_ref(cs).as_ref().map_or(0, |adc| adc.adc_get_result());
            (LAST.borrow(cs).get(), count)
        });
        led.set_state((last == AdcVector::InsideWindow).into()).ok();

        // count × 3300 / 1024, as `Adc::count_to_mv` works it out
        let mv = count as u32 * AVCC_MV as u32 / 1024;
        print_num(&mut tx, mv);
        let window = match last {
            AdcVector::BelowWindow => " mV: BelowWindow\r\n",
            AdcVector::InsideWindow => " mV: InsideWindow\r\n",
            AdcVector::AboveWindow => " mV: AboveWindow\r\n",
            _ => " mV: no result yet\r\n",
        };
        print(&mut tx, window);
        delay.delay_ms(500);
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

// The ADC vector (FFE8h: SLASEE4C Table 6-2, p. 46). Reading ADCIV returns the highest-priority pending
// flag and clears it (SLAU445I 21.2.7.10.1, p. 558).
#[interrupt]
fn ADC() {
    with(|cs| {
        if let Some(adc) = ADC.borrow_ref_mut(cs).as_mut() {
            loop {
                match adc.interrupt_source() {
                    AdcVector::None => break,
                    window @ (AdcVector::BelowWindow | AdcVector::InsideWindow | AdcVector::AboveWindow) => {
                        LAST.borrow(cs).set(window)
                    }
                    _ => {}
                }
            }
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
