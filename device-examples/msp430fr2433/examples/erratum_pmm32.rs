//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A stress test for erratum PMM32: the device can lock up, or run code it shouldn't, when it enters LPM3
//! or LPM4 just as an interrupt arrives, if MODCLK starts or stops at that moment or SMCLK runs slower
//! than MCLK. The program goes to sleep and is woken up again and again, while the ADC, clocked by MODCLK,
//! converts once per period of TA1. Every 1024 sleeps LED1 toggles and the backchannel UART prints the
//! count. A lock-up stops both, and a reset prints a new `start` line.
//!
//! The erratum: "The device might enter lockup state or start executing unintentional code" when "1) The
//! device transitions from AM to LPM3/4", "2) An interrupt is requested" and "3) MODCLK is requested (e.g.
//! triggered by ADC) or removed (e.g. end of ADC conversion)" at the same time (condition 1), or when the
//! first two happen while "Neither MODCLK nor SMCLK are running" and "SMCLK is configured with a different
//! frequency than MCLK" (condition 2) (SLAZ664S PMM32, p. 11 to p. 12). `request_lpm3()` and
//! `request_lpm4()` apply its workaround 2 (SLAZ664S PMM32, p. 12): "Place the FRAM in INACTIVE mode before
//! any entry to LPM3/4 by clearing the FRPWR bit and FRLPMPWR bit (if exist) in the GCCTL0 register. This
//! must be performed from RAM". They keep interrupts disabled from there to the start of the sleep, so no
//! handler powers the FRAM up again in between (see the HAL's `lpm` module).
//!
//! In LPM3, TA1 counts ACLK from REFO, 33 cycles per period, about 1 ms. As the count reaches CCR0, its
//! CCR1 output goes high, which starts an ADC conversion and so requests MODCLK, and CCIFG0 requests TA1's
//! CCR0 interrupt. The end of the conversion, about 0.25 ms later, removes MODCLK and requests the ADC
//! interrupt. Both handlers wake the CPU. Before each sleep the main loop waits a random 0 to 1023 µs, so the
//! sleeps start at every point of the period, some of them just as one of these events happens: condition
//! 1. LPM4 stops ACLK and the ADC, and only an I/O wakes the device from it: with `LPM4_FROM_GENERATOR` the
//! program sleeps in LPM4 instead, without TA1 and the ADC, woken by each rising edge of a square wave from
//! a function generator on P2.2: condition 2 alone. SMCLK runs at half MCLK's frequency, for condition 2,
//! either way. MCLK runs at 1 MHz, so the conditions of erratum CS13 aren't met (DCO above 2 MHz: SLAZ664S
//! CS13, p. 9) and the HAL leaves the DCO alone: a lock-up here is PMM32.
//! (REFO: 32768 Hz, SLASE59F Table 5-7, p. 25. REFO, ACLK, TA1 and the ADC run in LPM3, the ADC is off in
//! LPM4, and only I/O wakes the device from LPM4: SLASE59F Table 6-1, p. 40 to p. 41. TA1's CCR1 output is
//! the ADC's timer trigger, ADCSHSx = 10b, "TA1.1B": SLASE59F Table 6-16, p. 53; SLASE59F Table 6-12, p. 51.
//! The reset/set output and CCIFG0 both change "when the timer counts to the TAxCCR0 value": SLAU445I
//! Table 13-2, p. 376; SLAU445I 13.2.3.1, p. 371. The ADC requests MODCLK during a conversion only: SLAU445I
//! 3.2.15.1, p. 111; SLAU445I 21.2.4, p. 542. The conversion: 64 + 12 cycles of MODCLK / 16, with MODCLK at
//! 4.8 MHz typical: SLAU445I Table 21-3, p. 561; SLAU445I Table 21-5, p. 565; SLASE59F Table 5-9, p. 26.
//! LED1 on P1.0 is red, S3 is the reset button, and P2.2 goes to the header only: SLAU739 Figure 18, p. 23.)
//!
//! How to test (for LPM4 a function generator, and the scope to check its signal):
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 2. Expected: `start, reset cause: 00h`, then `sleeps: 1024`, `sleeps: 2048` and so on, with LED1
//!    toggling at each line. The cause is the value of SYSRSTIV (SLASE59F Table 6-9, p. 48): 00h, none,
//!    straight after flashing, and 04h after S3.
//! 3. Leave it running for at least an hour, overnight to be sure: the errata sheet gives no failure rate.
//!    Pass: the count keeps going up. Fail: LED1 and the count stop (a lock-up: press S3 to recover), a new
//!    `start` line appears (a reset), or anything else unexpected happens (code run by mistake).
//! 4. To check that the test can catch the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the
//!    loop then goes to sleep with a plain write to the status register, with the FRAM on. If it fails that
//!    way but not with the HAL, the workaround works. If it doesn't fail either way, the test can't tell.
//! 5. LPM4: set the generator to a square wave, 2 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z, and check the levels on the scope first: a negative or >3.6 V signal can damage the pin.
//!    Connect it to P2.2 (J2 pin 18), its ground to GND (J2 pin 20), set `LPM4_FROM_GENERATOR` to true, and
//!    repeat steps 1 to 4. Keep the generator on: nothing else wakes the device, so without it the count
//!    stops.
//! 6. TA1 and the ADC keep running, interrupts and all, through a reset that is only a PUC, so the next
//!    example you flash could hang in the default interrupt handler once it enables interrupts. After
//!    flashing it, press S3 once. (Timer_A and ADC registers reset at a POR, "rw-(0)": SLAU445I
//!    Figure 13-16, p. 384; SLAU445I Figure 21-21, p. 561; SLAU445I Table 0-1, p. 28. S3, the RST pin,
//!    resets with a BOR, which includes a POR: SLAU445I 1.2, p. 30.)
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::{arch::asm, cell::RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    adc::{
        adc_ch14_vss, Adc, AdcConfig, AdcInterruptFlags, ClockDivider, ConversionConfig, ConversionMode,
        Predivider, Resolution, SampleMode, SampleTime, SamplingRate, TriggerSource,
    },
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV},
    lpm::{request_lpm3, request_lpm4},
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    serial::*,
    timer::TimerParts3,
    watchdog::Wdt,
};
use msp430fr2433::{interrupt, Ta1, P2};
use panic_msp430 as _;

/// `false` goes to sleep without the HAL's workaround, to check that this test can catch the erratum
const USE_HAL_WORKAROUND: bool = true;
/// `true` sleeps in LPM4, woken by a function generator on P2.2, instead of in LPM3, woken by TA1 and the ADC
const LPM4_FROM_GENERATOR: bool = false;

/// The status register bits of LPM3, SCG1, SCG0 and CPUOFF, and of LPM4, with OSCOFF as well (SLAU445I
/// Table 1-2, p. 39; SLAU445I Figure 4-9, p. 130)
const LPM3_BITS: u16 = 0x00D0;
const LPM4_BITS: u16 = 0x00F0;
/// TA1's period in ACLK cycles: about 1 ms, with ACLK from REFO at 32768 Hz (SLASE59F Table 5-7, p. 25)
const TA1_PERIOD: u16 = 33;

static ADC: Mutex<RefCell<Option<Adc>>> = Mutex::new(RefCell::new(None));
static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next reset (reading
    // SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let cause = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = DCOCLKDIV in the 1 MHz range, SMCLK = MCLK / 2, and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_2)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
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
    tx.write_all(b"start, reset cause: ").ok();
    print_hex(&mut tx, cause.map_or(0, u16::from));
    tx.write_all(b"\r\n").ok();

    // The ADC counts MODCLK / 4 / 4 (ADCSSELx = 00b, ADCDIVx = 011b: SLAU445I Table 21-4, p. 563;
    // ADCPDIVx = 01b: SLAU445I Table 21-5, p. 565) and samples for 64 of those cycles (ADCSHTx = 0100b:
    // SLAU445I Table 21-3, p. 561). It converts channel 14, DVSS, which needs no pin (SLASE59F
    // Table 6-15, p. 53), once for each rising edge of TA1's CCR1 output (repeat-single-channel mode,
    // ADCCONSEQx = 10b; ADCSHSx = 10b, ADCSHP = 1: SLAU445I Table 21-4, p. 563 to p. 564). ADCIFG0
    // requests the ADC interrupt at the end of each conversion (SLAU445I Table 21-13, p. 570). Setting
    // it up switches it off, ADCON = 0 (SLAU445I Table 21-3, p. 561), and only LPM3 mode starts it: in
    // LPM4 mode it stays off, even if a run in LPM3 mode left it converting.
    let mut adc = AdcConfig::new(
        ClockDivider::_4,
        Predivider::_4,
        Resolution::Bits10,
        SamplingRate::Max200ksps,
        SampleTime::Cycles64,
    )
    .use_modclk()
    .configure(periph.adc);
    adc.enable_interrupts(AdcInterruptFlags::ResultReady);

    if LPM4_FROM_GENERATOR {
        // TA1 stays stopped: setting it up stops it, MC = 0 (SLAU445I Table 13-4, p. 384), even if a run in
        // LPM3 mode left it running. Stopped, it requests no ACLK, which would turn LPM4 into LPM3: a timer
        // does when it "selects ACLK as its clock source and the timer is enabled" (SLAU445I 3.2.12,
        // p. 108).
        TimerParts3::new(periph.ta1, TimerConfig::aclk(&aclk));

        // P2.2 with its pulldown, so it doesn't float without the generator (PxDIR = 0, PxREN = 1, PxOUT = 0:
        // SLAU445I Table 8-1, p. 313), and a rising edge requests the port 2 interrupt (PxIES = 0: SLAU445I
        // Table 8-16, p. 336; PxIE: SLAU445I Table 8-17, p. 336)
        let p2 = Batch::new(periph.p2).config_pin2(|p| p.pulldown()).split(&pmm);
        let mut input = p2.pin2;
        input.select_rising_edge_trigger().enable_interrupts();
        with(|cs| P2IV.borrow_ref_mut(cs).replace(p2.pxiv));
    } else {
        adc.start(
            &mut adc_ch14_vss(),
            ConversionConfig {
                mode: ConversionMode::RepeatSingle,
                trigger: TriggerSource::Timer,
                sample_mode: SampleMode::RisingEdge,
                back_to_back: false,
            },
        );

        // TA1 counts ACLK (TASSEL = 01b: SLASE59F Table 6-7, p. 46) in up mode, TA1_PERIOD counts per period
        // (SLAU445I 13.2.3.1, p. 371). Its CCR1 output, in reset/set mode, goes high as the count reaches
        // CCR0 and low again at 1 (SLAU445I Table 13-2, p. 376). It drives no pin.
        let ta1 = PwmParts3::new(periph.ta1, TimerConfig::aclk(&aclk), TA1_PERIOD - 1);
        let _trigger = ta1.pwm1.into_adc_trigger(1);
        // CCIE requests TA1's CCR0 interrupt for CCIFG0 (SLAU445I Table 13-6, p. 386). The HAL's PWM has no
        // call for it, so this sets the bit in TA1CCTL0 itself.
        unsafe { &*Ta1::ptr() }.ta1cctl0().modify(|_, w| w.ccie().set_bit());
    }
    with(|cs| ADC.borrow_ref_mut(cs).replace(adc));

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    let mut random: u16 = 1;
    let mut sleeps: u32 = 0;
    loop {
        // Wait a random 0 to 1023 µs, so the sleep starts at another point of the period
        random = next_random(random);
        delay.delay_us((random & 0x3FF) as u32);

        if USE_HAL_WORKAROUND {
            if LPM4_FROM_GENERATOR {
                request_lpm4();
            } else {
                request_lpm3();
            }
        } else if LPM4_FROM_GENERATOR {
            // A plain entry, as the errata sheet's own code does it, with `__bis_SR_register()` (SLAZ664S
            // PMM32, p. 12)
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM4_BITS, options(nostack)) };
        } else {
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM3_BITS, options(nostack)) };
        }
        sleeps += 1;

        if sleeps % 1024 == 0 {
            led1.toggle().ok();
            tx.write_all(b"sleeps: ").ok();
            print_decimal(&mut tx, sleeps);
            tx.write_all(b"\r\n").ok();
            // Finish sending before the next sleep: a busy eUSCI keeps its clock, SMCLK, running, which
            // turns LPM3 and LPM4 into LPM0 (SLAU445I 22.3.14, p. 590; SLAU445I Table 1-3, p. 39)
            tx.flush().ok();
        }
    }
}

/// The next number of a xorshift generator, for the random waits
fn next_random(x: u16) -> u16 {
    let x = x ^ (x << 7);
    let x = x ^ (x >> 9);
    x ^ (x << 8)
}

/// Print `n` in decimal. `write!` would do it too, but `core::fmt` takes several KiB of FRAM, more than the
/// MSP430FR2522 version of this test has to spare.
fn print_decimal(tx: &mut impl Write, n: u32) {
    let mut divisor: u32 = 1_000_000_000;
    while divisor > 1 && n < divisor {
        divisor /= 10;
    }
    while divisor > 0 {
        tx.write_all(&[b'0' + (n / divisor % 10) as u8]).ok();
        divisor /= 10;
    }
}

/// Print the low byte of `n` as two hexadecimal digits and an `h`, as the data sheet writes SYSRSTIV values
fn print_hex(tx: &mut impl Write, n: u16) {
    const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[DIGITS[(n >> 4 & 0xF) as usize], DIGITS[(n & 0xF) as usize], b'h']).ok();
}

// The TA1 CCR0 vector (FFF4h: SLASE59F Table 6-2, p. 42). CCIFG0 "is automatically reset when the TAxCCR0
// interrupt request is serviced" (SLAU445I 13.2.6.1, p. 380). `wake_cpu` returns the CPU to active mode, so
// the main loop carries on after the sleep (the SR saved on the stack: SLAU445I 1.4.2, p. 40).
#[interrupt(wake_cpu)]
fn TIMER1_A0() {}

// The ADC vector (FFDEh: SLASE59F Table 6-2, p. 42), with `wake_cpu` as above
#[interrupt(wake_cpu)]
fn ADC() {
    // Reading the result clears ADCIFG0 (SLAU445I Table 21-14, p. 571)
    with(|cs| {
        if let Some(adc) = ADC.borrow_ref_mut(cs).as_mut() {
            let _ = adc.result();
        }
    });
}

// The port 2 vector, P2IFG.0 to P2IFG.7 through P2IV (FFDAh: SLASE59F Table 6-2, p. 42), with `wake_cpu` as
// above. Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
#[interrupt(wake_cpu)]
fn PORT2() {
    with(|cs| {
        if let Some(p2iv) = P2IV.borrow_ref_mut(cs).as_mut() {
            let _: GpioVector = p2iv.get_interrupt_vector();
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
