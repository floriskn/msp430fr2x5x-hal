//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Tests `ClockConfig::mclk_dcoclk_hz` and the delays of the `SysDelay` it returns. The device checks
//! itself and reports the results on eUSCI_A0, and the scope checks them independently.
//!
//! Set `TARGET_HZ` to the MCLK frequency to test, from 1 MHz to 16 MHz. The FLL locks MCLK to the
//! largest multiple of its 32.768 kHz reference that doesn't exceed it: 5 MHz becomes 152 ×
//! 32.768 kHz = 4.980736 MHz. (MCLK runs from DCOCLKDIV, fDCOCLKDIV = (FLLN + 1) × (fFLLREFCLK ÷ n)
//! with n = 1 by default: SLAU445I 3.2.5, p. 104. MCLK may be 16 MHz at most: SLASEE4C 5.3, p. 17.)
//!
//! The device measures MCLK against ACLK, which runs from the FLL's reference, REFO, and times each delay
//! in MCLK cycles. An LED on P1.0 lights if every check passes.
//!
//! The MSP430FR2476 version of this example can lock the FLL to a function generator on XIN instead of
//! REFO, for the generator's accuracy. That option doesn't fit into the 7.25 KB of program FRAM of this
//! device (SLASEE4C Table 6-19, p. 62), so here the FLL always runs from REFO.
//! (MCLK on P1.3, and P1.0 and P1.6 as GPIO outputs: SLASEE4C Table 6-15, p. 58. UCA0TXD is P1.4:
//! SLASEE4C Table 6-11, p. 53. No board document covers the parts to connect: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (the scope, an LED and a resistor, and a 3.3-V USB-to-UART adapter):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND. Connect the adapter: its RX to
//!    P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Flash this example.
//! 3. Reset the device, with RST/NMI low for a moment, to run the test again while the terminal is open.
//!    Expected: every check ends in `: PASS`, then comes `ALL PASS`, and the LED lights.
//! 4. Scope, with a 10X probe and its ground clip on GND:
//!    - P1.3/MCLK: Analysis > Counter shows the MCLK the report prints, within the ±3.5 % REFO is
//!      specified to (SLASEE4C Table 5-7, p. 27).
//!    - P1.6: a square wave, high and low for `PULSE_US` each. Measure its +Width: it's `PULSE_US` plus a
//!      few loop instructions, with the same accuracy.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    capture::{CapTrigger, Capture, CaptureParts3, OverCapture, TimerConfig, CCR1},
    clock::{fll_status, ClockConfig, FllStatus, MclkDiv, SmclkDiv},
    delay::SysDelay,
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    pwm::PwmParts3,
    serial::*,
    watchdog::Wdt,
};
use msp430fr25x2::Ta1;
use nb::block;
use panic_msp430 as _;

/// MCLK frequency to test, from 1 MHz to 16 MHz
const TARGET_HZ: u32 = 5_000_000;
/// How long the square wave on P1.6 stays high, and then low, in microseconds
const PULSE_US: u32 = 100;

/// How many MCLK cycles a delay may take beyond the time requested: the function call, and the
/// rounding of the loop count
const DELAY_SLACK_CYCLES: u32 = 100;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut square_wave = p1.pin6;
    led.set_low().ok();
    square_wave.set_low().ok();

    // P1.3 = MCLK with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let _mclk_out = p1.pin3.to_output().to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I
    // Table 3-9, p. 118). REFO is the FLL reference and ACLK (SELREF = 01b: SLAU445I Table 3-7, p. 116;
    // SELA = 01b: SLAU445I Table 3-8, p. 117).
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk_hz(TARGET_HZ, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);
    let mclk_hz = smclk.freq();
    let aclk_hz = aclk.freq();

    // The UART runs from ACLK, so the report stays readable even if MCLK is wrong
    // P1.4 = UCA0TXD with P1SELx = 01 (SLASEE4C Table 6-15, p. 58)
    // (8N1, LSB first: UCMSB, UC7BIT, UCSPB, UCPEN in SLAU445I Table 22-8, p. 593; ACLK is UCSSEL = 01b:
    // SLASEE4C Table 6-8, p. 49)
    let mut tx = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_aclk(&aclk)
    .tx_only(p1.pin4.to_alternate1());

    // TA0 counts ACLK, and toggles its CCR0 output every 32 cycles (Toggle mode: "The output period is
    // double the timer period", SLAU445I Table 13-2, p. 376). TA1 counts SMCLK (= MCLK) and
    // captures the rising edges of that output on CCR0 (input B of TA1's CCR0 is TA0's CCR0 output:
    // SLASEE4C Figure 6-2, p. 54), so two captures are 64 ACLK cycles apart. CCR1 captures from software,
    // to time the delays.
    // (TASSEL: ACLK = 01b, SMCLK = 10b: SLASEE4C Table 6-8, p. 49. CCR0 captures on rising edges of
    // input B: CM = 01b, CCIS = 01b, SLAU445I Table 13-6, p. 386. A software capture switches CCIS
    // between GND and VCC: SLAU445I 13.2.4.1.1, p. 376.)
    let _ta0 = PwmParts3::new(periph.ta0, TimerConfig::aclk(&aclk), 31);
    let captures = CaptureParts3::config(periph.ta1, TimerConfig::smclk(&smclk))
        .config_cap0_input_B()
        .config_cap0_trigger(CapTrigger::RisingEdge)
        .config_cap1_software()
        .commit();
    let mut aclk_edges = captures.cap0;
    let mut stopwatch = captures.cap1;

    let mut pass = true;

    print(&mut tx, "\r\n--- dco_delay_test ---\r\nTarget ");
    print_num(&mut tx, TARGET_HZ, 0);
    print(&mut tx, " Hz, FLL reference REFO\r\n");

    // The frequency the FLL locks to, from the HAL
    let multiplier = mclk_hz / aclk_hz;
    print(&mut tx, "MCLK ");
    print_num(&mut tx, mclk_hz, 0);
    print(&mut tx, " Hz = ");
    print_num(&mut tx, multiplier, 0);
    print(&mut tx, " x ");
    print_num(&mut tx, aclk_hz, 0);
    print(&mut tx, " Hz\r\n");

    // The FLL keeps fine-tuning after it reports lock (measured on an MSP430FR2476). Its lock time is
    // typically 280 ms (SLASEE4C Table 5-5, p. 26: tFLL,lock at 16 MHz, a typical value, no maximum)
    delay.delay_ms(280);
    // MCLK cycles in 8 x 64 ACLK cycles. Each capture is read long before the next one arrives.
    let mut previous = read_capture(&mut aclk_edges);
    let mut mclk_cycles = 0u32;
    for _ in 0..8 {
        let now = read_capture(&mut aclk_edges);
        mclk_cycles += now.wrapping_sub(previous) as u32;
        previous = now;
    }
    let expected = multiplier * 8 * 64;
    // The FLL keeps MCLK within about 1 % of its target (SLASEE4C Table 5-5, p. 25: fDCO,FLL is within
    // ±1.0 % at 25°C with REFO as the reference, ±2.0 % from –40°C to 85°C)
    let ratio_ok = mclk_cycles.abs_diff(expected) * 100 <= expected;
    // CSCTL7.FLLUNLOCK = 00b (SLAU445I Table 3-11, p. 121)
    let locked = fll_status() == FllStatus::Locked;
    print(&mut tx, "MCLK / ACLK measured: ");
    print_num(&mut tx, mclk_cycles * 100 / (8 * 64), 2);
    print(&mut tx, if locked { ", FLL locked" } else { ", FLL NOT locked" });
    print(&mut tx, pass_fail(ratio_ok && locked));
    pass &= ratio_ok && locked;

    // How the DCO was set up, from the clock registers (the HAL has no API for these).
    // CSCTL1: DCOFTRIMEN is bit 7 (clear: the factory trim), DCOFTRIM bits 6-4, DCORSEL bits 3-1
    // (SLAU445I Table 3-5, p. 114). CSCTL0: the DCO tap is bits 8-0 (SLAU445I Table 3-4, p. 113). The
    // software trim aims for a tap close to 256 (SLAU445I 3.2.11.2, p. 107).
    let cs = unsafe { &*msp430fr25x2::Cs::ptr() };
    let csctl0 = cs.csctl0().read().bits();
    let csctl1 = cs.csctl1().read().bits();
    print(&mut tx, "DCO range ");
    print_num(&mut tx, (csctl1 >> 1 & 0b111) as u32, 0);
    if csctl1 & 0x80 != 0 {
        print(&mut tx, ", trim ");
        print_num(&mut tx, (csctl1 >> 4 & 0b111) as u32, 0);
    } else {
        print(&mut tx, ", factory trim");
    }
    print(&mut tx, ", tap ");
    print_num(&mut tx, (csctl0 & 0x1FF) as u32, 0);
    print(&mut tx, " (the software trim aims for 256)\r\n");

    // Each delay, the time it requests in ns, and the time it may round up to
    let delays: [(&str, u32, u32, fn(&mut SysDelay)); 11] = [
        ("delay_ns(250)", 250, 1_000, |d| d.delay_ns(250)),
        ("delay_ns(1500)", 1_500, 2_000, |d| d.delay_ns(1500)),
        ("delay_ns(2500000)", 2_500_000, 2_501_000, |d| d.delay_ns(2_500_000)),
        ("delay_us(1)", 1_000, 1_000, |d| d.delay_us(1)),
        ("delay_us(10)", 10_000, 10_000, |d| d.delay_us(10)),
        ("delay_us(100)", 100_000, 100_000, |d| d.delay_us(100)),
        ("delay_us(999)", 999_000, 999_000, |d| d.delay_us(999)),
        ("delay_us(1000)", 1_000_000, 1_000_000, |d| d.delay_us(1000)),
        ("delay_us(1500)", 1_500_000, 1_500_000, |d| d.delay_us(1500)),
        ("delay_ms(1)", 1_000_000, 1_000_000, |d| d.delay_ms(1)),
        ("delay_ms(3)", 3_000_000, 3_000_000, |d| d.delay_ms(3)),
    ];
    // Time every delay first, and only then work out what to expect: the compiler may move
    // calculations between the stopwatch readings, but not past `black_box`
    let overhead = time(&mut stopwatch, || {});
    let mut measured = [0u16; 11];
    for (cycles, (_, _, _, run)) in measured.iter_mut().zip(delays) {
        *cycles = time(&mut stopwatch, || run(&mut delay)).saturating_sub(overhead);
    }
    let mclk_hz = core::hint::black_box(mclk_hz);
    for ((name, min_ns, max_ns, _), cycles) in delays.into_iter().zip(measured) {
        let cycles = cycles as u32;
        let min_cycles = ns_to_cycles(min_ns, mclk_hz);
        let max_cycles = ns_to_cycles(max_ns, mclk_hz) * 101 / 100 + DELAY_SLACK_CYCLES;
        let ok = (min_cycles..=max_cycles).contains(&cycles);
        print(&mut tx, name);
        print(&mut tx, ": ");
        // Tenths of a microsecond, cycles × 10^7 / MCLK, in 32 bits: MCLK in kHz
        print_num(&mut tx, cycles * 10_000 / (mclk_hz / 1000), 1);
        print(&mut tx, " us, ");
        print_num(&mut tx, cycles, 0);
        print(&mut tx, " cycles");
        print(&mut tx, pass_fail(ok));
        pass &= ok;
    }

    print(&mut tx, if pass { "ALL PASS\r\n" } else { "FAILED\r\n" });
    print(&mut tx, "Scope: MCLK on P1.3, square wave on P1.6, high and low for ");
    print_num(&mut tx, PULSE_US, 0);
    print(&mut tx, " us each\r\n");
    led.set_state(pass.into()).ok();

    loop {
        square_wave.set_high().ok();
        delay.delay_us(PULSE_US);
        square_wave.set_low().ok();
        delay.delay_us(PULSE_US);
    }
}

/// The SMCLK cycles from just before `f` runs to just after it returns, from two software captures
/// (SLAU445I 13.2.4.1.1, p. 376)
fn time(stopwatch: &mut Capture<Ta1, CCR1>, f: impl FnOnce()) -> u16 {
    stopwatch.trigger_capture();
    let start = read_capture(stopwatch);
    f();
    stopwatch.trigger_capture();
    read_capture(stopwatch).wrapping_sub(start)
}

/// The next captured count. After a missed capture (COV: SLAU445I 13.2.4.1, p. 375) this is the
/// latest one.
fn read_capture<C>(capture: &mut Capture<Ta1, C>) -> u16
where
    Ta1: msp430_hal::timer::CapCmp<C>,
{
    match block!(capture.capture()) {
        Ok(count) | Err(OverCapture(count)) => count,
    }
}

/// MCLK cycles in `ns` nanoseconds, rounded up: `ns` × MCLK in kHz / 10^6, with the whole microseconds and
/// the rest of `ns` apart, so that it fits in 32 bits. MCLK in whole kHz is at most 0.1 % low.
fn ns_to_cycles(ns: u32, mclk_hz: u32) -> u32 {
    let khz = mclk_hz / 1000;
    (ns / 1000 * khz + ns % 1000 * khz / 1000).div_ceil(1000)
}

fn pass_fail(ok: bool) -> &'static str {
    if ok { ": PASS\r\n" } else { ": FAIL\r\n" }
}

fn print(tx: &mut impl Write, text: &str) {
    tx.write_all(text.as_bytes()).ok();
}

/// Print `value` / 10^`decimals`, with that many decimals
fn print_num(tx: &mut impl Write, value: u32, decimals: usize) {
    let mut buf = [0u8; 12];
    let mut pos = buf.len();
    let mut rest = value;
    let mut digits = 0;
    loop {
        if decimals > 0 && digits == decimals {
            pos -= 1;
            buf[pos] = b'.';
        }
        pos -= 1;
        buf[pos] = b'0' + (rest % 10) as u8;
        rest /= 10;
        digits += 1;
        if rest == 0 && digits > decimals {
            break;
        }
    }
    tx.write_all(&buf[pos..]).ok();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
