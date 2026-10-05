//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Tests `ClockConfig::mclk_dcoclk_hz` and the delays of the `SysDelay` it returns. The board checks
//! itself and reports the results on the backchannel UART, and the scope checks them independently.
//!
//! Set `TARGET_HZ` to the MCLK frequency to test, from 1 MHz to 24 MHz. The FLL locks MCLK to the
//! largest multiple of its 32.768 kHz reference that doesn't exceed it: 5 MHz becomes 152 ×
//! 32.768 kHz = 4.980736 MHz. (MCLK runs from DCOCLKDIV, fDCOCLKDIV = (FLLN + 1) × (fFLLREFCLK ÷ n)
//! with n = 1 by default: SLAU445I 3.2.5, p. 104. MCLK may be 24 MHz at most: SLASEC4D 5.3, p. 27.)
//!
//! The board measures MCLK against ACLK, which runs from the FLL reference, and times each delay in
//! MCLK cycles. LED2 (green) lights if every check passes, LED1 (red) if one fails (SLAU680 Figure 18,
//! p. 26).
//!
//! With `FLL_REF_FROM_XT1` set to `true`, as it is, the FLL locks to the LaunchPad's 32.768-kHz crystal on
//! XT1 instead of REFO, so MCLK and the delays have the crystal's accuracy. Set it to `false` to test with
//! REFO. (The crystal Q1 is connected to XIN, P2.7, and XOUT, P2.6: SLAU680 2.5, p. 13; SLAU680
//! Figure 18, p. 26.)
//!
//! How to test (optionally the scope):
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 2. Press the reset button S3 to run the test again while the terminal is open. Expected: every
//!    check ends in `: PASS`, then comes `ALL PASS`, and LED2 lights green. If the crystal doesn't start
//!    within 3 s, the report says `XT1 didn't start: FAIL`.
//! 3. Scope, with a 10X probe and its ground clip on GND (J3 pin 22):
//!    - P3.0/MCLK (J2 pin 11): Analysis > Counter shows the MCLK the report prints, within the ±0.5 % the
//!      FLL is specified to with an XT1 crystal as the reference (SLASEC4D Table 5-5, p. 37), or with
//!      `FLL_REF_FROM_XT1` set to `false` within the ±3.5 % of REFO (SLASEC4D Table 5-7, p. 40).
//!    - P1.6 (J1 pin 3): a square wave, high and low for `PULSE_US` each. Measure its +Width: it's
//!      `PULSE_US` plus a few loop instructions, with the same accuracy.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    capture::{CapTrigger, Capture, CaptureParts3, OverCapture, TimerConfig, CCR1},
    clock::{fll_status, ClockConfig, FllStatus, MclkDiv, SmclkDiv, Xt1Config},
    delay::SysDelay,
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    prelude::*,
    pwm::PwmParts3,
    serial::*,
    watchdog::Wdt,
};
use msp430fr2355::Tb1;
use nb::block;
use panic_msp430 as _;

/// MCLK frequency to test, from 1 MHz to 24 MHz
const TARGET_HZ: u32 = 5_000_000;
/// How long the square wave on P1.6 stays high, and then low, in microseconds
const PULSE_US: u32 = 100;
/// Lock the FLL to the LaunchPad's 32.768 kHz crystal on XT1 instead of REFO
const FLL_REF_FROM_XT1: bool = true;

/// How many MCLK cycles a delay may take beyond the time requested: the function call, and the
/// rounding of the loop count
const DELAY_SLACK_CYCLES: u32 = 100;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let p6 = Batch::new(periph.p6)
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    // LED1 on P1.0 is red and LED2 on P6.6 green (SLAU680 Figure 18, p. 26). P1.6 is J1 pin 3 (SLAU680
    // Figure 10, p. 15).
    let mut led1 = p1.pin0;
    let mut led2 = p6.pin6;
    let mut square_wave = p1.pin6;
    led1.set_low().ok();
    led2.set_low().ok();
    square_wave.set_low().ok();

    // P3.0 = MCLK with P3SEL = 01 and P3DIR = 1 (SLASEC4D Table 6-65, p. 100)
    let _mclk_out = p3.pin0.to_output().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I
    // Table 3-9, p. 118). With the crystal, XT1 runs in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10,
    // p. 120) and is the FLL reference (SELREF = 00b: SLAU445I Table 3-7, p. 116) and ACLK (SELA = 00b:
    // SLAU445I Table 3-8, p. 117); otherwise REFO is both (SELREF = 01b, SELA = 01b).
    let (smclk, aclk, mut delay, xt1_ok) = if FLL_REF_FROM_XT1 {
        // XIN is P2.7 and XOUT P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
        let config = ClockConfig::new(periph.cs)
            .mclk_dcoclk_hz(TARGET_HZ, MclkDiv::_1)
            .smclk_on(SmclkDiv::_1)
            .xt1clk_on(Xt1Config::crystal(32_768, p2.pin7.to_alternate2(), p2.pin6.to_alternate2()))
            .fll_ref_xt1()
            .aclk_xt1clk();
        // The crystal starts up in 1000 ms typically (tSTART,LFXT: SLASEC4D Table 5-3, p. 35), so give it
        // three times that
        match config.try_freeze(&mut fram, 3000) {
            Ok((smclk, aclk, _xt1clk, delay)) => (smclk, aclk, delay, true),
            // The crystal didn't start: carry on with REFO, to report it
            Err(config) => {
                let (smclk, aclk, delay) = config.xt1clk_off().freeze(&mut fram);
                (smclk, aclk, delay, false)
            }
        }
    } else {
        let (smclk, aclk, delay) = ClockConfig::new(periph.cs)
            .mclk_dcoclk_hz(TARGET_HZ, MclkDiv::_1)
            .smclk_on(SmclkDiv::_1)
            .aclk_refoclk()
            .freeze(&mut fram);
        (smclk, aclk, delay, true)
    };
    let mclk_hz = smclk.freq();
    let aclk_hz = aclk.freq();

    // The UART runs from ACLK, so the report stays readable even if MCLK is wrong
    // P4.3 = UCA1TXD with P4SEL = 01 (SLASEC4D Table 6-66, p. 102), the backchannel UART (SLAU680 2.2.4,
    // p. 11)
    // (8N1, LSB first: UCMSB, UC7BIT, UCSPB, UCPEN in SLAU445I Table 22-8, p. 593; ACLK is UCSSEL = 01b:
    // SLASEC4D Table 6-9, p. 68)
    let mut tx = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_aclk(&aclk)
    .tx_only(p4.pin3.to_alternate1());

    // TB0 counts ACLK, and toggles its CCR0 output every 32 cycles (Toggle mode: "The output period is
    // double the timer period", SLAU445I Table 14-4, p. 401). TB1 counts SMCLK (= MCLK) and captures the
    // rising edges of that output on CCR0 (input B: SLASEC4D Table 6-17, p. 74, CCI0B = "Timer0_B3 CCR0B
    // output (internal)"), so two captures are 64 ACLK cycles apart. CCR1 captures from software, to time
    // the delays.
    // (TBSSEL: ACLK = 01b, SMCLK = 10b: SLASEC4D Table 6-9, p. 68. CCR0 captures on rising edges of
    // input B: CM = 01b, CCIS = 01b, SLAU445I Table 14-8, p. 411. A software capture switches CCIS
    // between GND and VCC: SLAU445I 14.2.4.1.1, p. 399.)
    let _tb0 = PwmParts3::new(periph.tb0, TimerConfig::aclk(&aclk), 31);
    let captures = CaptureParts3::config(periph.tb1, TimerConfig::smclk(&smclk))
        .config_cap0_input_B()
        .config_cap0_trigger(CapTrigger::RisingEdge)
        .config_cap1_software()
        .commit();
    let mut aclk_edges = captures.cap0;
    let mut stopwatch = captures.cap1;

    let mut pass = true;

    print(&mut tx, "\r\n--- dco_delay_test ---\r\nTarget ");
    print_num(&mut tx, TARGET_HZ, 0);
    print(&mut tx, " Hz, FLL reference ");
    print(&mut tx, if FLL_REF_FROM_XT1 { "XT1 (crystal)\r\n" } else { "REFO\r\n" });
    if !xt1_ok {
        print(&mut tx, "XT1 didn't start: FAIL\r\n");
        pass = false;
    }

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
    // typically 200 ms (SLASEC4D Table 5-5, p. 37: tFLL,lock, a typical value, no maximum)
    delay.delay_ms(200);
    // MCLK cycles in 8 x 64 ACLK cycles. Each capture is read long before the next one arrives.
    let mut previous = read_capture(&mut aclk_edges);
    let mut mclk_cycles = 0u32;
    for _ in 0..8 {
        let now = read_capture(&mut aclk_edges);
        mclk_cycles += now.wrapping_sub(previous) as u32;
        previous = now;
    }
    let expected = multiplier * 8 * 64;
    // The FLL keeps MCLK within about 1 % of its target (SLASEC4D Table 5-5, p. 37: fDCO,FLL at 24 MHz is
    // within ±1.0 % at 25°C with REFO as the reference, ±2.0 % over temperature, and ±0.5 % with an XT1
    // crystal as the reference)
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
    let cs = unsafe { &*msp430fr2355::Cs::ptr() };
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
        print_num(&mut tx, (cycles as u64 * 10_000_000 / mclk_hz as u64) as u32, 1);
        print(&mut tx, " us, ");
        print_num(&mut tx, cycles, 0);
        print(&mut tx, " cycles");
        print(&mut tx, pass_fail(ok));
        pass &= ok;
    }

    print(&mut tx, if pass { "ALL PASS\r\n" } else { "FAILED\r\n" });
    print(&mut tx, "Scope: MCLK on P3.0, square wave on P1.6, high and low for ");
    print_num(&mut tx, PULSE_US, 0);
    print(&mut tx, " us each\r\n");
    if pass {
        led2.set_high().ok();
    } else {
        led1.set_high().ok();
    }

    loop {
        square_wave.set_high().ok();
        delay.delay_us(PULSE_US);
        square_wave.set_low().ok();
        delay.delay_us(PULSE_US);
    }
}

/// The SMCLK cycles from just before `f` runs to just after it returns, from two software captures
/// (SLAU445I 14.2.4.1.1, p. 399)
fn time(stopwatch: &mut Capture<Tb1, CCR1>, f: impl FnOnce()) -> u16 {
    stopwatch.trigger_capture();
    let start = read_capture(stopwatch);
    f();
    stopwatch.trigger_capture();
    read_capture(stopwatch).wrapping_sub(start)
}

/// The next captured count. After a missed capture (COV: SLAU445I 14.2.4.1, p. 398) this is the
/// latest one.
fn read_capture<C>(capture: &mut Capture<Tb1, C>) -> u16
where
    Tb1: msp430_hal::timer::CapCmp<C>,
{
    match block!(capture.capture()) {
        Ok(count) | Err(OverCapture(count)) => count,
    }
}

/// MCLK cycles in `ns` nanoseconds, rounded up
fn ns_to_cycles(ns: u32, mclk_hz: u32) -> u32 {
    (ns as u64 * mclk_hz as u64).div_ceil(1_000_000_000) as u32
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
