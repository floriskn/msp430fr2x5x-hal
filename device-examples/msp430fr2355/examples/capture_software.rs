//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Captures started from software: the timer's count is recorded when button S1 is pressed and when
//! it's released, and the backchannel UART prints how long S1 was held.
//!
//! A capture/compare register set up with `config_capN_software()` captures when software switches its
//! input between GND and VCC (`trigger_capture()`), at the timer's next clock edge, so the time comes
//! from the timer rather than from when the CPU reads it.
//! (SLAU445I 14.2.4.1.1, p. 399: "Capture Initiated by Software". S1 is P4.1: SLAU680 Figure 18, p. 26.)
//!
//! TB0 counts ACLK, from REFO at 32.768 kHz, so it counts for 2 s before it wraps around: hold S1 for less
//! than 2 s. (REFO: SLASEC4D Table 5-7, p. 40.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on (SLAU680 Table 2, p. 10).
//! 2. Open the COM port of "MSP Application UART1" at 9600 baud in a serial terminal such as PuTTY
//!    (SLAU680 2.2.4, p. 11).
//! 3. Press and hold S1, then release it: `S1 held for 734 ms`, for example. A short tap gives a few
//!    tens of milliseconds.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    capture::{CaptureParts3, CapturePin, TimerConfig},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// ACLK cycles per second: REFO's 32.768 kHz (SLASEC4D Table 5-7, p. 40)
const ACLK_HZ: u32 = 32_768;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // S1 pulls P4.1 low, with the internal pullup on (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313), as the board has none (SLAU680 Figure 18, p. 26)
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let mut s1 = p4.pin1;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
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

    // TB0 counts ACLK (TBSSEL = 01b: SLAU445I Table 14-6, p. 409) in continuous mode. CCR1's input starts
    // at GND (CCIS = 10b) and captures on both edges (CM = 11b), so each switch between GND and VCC is a
    // capture (SLAU445I 14.2.4.1.1, p. 399; SLAU445I Table 14-8, p. 411).
    let captures = CaptureParts3::config(periph.tb0, TimerConfig::aclk(&aclk))
        .config_cap1_software()
        .commit();
    let mut capture = captures.cap1;

    writeln!(tx, "\r\nPress and hold S1\r").ok();
    loop {
        while s1.is_high().unwrap() {}
        capture.trigger_capture();
        let pressed = block!(capture.capture()).unwrap_or_else(|over| over.0);

        // Let the bouncing stop before looking for the release
        for _ in 0..10_000 {
            msp430::asm::nop();
        }
        while s1.is_low().unwrap() {}
        capture.trigger_capture();
        let released = block!(capture.capture()).unwrap_or_else(|over| over.0);

        let ticks = released.wrapping_sub(pressed) as u32;
        writeln!(tx, "S1 held for {} ms\r", ticks * 1000 / ACLK_HZ).ok();

        for _ in 0..10_000 {
            msp430::asm::nop();
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
