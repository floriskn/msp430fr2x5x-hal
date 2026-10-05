//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Captures started from software: the timer's count is recorded when a button on P2.3 is pressed and when
//! it's released, and eUSCI_A0 prints how long the button was held.
//!
//! A capture/compare register set up with `config_capN_software()` captures when software switches its
//! input between GND and VCC (`trigger_capture()`), at the timer's next clock edge, so the time comes
//! from the timer rather than from when the CPU reads it.
//! (SLAU445I 13.2.4.1.1, p. 376: "Capture Initiated by Software". P2.3 is a GPIO input with its pullup:
//! SLASEE4C Table 6-16, p. 60; SLAU445I Table 8-1, p. 313. P2.3 only exists on the 20-pin RHL package:
//! SLASEE4C Table 4-2, p. 14. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document covers the
//! parts to connect: there is none for the MSP430FR25x2.)
//!
//! TA0 counts ACLK, from REFO at 32.768 kHz, so it counts for 2 s before it wraps around: hold the button
//! for less than 2 s. (REFO: SLASEE4C Table 5-7, p. 27.)
//!
//! How to test (a push button and a 3.3-V USB-to-UART adapter):
//! 1. Connect a push button from P2.3 to GND (the internal pullup is on). Connect the adapter: its RX to
//!    P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud.
//! 2. Flash this example. Expected: `Press and hold the button`.
//! 3. Press and hold the button, then release it: `Button held for 734 ms`, for example. A short tap gives
//!    a few tens of milliseconds.
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
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// ACLK cycles per second: REFO's 32.768 kHz (SLASEE4C Table 5-7, p. 27)
const ACLK_HZ: u32 = 32_768;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The button pulls P2.3 low, against the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut button = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
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

    // TA0 counts ACLK (TASSEL = 01b: SLAU445I Table 13-4, p. 384) in continuous mode. CCR1's input starts
    // at GND (CCIS = 10b) and captures on both edges (CM = 11b), so each switch between GND and VCC is a
    // capture (SLAU445I 13.2.4.1.1, p. 376; SLAU445I Table 13-6, p. 386).
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::aclk(&aclk))
        .config_cap1_software()
        .commit();
    let mut capture = captures.cap1;

    print(&mut tx, "\r\nPress and hold the button\r\n");
    loop {
        while button.is_high().unwrap() {}
        capture.trigger_capture();
        let pressed = block!(capture.capture()).unwrap_or_else(|over| over.0);

        // Let the bouncing stop before looking for the release
        for _ in 0..10_000 {
            msp430::asm::nop();
        }
        while button.is_low().unwrap() {}
        capture.trigger_capture();
        let released = block!(capture.capture()).unwrap_or_else(|over| over.0);

        let ticks = released.wrapping_sub(pressed) as u32;
        print(&mut tx, "Button held for ");
        print_num(&mut tx, ticks * 1000 / ACLK_HZ);
        print(&mut tx, " ms\r\n");

        for _ in 0..10_000 {
            msp430::asm::nop();
        }
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
