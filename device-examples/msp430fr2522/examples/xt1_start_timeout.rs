//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! `try_freeze` with a timeout: if XT1 hasn't started within 1 s, the example falls back to the internal
//! oscillators. An LED on P2.2 lights when XT1 started, and an LED on P1.0 when the example gave up on it.
//!
//! ACLK comes out on P1.1: from XT1 it runs at the generator's 20 kHz, after the fallback from REFO at
//! 32.768 kHz, so the two are easy to tell apart.
//! (While XT1 has no signal, its fault flag XT1OFFG keeps being set again: SLAU445I 3.2.13, p. 109. REFO runs
//! at 32.768 kHz ±3.5 %: SLASEE4C Table 5-7, p. 27. XIN is P2.1: SLASEE4C Table 6-16, p. 60. ACLK is P1.1:
//! SLASEE4C Table 6-15, p. 58. A low level on RST/NMI resets the device: SLAU445I 1.2, p. 30. No board
//! document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, two LEDs and resistors, and optionally the scope):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and another from P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 20 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on.
//! 4. Flash this example. Expected: the LED on P2.2 lights. The scope on ACLK, P1.1, ground clip on GND,
//!    counts 20 kHz.
//! 5. Switch the generator output off, and reset the device, with RST/NMI low for a moment. Expected: about
//!    1 s later the LED on P1.0 turns on (the LED on P2.2 stays off), and ACLK counts about 32.8 kHz, from
//!    REFO.
//! 6. With the output off, reset the device and switch the output on within half a second: the LED on P2.2
//!    lights instead, since XT1 started before the timeout.
//! 7. To time the timeout: probes on RST/NMI and on the LED on P1.0, single-shot trigger on RST/NMI's rising
//!    edge, 200 ms/div. With the output off, pull RST/NMI low and release it: P1.0 rises about 1 s after
//!    RST/NMI. Take the probe off RST/NMI before flashing again: it is also the Spy-Bi-Wire data line,
//!    SBWTDIO (SLASEE4C Table 6-7, p. 48).
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 20_000;
/// How long to wait for XT1 before falling back to REFO
const XT1_TIMEOUT_MS: u16 = 1000;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // The LEDs on P1.0 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.to_output())
        .split(&pmm);
    let mut led_gave_up = p1.pin0;
    let mut led_started = p2.pin2;
    led_gave_up.set_low().ok();
    led_started.set_low().ok();

    // ACLK on P1.1 with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58), XIN on P2.1 with
    // P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let clocks = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk();

    match clocks.try_freeze(&mut fram, XT1_TIMEOUT_MS) {
        Ok((_smclk, _aclk, _xt1clk, _delay)) => {
            led_started.set_high().ok();
        }
        Err(clocks) => {
            // XT1 did not start: run everything that was sourced from XT1 from REFO instead
            // (XT1OFFG kept being set again while the fault lasted: SLAU445I 3.2.13, p. 109. ACLK from
            // REFO is SELA = 01b: SLAU445I Table 3-8, p. 117.)
            let (_smclk, _aclk, _delay) = clocks.xt1clk_off().freeze(&mut fram);
            led_gave_up.set_high().ok();
        }
    }

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
