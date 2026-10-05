//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in bypass mode as the CPU clock: MCLK and SMCLK run from the generator's 32.768 kHz on XIN, and an LED
//! on P1.0 blinks. When the signal stops, the fail-safe moves them to REFO: the CPU keeps running, and an LED
//! on P2.2 reports the fault.
//!
//! The blink comes from `SysDelay`, a delay loop timed for 32.768 kHz, so it follows MCLK: half the
//! frequency, half the speed. At each blink the main loop clears the sticky fault flag, which moves MCLK back
//! to XT1 once the signal is back. Bypass mode leaves the start counter off, so that happens as soon as the
//! signal returns.
//! (Fail-safe for MCLK and SMCLK: SLAU445I 3.2.13, p. 109. Clearing the flags switches the clocks back once
//! no fault remains: SLAU445I 3.2.13, p. 110. Start counter, ENSTFCNT1: SLAU445I Table 3-11, p. 121. REFO
//! runs at 32.768 kHz: SLASEE4C Table 5-7, p. 27. XIN is P2.1: SLASEE4C Table 6-16, p. 60. SMCLK and MCLK
//! are P1.2 and P1.3: SLASEE4C Table 6-15, p. 58. No board document covers the LEDs: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (function generator, two LEDs and resistors, and optionally the scope):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and another from P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on. Do this before flashing:
//!    `freeze()` waits for XT1.
//! 4. Flash this example. Expected: the LED on P1.0 blinks, roughly 0.5 s on and 0.5 s off (a delay loop is
//!    coarse at 32 kHz). The scope on MCLK, P1.3, ground clip on GND, counts 32.768 kHz, and so does SMCLK
//!    on P1.2.
//! 5. Set the generator to 16.384 kHz: MCLK follows, and the LED on P1.0 blinks exactly half as fast.
//! 6. Switch the generator output off: the LED on P1.0 keeps blinking, at the first speed again, because the
//!    CPU now runs from REFO, and the LED on P2.2 lights.
//! 7. Switch the output back on (still 16.384 kHz): at the next blink the LED on P2.2 turns off, and the LED
//!    on P1.0 slows down again.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;

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
    let mut led_blink = p1.pin0;
    let mut led_fault = p2.pin2;

    // SMCLK on P1.2 and MCLK on P1.3 with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58);
    // XIN on P2.1 with P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _smclk_out = p1.pin2.to_output().to_alternate2();
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120) sources MCLK and SMCLK
    // (SELMS = 010b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, _aclk, mut xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .mclk_xt1clk(MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .freeze(&mut fram);

    loop {
        led_blink.toggle().ok();
        delay.delay_ms(500);
        // The fault flag is sticky and the fail-safe stays engaged until it is cleared, so
        // clear it and see whether it comes straight back
        // (The fault bits "remain set until software resets them": SLAU445I 3.2.13, p. 109. XT1OFFG:
        // SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10, p. 63.)
        xt1clk.clear_fault();
        led_fault.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
