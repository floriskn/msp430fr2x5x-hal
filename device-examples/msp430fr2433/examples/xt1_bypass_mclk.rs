//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in bypass mode as the CPU clock: MCLK and SMCLK run from the generator's 32.768 kHz on XIN, and LED2
//! blinks. When the signal stops, the fail-safe moves them to REFO: the CPU keeps running, and LED1 reports
//! the fault.
//!
//! The blink comes from `SysDelay`, a delay loop timed for 32.768 kHz, so it follows MCLK: half the
//! frequency, half the speed. At each blink the main loop clears the sticky fault flag, which moves MCLK back
//! to XT1 once the signal is back. Bypass mode leaves the start counter off, so that happens as soon as the
//! signal returns.
//! (Fail-safe for MCLK and SMCLK: SLAU445I 3.2.13, p. 109. Clearing the flags switches the clocks back once
//! no fault remains: SLAU445I 3.2.13, p. 110. Start counter, ENSTFCNT1: SLAU445I Table 3-11, p. 121. REFO
//! runs at 32.768 kHz: SLASE59F Table 5-7, p. 25. LED1 on P1.0 is red and LED2 on P1.1 green: SLAU739
//! Figure 18, p. 23.)
//!
//! How to test (function generator, and optionally the scope):
//! 1. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 2. Connect it to XIN, P2.1 (J2 pin 12), its ground to GND (J2 pin 20), and switch the output on. Do this
//!    before flashing: `freeze()` waits for XT1.
//! 3. Flash this example. Expected: LED2 blinks, roughly 0.5 s on and 0.5 s off (a delay loop is coarse at
//!    32 kHz). The scope on MCLK, P1.3 (J1 pin 9), ground clip on GND (J3 pin 22), counts 32.768 kHz, and so
//!    does SMCLK on P1.7 (J1 pin 6).
//! 4. Set the generator to 16.384 kHz: MCLK follows, and LED2 blinks exactly half as fast.
//! 5. Switch the generator output off: LED2 keeps blinking, at the first speed again, because the CPU now
//!    runs from REFO, and LED1 lights.
//! 6. Switch the output back on (still 16.384 kHz): at the next blink LED1 turns off, and LED2 slows down
//!    again.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p1.pin1;

    // MCLK on P1.3 and SMCLK on P1.7 with P1SELx = 10 and P1DIR = 1 (SLASE59F Table 6-17, p. 55);
    // XIN on P2.1 with P2SELx = 01 (SLASE59F Table 6-18, p. 56)
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    let _smclk_out = p1.pin7.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate1();

    // XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120) sources MCLK and SMCLK
    // (SELMS = 010b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, _aclk, mut xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .mclk_xt1clk(MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .freeze(&mut fram);

    loop {
        led2.toggle().ok();
        delay.delay_ms(500);
        // The fault flag is sticky and the fail-safe stays engaged until it is cleared, so
        // clear it and see whether it comes straight back
        // (The fault bits "remain set until software resets them": SLAU445I 3.2.13, p. 109. XT1OFFG:
        // SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10, p. 63.)
        xt1clk.clear_fault();
        led1.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
