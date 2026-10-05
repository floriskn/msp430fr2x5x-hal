//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in bypass mode as the CPU clock: MCLK and SMCLK run from the generator's 32.768 kHz on XIN, and LED2
//! blinks green. When the signal stops, the fail-safe moves them to REFO: the CPU keeps running, and LED1
//! reports the fault in red.
//!
//! The blink comes from `SysDelay`, a delay loop timed for 32.768 kHz, so it follows MCLK: half the
//! frequency, half the speed. At each blink the main loop clears the sticky fault flag, which moves MCLK back
//! to XT1 once the signal is back. Bypass mode leaves the start counter off, so that happens as soon as the
//! signal returns.
//! (Fail-safe for MCLK and SMCLK: SLAU445I 3.2.13, p. 109. Clearing the flags switches the clocks back once
//! no fault remains: SLAU445I 3.2.13, p. 110. Start counter, ENSTFCNT1: SLAU445I Table 3-11, p. 121. REFO
//! runs at 32.768 kHz: SLASEC4D Table 5-7, p. 40. LED1 on P1.0 is red, LED2 on P6.6 green: SLAU680
//! Figure 18, p. 26.)
//!
//! This test needs a board change: on the MSP-EXP430FR2355, XIN, P2.7, isn't on the BoosterPack headers but
//! goes only to the 32.768-kHz crystal Q1 and its capacitor C3 (SLAU680 Figure 18, p. 26). Desolder Q1, and
//! solder a wire to the pad of its pin 1, on the XIN side, for the generator. Solder Q1 back afterwards, for
//! the examples that use the crystal. (Bypass mode: SLAU445I 3.2.4, p. 103. Any pin may see –0.3 V to
//! VCC + 0.3 V at most: SLASEC4D 5.1, p. 27, and the LaunchPad's VCC is 3.3 V: SLAU680 2.3.1, p. 12.)
//!
//! How to test (a board change, the function generator, and optionally the scope):
//! 1. Make the board change above.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to the wire on XIN, its ground to GND (J2 pin 20), and switch the output on. Do this before
//!    flashing: `freeze()` waits for XT1.
//! 4. Flash this example. Expected: LED2 blinks, roughly 0.5 s on and 0.5 s off (a delay loop is coarse at
//!    32 kHz). The scope on MCLK, P3.0 (J2 pin 11), ground clip on GND (J3 pin 22), counts 32.768 kHz, and so
//!    does SMCLK on P3.4 (J1 pin 8).
//! 5. Set the generator to 16.384 kHz: MCLK follows, and LED2 blinks exactly half as fast.
//! 6. Switch the generator output off: LED2 keeps blinking, at the first speed again, because the CPU now
//!    runs from REFO, and LED1 lights red.
//! 7. Switch the output back on (still 16.384 kHz): at the next blink LED1 turns off, and LED2 slows down
//!    again.
//! (Header pins: SLAU680 Figure 10, p. 15.)
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
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    let p6 = Batch::new(periph.p6)
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p6.pin6;

    // MCLK on P3.0 and SMCLK on P3.4 with P3SEL = 01 and P3DIR = 1 (SLASEC4D Table 6-65, p. 100);
    // XIN on P2.7 with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let _mclk_out = p3.pin0.to_output().to_alternate1();
    let _smclk_out = p3.pin4.to_output().to_alternate1();
    let xin = p2.pin7.to_alternate2();

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
