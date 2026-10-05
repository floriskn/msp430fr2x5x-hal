//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 fault detection and the fail-safe: when the signal on XIN stops, ACLK switches from XT1 to REFO and
//! LED1 turns on. When the signal returns, the example clears the fault and ACLK goes back to XT1.
//!
//! The generator runs at 20 kHz instead of 32.768 kHz, so ACLK on P2.2 shows which clock it runs from: 20 kHz
//! from XT1, about 32.8 kHz from REFO. The fault flags are sticky, and the fail-safe stays engaged until
//! software clears them, which the main loop does all the time, except while S1 is held.
//! (Fail-safe: SLAU445I 3.2.13, p. 109 to p. 110. REFO runs at 32.768 kHz ±3.5 %: SLASE59F Table 5-7, p. 25.
//! LED1 on P1.0 is red, and S1 pulls P2.3 low: SLAU739 Figure 18, p. 23.)
//!
//! How to test (function generator and the scope):
//! 1. Generator: square wave, 20 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 2. Connect it to XIN, P2.1 (J2 pin 12), its ground to GND (J2 pin 20), and switch the output on.
//! 3. Flash this example, and put the scope on ACLK, P2.2 (J2 pin 18), ground clip on GND (J3 pin 22).
//!    Expected: ACLK is 20 kHz, and LED1 is off.
//! 4. Switch the generator output off: LED1 turns on, and ACLK jumps to about 32.8 kHz.
//! 5. Switch it back on: LED1 turns off almost at once, and ACLK is back at 20 kHz. Bypass mode leaves the
//!    start counter off; with `.enable_start_counter()` the fault would only clear after 1024 clean cycles,
//!    51 ms at 20 kHz (start-up counter: SLASE59F Table 5-4 note 7, p. 23).
//! 6. Hold S1 and repeat steps 4 and 5: ACLK stays on REFO and LED1 stays on, even with the signal back.
//!    Release S1: LED1 turns off, and ACLK returns to 20 kHz.
//! 7. Optional: lower the frequency step by step (10, 5, 3.5, 2, 1 kHz) to find where the fault detector
//!    trips. The data sheet only promises that frequencies above 3.5 kHz don't set the fault flag
//!    (fFault,LFXT: SLASE59F Table 5-4, p. 23). Or vary the duty cycle: bypass mode is specified for 40 % to
//!    60 % (DCXT1,SW: SLASE59F Table 5-4, p. 23).
//! (Header pins: SLAU739 Figure 18, p. 23.)
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

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    // S1 on P2.3 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313; SLAU739 Figure 18, p. 23)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut button = p2.pin3;

    // ACLK on P2.2 with P2SELx = 10 and P2DIR = 1, XIN on P2.1 with P2SELx = 01 (SLASE59F Table 6-18,
    // p. 56)
    let _aclk_out = p2.pin2.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
        .freeze(&mut fram);

    loop {
        // While S1 is held the flags are left alone, so the latched fault keeps ACLK on REFO
        // (SLAU445I 3.2.13, p. 109 to p. 110. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I
        // Table 1-10, p. 63.)
        let s1_held = button.is_low().unwrap_or(false);
        if !s1_held {
            xt1clk.clear_fault();
        }
        led.set_state(xt1clk.is_faulted().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
