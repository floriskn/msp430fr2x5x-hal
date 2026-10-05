//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 fault detection and the fail-safe: when the signal on XIN stops, ACLK switches from XT1 to REFO and
//! an LED on P1.0 turns on. When the signal returns, the example clears the fault and ACLK goes back to XT1.
//!
//! The generator runs at 20 kHz instead of 32.768 kHz, so ACLK on P1.1 shows which clock it runs from: 20 kHz
//! from XT1, about 32.8 kHz from REFO. The fault flags are sticky, and the fail-safe stays engaged until
//! software clears them, which the main loop does all the time, except while a button on P1.5 is held.
//! (Fail-safe: SLAU445I 3.2.13, p. 109 to p. 110. REFO runs at 32.768 kHz ±3.5 %: SLASEE4C Table 5-7, p. 27.
//! XIN is P2.1: SLASEE4C Table 6-16, p. 60. ACLK is P1.1: SLASEE4C Table 6-15, p. 58. No board document
//! covers the LED or the button: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator and the scope, an LED and a resistor, and a button or a jumper wire):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and a button from P1.5 to GND (the internal pull-up is on). A wire from P1.5 that you hold on
//!    GND works as the button too. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 20 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on.
//! 4. Flash this example, and put the scope on ACLK, P1.1, ground clip on GND. Expected: ACLK is 20 kHz, and
//!    the LED is off.
//! 5. Switch the generator output off: the LED turns on, and ACLK jumps to about 32.8 kHz.
//! 6. Switch it back on: the LED turns off almost at once, and ACLK is back at 20 kHz. Bypass mode leaves the
//!    start counter off; with `.enable_start_counter()` the fault would only clear after 1024 clean cycles,
//!    51 ms at 20 kHz (start-up counter: SLASEE4C Table 5-4 note 8, p. 25).
//! 7. Hold the button and repeat steps 5 and 6: ACLK stays on REFO and the LED stays on, even with the
//!    signal back. Release the button: the LED turns off, and ACLK returns to 20 kHz.
//! 8. Optional: lower the frequency step by step (10, 5, 3.5, 2, 1 kHz) to find where the fault detector
//!    trips. The data sheet only promises that frequencies above 3.5 kHz don't set the fault flag
//!    (fFault,LFXT: SLASEE4C Table 5-4, p. 25). Or vary the duty cycle: bypass mode is specified for 40 % to
//!    60 % (DCXT1,SW: SLASEE4C Table 5-4, p. 25).
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
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // The LED on P1.0, a GPIO output, and the button on P1.5, an input with its pullup (P1SELx = 00 and
    // P1DIR = 1 or 0: SLASEE4C Table 6-15, p. 58; P1REN = 1, P1OUT = 1: SLAU445I Table 8-1, p. 313)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin5(|p| p.pullup())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let mut led = p1.pin0;
    let mut button = p1.pin5;

    // ACLK on P1.1 with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58), XIN on P2.1 with
    // P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, mut xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
        .freeze(&mut fram);

    loop {
        // While the button is held the flags are left alone, so the latched fault keeps ACLK on REFO
        // (SLAU445I 3.2.13, p. 109 to p. 110. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I
        // Table 1-10, p. 63.)
        let button_held = button.is_low().unwrap_or(false);
        if !button_held {
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
