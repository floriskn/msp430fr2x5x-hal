//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The XT1 fault as an interrupt: when the signal on XIN stops, the oscillator fault requests the user NMI,
//! and an LED on P2.2 turns on. An LED on P1.0 blinks meanwhile, to show that the main loop runs.
//!
//! `xt1_fault_failsafe.rs` polls `Xt1clk::is_faulted`; the interrupt also arrives while the program is busy
//! or asleep. The user NMI is non-maskable, so it can't share data with the program through a critical
//! section: the handler sets an atomic flag from `msp430-atomic`. The handler also disables the interrupt,
//! and the main loop enables it again once XT1 is back, ready for the next fault.
//! (An oscillator fault is a user NMI source, and NMIs are not masked by GIE: SLAU445I 1.3.1, p. 33.
//! SYSUNIV 04h, OFIFG: SLASEE4C Table 6-10, p. 52. The fail-safe switches ACLK to REFO: SLAU445I 3.2.13,
//! p. 109. XIN is P2.1: SLASEE4C Table 6-16, p. 60. ACLK is P1.1: SLASEE4C Table 6-15, p. 58. No board
//! document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (function generator, two LEDs and resistors, and optionally the scope):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and another from P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on. Do this before flashing:
//!    `freeze()` waits for XT1.
//! 4. Flash this example. Expected: the LED on P1.0 toggles every 0.5 s, and the LED on P2.2 is off. The
//!    scope on ACLK, P1.1, ground clip on GND, counts the generator's 32.768 kHz.
//! 5. Switch the generator output off: within half a second the LED on P2.2 lights. The LED on P1.0 keeps
//!    toggling, and ACLK runs from REFO, at about 32.8 kHz.
//! 6. Switch the output back on: at the next toggle of the LED on P1.0, the LED on P2.2 turns off, and ACLK
//!    follows the generator again.
//! 7. Repeat steps 5 and 6: every fault lights the LED on P2.2 again.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{self, ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use msp430_atomic::AtomicBool;
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;

/// Set by the interrupt handler when XT1 fails, cleared by the main loop once XT1 is back
static XT1_FAULT: AtomicBool = AtomicBool::new(false);

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

    // ACLK on P1.1 with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58), XIN on P2.1 with
    // P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, mut xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_xt1clk()
        .freeze(&mut fram);

    // OFIE enables the oscillator fault interrupt (SLAU445I Table 1-9, p. 62)
    xt1clk.enable_fault_interrupt();

    loop {
        led_blink.toggle().ok();
        delay.delay_ms(500);

        if XT1_FAULT.load() {
            led_fault.set_high().ok();
            // The fault can only be cleared once XT1 runs again
            // (Cleared while the fault remains, the bits "are automatically set again":
            // SLAU445I 3.2.13, p. 109. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10,
            // p. 63.)
            xt1clk.clear_fault();
            if !xt1clk.is_faulted() {
                led_fault.set_low().ok();
                XT1_FAULT.store(false);
                xt1clk.enable_fault_interrupt();
            }
        }
    }
}

// The user NMI vector: the NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEE4C Table 6-2,
// p. 46)
#[interrupt]
fn UNMI() {
    // This also disables the fault interrupt, which would otherwise be requested again straight
    // away for as long as the fault lasts
    // ("as long as a fault condition still exists, the OFIFG remains set": SLAU445I 3.2.13, p. 110;
    // it clears OFIE: SLAU445I Table 1-9, p. 62)
    if clock::take_fault_interrupt() {
        XT1_FAULT.store(true);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
