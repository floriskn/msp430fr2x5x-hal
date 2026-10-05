//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The FLL unlock interrupt: when the FLL loses its lock, an interrupt reports it, and the FLL's unlock
//! history tells whether the DCO ran too slow or too fast. (`fll_unlock_reset.rs` resets the device
//! instead.) XT1 in bypass mode, from a function generator, is the FLL reference: changing the generator's
//! frequency unlocks the FLL until it has pulled the DCO to the new frequency.
//! (FLLWARNEN: "If FLLUNLOCKHIS is not equal to 00, an OFIFG is generated", SLAU445I Table 3-11, p. 121.
//! OFIFG requests the user NMI with OFIE set: SLAU445I 3.2.13, p. 109; SYSUNIV 04h, OFIFG: SLASEE4C
//! Table 6-10, p. 52.)
//!
//! The interrupt is the user NMI, which is non-maskable: it can't share data with the rest of the
//! program through a critical section, so this example uses an atomic flag from `msp430-atomic`.
//!
//! An LED on P1.0 is on while the program runs. After an unlock, an LED lights for half a second: the one
//! on P1.7 if the DCO was too slow, the one on P2.2 if it was too fast, both if it was both.
//! (XIN is P2.1: SLASEE4C Table 6-16, p. 60. No board document covers the LEDs: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (function generator, three LEDs and resistors):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from each of
//!    P1.0, P1.7 and P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), 50 % duty, output load
//!    High-Z. Check the levels on the scope first, then connect it to XIN, P2.1, with its ground on GND.
//!    See `xt1_bypass_aclk.rs`.
//! 3. Switch the generator output on, then flash this example. The LED on P1.0 turns on.
//! 4. Change the generator to 30 kHz. The LED on P2.2 flashes: the FLL now aims for 244 × 30 kHz =
//!    7.32 MHz, and the DCO, still at 7.995 MHz, was too fast until the FLL pulled it down.
//! 5. Change the generator back to 32.768 kHz. The LED on P1.7 flashes: the DCO was too slow.
//!
//! Don't set the generator above 32.768 kHz: MCLK would follow it above 8 MHz, the most the FRAM runs at
//! without wait states (SLASEE4C 5.3, p. 17). Keep the generator on: switching it off is an XT1 fault,
//! which requests the same interrupt (see `xt1_fault_interrupt.rs`).
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{self, ClockConfig, DcoclkFreqSel, FllStatus, FllUnlockHistory, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use msp430_atomic::AtomicBool;
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// The frequency the generator is set to at the start
const XT1_FREQ_HZ: u32 = 32_768;

/// Set by the interrupt handler after an unlock, cleared by the main loop
static UNLOCKED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // The LEDs on P1.0, P1.7 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let mut led_running = p1.pin0.to_output_low();
    let mut led_too_slow = p1.pin7.to_output_low();
    let mut led_too_fast = p2.pin2.to_output_low();

    // XIN is P2.1 with P2SELx = 10 (SLASEE4C Table 6-16, p. 60). In bypass mode XOUT stays a GPIO
    // (SLAU445I 3.2.4, p. 103).
    let xin = p2.pin1.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b: SLAU445I Table 3-8, p. 117), with the FLL locked to XT1CLK
    // (SELREF: SLAU445I 3.2.5, p. 104)
    let (_smclk, _aclk, _xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .fll_ref_xt1()
        .aclk_refoclk()
        .freeze(&mut fram);

    // The FLL reports the DCO as too slow for a few ms after `freeze()` while it settles, so wait before
    // enabling the interrupt (see `clock::fll_unlock_history`)
    delay.delay_ms(20);
    clock::enable_fll_unlock_interrupt();
    led_running.set_high().ok();

    loop {
        if UNLOCKED.load() {
            UNLOCKED.store(false);
            // The history stays until the interrupt is enabled again (FLLUNLOCKHIS: SLAU445I Table 3-11,
            // p. 121)
            let history = clock::fll_unlock_history();
            let too_slow = matches!(history, FllUnlockHistory::TooSlow | FllUnlockHistory::TooSlowAndFast);
            let too_fast = matches!(history, FllUnlockHistory::TooFast | FllUnlockHistory::TooSlowAndFast);
            led_too_slow.set_state(too_slow.into()).ok();
            led_too_fast.set_state(too_fast.into()).ok();
            delay.delay_ms(500);
            led_too_slow.set_low().ok();
            led_too_fast.set_low().ok();

            // Enable the interrupt again once the FLL has locked (FLLUNLOCK: SLAU445I Table 3-11, p. 121).
            // This clears the history.
            while clock::fll_status() != FllStatus::Locked {}
            clock::enable_fll_unlock_interrupt();
        }
    }
}

// The user NMI vector: the NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEE4C Table 6-2,
// p. 46)
#[interrupt]
fn UNMI() {
    // This also disables the interrupt, which would otherwise be requested again straight away, as the
    // history keeps OFIFG set until it's cleared ("When the interrupt is granted, the OFIE is not reset
    // automatically": SLAU445I 3.2.13, p. 109)
    if clock::take_fault_interrupt() {
        UNLOCKED.store(true);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
