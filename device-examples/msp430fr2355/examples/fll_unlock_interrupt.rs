//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The FLL unlock interrupt: when the FLL loses its lock, an interrupt reports it, and the FLL's unlock
//! history tells whether the DCO ran too slow or too fast. (`fll_unlock_reset.rs` resets the device
//! instead.) Each press of S2 switches the FLL's multiplier between 244 and 223, as a step of its reference
//! frequency would, which unlocks the FLL until it has pulled the DCO to the new frequency.
//! (FLLWARNEN: "If FLLUNLOCKHIS is not equal to 00, an OFIFG is generated", SLAU445I Table 3-11, p. 121.
//! OFIFG requests the user NMI with OFIE set: SLAU445I 3.2.13, p. 109; SYSUNIV 04h, OFIFG: SLASEC4D
//! Table 6-12, p. 70.)
//!
//! The FLL locks MCLK to 244 × 32.768 kHz = 7.995 MHz, from REFO, or after a press to 223 × 32.768 kHz =
//! 7.307 MHz. That's 8.6 % lower, about 90 of the DCO's 512 taps, which are about 0.1 % apart, and the
//! software trim sets the tap near the middle, so the FLL can follow without running out of taps. The HAL
//! has no function for changing FLLN while the FLL runs, as that only serves to unlock it, so the example
//! writes CSCTL2 itself.
//! (fDCOCLKDIV = (FLLN + 1) × fFLLREFCLK: SLAU445I 3.2.5, p. 104. FLLN: SLAU445I Table 3-6, p. 115. The
//! FLL moves the DCO tap, each about 0.1 % above the one before: SLAU445I 3.2.6, p. 104. The tap near the
//! middle: SLAU445I 3.2.11.2, p. 107.)
//!
//! The interrupt is the user NMI, which is non-maskable: it can't share data with the rest of the
//! program through a critical section, so this example uses an atomic flag from `msp430-atomic`.
//!
//! After an unlock, LED1 lights red for about half a second if the DCO was too fast, LED2 green if it was
//! too slow, both if it was both. (LED1 is P1.0, LED2 is P6.6, and S2 connects P2.3 to GND: SLAU680
//! Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example. Both LEDs are off.
//! 2. Press S2. LED1 flashes red: the FLL now aims for 7.307 MHz, and the DCO, still at 7.995 MHz, was too
//!    fast until the FLL pulled it down.
//! 3. Press S2 again. LED2 flashes green: back at 244 × 32.768 kHz, the DCO was too slow.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{self, ClockConfig, DcoclkFreqSel, FllStatus, FllUnlockHistory, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use msp430_atomic::AtomicBool;
use msp430fr2355::interrupt;
use panic_msp430 as _;

/// The FLL multiplier, FLLN + 1, of the 8 MHz setting: 244 × 32.768 kHz = 7.995 MHz (SLAU445I 3.2.5,
/// p. 104)
const MULTIPLIER: u16 = 244;
/// The multiplier S2 switches to: 223 × 32.768 kHz = 7.307 MHz
const OTHER_MULTIPLIER: u16 = 223;

/// Set by the interrupt handler after an unlock, cleared by the main loop
static UNLOCKED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // S2 on P2.3 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let p6 = Batch::new(periph.p6).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p6.pin6.to_output_low();
    let mut s2 = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range (SELMS = 000b: SLAU445I Table 3-8, p. 117), with the FLL
    // locked to REFO (SELREF = 01b: SLAU445I Table 3-7, p. 116)
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The FLL reports the DCO as too slow for a few ms after `freeze()` while it settles, so wait before
    // enabling the interrupt (see `clock::fll_unlock_history`)
    delay.delay_ms(20);
    clock::enable_fll_unlock_interrupt();

    // S2 pulls P2.3 low, so a press is a high-to-low transition (PxIES = 1: SLAU445I Table 8-16, p. 336)
    s2.select_falling_edge_trigger();
    let cs = unsafe { &*msp430fr2355::Cs::ptr() };
    let mut multiplier = MULTIPLIER;

    loop {
        if s2.wait_for_ifg().is_ok() {
            // A new multiplier while the FLL runs (FLLN: SLAU445I Table 3-6, p. 115)
            multiplier = if multiplier == MULTIPLIER { OTHER_MULTIPLIER } else { MULTIPLIER };
            cs.csctl2().modify(|_, w| w.flln().set(multiplier - 1));
            // Wait for the button's release and its bouncing to end, then forget the edges it caused
            while s2.is_low().unwrap_or(false) {}
            delay.delay_ms(100);
            s2.clear_ifg();
        }

        if UNLOCKED.load() {
            UNLOCKED.store(false);
            // The history stays until the interrupt is enabled again (FLLUNLOCKHIS: SLAU445I Table 3-11,
            // p. 121)
            let history = clock::fll_unlock_history();
            let too_slow = matches!(history, FllUnlockHistory::TooSlow | FllUnlockHistory::TooSlowAndFast);
            let too_fast = matches!(history, FllUnlockHistory::TooFast | FllUnlockHistory::TooSlowAndFast);
            led1.set_state(too_fast.into()).ok();
            led2.set_state(too_slow.into()).ok();
            delay.delay_ms(500);
            led1.set_low().ok();
            led2.set_low().ok();

            // Enable the interrupt again once the FLL has locked (FLLUNLOCK: SLAU445I Table 3-11, p. 121).
            // This clears the history.
            while clock::fll_status() != FllStatus::Locked {}
            clock::enable_fll_unlock_interrupt();
        }
    }
}

// The user NMI vector: the NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEC4D Table 6-2,
// p. 63)
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
