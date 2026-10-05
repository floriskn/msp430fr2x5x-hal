//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A reset when the DCO runs too fast for the FLL (`ClockConfig::reset_on_fll_unlock`). Pressing S2 halves
//! the FLL's multiplier, as halving the FLL's reference frequency would: the DCO then runs twice as fast as
//! the FLL wants, which resets the device.
//! (The PUC comes when FLLULPUC = 1 and FLLUNLOCK = 10b, too fast: SLAU445I Figure 3-4, p. 106. The reset
//! cause: FLL unlock, SYSRSTIV 24h: SLASEC4D Table 6-12, p. 70.)
//!
//! The FLL locks MCLK to 244 × 32.768 kHz = 7.995 MHz, from REFO, and S2 writes FLLN while the FLL runs.
//! The HAL has no function for that, as it only serves to make the DCO run too fast, so the example writes
//! CSCTL2 itself. A DCO that runs too slow only unlocks the FLL, without a reset; `fll_unlock_interrupt.rs`
//! shows both.
//! (fDCOCLKDIV = (FLLN + 1) × fFLLREFCLK: SLAU445I 3.2.5, p. 104. FLLN: SLAU445I Table 3-6, p. 115.)
//!
//! LED2 (green) is on while the program runs, and LED1 (red) shows that the last reset was an FLL unlock
//! reset. (LED1 is P1.0, LED2 is P6.6, and S2 connects P2.3 to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example. LED2 lights green.
//! 2. Press S2. The board resets and starts over: LED1 lights red.
//! 3. Press the reset button S3: LED1 turns off, as this reset came from the RST pin.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::{Pmm, ResetCause},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The FLL multiplier, FLLN + 1, of the 8 MHz setting: 244 × 32.768 kHz = 7.995 MHz (SLAU445I 3.2.5,
/// p. 104)
const MULTIPLIER: u16 = 244;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next reset (reading
    // SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let cause = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

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
    led1.set_state((cause == Some(ResetCause::FllUnlock)).into()).ok();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM, DIVS:
    // SLAU445I Table 3-9, p. 118), the FLL locked to REFO (SELREF = 01b: SLAU445I Table 3-7, p. 116), and
    // the FLL unlock reset (FLLULPUC: SLAU445I Table 3-11, p. 121)
    let (_smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .reset_on_fll_unlock()
        .freeze(&mut fram);

    led2.set_high().ok();

    // S2 pulls P2.3 low, so a press is a high-to-low transition (PxIES = 1: SLAU445I Table 8-16, p. 336).
    // The flag waits for a new press, so S2 still held after the reset doesn't count again.
    s2.select_falling_edge_trigger();
    loop {
        if s2.wait_for_ifg().is_ok() {
            // Half the multiplier: the DCO, still at 7.995 MHz, is now twice as fast as the FLL's new target
            // of 3.998 MHz (FLLN: SLAU445I Table 3-6, p. 115)
            let cs = unsafe { &*msp430fr2355::Cs::ptr() };
            cs.csctl2().modify(|_, w| w.flln().set(MULTIPLIER / 2 - 1));
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
