//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A reset when the DCO runs too fast for the FLL (`ClockConfig::reset_on_fll_unlock`). XT1 in bypass
//! mode, from a function generator, is the FLL reference. Lowering the generator's frequency suddenly
//! leaves the DCO faster than the FLL can pull it down, which resets the device.
//! (The PUC comes when FLLULPUC = 1 and FLLUNLOCK = 10b, too fast: SLAU445I Figure 3-4, p. 106. The
//! reset cause: FLL unlock, SYSRSTIV 24h: SLASEE4C Table 6-10, p. 52.)
//!
//! An LED on P1.0 is on while the program runs, and an LED on P2.2 shows that the last reset was an FLL
//! unlock reset. MCLK is on P1.3 and MCLK / 8 on P1.2, for the scope.
//! (XIN is P2.1: SLASEE4C Table 6-16, p. 60. SMCLK and MCLK are P1.2 and P1.3: SLASEE4C Table 6-15, p. 58.
//! A low level on RST/NMI resets the device: SLAU445I 1.2, p. 30. No board document covers the LEDs: there
//! is none for the MSP430FR25x2.)
//!
//! How to test (function generator and scope, two LEDs and resistors):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND, and another from P2.2 to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 32.768 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), 50 % duty, output load
//!    High-Z. Check the levels on the scope first, then connect it to XIN, P2.1, with its ground on GND.
//!    See `xt1_bypass_aclk.rs`.
//! 3. Flash this example and switch the generator output on. The LED on P1.0 is on, and the scope on P1.2
//!    shows 999.4 kHz: MCLK is 244 × 32.768 kHz = 7.995 MHz.
//! 4. Change the generator to 20 kHz. The device resets: the LED on P2.2 turns on, and P1.2 shows 610 kHz,
//!    as the FLL locks MCLK to 244 × 20 kHz = 4.88 MHz after the restart.
//! 5. Change the generator back to 32.768 kHz. No reset this time: the DCO is now too slow, which only
//!    sets FLLUNLOCK. Reset the device, with RST/NMI low for a moment, to start again at 7.995 MHz; the LED
//!    on P2.2 turns off.
//!
//! Don't set the generator above 32.768 kHz: MCLK would follow it above 8 MHz, the most the FRAM runs at
//! without wait states (SLASEE4C 5.3, p. 17).
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::{Pmm, ResetCause},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The frequency the generator is set to at the start
const XT1_FREQ_HZ: u32 = 32_768;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next reset (reading
    // SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let cause = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // The LEDs on P1.0 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let mut led_running = p1.pin0.to_output_low();
    let mut led_fll_reset = p2.pin2.to_output_low();
    led_fll_reset.set_state((cause == Some(ResetCause::FllUnlock)).into()).ok();

    // SMCLK on P1.2 and MCLK on P1.3, each with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let _smclk_out = p1.pin2.to_output().to_alternate2();
    let _mclk_out = p1.pin3.to_output().to_alternate2();
    // XIN is P2.1 with P2SELx = 10 (SLASEE4C Table 6-16, p. 60). In bypass mode XOUT stays a GPIO
    // (SLAU445I 3.2.4, p. 103).
    let xin = p2.pin1.to_alternate2();

    // MCLK from DCOCLKDIV (SELMS = 000b: SLAU445I Table 3-8, p. 117), the FLL locked to XT1CLK (SELREF:
    // SLAU445I 3.2.5, p. 104), SMCLK = MCLK / 8 (DIVS = 11b: SLAU445I Table 3-9, p. 118), and the FLL
    // unlock reset (FLLULPUC: SLAU445I Table 3-11, p. 121)
    let (_smclk, _aclk, _xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .fll_ref_xt1()
        .aclk_refoclk()
        .reset_on_fll_unlock()
        .freeze(&mut fram);

    led_running.set_high().ok();

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
