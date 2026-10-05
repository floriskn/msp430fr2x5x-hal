//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The XT1 fault as an interrupt: when the signal on XIN stops, the oscillator fault requests the user NMI,
//! and LED1 turns on. LED2 blinks meanwhile, to show that the main loop runs.
//!
//! `xt1_fault_failsafe.rs` polls `Xt1clk::is_faulted`; the interrupt also arrives while the program is busy
//! or asleep. The user NMI is non-maskable, so it can't share data with the program through a critical
//! section: the handler sets an atomic flag from `msp430-atomic`. The handler also disables the interrupt,
//! and the main loop enables it again once XT1 is back, ready for the next fault.
//! (An oscillator fault is a user NMI source, and NMIs are not masked by GIE: SLAU445I 1.3.1, p. 33.
//! SYSUNIV 04h, OFIFG: SLASE59F Table 6-9, p. 48. The fail-safe switches ACLK to REFO: SLAU445I 3.2.13,
//! p. 109. LED1 on P1.0 is red and LED2 on P1.1 green: SLAU739 Figure 18, p. 23.)
//!
//! How to test (function generator, and optionally the scope):
//! 1. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 2. Connect it to XIN, P2.1 (J2 pin 12), its ground to GND (J2 pin 20), and switch the output on. Do this
//!    before flashing: `freeze()` waits for XT1.
//! 3. Flash this example. Expected: LED2 toggles every 0.5 s, and LED1 is off. The scope on ACLK, P2.2
//!    (J2 pin 18), ground clip on GND (J3 pin 22), counts the generator's 32.768 kHz.
//! 4. Switch the generator output off: within half a second LED1 lights. LED2 keeps toggling, and ACLK runs
//!    from REFO, at about 32.8 kHz.
//! 5. Switch the output back on: at the next toggle of LED2, LED1 turns off, and ACLK follows the generator
//!    again.
//! 6. Repeat steps 4 and 5: every fault lights LED1 again.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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
use msp430fr2433::interrupt;
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;

/// Set by the interrupt handler when XT1 fails, cleared by the main loop once XT1 is back
static XT1_FAULT: AtomicBool = AtomicBool::new(false);

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

    // ACLK on P2.2 with P2SELx = 10 and P2DIR = 1, XIN on P2.1 with P2SELx = 01 (SLASE59F Table 6-18,
    // p. 56)
    let _aclk_out = p2.pin2.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate1();

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
        led2.toggle().ok();
        delay.delay_ms(500);

        if XT1_FAULT.load() {
            led1.set_high().ok();
            // The fault can only be cleared once XT1 runs again
            // (Cleared while the fault remains, the bits "are automatically set again":
            // SLAU445I 3.2.13, p. 109. XT1OFFG: SLAU445I Table 3-11, p. 122; OFIFG: SLAU445I Table 1-10,
            // p. 63.)
            xt1clk.clear_fault();
            if !xt1clk.is_faulted() {
                led1.set_low().ok();
                XT1_FAULT.store(false);
                xt1clk.enable_fault_interrupt();
            }
        }
    }
}

// The user NMI vector: the NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASE59F Table 6-2,
// p. 41)
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
