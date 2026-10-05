//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The XT1 fault as an interrupt: when the crystal stops, the oscillator fault requests the user NMI, and
//! LED1 lights red. LED2 toggles green meanwhile, to show that the main loop runs.
//!
//! The LaunchPad's 32.768-kHz crystal can't be stopped from outside, so holding S1 stops it: the main loop
//! then makes XIN a general-purpose I/O, which disables XT1, as a broken crystal would stop (see
//! `set_crystal_running` below). `xt1_fault_failsafe.rs` polls `Xt1clk::is_faulted`; the interrupt also
//! arrives while the program is busy or asleep. The user NMI is non-maskable, so it can't share data with
//! the program through a critical section: the handler sets an atomic flag from `msp430-atomic`. The handler
//! also disables the interrupt, and the main loop enables it again once XT1 is back, ready for the next
//! fault.
//! (An oscillator fault is a user NMI source, and NMIs are not masked by GIE: SLAU445I 1.3.1, p. 33.
//! SYSUNIV 04h, OFIFG: SLASEC4D Table 6-12, p. 70. The fail-safe switches ACLK to REFO: SLAU445I 3.2.13,
//! p. 109. The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, LED1 on P1.0 is red, LED2 on P6.6 green, and S1
//! connects P4.1 to GND: SLAU680 Figure 18, p. 26.)
//!
//! How to test (optionally the scope):
//! 1. Flash this example. Expected, once the crystal has started, about a second after reset (1000 ms
//!    typical: SLASEC4D Table 5-3, p. 35): LED2 toggles every 0.5 s, and LED1 is off. The scope on ACLK,
//!    P1.1 (J3 pin 28), ground clip on GND (J3 pin 22), counts the crystal's 32.768 kHz.
//! 2. Hold S1: within a second LED1 lights red. LED2 keeps toggling, and ACLK runs from REFO, whose
//!    frequency can differ by up to 3.5 % (SLASEC4D Table 5-7, p. 40).
//! 3. Release S1: about a second later, once the crystal has started again, LED1 turns off at a toggle of
//!    LED2, and ACLK runs from the crystal again.
//! 4. Repeat steps 2 and 3: every fault lights LED1 again.
//! (Header pins: SLAU680 Figure 10, p. 15.)
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
use msp430fr2355::interrupt;
use panic_msp430 as _;

/// Frequency of the LaunchPad's crystal Q1 (SLAU680 2.5, p. 13)
const XT1_FREQ_HZ: u32 = 32_768;

/// Set by the interrupt handler when XT1 fails, cleared by the main loop once XT1 is back
static XT1_FAULT: AtomicBool = AtomicBool::new(false);

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
    // S1 on P4.1 with the internal pullup, as the board has none (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313)
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let p6 = Batch::new(periph.p6)
        .config_pin6(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p6.pin6;
    let mut s1 = p4.pin1;

    // ACLK on P1.1 with P1SEL = 10 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96); XIN on P2.7 and XOUT on
    // P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117); XT1 in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120)
    let (_smclk, _aclk, mut xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .xt1clk_on(Xt1Config::crystal(XT1_FREQ_HZ, xin, xout))
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_xt1clk()
        .freeze(&mut fram);

    // OFIE enables the oscillator fault interrupt (SLAU445I Table 1-9, p. 62)
    xt1clk.enable_fault_interrupt();

    loop {
        led2.toggle().ok();
        delay.delay_ms(500);
        // While S1 is held the crystal stands still
        set_crystal_running(s1.is_high().unwrap_or(true));

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

/// Stop the crystal, as a broken one would stop, or let it start again: this test's stand-in for an XT1
/// fault, which the HAL has no function for. With the P2SEL1 bit of XIN, P2.7, cleared, "both XT1IN and
/// XT1OUT ports are configured as general-purpose I/Os, and XT1 is disabled"; setting it configures them
/// "for XT1 operation" again (SLAU445I 3.2.4, p. 103; XIN is P2SELx = 10: SLASEC4D Table 6-64, p. 98).
/// Meanwhile XIN is a GPIO output, held low, so that it passes no stray edges on to XT1CLK: the workaround
/// for erratum RTC15 toggles XIN as a GPIO output to clock XT1CLK (SLAZ695J RTC15, p. 11). PxDIR and PxOUT:
/// SLAU445I Table 8-1, p. 313.
fn set_crystal_running(running: bool) {
    const XIN: u8 = 1 << 7;
    let p2 = unsafe { &*msp430fr2355::P2::ptr() };
    if running {
        p2.p2dir().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2sel1().modify(|r, w| unsafe { w.bits(r.bits() | XIN) });
    } else {
        p2.p2sel1().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2out().modify(|r, w| unsafe { w.bits(r.bits() & !XIN) });
        p2.p2dir().modify(|r, w| unsafe { w.bits(r.bits() | XIN) });
    }
}

// The user NMI vector: the NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEC4D Table 6-2,
// p. 63)
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
