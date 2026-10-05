//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The RST/NMI pin, in reset mode and in NMI mode. In reset mode a button on RST/NMI resets the device, and
//! the next start lights an LED on P1.1, because `Pmm::take_reset_cause()` reports the RST pin. A button on
//! P1.2 switches the pin to NMI mode, where the RST/NMI button toggles an LED on P1.0 instead of resetting
//! the device, and back. In NMI mode a button on P1.3 switches the edge that requests the interrupt, so the
//! LED on P1.0 toggles when the RST/NMI button is released instead of pressed.
//!
//! In reset mode the program selects the pin's pullup and the filter that suppresses short pulses, both
//! as after a reset; without an external resistor, the internal pullup keeps the pin high. `into_nmi()` and
//! `into_reset()` switch the mode, and `set_edge()` the edge, which requests the user NMI.
//! (The pin's functions, its resistor and its filter: SLAU445I 1.7, p. 43; SYSFLTE, SYSRSTRE, SYSRSTUP,
//! SYSNMIIES and SYSNMI: SLAU445I Table 1-11, p. 64. A low level in reset mode causes a BOR: SLAU445I 1.2,
//! p. 30, and a BOR sets the I/O pins to inputs: SLAU445I 1.2.1, p. 32. RST/NMI, SYSRSTIV 04h, and
//! brownout, 02h: SLASEE4C Table 6-10, p. 52. The user NMI: SLAU445I 1.3.1, p. 33. RST/NMI is pin 4, also
//! SBWTDIO: SLASEE4C Table 4-1, p. 11. P1.0 to P1.3 are GPIO with P1SELx = 00: SLASEE4C Table 6-15, p. 58.
//! No board document covers the LEDs or the buttons: there is none for the MSP430FR25x2.)
//!
//! How to test (two LEDs, two resistors, and three buttons or jumper wires):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, another from P1.1 to GND, and a
//!    button from each of RST/NMI, P1.2 and P1.3 to GND (the internal pullups are on). A wire that you touch
//!    to GND works as a button too.
//! 2. Flash this example, then press the RST/NMI button. Expected: the LED on P1.1 lights.
//! 3. Hold the RST/NMI button. Expected: the LED on P1.1 is off while it is held, as the device is held in
//!    reset, and lights again when you let go.
//! 4. Press the P1.2 button (NMI mode), then press the RST/NMI button a few times. Expected: the LED on P1.0
//!    toggles at each press, and the LED on P1.1 stays lit: the device doesn't reset.
//! 5. Press the P1.3 button, then press the RST/NMI button and hold it for a moment each time. Expected: the
//!    LED on P1.0 toggles when you let go. Press the P1.3 button again to have it toggle at the press.
//! 6. Press the P1.2 button (reset mode), then hold the RST/NMI button. Expected: as in step 3, and the LED
//!    on P1.0 is off.
//! 7. Switch the power off and on again. Expected: the LED on P1.1 stays off: the first reset cause is now
//!    the power-up, a brownout reset, not the RST pin.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::digital::*;
use msp430_atomic::AtomicBool;
use msp430_rt::entry;
use msp430_hal::{
    gpio::Batch,
    pmm::{Pmm, ResetCause},
    sys::{self, NmiEdge, RstPull, SysParts},
    watchdog::Wdt,
};
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// Set by the interrupt handler for each edge on the RST/NMI pin in NMI mode. The user NMI is
/// non-maskable, so it can't share data through a critical section, and this uses an atomic flag from
/// `msp430-atomic`. (NMIs "are not masked by the general interrupt enable (GIE) bit": SLAU445I 1.3.1, p. 33)
static EDGE: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next start (reading SYSRSTIV
    // clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    // The LEDs on P1.0 and P1.1 are outputs, and the buttons pull P1.2 and P1.3 low against their internal
    // pullups (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .config_pin2(|p| p.pullup())
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p1.pin1;
    let mut mode_button = p1.pin2;
    let mut edge_button = p1.pin3;
    led1.set_low().ok();
    led2.set_state((first == Some(ResetCause::ResetPin)).into()).ok();

    let mut rst = SysParts::new(periph.sfr).rst_nmi_pin;
    loop {
        // Reset mode, with the pullup (SYSRSTRE = 1, SYSRSTUP = 1) and the filter (SYSFLTE = 1), as after a
        // reset (SLAU445I Table 1-11, p. 64)
        rst.set_pull(RstPull::Up);
        rst.set_filter(true);
        while !pressed(&mut mode_button) {}

        // NMI mode (SYSNMI = 1), on the falling edge to begin with (SYSNMIIES = 1), as the button pulls the
        // pin low, and with the pin's interrupt enabled (NMIIE: SLAU445I Table 1-9, p. 62)
        let mut edge = NmiEdge::Falling;
        let mut nmi = rst.into_nmi(edge, RstPull::Up);
        nmi.enable_interrupts();
        while !pressed(&mut mode_button) {
            if EDGE.load() {
                EDGE.store(false);
                led1.toggle().ok();
            }
            if pressed(&mut edge_button) {
                // The other edge: the rising one is SYSNMIIES = 0 (SLAU445I Table 1-11, p. 64)
                edge = if edge == NmiEdge::Falling { NmiEdge::Rising } else { NmiEdge::Falling };
                nmi.set_edge(edge);
            }
        }

        // Back to reset mode (SYSNMI = 0: SLAU445I Table 1-11, p. 64)
        rst = nmi.into_reset(RstPull::Up);
        led1.set_low().ok();
    }
}

/// Whether `button` is pressed. If it is, wait until it is released, and a little longer for the bouncing
/// to stop.
fn pressed(button: &mut impl InputPin) -> bool {
    if button.is_high().unwrap() {
        return false;
    }
    while button.is_low().unwrap() {}
    for _ in 0..10_000 {
        msp430::asm::nop();
    }
    true
}

// The user NMI vector: the RST/NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEE4C Table 6-2,
// p. 46; NMIIFG: SLAU445I Table 1-10, p. 63)
#[interrupt]
fn UNMI() {
    if sys::take_nmi_pin_interrupt() {
        EDGE.store(true);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
