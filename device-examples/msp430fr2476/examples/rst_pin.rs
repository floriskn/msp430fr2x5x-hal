//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The RST/NMI pin, in reset mode and in NMI mode. In reset mode the reset button S3 resets the device, and
//! the next start lights LED2 blue, because `Pmm::take_reset_cause()` reports the RST pin. S1 switches the
//! pin to NMI mode, where S3 toggles LED1 instead of resetting the device, and back. In NMI mode S2
//! switches the edge that requests the interrupt, so LED1 toggles when S3 is released instead of pressed.
//!
//! In reset mode the program selects the pin's pullup and the filter that suppresses short pulses, both
//! as after a reset, so S3 works as usual; the board's own pullup, R11, keeps the pin high in any case.
//! `into_nmi()` and `into_reset()` switch the mode, and `set_edge()` the edge, which requests the user NMI.
//! (The pin's functions, its resistor and its filter: SLAU445I 1.7, p. 43; SYSFLTE, SYSRSTRE, SYSRSTUP,
//! SYSNMIIES and SYSNMI: SLAU445I Table 1-11, p. 64. A low level in reset mode causes a BOR: SLAU445I 1.2,
//! p. 30, and a BOR sets the I/O pins to inputs: SLAU445I 1.2.1, p. 32. RST/NMI, SYSRSTIV 04h, and
//! brownout, 02h: SLASEO7C Table 9-10, p. 52. The user NMI: SLAU445I 1.3.1, p. 33. S3 pulls RST to GND
//! against the 47-kΩ pullup R11; LED1 on P1.0 is green, the blue part of LED2 is P4.7, S1 is P4.0 and S2
//! is P2.3: SLAU802 Figure 19, p. 25.)
//!
//! How to test:
//! 1. Flash this example, then press S3. Expected: LED2 lights blue.
//! 2. Hold S3. Expected: LED2 is off while S3 is held, as the device is held in reset, and lights blue again
//!    when you let go.
//! 3. Press S1 (NMI mode), then press S3 a few times. Expected: LED1 toggles at each press, and LED2 stays
//!    lit: the device doesn't reset.
//! 4. Press S2, then press S3 and hold it for a moment each time. Expected: LED1 toggles when you let go of
//!    S3. Press S2 again to have it toggle at the press.
//! 5. Press S1 (reset mode), then hold S3. Expected: as in step 2, and LED1 is off.
//! 6. Unplug the USB cable and plug it back in. Expected: LED2 stays off: the first reset cause is now the
//!    power-up, a brownout reset, not the RST pin.
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
use msp430fr247x::interrupt;
use panic_msp430 as _;

/// Set by the interrupt handler for each edge on the RST/NMI pin in NMI mode. The user NMI is
/// non-maskable, so it can't share data through a critical section, and this uses an atomic flag from
/// `msp430-atomic`. (NMIs "are not masked by the general interrupt enable (GIE) bit": SLAU445I 1.3.1, p. 33)
static EDGE: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next start (reading SYSRSTIV
    // clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    // S1 and S2 inputs with their internal pullups (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1,
    // p. 313), on top of R9 and R10 (SLAU802 Figure 19, p. 25)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.pullup())
        .config_pin7(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut s1 = p4.pin0;
    let mut s2 = p2.pin3;
    let mut blue = p4.pin7;
    led1.set_low().ok();
    blue.set_state((first == Some(ResetCause::ResetPin)).into()).ok();

    let mut rst = SysParts::new(periph.sfr).rst_nmi_pin;
    loop {
        // Reset mode, with the pullup (SYSRSTRE = 1, SYSRSTUP = 1) and the filter (SYSFLTE = 1), as after a
        // reset (SLAU445I Table 1-11, p. 64)
        rst.set_pull(RstPull::Up);
        rst.set_filter(true);
        while !pressed(&mut s1) {}

        // NMI mode (SYSNMI = 1), on the falling edge to begin with (SYSNMIIES = 1), as S3 pulls the pin low,
        // and with the pin's interrupt enabled (NMIIE: SLAU445I Table 1-9, p. 62)
        let mut edge = NmiEdge::Falling;
        let mut nmi = rst.into_nmi(edge, RstPull::Up);
        nmi.enable_interrupts();
        while !pressed(&mut s1) {
            if EDGE.load() {
                EDGE.store(false);
                led1.toggle().ok();
            }
            if pressed(&mut s2) {
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

// The user NMI vector: the RST/NMI pin (NMIIFG) and oscillator faults (OFIFG) (FFFAh: SLASEO7C Table 9-2,
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
