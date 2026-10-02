//! The RST/NMI pin as an interrupt input: the reset button S3 toggles LED1, which is green, instead
//! of resetting the device (SLAU802 Figure 19, p. 25).
//!
//! The pin's interrupt is the user NMI, which is non-maskable: it can't share data with the rest of
//! the program through a critical section, so this example uses an atomic flag from `msp430-atomic`.
//! (NMIs "are not masked by the general interrupt enable (GIE) bit", and an edge on the RST/NMI pin
//! in NMI mode is a user NMI source: SLAU445I 1.3.1, p. 33. The NMI function: SLAU445I 1.7, p. 43.)
//!
//! The pin stays an NMI input until the next reset (SYSNMI is cleared by a PUC: SLAU445I Figure 1-20,
//! p. 64, with the key in SLAU445I Table 0-1, p. 28), and S3 no longer causes one, so to get the
//! reset button back, unplug the USB cable or flash another program.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    gpio::Batch,
    pmm::Pmm,
    sys::{self, NmiEdge, RstPull, SysParts},
    watchdog::Wdt,
};
use msp430_atomic::AtomicBool;
use msp430fr247x::interrupt;
use panic_msp430 as _;

/// Set by the interrupt handler for each press of S3
static PRESSED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;

    // S3 pulls the pin low, so a press is a falling edge
    // (S3 connects RST to GND, R11 pulls it up: SLAU802 Figure 19, p. 25)
    let sys = SysParts::new(periph.sfr);
    let mut rst = sys.rst_nmi_pin.into_nmi(NmiEdge::Falling, RstPull::Up);
    rst.enable_interrupts();

    loop {
        if PRESSED.load() {
            PRESSED.store(false);
            led1.toggle().ok();
        }
    }
}

#[interrupt]
fn UNMI() {
    if sys::take_nmi_pin_interrupt() {
        PRESSED.store(true);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
