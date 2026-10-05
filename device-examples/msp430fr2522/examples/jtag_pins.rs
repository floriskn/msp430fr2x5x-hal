//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Dedicating the JTAG pins: from a press of a button on P2.2 until the next BOR, P1.4 to P1.7 are JTAG
//! pins only (TCK, TMS, TDI and TDO), whatever their port registers say. An LED on P1.0 shows the level of
//! a jumper wire from P1.6 to P1.1. P1.6, a GPIO output, drives the wire low, so the LED is off. After the
//! press, P1.6 is the JTAG input TDI, which doesn't drive the wire, so the pullup of P1.1 pulls it high
//! and the LED lights.
//! (SYSJTAGPIN: "Setting this bit disables the shared digital functionality of the JTAG pins and
//! permanently enables the JTAG function. This bit can only be set once. After the bit is set, it remains
//! set until a BOR occurs": SLAU445I Table 1-13, p. 66. In JTAG mode P1DIR and P1SELx don't matter:
//! SLASEE4C Table 6-15, p. 58. TDI is an input: SLASEE4C Table 6-6, p. 47. P1.0, P1.1 and P2.2 are GPIO
//! in both packages: SLASEE4C Table 4-2, p. 14; SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16, p. 60.
//! No board document covers the LED or the button: there is none for the MSP430FR25x2.)
//!
//! After the press a Spy-Bi-Wire debugger, on TEST and RST, may not connect until a BOR: SYSJTAGPIN = 1
//! means "explicit 4-wire JTAG mode selection" (SLAU445I Table 1-13, p. 66). Pull RST low, a BOR
//! (SLAU445I 1.2, p. 30), before flashing anything else: the program only dedicates the pins on a press,
//! so after the BOR the debugger can connect. If flashing still fails, switch the power off and on.
//!
//! How to test (an LED, a resistor, a jumper wire, and a button or a second jumper wire):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, a jumper wire from P1.6 to
//!    P1.1, and a button from P2.2 to GND (the internal pullup is on); a wire from P2.2 that you touch to
//!    GND works as the button too.
//! 2. Flash this example by Spy-Bi-Wire, not 4-wire JTAG, which uses P1.4 to P1.7 (SLASEE4C Table 6-6,
//!    p. 47). Expected: the LED is off. (On means the wire isn't connected.)
//! 3. Press the button. Expected: the LED lights.
//! 4. Pull RST low for a moment, with a wire to GND. Expected: the LED is off again: the BOR has cleared
//!    SYSJTAGPIN, and P1.6 drives the wire low again.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{gpio::Batch, pmm::Pmm, sys::SysParts, watchdog::Wdt};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // P1.1 reads the wire, with its pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313;
    // SLASEE4C Table 6-15, p. 58)
    let p1 = Batch::new(periph.p1)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    // The button pulls P2.2 low, with the internal pullup on
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0.to_output_low();
    let mut wire = p1.pin1;
    let mut button = p2.pin2;
    // P1.6 drives the wire low as a GPIO output, P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let tdi = p1.pin6.to_output_low();

    // Until the button is pressed, the LED shows the wire's level
    while button.is_high().unwrap() {
        led.set_state(wire.is_high().unwrap().into()).ok();
    }

    // SYSJTAGPIN = 1 (SLAU445I Table 1-13, p. 66). It takes the four pins, which can't be used for
    // anything else afterwards.
    SysParts::new(periph.sfr).jtag_pins.dedicate(p1.pin4, p1.pin5, tdi, p1.pin7);

    loop {
        led.set_state(wire.is_high().unwrap().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
