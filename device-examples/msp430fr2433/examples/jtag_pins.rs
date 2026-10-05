//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Dedicating the JTAG pins: from a press of S1 until the next BOR, P1.4 to P1.7 are JTAG pins only (TCK,
//! TMS, TDI and TDO), whatever their port registers say. LED1 shows the level of a jumper wire from P1.6
//! to P2.4. P1.6, a GPIO output, drives the wire low, so LED1 is off. Once S1 is pressed, P1.6 is the
//! JTAG input TDI, which doesn't drive the wire, so the pullup of P2.4 pulls it high and LED1 lights.
//! (SYSJTAGPIN: "Setting this bit disables the shared digital functionality of the JTAG pins and
//! permanently enables the JTAG function. This bit can only be set once. After the bit is set, it remains
//! set until a BOR occurs": SLAU445I Table 1-13, p. 66. In JTAG mode P1DIR and P1SELx don't matter:
//! SLASE59F Table 6-17, p. 55. TDI is an input: SLASE59F Table 6-5, p. 43. LED1 on P1.0 is red, S1 is
//! P2.3, and S3 pulls RST low: SLAU739 Figure 18, p. 23.)
//!
//! After S1 the LaunchPad's debugger may not connect until a BOR: it uses Spy-Bi-Wire on TEST and RST
//! (SLAU739 2.2.3, p. 8), while SYSJTAGPIN = 1 means "explicit 4-wire JTAG mode selection" (SLAU445I
//! Table 1-13, p. 66). Press S3, a BOR (SLAU445I 1.2, p. 30), before flashing anything else: the program
//! only dedicates the pins on S1, so after the BOR the debugger can connect. If flashing still fails,
//! unplug the USB cable and plug it back in. Until the BOR the backchannel UART can't work either, as it
//! is on P1.4 and P1.5 (SLAU739 Figure 18, p. 23).
//!
//! How to test (a jumper wire):
//! 1. Connect P1.6 (J1 pin 5) to P2.4 (J1 pin 7) with a jumper wire. (Header pins: SLAU739 Figure 18,
//!    p. 23.)
//! 2. Flash this example. Expected: LED1 is off. (On means the wire isn't connected.)
//! 3. Press S1. Expected: LED1 lights.
//! 4. Press S3. Expected: LED1 is off again: the BOR has cleared SYSJTAGPIN, and P1.6 drives the wire low
//!    again.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{gpio::Batch, pmm::Pmm, sys::SysParts, watchdog::Wdt};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // P2.4 reads the wire, with its pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313;
    // SLASE59F Table 6-19, p. 58). S1 pulls P2.3 low; the board has no pullup on it, so the internal one is
    // on (SLAU739 Figure 18, p. 23).
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .config_pin4(|p| p.pullup())
        .split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut wire = p2.pin4;
    let mut s1 = p2.pin3;
    // P1.6 drives the wire low as a GPIO output, P1SELx = 00 and P1DIR = 1 (SLASE59F Table 6-17, p. 55).
    // P1.4 and P1.5, the backchannel UART's pins, stay inputs.
    let tdi = p1.pin6.to_output_low();

    // Until S1 is pressed, LED1 shows the wire's level
    while s1.is_high().unwrap() {
        led1.set_state(wire.is_high().unwrap().into()).ok();
    }

    // SYSJTAGPIN = 1 (SLAU445I Table 1-13, p. 66). It takes the four pins, which can't be used for
    // anything else afterwards.
    SysParts::new(periph.sfr).jtag_pins.dedicate(p1.pin4, p1.pin5, tdi, p1.pin7);

    loop {
        led1.set_state(wire.is_high().unwrap().into()).ok();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
