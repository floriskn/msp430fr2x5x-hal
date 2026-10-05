//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Dedicating the JTAG pins: from a press of S1 until the next BOR, P1.4 to P1.7 are JTAG pins only (TCK,
//! TMS, TDI and TDO), whatever their port registers say. LED1 shows the level of a jumper wire from P1.6
//! to P3.6. P1.6, a GPIO output, drives the wire low, so LED1 is off. Once S1 is pressed, P1.6 is the
//! JTAG input TDI, which doesn't drive the wire, so the pullup of P3.6 pulls it high and LED1 lights.
//! (SYSJTAGPIN: "Setting this bit disables the shared digital functionality of the JTAG pins and
//! permanently enables the JTAG function. This bit can only be set once. After the bit is set, it remains
//! set until a BOR occurs": SLAU445I Table 1-13, p. 66. In JTAG mode P1DIR and P1SELx don't matter:
//! SLASEC4D Table 6-63, p. 96. TDI is an input: SLASEC4D Table 6-7, p. 66. LED1 on P1.0 is red, S1 is
//! P4.1, and S3 pulls RST low: SLAU680 Figure 18, p. 26.)
//!
//! After S1 the LaunchPad's debugger may not connect until a BOR: it uses Spy-Bi-Wire on TEST and RST
//! (SLAU680 2.2.3, p. 10), while SYSJTAGPIN = 1 means "explicit 4-wire JTAG mode selection" (SLAU445I
//! Table 1-13, p. 66). Press S3, a BOR (SLAU445I 1.2, p. 30), before flashing anything else: the program
//! only dedicates the pins on S1, so after the BOR the debugger can connect. If flashing still fails,
//! unplug the USB cable and plug it back in. Nothing else on this LaunchPad uses P1.4 to P1.7: they go to
//! the headers, and P1.4 also to the Grove connector J12 (SLAU680 Figure 18, p. 26). The backchannel UART
//! is on P4.2 and P4.3.
//!
//! How to test (a jumper wire):
//! 1. Connect P1.6 (J1 pin 3) to P3.6 (J1 pin 5) with a jumper wire. (Header pins: SLAU680 Figure 10,
//!    p. 15.)
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
    let periph = msp430fr2355::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // P3.6 reads the wire, with its pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313;
    // SLASEC4D Table 6-65, p. 100)
    let p3 = Batch::new(periph.p3)
        .config_pin6(|p| p.pullup())
        .split(&pmm);
    // S1 pulls P4.1 low. The board has no pullup on it, so the internal one is on (SLAU680 Figure 18,
    // p. 26).
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut wire = p3.pin6;
    let mut s1 = p4.pin1;
    // P1.6 drives the wire low as a GPIO output, P1SELx = 00 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96)
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
