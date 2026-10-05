//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The watchdog in watchdog mode, fed while the program runs: an LED on P1.0 blinks as long as the program
//! feeds the watchdog. Hold down a button on P1.2, and the program stops feeding it, as a program that hangs
//! would: about a second later the watchdog resets the device, and an LED on P1.1 lights, because
//! `Pmm::take_reset_cause()` reports a watchdog time-out.
//!
//! The watchdog counts ACLK, 32768 Hz from REFO, and resets the device with a PUC once it has counted 2^15
//! cycles, 1 s, since `Wdt::feed()` last cleared the count. The program feeds it and toggles the LED on
//! P1.0 every 100 ms. At each start it reads the reset causes, which also clears them, and lights the LED on
//! P1.1 if the first one is a watchdog time-out. WDTIFG can't tell: in watchdog mode it "self clears upon a
//! watchdog timeout event".
//! (Watchdog mode: SLAU445I 12.2.2, p. 363. WDTSSEL = 01b, ACLK; WDTIS = 100b, "1 s at 32.768 kHz";
//! WDTCNTCL clears the count: SLAU445I Table 12-2, p. 366. REFO: SLASEE4C Table 5-7, p. 27. Watchdog
//! time-out, SYSRSTIV 16h: SLASEE4C Table 6-10, p. 52. WDTIFG: SLAU445I Table 1-10, p. 63. A low level on
//! RST/NMI resets the device: SLAU445I 1.2, p. 30. P1.0 to P1.2 are GPIO with P1SELx = 00: SLASEE4C
//! Table 6-15, p. 58. No board document covers the LEDs or the button: there is none for the MSP430FR25x2.)
//!
//! How to test (two LEDs, two resistors, and a button or a jumper wire):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, another from P1.1 to GND, and a
//!    button from P1.2 to GND (the internal pullup is on). A wire from P1.2 that you touch to GND works as
//!    the button too.
//! 2. Flash this example, then reset the device, with RST/NMI low for a moment. Expected: the LED on P1.0
//!    blinks, on for 0.1 s and off for 0.1 s, and the LED on P1.1 is off.
//! 3. Hold the button. Expected: the LED on P1.0 stops, and about a second later the LED on P1.1 lights: the
//!    watchdog has reset the device. As long as the button is held, the watchdog resets the device again
//!    about every second.
//! 4. Let go of the button. Expected: the LED on P1.0 blinks again, and the LED on P1.1 stays lit.
//! 5. Reset the device with RST/NMI again. Expected: the LED on P1.1 goes off, as the reset came from the
//!    RST pin this time.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::{Pmm, ResetCause},
    watchdog::{Wdt, WdtClkPeriods},
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Hold the watchdog while the program starts: after every PUC it runs in watchdog mode, with an
    // interval of about 32 ms (WDTHOLD = 1: SLAU445I Table 12-2, p. 366; SLAU445I 12.2.2, p. 363)
    let mut wdt = Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next start (reading SYSRSTIV
    // clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    // The LEDs on P1.0 and P1.1 are outputs, and the button pulls P1.2 low against its internal pullup
    // (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .config_pin2(|p| p.pullup())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p1.pin1;
    let mut button = p1.pin2;
    led2.set_state((first == Some(ResetCause::WatchdogTimeout)).into()).ok();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO, 32.768 kHz (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; REFO: SLASEE4C Table 5-7, p. 27)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // Watchdog mode, counting ACLK (WDTSSEL = 01b), with an interval of 2^15 cycles, "1 s at 32.768 kHz"
    // (WDTIS = 100b), from a cleared count (WDTCNTCL = 1) (SLAU445I Table 12-2, p. 366)
    wdt.set_aclk(&aclk).set_interval_and_start(WdtClkPeriods::_32k);

    loop {
        // While the button is held, the program neither feeds the watchdog nor toggles the LED, like a
        // program that hangs
        if button.is_high().unwrap() {
            // Clear the count, so the interval starts again (WDTCNTCL = 1: SLAU445I Table 12-2, p. 366)
            wdt.feed();
            led1.toggle().ok();
        }
        delay.delay_ms(100);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
