//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The watchdog in watchdog mode, fed while the program runs: LED1 blinks as long as the program feeds the
//! watchdog. Hold S1, and the program stops feeding it, as a program that hangs would: about a second
//! later the watchdog resets the device, and LED2 lights red, because `Pmm::take_reset_cause()` reports a
//! watchdog time-out.
//!
//! The watchdog counts ACLK, 32768 Hz from REFO, and resets the device with a PUC once it has counted 2^15
//! cycles, 1 s, since `Wdt::feed()` last cleared the count. The program feeds it and toggles LED1 every
//! 100 ms. At each start it reads the reset causes, which also clears them, and lights LED2 if the first
//! one is a watchdog time-out. WDTIFG can't tell: in watchdog mode it "self clears upon a watchdog timeout
//! event".
//! (Watchdog mode: SLAU445I 12.2.2, p. 363. WDTSSEL = 01b, ACLK; WDTIS = 100b, "1 s at 32.768 kHz";
//! WDTCNTCL clears the count: SLAU445I Table 12-2, p. 366. REFO: SLASEO7C 8.12.3.4, p. 30. Watchdog
//! time-out, SYSRSTIV 16h: SLASEO7C Table 9-10, p. 52. WDTIFG: SLAU445I Table 1-10, p. 63. LED1 on P1.0
//! is green, the red part of LED2 is P5.1, S1 is P4.0, and S3 is the reset button: SLAU802 Figure 19,
//! p. 25.)
//!
//! How to test:
//! 1. Flash this example, then press S3. Expected: LED1 blinks, on for 0.1 s and off for 0.1 s, and LED2
//!    is off.
//! 2. Hold S1. Expected: LED1 stops, and about a second later LED2 lights red: the watchdog has reset the
//!    device. As long as S1 is held, the watchdog resets the device again about every second.
//! 3. Let go of S1. Expected: LED1 blinks again, and LED2 stays red.
//! 4. Press S3. Expected: LED2 goes off, as the reset came from the reset pin this time.
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
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Hold the watchdog while the program starts: after every PUC it runs in watchdog mode, with an
    // interval of about 32 ms (WDTHOLD = 1: SLAU445I Table 12-2, p. 366; SLAU445I 12.2.2, p. 363)
    let mut wdt = Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next start (reading SYSRSTIV
    // clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    // S1 pulls P4.0 low, with the internal pullup on (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313) besides R9 (SLAU802 Figure 19, p. 25)
    let p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.pullup())
        .split(&pmm);
    let p5 = Batch::new(periph.p5)
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut s1 = p4.pin0;
    let mut red = p5.pin1;
    red.set_state((first == Some(ResetCause::WatchdogTimeout)).into()).ok();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO, 32.768 kHz (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; REFO: SLASEO7C 8.12.3.4, p. 30)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // Watchdog mode, counting ACLK (WDTSSEL = 01b), with an interval of 2^15 cycles, "1 s at 32.768 kHz"
    // (WDTIS = 100b), from a cleared count (WDTCNTCL = 1) (SLAU445I Table 12-2, p. 366)
    wdt.set_aclk(&aclk).set_interval_and_start(WdtClkPeriods::_32k);

    loop {
        // While S1 is held, the program neither feeds the watchdog nor toggles LED1, like a program that
        // hangs
        if s1.is_high().unwrap() {
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
