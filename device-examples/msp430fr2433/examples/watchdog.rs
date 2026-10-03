#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{gpio::Batch, pmm::Pmm, watchdog::{Wdt, WdtClkPeriods}};
use panic_msp430 as _;

// The LED on P1.0 should toggle about once per second (red LED1, SLAU739 Figure 18, p. 23)

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    // Configure watchdog for ~1 sec timeout
    // (VLOCLK: WDTSSEL = 10b, SLASE59F Table 6-8, p. 47; typically 10 kHz, SLASE59F Table 5-8, p. 26.
    // 8192 clocks: WDTIS = 101b, SLAU445I 12.3.1, Table 12-2, p. 366.)
    Wdt::constrain(periph.watchdog_timer)
        .set_vloclk() // ~10kHz
        .set_interval_and_start(WdtClkPeriods::_8192); // ~10kHz / 8192 ~= 1 sec

    // Pmm::new clears LOCKLPM5 (SLAU445I Table 2-7, p. 97). SLASE59F 6.10.3, p. 46 sets the ports up before
    // that; clearing it first leaves the pins inputs until they are set up (SLAU445I 8.3.1, p. 316).
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let mut red_led = Batch::new(periph.p1).split(&pmm).pin0.to_output();

    // The LED changes state only if P1OUT keeps its value through the watchdog's PUC, which SLAU445I
    // Table 8-10, p. 334 doesn't promise: the reset value of PxOUT is "Undefined".
    red_led.toggle();

    // The watchdog will reset program execution when it times out (a PUC in watchdog mode: SLAU445I
    // 12.2.2, p. 363)
    loop {}
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
