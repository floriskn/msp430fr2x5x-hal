//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1's start counter in bypass mode: with ENSTFCNT1 set, `freeze()` only returns once XT1 has run cleanly
//! for 1024 cycles. An LED on P1.0 turns on when `freeze()` returns: 250 ms after the generator starts at
//! 4.096 kHz.
//!
//! At 32.768 kHz those 1024 cycles take only 31 ms, too short to see, so the generator runs at 4.096 kHz
//! here. Don't go below about 4 kHz: XT1 may count as faulty under 3.5 kHz. The data sheet's 1024 cycles
//! match a measurement on an MSP430FR2476; the family user's guide gives 8192 for bypass mode instead.
//! (SLASEE4C Table 5-4 note 8, p. 25: "start-up counter of 1024 clock cycles". 8192: SLAU445I 3.2.13,
//! p. 110. ENSTFCNT1: SLAU445I Table 3-11, p. 121. fFault,LFXT is at most 3500 Hz: SLASEE4C Table 5-4,
//! p. 25. XIN is P2.1: SLASEE4C Table 6-16, p. 60. ACLK is P1.1: SLASEE4C Table 6-15, p. 58. A low level on
//! RST/NMI resets the device: SLAU445I 1.2, p. 30. No board document covers the LED: there is none for
//! the MSP430FR25x2.)
//!
//! How to test (function generator and the scope, an LED and a resistor):
//! 1. Power the MSP430FR2522 from 3.3 V, and connect an LED with a series resistor (about 1 kΩ) from P1.0
//!    to GND. XIN, P2.1, must have no crystal on it.
//! 2. Generator: square wave, 4.096 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before connecting: a
//!    negative or >3.6 V signal can damage the pin.
//! 3. Connect it to XIN, P2.1, its ground to GND, and switch the output on. Put the generator's signal on a
//!    second scope channel (1X) as well, with the BNC T-piece.
//! 4. Flash this example. Expected: the LED turns on.
//! 5. Switch the generator output off, and reset the device, with RST/NMI low for a moment. Expected: the
//!    LED stays off, because `freeze()` waits for XT1.
//! 6. Scope on the LED, P1.0, ground clip on GND: single-shot trigger on its rising edge, 100 ms/div, trigger
//!    point near the right of the screen. Switch the generator output on. Expected: the LED turns on 250 ms
//!    after the first edge on XIN.
//! 7. Repeat steps 5 and 6 at 8.192 kHz: 125 ms. With `START_COUNTER` set to false, the LED turns on almost
//!    at once after the first edge.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 4_096;
/// Whether `freeze()` waits for the 1024-cycle start counter
const START_COUNTER: bool = true;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let mut led = p1.pin0;
    led.set_low().ok();

    // ACLK on P1.1 with P1SELx = 10 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58), XIN on P2.1 with
    // P2SELx = 10 (SLASEE4C Table 6-16, p. 60)
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin1.to_alternate2();

    // XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120), optionally with the start
    // counter (ENSTFCNT1 = 1: SLAU445I Table 3-11, p. 121)
    let xt1 = Xt1Config::bypass(XT1_FREQ_HZ, xin);
    let xt1 = if START_COUNTER { xt1.enable_start_counter() } else { xt1 };

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117)
    let (_smclk, _aclk, _xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(xt1)
        .aclk_xt1clk()
        .freeze(&mut fram);

    led.set_high().ok();

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
