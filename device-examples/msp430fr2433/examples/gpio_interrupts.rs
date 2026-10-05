//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! GPIO interrupts and the watchdog's interval interrupt: LED1 toggles every second, and each press of
//! S1 toggles both LED2 and LED1.
//!
//! The watchdog, as an interval timer, interrupts every 2^15 ACLK cycles, 1 s with ACLK from REFO, and
//! its interrupt toggles LED1. The main loop polls the interrupt flag of S1 (P2IFG.3, with its
//! interrupt disabled): at each press it toggles LED2 and sets the flag of P2.2 in software, which
//! requests the port 2 interrupt, and that toggles LED1.
//! (WDTIS = 100b, "1 s at 32.768 kHz": SLAU445I Table 12-2, p. 366. REFO: SLASE59F Table 5-7, p. 25.
//! Software can set PxIFG: SLAU445I 8.2.6, p. 315. LED1 on P1.0 is red, LED2 on P1.1 is green, and S1
//! is P2.3, with no pull-up on the board: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example. Leave P2.2 (J2 pin 18) open. (Header pins: SLAU739 Figure 18, p. 23.)
//! 2. Expected: LED1 toggles every second, on for 1 s and off for 1 s.
//! 3. Press S1: LED2 toggles, and so does LED1.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use critical_section::with;
use msp430fr2433::interrupt;

use core::cell::RefCell;
use embedded_hal::digital::*;
use msp430::interrupt::{enable as enable_int, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin0, PxIV, P1, P2},
    pmm::Pmm,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
use panic_msp430 as _;

static LED1: Mutex<RefCell<Option<Pin<P1, Pin0, Output>>>> = Mutex::new(RefCell::new(None));
static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();
    // Stop the watchdog, then use it as an interval timer (WDTHOLD, WDTTMSEL: SLAU445I Table 12-2,
    // p. 366; interval timer mode: SLAU445I 12.2.3, p. 363)
    let mut wdt = Wdt::constrain(periph.wdt_a).to_interval();

    // REFO runs at 32.768 kHz (SLASE59F Table 5-7, p. 25)
    // (MCLK from REFOCLK, SELMS = 001b, and ACLK from REFO, SELA = 01b: SLAU445I Table 3-8, p. 117;
    // SMCLK = MCLK / 2, DIVS = 01b: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_refoclk(MclkDiv::_1) // 32 kHz MCLK
        .smclk_on(SmclkDiv::_2) // 16 kHz SMCLK
        .aclk_refoclk()
        .freeze(&mut Fram::new(periph.frctl));

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // P2.3 with its pullup and P2.2 with its pulldown (PxREN = 1, PxOUT = 1 or 0: SLAU445I Table 8-1,
    // p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);

    let led1 = p1.pin0.to_output();
    // Onboard button with interrupt disabled (S1: SLAU739 Figure 18, p. 23)
    let mut button = p2.pin3;
    // Some random pin with interrupt enabled. IFG will be set manually.
    // (Software can set PxIFG to request the interrupt: SLAU445I 8.2.6, p. 315)
    let mut pin = p2.pin2.pulldown();
    // P1.1 drives LED2 (SLAU739 Figure 18, p. 23)
    let mut led2 = p1.pin1.to_output();
    let p2iv = p2.pxiv;

    with(|cs| LED1.borrow_ref_mut(cs).replace(led1));
    with(|cs| P2IV.borrow_ref_mut(cs).replace(p2iv));

    // ACLK is WDTSSEL = 01b (SLAU445I Table 12-2, p. 366); WDTIE enables the interval interrupt
    // (SLAU445I Table 1-9, p. 62). PxIES = 0 sets PxIFG on a low-to-high transition, PxIES = 1 on a
    // high-to-low one (SLAU445I Table 8-16, p. 336).
    wdt.set_aclk(&aclk)
        .enable_interrupts()
        .set_interval_and_start(WdtClkPeriods::_32k);
    pin.select_rising_edge_trigger().enable_interrupts();
    button.select_falling_edge_trigger();

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_int() };

    // P2IFG.3 is set on the selected edge even with its interrupt disabled, so it can be polled
    // (SLAU445I 8.2.6, p. 315)
    loop {
        block!(button.wait_for_ifg()).ok();
        led2.toggle().ok();
        pin.set_ifg();
    }
}

// The port 2 interrupt vector, P2IFG.0 to P2IFG.7 through P2IV (FFDAh: SLASE59F Table 6-2, p. 42).
// Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
#[interrupt]
fn PORT2() {
    with(|cs| {
        let Some(ref mut led1) = *LED1.borrow_ref_mut(cs) else { return; };
        let Some(ref mut p2iv) = *P2IV.borrow_ref_mut(cs) else { return; };

        if let GpioVector::Pin2Isr = p2iv.get_interrupt_vector() {
            led1.toggle().ok();
        }
    });
}

// The watchdog interval mode vector, WDTIFG (FFE6h: SLASE59F Table 6-2, p. 42). In interval mode
// "WDTIFG is reset automatically by servicing the interrupt" (SLAU445I Table 1-10, p. 63).
#[interrupt]
fn WDT() {
    with(|cs| {
        LED1.borrow_ref_mut(cs).as_mut().map(|led1| {
            led1.toggle().ok();
        })
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
