//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A capture input handled in its interrupt: each press of button S2, which reaches P1.2 through a jumper
//! wire, toggles LED1.
//!
//! P1.2 is TA0.CCI2A, input A of TA0's CCR2. S2's pin, P2.7, has no timer function, so a jumper wire
//! connects it to P1.2: S2 pulls both low when pressed, and P2.7's internal pullup pulls them high again.
//! CCR2 captures each falling edge. Its interrupt reads TA0IV, which gives the capture, and toggles LED1.
//! The example also shows how to write panic-free code, with `panic_never` in release builds.
//! (TA0.CCI2A on P1.2: SLASE59F Table 6-11, p. 50; SLASE59F Table 6-17, p. 55. P2.7 is a GPIO only:
//! SLASE59F Table 6-19, p. 58. TA0IV: SLAU445I 13.2.6.2, p. 380. S2 on P2.7, with no pull-up on the
//! board, and LED1 on P1.0, red: SLAU739 Figure 18, p. 23.)
//!
//! How to test (a jumper wire):
//! 1. Connect P2.7 (J1 pin 8) to P1.2 (J1 pin 10) with a jumper wire. (Header pins: SLAU739 Figure 18,
//!    p. 23.)
//! 2. Flash this example.
//! 3. Press S2: LED1 toggles. Press it again: LED1 toggles back.
//! 4. S2 isn't debounced, so a press can toggle LED1 more than once, and a release can toggle it too.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::UnsafeCell;
use critical_section::with;
use embedded_hal::digital::StatefulOutputPin;
use msp430::interrupt::{enable, Mutex};
use msp430_rt::entry;
use msp430fr2433::interrupt;
use msp430_hal::{
    capture::{
        CCR2, CapCmp, CapTrigger, Capture, CaptureParts3, CaptureVector, TBxIV, TimerConfig
    }, clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::{Batch, *}, pmm::Pmm, watchdog::Wdt
};

#[cfg(debug_assertions)]
use panic_msp430 as _;

#[cfg(not(debug_assertions))]
use panic_never as _;

// We use UnsafeCell as a panic-free version of RefCell. If you aren't using `panic_never` then RefCell is more ergonomic.
static CAPTURE: Mutex<UnsafeCell<Option<Capture<msp430fr2433::Ta0, CCR2>>>> =
    Mutex::new(UnsafeCell::new(None));
static VECTOR: Mutex<UnsafeCell<Option<TBxIV<msp430fr2433::Ta0>>>> =
    Mutex::new(UnsafeCell::new(None));
static LED1: Mutex<UnsafeCell<Option<Pin<P1, Pin0, Output>>>> =
    Mutex::new(UnsafeCell::new(None));

#[entry]
fn main() -> ! {
    let Some(periph) = msp430fr2433::Peripherals::take() else { loop {} };
    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    // S2 on P2.7 with the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313),
    // which also pulls P1.2 high through the jumper wire
    let _p2 = Batch::new(periph.p2)
        .config_pin7(|p| p.pullup())
        .split(&pmm);
    let led1 = p1.pin0;

    with(|cs| unsafe { *LED1.borrow(cs).get() = Some(led1) });

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range, ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA0 CCR2 input A (CCI2A) is P1.2, with P1SELx = 10 and P1DIR = 0 (SLASE59F Table 6-11, p. 50;
    // SLASE59F Table 6-17, p. 55). ACLK is TASSEL = 01b (SLASE59F Table 6-7, p. 46). Capture on the
    // falling edge, when S2 is pressed: CM = 10b, CCIS = 00b (SLAU445I Table 13-6, p. 386).
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::aclk(&aclk))
        .config_cap2_input_A(p1.pin2.to_alternate2())
        .config_cap2_trigger(CapTrigger::FallingEdge)
        .commit();
    let mut capture = captures.cap2;
    let vectors = captures.tbxiv;

    setup_capture(&mut capture);
    with(|cs| {
        unsafe { *CAPTURE.borrow(cs).get() = Some(capture) }
        unsafe { *VECTOR.borrow(cs).get() = Some(vectors) }
    });
    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable() };

    loop {}
}

/// Enable the capture interrupt (CCIE: SLAU445I Table 13-6, p. 386)
fn setup_capture<T: CapCmp<C>, C>(capture: &mut Capture<T, C>) {
    capture.enable_interrupts();
}

// The TA0 CCR1, CCR2 and overflow interrupt vector (FFF6h: SLASE59F Table 6-2, p. 41). Reading TA0IV
// gives the highest pending source and clears its flag (SLAU445I 13.2.6.2, p. 380; SLAU445I
// Table 13-8, p. 388).
#[interrupt]
fn TIMER0_A1() {
    with(|cs| {
        let Some(vector) = unsafe { &mut *VECTOR.borrow(cs).get() }.as_mut() else { return; };
        let Some(capture) = unsafe { &mut *CAPTURE.borrow(cs).get() }.as_mut() else { return; };
        let Some(led) = unsafe { &mut *LED1.borrow(cs).get() }.as_mut() else { return; };

        if let CaptureVector::Capture2(cap) = vector.interrupt_vector() {
            if cap.interrupt_capture(capture).is_ok() {
                led.toggle().unwrap();
            }
        };
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
