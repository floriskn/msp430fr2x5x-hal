//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The Interrupt Compare Controller (ICC): interrupt priorities, and an interrupt handler interrupted by
//! one of a higher priority. LED2 blinks from the watchdog's interval interrupt, and each press of S2 runs
//! the port 2 interrupt handler, which keeps LED1 on for 2 s. While the watchdog has the higher priority,
//! LED2 keeps blinking during those 2 s. S1 swaps the two priorities, and then LED2 stops while LED1 is on.
//!
//! The watchdog interrupts every 2^13 ACLK cycles, 0.25 s with ACLK from REFO, and its handler toggles LED2.
//! The port 2 handler reads P2IV, which clears S2's flag, and sets GIE, as each handler must for the ICC to
//! nest them; then it waits 2 s with LED1 on. The ICC passes an interrupt to the CPU during a handler only
//! if its priority is higher than that handler's, so the watchdog's interrupts get through during the wait
//! only while the watchdog has the higher priority. Otherwise they wait until the handler has returned.
//! (The ICC: SLAU445I 5.2, p. 282. Clearing the flag and then setting GIE in each handler: SLAU445I 5.2.6.2,
//! p. 287; SLASEC4D 6.10.7, p. 71. The sources' priorities, ILSR2 for port 2 and ILSR12 for the watchdog:
//! SLASEC4D Table 6-13, p. 71. WDTIS = 101b: SLAU445I Table 12-2, p. 366. LED1 on P1.0 is red, LED2 on
//! P6.6 green, S1 is P4.1 and S2 P2.3, and the board has no pull-ups for the buttons: SLAU680 Figure 18,
//! p. 26.)
//!
//! How to test:
//! 1. Flash this example. Expected: LED2 blinks, 0.25 s on and 0.25 s off.
//! 2. Press S2: LED1 lights for 2 s, and LED2 keeps blinking all the while.
//! 3. Press S1, which swaps the priorities, and then S2 again: LED1 lights for 2 s, and LED2 stays as it
//!    is meanwhile, on or off, and blinks again once LED1 goes off.
//! 4. Each press of S1 swaps the priorities again. S2 isn't debounced, so let go of it within the 2 s:
//!    a bouncing contact after the handler has ended runs it again (SLAU445I 8.2.6, p. 315).
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    delay::SysDelay,
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin0, Pin6, PxIV, P1, P2, P6},
    icc::{Icc, IccSource, Priority},
    pmm::Pmm,
    watchdog::{Wdt, WdtClkPeriods},
};
use msp430fr2355::interrupt;
use panic_msp430 as _;

static LED1: Mutex<RefCell<Option<Pin<P1, Pin0, Output>>>> = Mutex::new(RefCell::new(None));
static LED2: Mutex<RefCell<Option<Pin<P6, Pin6, Output>>>> = Mutex::new(RefCell::new(None));
static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));
static DELAY: Mutex<Cell<Option<SysDelay>>> = Mutex::new(Cell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    let mut wdt = Wdt::constrain(periph.wdt_a).to_interval();

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // S2 on P2.3 and S1 on P4.1 are inputs with their pull-ups (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2).config_pin3(|p| p.pullup()).split(&pmm);
    let p4 = Batch::new(periph.p4).config_pin1(|p| p.pullup()).split(&pmm);
    let p6 = Batch::new(periph.p6).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    let mut s1 = p4.pin1;
    let mut s2 = p2.pin3;
    with(|cs| {
        LED1.borrow_ref_mut(cs).replace(p1.pin0.to_output_low());
        LED2.borrow_ref_mut(cs).replace(p6.pin6.to_output_low());
        P2IV.borrow_ref_mut(cs).replace(p2.pxiv);
        DELAY.borrow(cs).set(Some(delay));
    });

    // A press of S2 is a high-to-low transition (P2IES = 1), which sets P2IFG.3 and, with P2IE.3, requests
    // the port 2 interrupt (SLAU445I 8.2.6, p. 315)
    s2.select_falling_edge_trigger();
    s2.clear_ifg();
    s2.enable_interrupts();

    // The watchdog as an interval timer, from ACLK (WDTTMSEL = 1, WDTSSEL = 01b, WDTIE: SLAU445I Table 12-2,
    // p. 366; SLAU445I 12.2.3, p. 363)
    wdt.set_aclk(&aclk).enable_interrupts().set_interval_and_start(WdtClkPeriods::_8192);

    // The watchdog at the highest priority and port 2 at the lowest (ILSRx: SLAU445I 5.2.2, p. 283), and the
    // ICC on (ICCEN). `enable()` changes ICCEN with interrupts disabled (SLAU445I 5.2.6.3, p. 288).
    let mut icc = Icc::new(periph.icc);
    icc.set_priority(IccSource::Watchdog, Priority::Highest);
    icc.set_priority(IccSource::Port2, Priority::Lowest);
    icc.enable();

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    loop {
        wait_for_s1(&mut s1, &mut delay);
        // Swap the priorities. They can change at any time (SLAU445I 5.2.2, p. 283).
        let (high, low) = if icc.priority(IccSource::Watchdog) == Priority::Highest {
            (IccSource::Port2, IccSource::Watchdog)
        } else {
            (IccSource::Watchdog, IccSource::Port2)
        };
        icc.set_priority(high, Priority::Highest);
        icc.set_priority(low, Priority::Lowest);
    }
}

/// Wait until S1 is pressed and released. The 20 ms waits after each change keep a bouncing contact from
/// counting as more than one press.
fn wait_for_s1(s1: &mut impl InputPin, delay: &mut SysDelay) {
    while s1.is_high().unwrap() {}
    delay.delay_ms(20);
    while s1.is_low().unwrap() {}
    delay.delay_ms(20);
}

fn set_led1(on: bool) {
    with(|cs| {
        if let Some(led1) = LED1.borrow_ref_mut(cs).as_mut() {
            led1.set_state(on.into()).ok();
        }
    });
}

/// Read P2IV, which clears the highest pending flag of port 2 (SLAU445I 8.2.6, p. 315)
fn port2_source() -> Option<GpioVector> {
    with(|cs| P2IV.borrow_ref_mut(cs).as_mut().map(|p2iv| p2iv.get_interrupt_vector()))
}

// The port 2 vector at FFD2h (SLASEC4D Table 6-2, p. 64)
#[interrupt]
fn PORT2() {
    let source = port2_source();
    // With the flag cleared, let interrupts of a higher priority interrupt this handler (SLAU445I 5.2.6.2,
    // p. 287)
    unsafe { enable_interrupts() };

    if source == Some(GpioVector::Pin3Isr) {
        set_led1(true);
        // The delay outside a critical section, so interrupts stay enabled
        if let Some(mut delay) = with(|cs| DELAY.borrow(cs).get()) {
            delay.delay_ms(2000);
        }
        set_led1(false);
        // Clear the flag again: S2's contact can bounce on the press and the release, and each falling edge
        // sets it (SLAU445I 8.2.6, p. 315)
        port2_source();
    }
}

// The watchdog's interval mode vector at FFE6h. Serving it clears WDTIFG (SLASEC4D Table 6-2, p. 63;
// SLAU445I 12.2.3, p. 363), and then GIE is set, as in every handler (SLAU445I 5.2.6.2, p. 287).
#[interrupt]
fn WDT() {
    unsafe { enable_interrupts() };
    with(|cs| {
        if let Some(led2) = LED2.borrow_ref_mut(cs).as_mut() {
            led2.toggle().ok();
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
