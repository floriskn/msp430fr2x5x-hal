//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The supply current in LPM4, with the high-side supply voltage supervisor (SVSH) on and off. The board
//! sleeps in LPM4, and each press of button S2 switches SVSH, then sleeps again. A multimeter in series
//! with the supply shows the difference.
//! (LPM4 at 25 °C and 3 V: 1.00 µA typical with SVS, 0.82 µA without: SLASEC4D 5.7, p. 29. SVSHE turns
//! SVSH off in LPM2 to LPM4: SLAU445I Table 2-2, p. 91. S2 is P2.3: SLAU680 Figure 18, p. 26.)
//!
//! After each press, LED2 (green) blinks if SVSH is now on, and LED1 (red) if it's off.
//!
//! How to test (multimeter, following SLAU680 2.4, p. 12 to p. 13):
//! 1. Flash this example. Disconnect the function generator from the board.
//! 2. On J101, remove the TXD, RXD, SBW RST, SBW TST and 3V3 jumpers. Keep GND. The board is now off.
//! 3. Set the multimeter to DC current, with its leads in its current jacks (see its manual), and hold
//!    the leads on the two 3V3 pins of J101. The board starts.
//! 4. Read the current: about 1 µA, with SVSH on. Press S2: LED1 blinks and the current drops by about
//!    0.18 µA. Press S2 again: LED2 blinks and it rises again.
//!
//! The current flickers while the LED blinks, so read it a few seconds after. Put the jumpers back
//! afterwards: without them the board can't be flashed.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use msp430::interrupt::Mutex;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV},
    lpm::{request_lpm4_with_interrupts, SvsState},
    pmm::Pmm,
    watchdog::Wdt,
};
use msp430fr2355::{interrupt, P2};
use panic_msp430 as _;

static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). No peripheral uses them, so
    // they stop in LPM4.
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);

    // "Make sure there are no floating inputs/outputs" (SLAU680 2.4, p. 13): S2 (P2.3) gets its pullup,
    // which the button needs, as the board has none (SLAU680 Figure 18, p. 26), and `pulldown_unused` gives
    // every other pin its pulldown (SLAU445I 8.3.2, p. 317; pullup and pulldown: PxDIR = 0, PxREN = 1,
    // PxOUT = 1 or 0: SLAU445I Table 8-1, p. 313). That includes S1 (P4.1), which has no pullup on the
    // board either, so its pulldown draws no current.
    let p1 = Batch::new(periph.p1).pulldown_unused().split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .pulldown_unused()
        .split(&pmm);
    let _p3 = Batch::new(periph.p3).pulldown_unused().split(&pmm);
    let _p4 = Batch::new(periph.p4).pulldown_unused().split(&pmm);
    let _p5 = Batch::new(periph.p5).pulldown_unused().split(&pmm);
    let p6 = Batch::new(periph.p6).pulldown_unused().split(&pmm);

    // LED1 is P1.0 and LED2 is P6.6 (SLAU680 Figure 18, p. 26)
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p6.pin6.to_output_low();
    let mut s2 = p2.pin3;
    with(|cs| P2IV.borrow_ref_mut(cs).replace(p2.pxiv));

    // S2 pulls P2.3 low, so a press is a high-to-low transition (PxIES = 1: SLAU445I Table 8-16, p. 336;
    // PxIE: SLAU445I Table 8-17, p. 336). Port interrupts wake the device from LPM4 (SLASEC4D Table 6-1,
    // p. 61).
    s2.select_falling_edge_trigger().enable_interrupts();

    // SVSH is on after reset (SVSHE = 1: SLAU445I Table 2-2, p. 91)
    let mut svsh = SvsState::Enabled;
    loop {
        // No clock is in use, so this is LPM4 (SLAU445I Table 1-3, p. 39). The PORT2 handler wakes the
        // CPU, and the loop continues.
        request_lpm4_with_interrupts();

        svsh = match svsh {
            SvsState::Enabled => SvsState::Disabled,
            SvsState::Disabled => SvsState::Enabled,
        };
        pmm.set_svsh(svsh);

        let led: &mut dyn OutputPin<Error = _> = match svsh {
            SvsState::Enabled => &mut led2,
            SvsState::Disabled => &mut led1,
        };
        led.set_high().ok();
        delay.delay_ms(100);
        led.set_low().ok();

        // Wait for the button's release and its bouncing to end, then forget the edges it caused
        while s2.is_low().unwrap() {}
        delay.delay_ms(100);
        s2.clear_ifg();
    }
}

// The port 2 interrupt vector, P2IFG.0 to P2IFG.7 through P2IV (FFD2h: SLASEC4D Table 6-2, p. 64).
// Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
// `wake_cpu` returns the CPU to active mode afterwards (an interrupt returns to another operating mode
// if its handler changes the SR saved on the stack: SLAU445I 1.4, p. 36).
#[interrupt(wake_cpu)]
fn PORT2() {
    with(|cs| {
        if let Some(ref mut p2iv) = *P2IV.borrow_ref_mut(cs) {
            let _: GpioVector = p2iv.get_interrupt_vector();
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
