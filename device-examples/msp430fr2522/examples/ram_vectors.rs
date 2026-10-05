//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Interrupt vectors in RAM: the program changes which function handles an interrupt while it runs.
//!
//! The watchdog, as an interval timer, requests its interrupt every 0.25 s. Its vector in the RAM table
//! points at a handler that toggles an LED on P1.0, or at one that toggles an LED on P2.0. Each press of a
//! button on P2.3 points it at the other handler, so the other LED blinks.
//! (Interrupt vectors in RAM: SLAU445I 1.3.6.1, p. 36. The watchdog's interval mode: SLAU445I 12.2.3,
//! p. 363. P1.0, P2.0 and P2.3 are GPIO with PxSELx = 00: SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16,
//! p. 60. P2.3 only exists on the 20-pin RHL package: SLASEE4C Table 4-2, p. 14. No board document covers
//! the parts to connect: there is none for the MSP430FR25x2.)
//!
//! Before building this example, change `RAM LENGTH` in `memory.x` from 0x800 to 0x780. The RAM table
//! takes the top 128 bytes of RAM, 2780h to 27FFh, and the stack starts at the end of RAM, so RAM must
//! leave them out, or the table and the stack overwrite each other. The other examples work either way.
//! Cargo doesn't notice changes to `memory.x`, so clean the examples after the change, or they keep the old
//! RAM length: `cargo clean -p msp430fr25x2-hal-examples --target msp430-none-elf`. (RAM is 2000h to
//! 27FFh: SLASEE4C Table 6-19, p. 62.)
//!
//! How to test (two LEDs and resistors, and a push button):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, another from P2.0 to GND, and a
//!    push button from P2.3 to GND (the internal pullup is on).
//! 2. Change `memory.x` and clean the examples as above, then flash this example. The LED on P1.0 blinks
//!    twice a second.
//! 3. Press the button: the LED on P1.0 stops, and the one on P2.0 blinks instead.
//! 4. Press the button again: the LED on P1.0 blinks again.
//!
//! The RAM table stays in use until the next brownout reset (BOR), even when another program is flashed,
//! and that program's interrupts would then go to this example's handlers. So after flashing the next
//! example, reset the device with RST/NMI low for a moment, which is a BOR, or switch the power off and on.
//! (SYSRIVECT is reset by a BOR only: SLAU445I Table 1-13, p. 66. The RST pin causes a BOR: SLAU445I 1.2,
//! p. 30.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, Output, Pin, Pin0, P1, P2},
    pmm::Pmm,
    sys::SysParts,
    watchdog::{Wdt, WdtClkPeriods},
};
use msp430fr25x2::Interrupt;
use panic_msp430 as _;

static LED_P1_0: Mutex<RefCell<Option<Pin<P1, Pin0, Output>>>> = Mutex::new(RefCell::new(None));
static LED_P2_0: Mutex<RefCell<Option<Pin<P2, Pin0, Output>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog, then use it as an interval timer (WDTHOLD, WDTTMSEL: SLAU445I Table 12-2,
    // p. 366)
    let mut wdt = Wdt::constrain(periph.wdt_a).to_interval();

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // The button pulls P2.3 low, against the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut button = p2.pin3;
    with(|cs| {
        LED_P1_0.borrow_ref_mut(cs).replace(p1.pin0.to_output_low());
        LED_P2_0.borrow_ref_mut(cs).replace(p2.pin0.to_output_low());
    });

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO, 32.768 kHz (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; REFO: SLASEE4C Table 5-7, p. 27)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // Copy the interrupt vectors from FRAM to the top of RAM, and take them from there (SYSRIVECT = 1:
    // SLAU445I Table 1-13, p. 66). Then point the watchdog's vector at the first handler.
    let mut sys = SysParts::new(periph.sfr);
    unsafe {
        sys.interrupt_vectors.use_ram();
        sys.interrupt_vectors.set_handler(Interrupt::WDT, blink_p1_0);
    }

    // The watchdog counts ACLK (WDTSSEL = 01b) and requests its interrupt every 8192 cycles, 0.25 s
    // (WDTIS = 101b: SLAU445I Table 12-2, p. 366; WDTIE: SLAU445I Table 1-9, p. 62)
    wdt.set_aclk(&aclk)
        .enable_interrupts()
        .set_interval_and_start(WdtClkPeriods::_8192);

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    let mut p1_0_blinks = true;
    loop {
        // Wait for a press and release of the button, and for the bouncing to stop
        while button.is_high().unwrap() {}
        while button.is_low().unwrap() {}
        delay.delay_ms(20);

        // One word is written, so the interrupt never sees half a vector
        p1_0_blinks = !p1_0_blinks;
        let handler: extern "msp430-interrupt" fn() = if p1_0_blinks { blink_p1_0 } else { blink_p2_0 };
        unsafe { sys.interrupt_vectors.set_handler(Interrupt::WDT, handler) };

        // Turn both LEDs off, so only the blinking one lights
        with(|cs| {
            LED_P1_0.borrow_ref_mut(cs).as_mut().map(|led| led.set_low().ok());
            LED_P2_0.borrow_ref_mut(cs).as_mut().map(|led| led.set_low().ok());
        });
    }
}

// The handlers for the RAM table: functions with the interrupt calling convention. (`#[interrupt]` puts a
// handler into the FRAM table instead.) In interval mode "WDTIFG is reset automatically by servicing the
// interrupt" (SLAU445I Table 1-10, p. 63).
extern "msp430-interrupt" fn blink_p1_0() {
    with(|cs| {
        LED_P1_0.borrow_ref_mut(cs).as_mut().map(|led| led.toggle().ok());
    });
}

extern "msp430-interrupt" fn blink_p2_0() {
    with(|cs| {
        LED_P2_0.borrow_ref_mut(cs).as_mut().map(|led| led.toggle().ok());
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
