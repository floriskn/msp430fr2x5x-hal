//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! GPIO pins configured again while the program runs. Each press of S1 takes the next step, and the LEDs
//! show what it did:
//! 1. `batch()`: port 6 is configured again in one go, P6.6, LED2, as an output, which is then set high:
//!    LED2 on. This is the state after flashing.
//! 2. `to_output_high()`: LED1's pin, P1.0, becomes an output that starts high: LED1 on.
//! 3. `to_input_pulldown()`: P1.0 becomes an input with its pull-down, which doesn't drive the LED: LED1
//!    off.
//! 4. `to_output_high()`: LED1 on.
//! 5. `to_input_floating()`: P1.0 becomes an input without a pull resistor: LED1 off.
//! 6. `to_alternate2()`: P1.0, made an output driving low, takes its SMCLK function: LED1 lights, driven by
//!    SMCLK's square wave of about 1 MHz.
//! 7. `to_gpio()`: P1.0 leaves the SMCLK function and drives its P1OUT bit, which is low: LED1 off.
//! 8. `to_input_pullup()`: S2's pin, P2.3, an output driving low until now, becomes an input with its
//!    pull-up, and LED2 follows it: LED2 goes off while S2 is held.
//!
//! The board has no pull-up resistors on the buttons, so S1 and S2 need the internal ones. The batch writes
//! each of the port's registers once instead of once per pin.
//! (S1 on P4.1 and S2 on P2.3, with no pull-ups; LED1 on P1.0 is red and LED2 on P6.6 green: SLAU680
//! Figure 18, p. 26. SMCLK on P1.0: SLASEC4D Table 6-63, p. 96. Pin directions and pull resistors: SLAU445I
//! Table 8-1, p. 313. The port registers: SLAU445I Tables 8-10 to 8-17, p. 334 to p. 336.)
//!
//! How to test:
//! 1. Flash this example. Expected: LED1 off, LED2 on (step 1).
//! 2. Press S1 seven times, and check the LEDs after each press against steps 2 to 8. In step 8, LED2 is
//!    on, and off while S2 is held.
//! 3. Press the reset button S3 to start again (SLAU680 Figure 18, p. 26).
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    delay::SysDelay,
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // S1 on P4.1 is an input with its pull-up (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p4 = Batch::new(periph.p4).config_pin1(|p| p.pullup()).split(&pmm);
    let p6 = Batch::new(periph.p6).split(&pmm);
    let mut s1 = p4.pin1;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118), for the delays and step 6
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // S2 connects P2.3 to GND, so with the pin driving low pressing S2 changes nothing. `to_output_low()`
    // clears P2OUT.3 before it sets P2DIR.3 (SLAU445I Table 8-10, p. 334; SLAU445I Table 8-11, p. 334).
    let s2 = p2.pin3.to_output_low();

    // 1. LED2 on: P6.6 becomes an output in a batch, and is then set high
    let mut p6 = p6.batch().config_pin6(|p| p.to_output()).split(&pmm);
    p6.pin6.set_high().ok();
    wait_for_s1(&mut s1, &mut delay);

    // 2. LED1 on: an output that starts high (P1OUT.0 is set before P1DIR.0)
    let led1 = p1.pin0.to_output_high();
    wait_for_s1(&mut s1, &mut delay);

    // 3. LED1 off: an input with its pull-down (P1DIR = 0, P1REN = 1, P1OUT = 0)
    let led1 = led1.to_input_pulldown();
    wait_for_s1(&mut s1, &mut delay);

    // 4. LED1 on
    let led1 = led1.to_output_high();
    wait_for_s1(&mut s1, &mut delay);

    // 5. LED1 off: a floating input (P1DIR = 0, P1REN = 0)
    let led1 = led1.to_input_floating();
    wait_for_s1(&mut s1, &mut delay);

    // 6. LED1 lit by SMCLK: P1SELx = 10 with P1DIR = 1, and P1OUT.0 low for step 7
    let led1 = led1.to_output_low().to_alternate2();
    wait_for_s1(&mut s1, &mut delay);

    // 7. LED1 off: P1.0 back to GPIO, P1SELx = 00 (SLAU445I Table 8-3, p. 314), driving P1OUT.0
    let _led1 = led1.to_gpio();
    wait_for_s1(&mut s1, &mut delay);

    // 8. S2's pin becomes an input with its pull-up (P2DIR = 0, P2REN = 1, P2OUT = 1), and LED2 goes off
    // while S2 pulls it low
    let mut s2 = s2.to_input_pullup();
    loop {
        p6.pin6.set_state(s2.is_high().unwrap().into()).ok();
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

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
