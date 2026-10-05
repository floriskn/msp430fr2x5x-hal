//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! GPIO pins configured again while the program runs. Each press of a button on P1.6 takes the next step,
//! and two LEDs show what it did:
//! 1. `batch()`: port 1 is configured again in one go, P1.0 and P1.4, the two LEDs' pins, as outputs, which
//!    are then set high: both LEDs on. This is the state after flashing.
//! 2. `to_input_pulldown()`: P1.0 becomes an input with its pull-down, which doesn't drive the LED: the LED
//!    on P1.0 off.
//! 3. `to_output_high()`: P1.0 becomes an output that starts high: the LED on P1.0 on.
//! 4. `to_adc_mode()`: P1.0 becomes the ADC input A0, which switches its output driver off: off.
//! 5. `from_adc_mode()`: P1.0 is an output again, still high: on.
//! 6. `to_input_floating()`: P1.0 becomes an input without a pull resistor: off.
//! 7. `to_alternate2()`: P1.4 takes its TA0.1 function, where the timer holds it low: the LED on P1.4 off.
//! 8. `to_gpio()`: P1.4 leaves the TA0.1 function and drives its P1OUT bit, which is still high: on.
//! 9. `to_input_pullup()`: P1.7, the second button's pin, an output driving low until now, becomes an input
//!    with its pull-up, and the LED on P1.4 follows it: off while that button is held.
//!
//! TA0 isn't set up, so its output 1 is the OUT bit of TA0CCTL1, which is low after reset. The batch writes
//! each of the port's registers once instead of once per pin.
//! (A0 and TA0.1, and ADCPCTLx "disables both the output driver and input Schmitt trigger": SLASEE4C
//! Table 6-15, p. 58. Output mode 0 and OUT after reset: SLAU445I Table 13-2, p. 376; SLAU445I Table 13-6,
//! p. 386. Pin directions and pull resistors: SLAU445I Table 8-1, p. 313. The port registers: SLAU445I
//! Tables 8-10 to 8-17, p. 334 to p. 336. No board document covers the LEDs or the buttons: there is none
//! for the MSP430FR25x2.)
//!
//! How to test (two LEDs, two resistors, and two buttons or jumper wires):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another one from P1.4 to
//!    GND. Connect a button from P1.6 to GND, and another one from P1.7 to GND: the program turns on their
//!    internal pull-ups. A wire that you touch to GND works as a button too.
//! 2. Flash this example. Expected: both LEDs on (step 1).
//! 3. Press the button on P1.6 eight times, and check the LEDs after each press against steps 2 to 9. In
//!    step 9, the LED on P1.4 is on, and off while the button on P1.7 is held.
//! 4. Reset the MSP430FR2522, or power it off and on, to start again.
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
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316). The first
    // button's pin, P1.6, is an input with its pull-up (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313).
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).config_pin6(|p| p.pullup()).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118), for the delays
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // 1. Both LEDs on: P1.0 and P1.4 become outputs in one batch, and are then set high
    let mut p1 = p1.batch().config_pin0(|p| p.to_output()).config_pin4(|p| p.to_output()).split(&pmm);
    p1.pin0.set_high().ok();
    p1.pin4.set_high().ok();

    let mut s1 = p1.pin6;
    // The second button connects P1.7 to GND, so with the pin driving low pressing it changes nothing.
    // `to_output_low()` clears P1OUT.7 before it sets P1DIR.7 (SLAU445I Table 8-10, p. 334; SLAU445I
    // Table 8-11, p. 334).
    let s2 = p1.pin7.to_output_low();
    wait_for_s1(&mut s1, &mut delay);

    // 2. LED on P1.0 off: an input with its pull-down (P1DIR = 0, P1REN = 1, P1OUT = 0)
    let led1 = p1.pin0.to_input_pulldown();
    wait_for_s1(&mut s1, &mut delay);

    // 3. On: an output that starts high (P1OUT.0 is set before P1DIR.0)
    let led1 = led1.to_output_high();
    wait_for_s1(&mut s1, &mut delay);

    // 4. Off: ADCPCTL0 = 1 in SYSCFG2 (SLAU445I Table 1-31, p. 82)
    let led1 = led1.to_adc_mode();
    wait_for_s1(&mut s1, &mut delay);

    // 5. On: ADCPCTL0 = 0, and P1.0 is the output it was before
    let led1 = led1.from_adc_mode();
    wait_for_s1(&mut s1, &mut delay);

    // 6. Off: a floating input (P1DIR = 0, P1REN = 0)
    let _led1 = led1.to_input_floating();
    wait_for_s1(&mut s1, &mut delay);

    // 7. LED on P1.4 off: P1.4 in its TA0.1 function, P1SELx = 10 with P1DIR = 1
    let led2 = p1.pin4.to_alternate2();
    wait_for_s1(&mut s1, &mut delay);

    // 8. On: P1.4 back to GPIO, P1SELx = 00 (SLAU445I Table 8-3, p. 314), driving P1OUT.4
    let mut led2 = led2.to_gpio();
    wait_for_s1(&mut s1, &mut delay);

    // 9. P1.7 becomes an input with its pull-up (P1DIR = 0, P1REN = 1, P1OUT = 1), and the LED on P1.4 goes
    // off while the second button pulls it low
    let mut s2 = s2.to_input_pullup();
    loop {
        led2.set_state(s2.is_high().unwrap().into()).ok();
    }
}

/// Wait until the button on P1.6 is pressed and released. The 20 ms waits after each change keep a
/// bouncing contact from counting as more than one press.
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
