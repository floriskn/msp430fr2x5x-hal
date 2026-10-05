//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! GPIO pins configured again while the program runs. Each press of S1 takes the next step, and the LEDs
//! show what it did:
//! 1. `batch()`: port 1 is configured again in one go, P1.0 and P1.1, LED1 and LED2, as outputs, which are
//!    then set high: both LEDs on. This is the state after flashing.
//! 2. `to_input_pulldown()`: LED1's pin, P1.0, becomes an input with its pull-down, which doesn't drive the
//!    LED: LED1 off.
//! 3. `to_output_high()`: P1.0 becomes an output that starts high: LED1 on.
//! 4. `to_adc_mode()`: P1.0 becomes the ADC input A0, which switches its output driver off: LED1 off.
//! 5. `from_adc_mode()`: P1.0 is an output again, still high: LED1 on.
//! 6. `to_input_floating()`: P1.0 becomes an input without a pull resistor: LED1 off.
//! 7. `to_alternate2()`: LED2's pin, P1.1, takes its TA0.1 function, where the timer holds it low: LED2
//!    off.
//! 8. `to_gpio()`: P1.1 leaves the TA0.1 function and drives its P1OUT bit, which is still high: LED2 on.
//! 9. `to_input_pullup()`: S2's pin, P2.7, an output driving low until now, becomes an input with its
//!    pull-up, and LED2 follows it: LED2 goes off while S2 is held.
//!
//! TA0 isn't set up, so its output 1 is the OUT bit of TA0CCTL1, which is low after reset. The board has no
//! pull-up resistors on the buttons, so S1 and S2 need the internal ones. The batch writes each of the
//! port's registers once instead of once per pin.
//! (S1 on P2.3 and S2 on P2.7, with no pull-ups; LED1 on P1.0 is red and LED2 on P1.1 green: SLAU739
//! Figure 18, p. 23. A0 and TA0.1, and ADCPCTLx "disables both the output driver and input Schmitt
//! trigger": SLASE59F Table 6-17, p. 55. Output mode 0 and OUT after reset: SLAU445I Table 13-2, p. 376;
//! SLAU445I Table 13-6, p. 386. Pin directions and pull resistors: SLAU445I Table 8-1, p. 313. The port
//! registers: SLAU445I Tables 8-10 to 8-17, p. 334 to p. 336.)
//!
//! How to test:
//! 1. Flash this example. Expected: LED1 and LED2 on (step 1).
//! 2. Press S1 eight times, and check the LEDs after each press against steps 2 to 9. In step 9, LED2 is
//!    on, and off while S2 is held.
//! 3. Press the reset button S3 to start again (SLAU739 Figure 18, p. 23).
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
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // S1 on P2.3 is an input with its pull-up (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2).config_pin3(|p| p.pullup()).split(&pmm);
    let mut s1 = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118), for the delays
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // S2 connects P2.7 to GND, so with the pin driving low pressing S2 changes nothing. `to_output_low()`
    // clears P2OUT.7 before it sets P2DIR.7 (SLAU445I Table 8-10, p. 334; SLAU445I Table 8-11, p. 334).
    let s2 = p2.pin7.to_output_low();

    // 1. Both LEDs on: P1.0 and P1.1 become outputs in one batch, and are then set high
    let mut p1 = p1.batch().config_pin0(|p| p.to_output()).config_pin1(|p| p.to_output()).split(&pmm);
    p1.pin0.set_high().ok();
    p1.pin1.set_high().ok();
    wait_for_s1(&mut s1, &mut delay);

    // 2. LED1 off: an input with its pull-down (P1DIR = 0, P1REN = 1, P1OUT = 0)
    let led1 = p1.pin0.to_input_pulldown();
    wait_for_s1(&mut s1, &mut delay);

    // 3. LED1 on: an output that starts high (P1OUT.0 is set before P1DIR.0)
    let led1 = led1.to_output_high();
    wait_for_s1(&mut s1, &mut delay);

    // 4. LED1 off: ADCPCTL0 = 1 in SYSCFG2 (SLAU445I Table 1-31, p. 82)
    let led1 = led1.to_adc_mode();
    wait_for_s1(&mut s1, &mut delay);

    // 5. LED1 on: ADCPCTL0 = 0, and P1.0 is the output it was before
    let led1 = led1.from_adc_mode();
    wait_for_s1(&mut s1, &mut delay);

    // 6. LED1 off: a floating input (P1DIR = 0, P1REN = 0)
    let _led1 = led1.to_input_floating();
    wait_for_s1(&mut s1, &mut delay);

    // 7. LED2 off: P1.1 in its TA0.1 function, P1SELx = 10 with P1DIR = 1
    let led2 = p1.pin1.to_alternate2();
    wait_for_s1(&mut s1, &mut delay);

    // 8. LED2 on: P1.1 back to GPIO, P1SELx = 00 (SLAU445I Table 8-3, p. 314), driving P1OUT.1
    let mut led2 = led2.to_gpio();
    wait_for_s1(&mut s1, &mut delay);

    // 9. S2's pin becomes an input with its pull-up (P2DIR = 0, P2REN = 1, P2OUT = 1), and LED2 goes off
    // while S2 pulls it low
    let mut s2 = s2.to_input_pullup();
    loop {
        led2.set_state(s2.is_high().unwrap().into()).ok();
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
