//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Timer cascading through a jumper wire: one timer counts the periods of another. LED1 toggles every
//! second, and LED2 every 5 s.
//!
//! TA0 counts ACLK (REFO, 32.768 kHz) with a period of 1 s, as PWM: its CCR2 output, TA0.2 on P1.2, is
//! high for the first half of each period, and LED1 toggles every period. A jumper wire takes TA0.2 to
//! TA1's clock pin, TA1CLK on P1.6, and TA1 counts its rising edges, the periods of TA0: LED2 toggles every
//! 5 periods, so every 5 s. TA1 could count up to 65536 s like this, about 18 hours. The other devices
//! can cascade timers inside the chip, through a timer's INCLK input (`TimerConfig::cascade`); the
//! MSP430FR2433's timers have no INCLK input, hence the wire.
//! (REFO: SLASE59F Table 5-7, p. 25. The timers' clock inputs, none of them INCLK: SLASE59F Tables 6-11
//! to 6-14, p. 50 to p. 52. TA0.2 is P1.2 and TA1CLK is P1.6: SLASE59F Table 6-11, p. 50; SLASE59F
//! Table 6-12, p. 51. A timer counts on the rising edges of its clock: SLAU445I 13.2.1, p. 370.
//! LED1 on P1.0 is red and LED2 on P1.1 is green: SLAU739 Figure 18, p. 23.)
//!
//! How to test (a jumper wire):
//! 1. Connect P1.2 (J1 pin 10) to P1.6 (J1 pin 5) with a jumper wire.
//! 2. Flash this example.
//! 3. Expected: LED1 (red) is on for 1 s, then off for 1 s. LED2 (green) is on for 5 s, then off for 5 s.
//! 4. Move the wire's end from P1.2 to GND (J2 pin 20): TA1 gets no more edges, so LED2 stops changing,
//!    while LED1 keeps blinking.
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]

use embedded_hal::{digital::*, pwm::SetDutyCycle};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    pwm::PwmParts3,
    timer::{TimerConfig, TimerParts3},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// ACLK cycles per TA0 period: 1 s (ACLK is REFO, 32768 Hz: SLASE59F Table 5-7, p. 25)
const ACLK_CYCLES: u16 = 32_768;
/// TA0 periods per LED2 toggle
const PERIODS: u16 = 5;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // LED1, red, on P1.0 and LED2, green, on P1.1 (SLAU739 Figure 18, p. 23)
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p1.pin1.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA1 counts the rising edges on TA1CLK, P1.6 with P1SELx = 10 and P1DIR = 0 (TASSEL = 00b: SLAU445I
    // Table 13-4, p. 384; SLASE59F Table 6-17, p. 55), from 0 up to and including PERIODS - 1 ("The
    // number of timer counts in the period is TAxCCR0 + 1": SLAU445I 13.2.3.1, p. 371). It starts first,
    // so it doesn't miss TA0's first period.
    let ta1clk = p1.pin6.to_alternate2();
    let mut ta1 = TimerParts3::new(periph.ta1, TimerConfig::tbclk(ta1clk)).timer;
    ta1.start(PERIODS - 1);

    // TA0 counts ACLK (TASSEL = 01b: SLAU445I Table 13-4, p. 384) in up mode, ACLK_CYCLES counts per
    // period. Its CCR2 output, TA0.2 on P1.2 with P1SELx = 10 and P1DIR = 1 (SLASE59F Table 6-17, p. 55),
    // is high from the start of each period until the timer counts to the duty cycle, half a period here
    // (reset/set mode: SLAU445I Table 13-2, p. 376). The pin is low until the timer takes it over, so TA1
    // sees no extra edge.
    let pwm = PwmParts3::new(periph.ta0, TimerConfig::aclk(&aclk), ACLK_CYCLES - 1);
    let mut ta0_2 = pwm.pwm2.init(p1.pin2.to_output_low().to_alternate2());
    ta0_2.set_duty_cycle(ACLK_CYCLES / 2).unwrap();

    loop {
        // Each timer sets its TAIFG when it counts from TAxCCR0 to zero (SLAU445I 13.2.3.1, p. 371)
        if ta0_2.take_period_flag() {
            led1.toggle().unwrap();
        }
        if ta1.wait().is_ok() {
            led2.toggle().unwrap();
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
