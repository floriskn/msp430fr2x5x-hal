//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Changing a PWM duty cycle exactly once per period, from the period interrupt.
//!
//! TA0 runs PWM at 1 kHz from SMCLK at 1 MHz, on P1.1 (TA0.1). The interrupt at the start of each period
//! switches the duty cycle between 250 µs and 750 µs, so the pulses alternate.
//!
//! A Timer_A takes a new duty cycle at once. From the period interrupt, early in a period, it applies to
//! that period. At a random moment, a duty cycle below the count the timer has already passed isn't
//! reached in that period, so the output stays high to its end, and on into the next period up to the
//! new duty cycle. (Updating TAxCCRn, and in reset/set mode "The output is reset when the timer counts to
//! the TAxCCRn value": SLAU445I 13.2.4.2 and Table 13-2, p. 376. Only Timer_B's compare registers "are
//! double-buffered": SLAU445I 14.1.1, p. 391.)
//! (TA0.1 is P1.1: SLASE59F Table 6-11, p. 50. P1.1 is also LED2's pin: SLAU739 Figure 18, p. 23.)
//!
//! How to test (scope):
//! 1. Probe P1.1 (J2 pin 19), ground clip on GND (J2 pin 20). Trigger on the rising edge, 500 µs/div.
//!    (Header pins: SLAU739 Figure 18, p. 23.)
//! 2. Flash this example, with `USE_PERIOD_INTERRUPT = true`: the high pulses alternate between 250 µs
//!    and 750 µs, and LED2 glows. Turn on the scope's persistence (Display > Persist): only these two
//!    pulse widths ever appear.
//! 3. Set `USE_PERIOD_INTERRUPT = false` and flash again: the duty cycle now changes every 1.3 ms, at
//!    random points of a period. Besides pulses of 250 µs and 750 µs, some are 1250 µs wide: a change
//!    from 750 µs to 250 µs that comes after the count passed 250 leaves the output high for the rest of
//!    that period and the first 250 µs of the next.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{delay::DelayNs, pwm::SetDutyCycle};
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    pwm::{Pwm, PwmParts3, TimerConfig, CCR1},
    watchdog::Wdt,
};
use msp430fr2433::{interrupt, Ta0};
use panic_msp430 as _;

/// Change the duty cycle from the period interrupt, or at random moments
const USE_PERIOD_INTERRUPT: bool = true;

/// The two duty cycles, in SMCLK cycles of 1 µs, out of a period of 1000
const SHORT: u16 = 250;
const LONG: u16 = 750;

static PWM: Mutex<RefCell<Option<Pwm<Ta0, CCR1>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = DCOCLKDIV in the 8 MHz range, so the interrupt handler runs quickly, SMCLK = MCLK / 8 for
    // the timer, and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS:
    // SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA0 counts SMCLK (TASSEL = 10b: SLASE59F Table 6-7, p. 46) in up mode: 1000 counts per period
    // (SLAU445I 13.2.3.1, p. 371). TA0.1 is P1.1 with P1SELx = 10 and P1DIR = 1 (SLASE59F Table 6-17,
    // p. 55).
    let pwm = PwmParts3::new(periph.ta0, TimerConfig::smclk(&smclk), 999);
    let mut pwm1 = pwm.pwm1.init(p1.pin1.to_output_low().to_alternate2());
    pwm1.set_duty_cycle(SHORT).unwrap();

    if USE_PERIOD_INTERRUPT {
        // TAIE: the interrupt at the start of each period (SLAU445I Table 13-4, p. 384). Set GIE, which
        // masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33).
        pwm1.enable_period_interrupt();
        with(|cs| PWM.borrow_ref_mut(cs).replace(pwm1));
        unsafe { enable_interrupts() };
        loop {
            msp430::asm::nop();
        }
    } else {
        // 1.3 ms between changes: they land anywhere in a period
        let mut duty = SHORT;
        loop {
            duty = if duty == SHORT { LONG } else { SHORT };
            pwm1.set_duty_cycle(duty).unwrap();
            delay.delay_us(1300);
        }
    }
}

// The TA0 vector of CCR1, CCR2 and the timer overflow, TAIFG (FFF6h: SLASE59F Table 6-2, p. 41)
#[interrupt]
fn TIMER0_A1() {
    with(|cs| {
        if let Some(pwm) = PWM.borrow_ref_mut(cs).as_mut() {
            // Clear TAIFG, or the interrupt repeats as soon as it returns. The new duty cycle takes effect
            // at once, while the count is still low, so the period that has just started has it.
            if pwm.take_period_flag() {
                let duty = if pwm.duty() == SHORT { LONG } else { SHORT };
                pwm.set_duty_cycle(duty).unwrap();
            }
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
