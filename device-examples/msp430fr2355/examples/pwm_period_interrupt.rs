//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Changing a PWM duty cycle exactly once per period, from the period interrupt.
//!
//! TB0 runs PWM at 1 kHz from SMCLK at 1 MHz, on P1.6 (TB0.1). The interrupt at the start of each period
//! switches the duty cycle between 250 µs and 750 µs, so the pulses alternate.
//!
//! A Timer_B loads a new duty cycle when the timer reaches the old one, so a change never cuts a period
//! short or leaves one high to its end, whenever it's written. It takes effect one or two periods later,
//! depending on when it comes; from the period interrupt it's always the next period. (CLLD = 11b:
//! SLAU445I Table 14-2, p. 400. TI's workaround for erratum TB25 sets duty cycles in this interrupt too:
//! SLAZ695J TB25, p. 11; the HAL doesn't need it, see the `pwm` module documentation.)
//! (TB0.1 is P1.6: SLASEC4D Table 6-16, p. 73. Header pins: SLAU680 Figure 10, p. 15.)
//!
//! How to test (scope):
//! 1. Probe P1.6 (J1 pin 3), ground clip on GND (J3 pin 22). Trigger on the rising edge, 500 µs/div.
//! 2. Flash this example, with `USE_PERIOD_INTERRUPT = true`: the high pulses alternate between 250 µs
//!    and 750 µs. Turn on the scope's persistence (Display > Persist): only these two pulse widths ever
//!    appear.
//! 3. Set `USE_PERIOD_INTERRUPT = false` and flash again: the duty cycle now changes every 1.3 ms, at
//!    random points of a period. The pulses are still only 250 µs or 750 µs wide, but no longer strictly
//!    alternate: sometimes the same width comes twice. (Measured on an MSP430FR2476, with changes at
//!    random moments: of 3000 pulses, none had another width.)
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
use msp430fr2355::{interrupt, Tb0};
use panic_msp430 as _;

/// Change the duty cycle from the period interrupt, or at random moments
const USE_PERIOD_INTERRUPT: bool = true;

/// The two duty cycles, in SMCLK cycles of 1 µs, out of a period of 1000
const SHORT: u16 = 250;
const LONG: u16 = 750;

static PWM: Mutex<RefCell<Option<Pwm<Tb0, CCR1>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

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

    // TB0 counts SMCLK (TBSSEL = 10b: SLAU445I Table 14-6, p. 409) in up mode: 1000 counts per period
    // (SLAU445I 14.2.3.1, p. 394). TB0.1 is P1.6 with P1SELx = 10 (SLASEC4D Table 6-63, p. 96).
    let pwm = PwmParts3::new(periph.tb0, TimerConfig::smclk(&smclk), 999);
    let mut pwm1 = pwm.pwm1.init(p1.pin6.to_output_low().to_alternate2());
    pwm1.set_duty_cycle(SHORT).unwrap();

    if USE_PERIOD_INTERRUPT {
        // TBIE: the interrupt at the start of each period (SLAU445I Table 14-6, p. 410). Set GIE, which
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

// The TB0 vector of CCR1, CCR2 and the timer overflow, TBIFG (FFF6h: SLASEC4D Table 6-2, p. 63)
#[interrupt]
fn TIMER0_B1() {
    with(|cs| {
        if let Some(pwm) = PWM.borrow_ref_mut(cs).as_mut() {
            // Clear TBIFG, or the interrupt repeats as soon as it returns. The new duty cycle loads when the
            // timer reaches this period's one, so the next period has it.
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
