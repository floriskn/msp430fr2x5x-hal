//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Center-aligned PWM on TA0, with one output active low, as for the two switches of a half bridge.
//!
//! TA0 counts up to 100 and back down (up/down mode), from SMCLK at 1 MHz, so a PWM period is 200 µs
//! (5 kHz). The pulses are centered on the moments the timer passes 0:
//! - P1.1 (TA0.1), duty 40, active high: high for 80 µs of each period. P1.1 is also LED2's pin, so LED2
//!   glows.
//! - P1.2 (TA0.2), duty 44, active low: low for 88 µs, around the high pulse of P1.1, and high for the
//!   rest of the period. The two outputs are never high together, with 4 µs between one going low and
//!   the other going high: the dead time.
//!
//! Each press of button S1 widens both pulses by 10 µs, up to a high pulse of 190 µs on P1.1, then
//! starts again at 10 µs. A Timer_A has no compare latches: a new duty cycle takes effect at once, also in
//! the middle of a pulse, where a toggling output can then miss its toggle or toggle twice, and stay out of
//! step until the timer next reaches the top, which resets or sets it. So for each change both pins go to
//! their GPIO level, low, for two periods.
//! (Center-aligned PWM and the dead time: SLAU445I 13.2.3.5 and Figure 13-9, p. 374. Up/down mode:
//! SLAU445I 13.2.3.4, p. 373. Updating TAxCCRn, and the toggle/reset and toggle/set output modes: SLAU445I
//! 13.2.4.2 and Table 13-2, p. 376. Only Timer_B's compare registers "are double-buffered": SLAU445I
//! 14.1.1, p. 391. TA0 outputs: SLASE59F Table 6-11, p. 50. S1 is P2.3, with no pull-up on the board, and
//! LED2 is P1.1: SLAU739 Figure 18, p. 23.)
//!
//! How to test (scope, two channels; ground clips on GND, J2 pin 20):
//! 1. CH1 on P1.1 (J2 pin 19), CH2 on P1.2 (J1 pin 10). Trigger on CH1, rising.
//! 2. Flash this example. CH1: 80 µs high pulses every 200 µs. CH2: 88 µs low pulses, centered on CH1's
//!    pulses. Zoom in on the edges: CH2 goes low 4 µs before CH1 goes high, and goes high 4 µs after CH1
//!    goes low.
//! 3. Press S1 a few times: both pulses widen, and CH1 and CH2 are still never high together. Each
//!    change leaves both low for about 0.4 ms.
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*, pwm::SetDutyCycle};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    pwm::{Polarity, PwmParts3, TimerConfig},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The timer counts from 0 up to this and back down, so a period is twice this many SMCLK cycles
const PERIOD: u16 = 100;
/// The dead time, in timer counts: P1.2's low pulse is this much longer on each side
const DEAD_TIME: u16 = 4;
/// The first duty cycle of P1.1, and the step for each press of S1, in timer counts on each side of 0
const DUTY: u16 = 40;
const STEP: u16 = 5;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // S1 pulls P2.3 low, with the internal pullup on (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut s1 = p2.pin3;

    // MCLK = DCOCLKDIV in the 8 MHz range, 244 × 32.768 kHz, and SMCLK = MCLK / 8, 999.4 kHz, for the
    // timer: the 1 MHz range, 32 × 32.768 kHz, is 5 % faster (SLAU445I 3.2.5, p. 104). ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA0 counts SMCLK (TASSEL = 10b: SLASE59F Table 6-7, p. 46) in up/down mode
    let pwm = PwmParts3::new_center_aligned(periph.ta0, TimerConfig::smclk(&smclk), PERIOD);
    // TA0.1 is P1.1 and TA0.2 is P1.2, with P1SELx = 10 and P1DIR = 1 (SLASE59F Table 6-11, p. 50;
    // SLASE59F Table 6-17, p. 55). Their GPIO level is low.
    let mut high_side = pwm.pwm1.init(p1.pin1.to_output_low().to_alternate2());
    let mut low_side = pwm.pwm2.init(p1.pin2.to_output_low().to_alternate2());

    // The pins stay at their GPIO level, low, while the duty cycles change: see the loop
    high_side.disable();
    low_side.disable();
    // The low side is active low: its output is low for its duty cycle (output mode toggle/set instead
    // of toggle/reset: SLAU445I Table 13-2, p. 376)
    low_side.set_polarity(Polarity::ActiveLow);

    let mut duty = DUTY;
    loop {
        // A Timer_A takes a new duty cycle at once: the HAL stops the timer, as SLAU445I 13.2.4.2, p. 376
        // asks, and writes TA0CCRn. If the count is between the old and the new value, the output misses
        // a toggle or toggles twice, and stays out of step until the timer counts to TA0CCR0, where
        // toggle/reset resets it and toggle/set sets it (SLAU445I Table 13-2, p. 376). Two PWM periods of
        // 200 µs reach TA0CCR0 at least once, so the outputs are in step again when the pins switch back.
        high_side.set_duty_cycle(duty).unwrap();
        low_side.set_duty_cycle(duty + DEAD_TIME).unwrap();
        delay.delay_us(400);
        low_side.enable();
        high_side.enable();

        // Wait for a press and release of S1, and for the bouncing to stop
        while s1.is_high().unwrap() {}
        while s1.is_low().unwrap() {}
        delay.delay_ms(20);

        duty = if duty + STEP + DEAD_TIME < PERIOD { duty + STEP } else { STEP };
        high_side.disable();
        low_side.disable();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
