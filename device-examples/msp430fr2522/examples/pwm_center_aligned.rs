//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Center-aligned PWM on TA0, with one output active low, as for the two switches of a half bridge.
//!
//! TA0 counts up to 100 and back down (up/down mode), from SMCLK at 1 MHz, so a PWM period is 200 µs
//! (5 kHz). The pulses are centered on the moments the timer passes 0:
//! - P1.4 (TA0.1), duty 40, active high: high for 80 µs of each period.
//! - P1.5 (TA0.2), duty 44, active low: low for 88 µs, around the high pulse of P1.4, and high for the
//!   rest of the period. The two outputs are never high together, with 4 µs between one going low and
//!   the other going high: the dead time.
//!
//! Each press of a button on P2.3 widens both pulses by 10 µs, up to a high pulse of 190 µs on P1.4, then
//! starts again at 10 µs. A Timer_A takes a new duty cycle at once, so one written while the count is
//! between the old and the new value misses its edge, and that output is wrong until the timer reaches the
//! top, where it is reset or set: both outputs could be high together. So the pins go to their GPIO level,
//! low, while the duty cycles change, and back to the timer two periods later.
//! (Center-aligned PWM and the dead time: SLAU445I 13.2.3.5 and Figure 13-9, p. 374. Up/down mode:
//! SLAU445I 13.2.3.4, p. 373. A new TAxCCRn value: SLAU445I 13.2.4.2, p. 376. Output modes toggle/reset and
//! toggle/set: SLAU445I Table 13-2, p. 376. TA0.1 is P1.4 and TA0.2 is P1.5: SLASEE4C Table 6-15, p. 58.
//! P2.3 is a GPIO input with its pullup: SLASEE4C Table 6-16, p. 60; SLAU445I Table 8-1, p. 313. P2.3 only
//! exists on the 20-pin RHL package: SLASEE4C Table 4-2, p. 14. No board document covers the parts to
//! connect: there is none for the MSP430FR25x2.)
//!
//! How to test (scope, two channels, and a push button):
//! 1. Connect a push button from P2.3 to GND (the internal pullup is on).
//! 2. CH1 on P1.4, CH2 on P1.5, ground clips on GND. Trigger on CH1, rising.
//! 3. Flash this example. CH1: 80 µs high pulses every 200 µs. CH2: 88 µs low pulses, centered on CH1's
//!    pulses. Zoom in on the edges: CH2 goes low 4 µs before CH1 goes high, and goes high 4 µs after CH1
//!    goes low.
//! 4. Press the button a few times: both pulses widen, and CH1 and CH2 are still never high together. At
//!    each press both stay low for two periods.
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
/// The dead time, in timer counts: P1.5's low pulse is this much longer on each side
const DEAD_TIME: u16 = 4;
/// The first duty cycle of P1.4, and the step for each press of the button, in timer counts on each side
/// of 0
const DUTY: u16 = 40;
const STEP: u16 = 5;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The button pulls P2.3 low, against the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut button = p2.pin3;

    // MCLK = DCOCLKDIV in the 8 MHz range, 244 × 32.768 kHz, and SMCLK = MCLK / 8, 999.4 kHz, for the
    // timer: the 1 MHz range, 32 × 32.768 kHz, is 5 % faster (SLAU445I 3.2.5, p. 104). ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TA0 counts SMCLK (TASSEL = 10b: SLAU445I Table 13-4, p. 384) in up/down mode
    let pwm = PwmParts3::new_center_aligned(periph.ta0, TimerConfig::smclk(&smclk), PERIOD);
    // TA0.1 is P1.4 and TA0.2 is P1.5, with P1SELx = 10 (SLASEE4C Table 6-15, p. 58). Their GPIO level is
    // low.
    let mut high_side = pwm.pwm1.init(p1.pin4.to_output_low().to_alternate2());
    let mut low_side = pwm.pwm2.init(p1.pin5.to_output_low().to_alternate2());
    // The low side is active low: its output is low for its duty cycle (output mode toggle/set instead
    // of toggle/reset: SLAU445I Table 13-2, p. 376)
    low_side.set_polarity(Polarity::ActiveLow);

    let mut duty = DUTY;
    loop {
        // The pins at their GPIO level while the duty cycles change. Each is written at once (SLAU445I
        // 13.2.4.2, p. 376), and an output that missed an edge is right again from the top on, where it's
        // reset or set (SLAU445I Table 13-2, p. 376): two periods of 200 µs give the timer time to get there.
        high_side.disable();
        low_side.disable();
        high_side.set_duty_cycle(duty).unwrap();
        low_side.set_duty_cycle(duty + DEAD_TIME).unwrap();
        delay.delay_us(400);
        low_side.enable();
        high_side.enable();

        // Wait for a press and release of the button, and for the bouncing to stop
        while button.is_high().unwrap() {}
        while button.is_low().unwrap() {}
        delay.delay_ms(20);

        duty = if duty + STEP + DEAD_TIME < PERIOD { duty + STEP } else { STEP };
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
