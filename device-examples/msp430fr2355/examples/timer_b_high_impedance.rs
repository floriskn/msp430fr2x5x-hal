//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Switching all TB0 outputs to high impedance with the TB0TRG pin, P1.2: while P1.2 is high, the PWM on
//! the outputs stops, for example to stop a motor driver on a fault.
//!
//! TB0 runs PWM at 1 kHz, 50 %, on both its outputs, P1.6 (TB0.1) and P1.7 (TB0.2). P1.2 has its
//! pulldown on, so with nothing connected it's low and the PWM runs. Pulling P1.2 high makes all TB0
//! outputs high impedance: both pins stop switching. Once P1.2 is low again, the PWM continues. Neither
//! LED is on a timer output on this LaunchPad, so the scope shows the outputs.
//! ("When the TBOUTH pin function is selected for the pin ... and when the pin is pulled high, all
//! Timer_B outputs are in a high-impedance state": SLAU445I 14.2.5, p. 401. TB0TRGSEL = 1 selects P1.2,
//! and the outputs it switches are P1.6 and P1.7: SLASEC4D Table 6-20, p. 76. TB0 outputs: SLASEC4D
//! Table 6-16, p. 73. The LEDs are on P1.0 and P6.6: SLAU680 Figure 18, p. 26. Header pins: SLAU680
//! Figure 10, p. 15.)
//!
//! After reset, eCOMP0's output is the trigger instead, so a TB0 PWM stops whenever eCOMP0's output is
//! high (TB0TRGSEL = 0: SLASEC4D Table 6-20, p. 76; SLAU445I Table 1-26, p. 77). `HighImpedanceTrigger::None`
//! switches the trigger off.
//!
//! How to test (a jumper wire, and the scope):
//! 1. Flash this example. Probe P1.6 (J1 pin 3) and P1.7 (J1 pin 4) with two channels of the scope, ground
//!    clips on GND (J3 pin 22): a 1 kHz square wave on each.
//! 2. Connect P1.2 (J1 pin 10) to 3.3 V (J1 pin 1) with a jumper wire: both outputs stop switching. The
//!    pins float now, so the scope may show a flat line at any level.
//! 3. Remove the wire: the square waves are back.
//!
//! Instead of the wire, the function generator can drive P1.2: a 1 Hz square wave from 0 V to 3.3 V
//! (output load High-Z; check the levels on the scope first). The PWM then stops every other half
//! second.
#![no_main]
#![no_std]

use embedded_hal::pwm::SetDutyCycle;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    timer::HighImpedanceTrigger,
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

    // MCLK = DCOCLKDIV in the 8 MHz range, 244 × 32.768 kHz, and SMCLK = MCLK / 8, 999.4 kHz, for the
    // timer: the 1 MHz range, 32 × 32.768 kHz, is 5 % faster (SLAU445I 3.2.5, p. 104). ACLK from REFO
    // (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // P1.2 is TB0TRG with P1SELx = 10 and P1DIR = 0 (SLASEC4D Table 6-63, p. 96), with its pulldown on
    // (PxREN = 1, PxOUT = 0: SLAU445I Table 8-1, p. 313), so it's low while nothing drives it
    let trigger = p1.pin2.pulldown().to_alternate2();

    // TB0 counts SMCLK (TBSSEL = 10b: SLAU445I Table 14-6, p. 409) in up mode: 1000 counts per period
    // (SLAU445I 14.2.3.1, p. 394). TB0TRGSEL = 1 selects P1.2 as the trigger (SLAU445I Table 1-26, p. 77).
    let config = TimerConfig::smclk(&smclk).high_impedance_trigger(HighImpedanceTrigger::Pin(&trigger));
    let pwm = PwmParts3::new(periph.tb0, config, 999);
    // TB0.1 is P1.6 and TB0.2 is P1.7, with P1SELx = 10 (SLASEC4D Table 6-63, p. 96)
    let mut out1 = pwm.pwm1.init(p1.pin6.to_output_low().to_alternate2());
    let mut out2 = pwm.pwm2.init(p1.pin7.to_output_low().to_alternate2());
    out1.set_duty_cycle(500).unwrap();
    out2.set_duty_cycle(500).unwrap();

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
