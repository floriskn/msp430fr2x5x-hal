//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum TB25, and of how the HAL's edge-aligned PWM avoids it: in up mode, a Timer_B compare
//! latch set to load at the start of a period loads at once instead, so a shorter duty cycle written late
//! in a period leaves that period high to its end. TB0 runs PWM on P4.3, and the program switches its duty
//! cycle between 25 % and 75 % at random moments. Through a jumper wire, TA0 captures each rising edge of
//! the PWM signal. They must be one period apart: a period left high runs into the next one without a
//! rising edge. Every 1024 changes LED1 toggles and the backchannel UART prints the counts.
//!
//! The erratum: "IF Timer B is configured for Up mode, AND the compare latch load event (TBxCCTLn.CLLD bits)
//! setting is configured to update TBxCCRn when TBxR reaches 0, THEN TBxCCRn will update immediately
//! instead of the described condition" (SLAZ726B TB25, p. 8). Its workaround loads at once (CLLD = 00b) and
//! writes the duty cycle in the timer's interrupt, at the start of each period. The HAL's edge-aligned PWM
//! loads the latch "when TBxR counts to the old TBxCLn value" instead (CLLD = 11b: SLAU445I Table 14-2,
//! p. 400), which the erratum doesn't name: each period ends at its old duty cycle, and the next one has
//! the new one, whenever the duty cycle is written (see the HAL's `pwm` module).
//! (TB0 and TA0 count SMCLK, about 1 MHz, so a PWM period of 1000 counts is 1000 counts of TA0 too. TB0.5 is
//! P4.3 with P4SEL = 10: SLASEO7C Table 9-15, p. 59; SLASEO7C Table 9-26, p. 68. TA0.CCI2A is P1.2 with
//! P1SEL = 10: SLASEO7C Table 9-12, p. 55; SLASEO7C Table 9-23, p. 65. Up mode: SLAU445I 14.2.3.1, p. 394.
//! Reset/set mode: SLAU445I Table 14-4, p. 401. Captures: SLAU445I 13.2.4.1, p. 374. LED1 on P1.0 is green:
//! SLAU802 Figure 19, p. 25.)
//!
//! How to test (a jumper wire):
//! 1. Connect P4.3 (J3 pin 24) to P1.2 (J1 pin 10) with a jumper wire.
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 3. Expected: about once a second a line such as `1076 periods, 0 left high`, the periods measured so far
//!    and how many of them stayed high to the end. LED1 toggles at each line.
//! 4. Pass: `0 left high` for a few minutes, some hundred thousand periods. Fail: that count goes up.
//!    Without the jumper wire, no periods are measured.
//! 5. To check that the test can catch the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the
//!    duty cycle then loads when the timer counts to 0, CLLD = 01b, which the erratum turns into loading at
//!    once, and the `left high` count goes up at every line.
//! (Header pins: SLAU802 Figure 10, p. 13.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*, pwm::SetDutyCycle};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    capture::{CapTrigger, Capture, CaptureParts3, CaptureVector, TBxIV, CCR2},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    pwm::{PwmParts7, TimerConfig},
    serial::*,
    watchdog::Wdt,
};
use msp430fr247x::{interrupt, Ta0, Tb0};
use panic_msp430 as _;

/// `false` sets the compare latch to load when the timer counts to 0, which the erratum breaks
const USE_HAL_WORKAROUND: bool = true;

/// The PWM period, and the two duty cycles, in SMCLK cycles
const PERIOD: u16 = 1000;
const SHORT: u16 = 250;
const LONG: u16 = 750;

static CAPTURE: Mutex<RefCell<Option<Capture<Ta0, CCR2>>>> = Mutex::new(RefCell::new(None));
static VECTOR: Mutex<RefCell<Option<TBxIV<Ta0>>>> = Mutex::new(RefCell::new(None));
/// The last rising edge's capture, if the one before it wasn't missed
static LAST: Mutex<Cell<Option<u16>>> = Mutex::new(Cell::new(None));
/// The periods measured, and those of them that weren't one period long
static PERIODS: Mutex<Cell<(u32, u32)>> = Mutex::new(Cell::new((0, 0)));

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = DCOCLKDIV in the 8 MHz range, so the interrupt handler runs quickly, SMCLK = MCLK / 8 for the
    // timers, and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I
    // Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9; SLASEO7C
    // Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p1.pin4.to_alternate1());

    // TB0 counts SMCLK in up mode, PERIOD counts per period (TBSSEL = 10b: SLAU445I Table 14-6, p. 409;
    // SLAU445I 14.2.3.1, p. 394), with TB0.5 on P4.3
    let pwm = PwmParts7::new(periph.tb0, TimerConfig::smclk(&smclk), PERIOD - 1);
    let mut pwm5 = pwm.pwm5.init(p4.pin3.to_output_low().to_alternate2());
    pwm5.set_duty_cycle(SHORT).unwrap();
    if !USE_HAL_WORKAROUND {
        // CLLD = 01b, load when TB0R counts to 0 (SLAU445I Table 14-2, p. 400; SLAU445I Table 14-8, p. 411)
        let tb0 = unsafe { &*Tb0::ptr() };
        tb0.tb0cctl5().modify(|_, w| w.clld().clld_1());
    }

    // TA0 counts SMCLK in continuous mode, and CCR2 captures the rising edges on P1.2 (TASSEL = 10b, CM =
    // 01b: SLAU445I Table 13-4, p. 384; SLAU445I Table 13-6, p. 386). CCIE requests its interrupt for each
    // capture (SLAU445I Table 13-6, p. 386).
    let captures = CaptureParts3::config(periph.ta0, TimerConfig::smclk(&smclk))
        .config_cap2_input_A(p1.pin2.to_alternate2())
        .config_cap2_trigger(CapTrigger::RisingEdge)
        .commit();
    let mut capture = captures.cap2;
    capture.enable_interrupts();
    with(|cs| {
        CAPTURE.borrow_ref_mut(cs).replace(capture);
        VECTOR.borrow_ref_mut(cs).replace(captures.tbxiv);
    });
    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    let mut random: u16 = 1;
    let mut changes: u32 = 0;
    let mut duty = SHORT;
    loop {
        // Wait a random 0 to 2047 µs, so the change comes at any point of a period
        random = next_random(random);
        delay.delay_us((random & 0x7FF) as u32);

        duty = if duty == SHORT { LONG } else { SHORT };
        pwm5.set_duty_cycle(duty).unwrap();
        changes += 1;

        if changes % 1024 == 0 {
            led1.toggle().ok();
            let (periods, left_high) = with(|cs| PERIODS.borrow(cs).get());
            writeln!(tx, "{} periods, {} left high\r", periods, left_high).ok();
        }
    }
}

/// The next number of a xorshift generator, for the random waits
fn next_random(x: u16) -> u16 {
    let x = x ^ (x << 7);
    let x = x ^ (x >> 9);
    x ^ (x << 8)
}

// The TA0 vector of CCR1, CCR2 and the timer overflow (FFF6h: SLASEO7C Table 9-2, p. 46). Reading TA0IV gives
// the highest pending source and clears its flag (SLAU445I 13.2.6.2, p. 380).
#[interrupt]
fn TIMER0_A1() {
    with(|cs| {
        let mut vector = VECTOR.borrow_ref_mut(cs);
        let mut capture = CAPTURE.borrow_ref_mut(cs);
        let (Some(vector), Some(capture)) = (vector.as_mut(), capture.as_mut()) else { return };
        if let CaptureVector::Capture2(token) = vector.interrupt_vector() {
            let last = LAST.borrow(cs);
            match token.interrupt_capture(capture) {
                Ok(count) => {
                    if let Some(previous) = last.get() {
                        // Both timers count the same clock, so a period is PERIOD counts, give or take one
                        let (periods, left_high) = PERIODS.borrow(cs).get();
                        let wrong = count.wrapping_sub(previous).abs_diff(PERIOD) > 1;
                        PERIODS.borrow(cs).set((periods + 1, left_high + wrong as u32));
                    }
                    last.set(Some(count));
                }
                // An edge was missed (COV), so this capture can't be compared with the last one
                Err(_) => last.set(None),
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
