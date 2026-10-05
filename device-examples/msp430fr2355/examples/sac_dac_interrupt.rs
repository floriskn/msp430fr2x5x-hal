//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The DAC of the Smart Analog Combo SAC0, loaded by a timer and refilled from its interrupt: its output on
//! P1.1 climbs a staircase of 8 steps, a step every half second, from 0 V to about 2.9 V, and starts again
//! at 0 V. LED1 toggles at each step.
//!
//! TB2 counts ACLK (REFO, 32.768 kHz) with a period of 0.5 s, and its CCR1 output, TB2.1, rises once per
//! period. Each rise loads the value waiting in the DAC's data register, and each load requests the
//! SAC0_SAC2 interrupt, whose handler writes the next step and toggles LED1. The DAC gives
//! 3.3 V × count / 4096, with the 3.3 V supply as its reference, and SAC0's op-amp, as a buffer, drives
//! that onto its output OA0O, P1.1. The LaunchPad's light sensor uses SAC2, not SAC0, so its jumpers J7 to
//! J9 can stay on.
//! (SACs are on the MSP430FR235x only: SLASEC4D 6.10.15, p. 79. REFO: SLASEC4D Table 5-7, p. 40. TB2.1 is
//! a DAC load trigger, DACLSEL = 10b: SLASEC4D Table 6-18, p. 74; SLASEC4D Table 6-32, p. 80. The DAC
//! loads "on the rising edge" of its trigger and then requests its interrupt: SLAU445I 20.2.3.4 and
//! 20.2.3.5, p. 529; the vector: SLASEC4D Table 6-2, p. 64. The DAC's output: SLAU445I Table 20-3,
//! p. 529; its reference, DVCC: SLASEC4D Table 6-31, p. 80. The supply: SLAU680 2.3.1, p. 12. SAC0's
//! pins: SLASEC4D Table 6-27, p. 79; SLASEC4D Table 6-63, p. 96. The light sensor, on SAC2's pins P3.1 to
//! P3.3 through J7 to J9: SLAU680 2.2.5.1, p. 11. LED1 on P1.0 is red: SLAU680 Figure 18, p. 26.)
//!
//! How to test (the multimeter, or the scope):
//! 1. Flash this example.
//! 2. Expected: LED1 (red) toggles every half second.
//! 3. The multimeter on DC volts, its probe on P1.1 (J3 pin 28) and COM on GND (J3 pin 22): the reading
//!    climbs by about 0.41 V every half second, 0 V, 0.41 V, 0.83 V and so on to 2.89 V, then drops back
//!    to 0 V.
//! 4. Or the scope on P1.1, 1 V/div and 500 ms/div: a staircase of 8 steps that repeats every 4 s.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::digital::*;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, Output, Pin, Pin0, P1},
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    sac::{BufferInput, Dac, LoadTrigger, PowerMode, SacConfig, VRef},
    watchdog::Wdt,
};
use msp430fr2355::{interrupt, Sac0};
use panic_msp430 as _;

/// ACLK cycles per TB2 period, so per step: 0.5 s (ACLK is REFO, 32768 Hz: SLASEC4D Table 5-7, p. 40)
const ACLK_CYCLES: u16 = 16_384;
/// The steps of the staircase, and the DAC counts per step: 8 × 512 is the DAC's 4096 counts
const STEPS: u16 = 8;
const STEP_COUNTS: u16 = 512;

static DAC: Mutex<RefCell<Option<Dac<'static, Sac0>>>> = Mutex::new(RefCell::new(None));
static LED1: Mutex<RefCell<Option<Pin<P1, Pin0, Output>>>> = Mutex::new(RefCell::new(None));
/// The step waiting in the DAC's data register for the next trigger
static WAITING_STEP: Mutex<Cell<u16>> = Mutex::new(Cell::new(1));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let led1 = p1.pin0.to_output_low();
    // OA0O on P1.1, P1SELx = 11 (SLASEC4D Table 6-63, p. 96)
    let p1_1 = p1.pin1.to_alternate3();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // TB2 counts ACLK (TBSSEL = 01b: SLAU445I Table 14-6, p. 409) in up mode, ACLK_CYCLES counts per
    // period (SLAU445I 14.2.3.1, p. 394). Its CCR1 output, TB2.1, becomes the DAC trigger: in reset/set
    // mode with CCR1 = 1 it rises once per period (SLAU445I Table 14-4, p. 401).
    let pwm = PwmParts3::new(periph.tb2, TimerConfig::aclk(&aclk), ACLK_CYCLES - 1);
    let trigger = pwm.pwm1.into_dac_trigger();

    // SAC0's DAC with DVCC as its reference (DACSREF = 0: SLASEC4D Table 6-31, p. 80), loading on the
    // rising edges of TB2.1 (DACLSEL = 10b: SLASEC4D Table 6-32, p. 80), with its interrupt (DACIE:
    // SLAU445I Table 20-8, p. 534). The first step waits in its data register for the first edge.
    let (dac_config, amp_config) = SacConfig::begin(periph.sac0);
    let mut dac = dac_config.configure_with_interrupts(VRef::Vcc, LoadTrigger::TB2_1(&trigger));
    dac.set_count(STEP_COUNTS);

    // SAC0's op-amp as a buffer of the DAC, driving OA0O (MSEL = 01b and PSEL = 01b: SLAU445I 20.2.2.3,
    // p. 524; SLASEC4D Table 6-27, p. 79)
    let _amp = amp_config.buffer(BufferInput::Dac(&dac), PowerMode::LowPower).output_pin(p1_1);

    with(|cs| {
        DAC.borrow_ref_mut(cs).replace(dac);
        LED1.borrow_ref_mut(cs).replace(led1);
    });
    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    loop {
        msp430::asm::nop();
    }
}

// The SAC0 and SAC2 vector, DACIFG (FFD8h: SLASEC4D Table 6-2, p. 64)
#[interrupt]
fn SAC0_SAC2() {
    with(|cs| {
        let mut dac = DAC.borrow_ref_mut(cs);
        let mut led1 = LED1.borrow_ref_mut(cs);
        let (Some(dac), Some(led1)) = (dac.as_mut(), led1.as_mut()) else { return };
        // Reading SAC0IV clears DACIFG, the interrupt request. The DAC set it when it loaded the waiting
        // step: "A set DACIFG bit indicates that the DAC is ready for new data" (SLAU445I 20.2.3.5, p. 529).
        if dac.data_loaded() {
            let waiting = WAITING_STEP.borrow(cs);
            let step = (waiting.get() + 1) % STEPS;
            dac.set_count(step * STEP_COUNTS);
            waiting.set(step);
            led1.toggle().ok();
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
