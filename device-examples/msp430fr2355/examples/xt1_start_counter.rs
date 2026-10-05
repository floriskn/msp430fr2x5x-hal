//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1's start counter in crystal mode: with ENSTFCNT1 set, `freeze()` only returns once the crystal has
//! run cleanly for 1024 cycles, 31 ms at 32.768 kHz. LED1 lights red, and P5.4 goes high, when `freeze()`
//! returns.
//!
//! The LaunchPad's crystal takes about a second to start, so to see the counter's share the example turns
//! off XT1's fault switch, XT1FAULTOFF, which only the enhanced clock system of the MSP430FR2355 has: ACLK on
//! P1.1 then shows XT1CLK from the crystal's first cycles, instead of REFO until the fault has cleared. The
//! data sheet gives 1024 cycles, and the family user's guide 8192 (250 ms); on an MSP430FR2476, in bypass
//! mode, 1024 were measured.
//! (tSTART,LFXT, 1000 ms typical, "Includes startup counter of 1024 clock cycles": SLASEC4D Table 5-3,
//! note 8, p. 35. 8192: SLAU445I 3.2.13, p. 110. ENSTFCNT1: SLAU445I Table 3-11, p. 121. XT1FAULTOFF:
//! SLAU445I 3.2.13, p. 110; SLAU445I Table 3-1, p. 98. The crystal Q1 is on XIN, P2.7, and XOUT, P2.6, and
//! LED1 on P1.0 is red: SLAU680 Figure 18, p. 26.)
//!
//! How to test (the scope):
//! 1. Flash this example. Expected: about a second after reset, LED1 lights red.
//! 2. Scope, ground clip on GND (J3 pin 22): CH1 on ACLK, P1.1 (J3 pin 28), CH2 on P5.4 (J3 pin 27).
//!    Single-shot trigger on CH2's rising edge, 50 ms/div, trigger point near the right of the screen.
//!    Press the reset button S3. Expected: ACLK is flat while the crystal starts, then runs at 32.768 kHz
//!    for about 31 ms (1024 cycles) before P5.4 rises. 250 ms would mean the user guide's 8192.
//! 3. Set `START_COUNTER` to false and flash again: P5.4 now rises much sooner after ACLK starts.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Frequency of the LaunchPad's crystal Q1 (SLAU680 2.5, p. 13)
const XT1_FREQ_HZ: u32 = 32_768;
/// Whether `freeze()` waits for the 1024-cycle start counter
const START_COUNTER: bool = true;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p5 = Batch::new(periph.p5)
        .config_pin4(|p| p.to_output())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut done = p5.pin4;
    led.set_low().ok();
    done.set_low().ok();

    // ACLK on P1.1 with P1SEL = 10 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96); XIN on P2.7 and XOUT on
    // P2.6, each with P2SEL = 10 (SLASEC4D Table 6-64, p. 98). P5.4 is a general-purpose I/O (SLASEC4D
    // Table 6-67, p. 104).
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin7.to_alternate2();
    let xout = p2.pin6.to_alternate2();

    // XT1 in crystal mode (XT1BYPASS = 0: SLAU445I Table 3-10, p. 120), with the start counter
    // (ENSTFCNT1 = 1: SLAU445I Table 3-11, p. 121) unless `START_COUNTER` is false, and without the switch
    // of ACLK to REFO on a fault (XT1FAULTOFF = 1: SLAU445I Table 3-10, p. 119)
    let xt1 = Xt1Config::crystal(XT1_FREQ_HZ, xin, xout).disable_fault_switch();
    let xt1 = if START_COUNTER { xt1 } else { xt1.disable_start_counter() };

    // MCLK = SMCLK = DCOCLKDIV (SELMS = 000b) and ACLK from XT1CLK (SELA = 00b) (SLAU445I Table 3-8,
    // p. 117)
    let (_smclk, _aclk, _xt1clk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(xt1)
        .aclk_xt1clk()
        .freeze(&mut fram);

    led.set_high().ok();
    done.set_high().ok();

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
