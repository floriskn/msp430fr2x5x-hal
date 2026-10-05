//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! XT1 in high-frequency mode, at 4 MHz: MCLK and SMCLK run from it, ACLK from it divided by 128, and LED2
//! blinks, timed by a delay loop on MCLK. When XT1 fails, LED1 lights and the clocks keep running from
//! their fail-safe sources; when XT1 runs again, they go back to it and LED1 turns off. The LaunchPad's
//! crystal is a 32.768-kHz one, so this needs a board change, and a 4-MHz clock signal or crystal.
//!
//! With XTS = 1, XT1 takes a crystal or a clock signal of 1 MHz to 24 MHz, in four ranges set by XT1HFFREQ;
//! 4 MHz is in the lowest. ACLK can't run faster than 40 kHz, so for ACLK the HAL divides XT1 by DIVA, with
//! the divider that lands closest to 32.768 kHz: 128, for 31.25 kHz. `CRYSTAL` picks a clock signal on XIN
//! (`Xt1Config::bypass_hf`, the default) or a crystal on XIN and XOUT (`Xt1Config::crystal_hf`). MCLK comes
//! out on P3.0 and ACLK on P1.1.
//!
//! When XT1 fails in high-frequency mode, the fail-safe switches MCLK and SMCLK to the DCO, DCOCLKDIV, and
//! ACLK to REFO, and keeps them there until the fault flags are cleared with XT1 running again. The HAL
//! references the FLL to REFO for that DCO, so it runs at 32 x 32.768 kHz, 1.048576 MHz, and the delay loop
//! takes about 3.8 times as long. The main loop clears the flags at each blink, and LED1 shows whether the
//! fault came back.
//! (XTS, XT1HFFREQ and DIVA: SLAU445I Table 3-10, p. 119 to p. 120. ACLK "must be approximately 32 kHz and
//! no faster than 40 kHz": SLAU445I 3.2.4, p. 103. 1 MHz to 24 MHz, and a 40 % to 60 % duty cycle for a
//! clock signal: SLASEC4D Table 5-4, p. 36. The fail-safe: SLAU445I 3.2.13, p. 109 to p. 110. DCOCLKDIV =
//! (FLLN + 1) x REFOCLK, with FLLN = 31: SLAU445I 3.2.5, p. 104. REFO runs at 32.768 kHz ±3.5 %: SLASEC4D
//! Table 5-7, p. 40. XIN on P2.7 and XOUT on P2.6 with P2SEL = 10: SLASEC4D Table 6-64, p. 98. MCLK on P3.0
//! with P3SEL = 01: SLASEC4D Table 6-65, p. 100. ACLK on P1.1 with P1SEL = 10: SLASEC4D Table 6-63, p. 96.
//! LED1 on P1.0 is red and LED2 on P6.6 green: SLAU680 Figure 18, p. 26.)
//!
//! How to test (a board change, the function generator or a 4-MHz crystal, and the scope):
//! 1. XIN and XOUT connect straight to the 32.768-kHz crystal Q1, with the 12-pF capacitors C3 and C2 to
//!    GND, and aren't on the headers (SLAU680 Figure 18, p. 26).
//!    - For a clock signal (`CRYSTAL` false): solder a wire to XIN, pin 1 of Q1 in the schematic.
//!      Generator: square wave, 4 MHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!      High-Z. Check the levels, and the frequency's unit (MHz), on the scope before connecting: a
//!      negative or >3.6 V signal can damage the pin. Connect it to the wire, its ground to GND (J3
//!      pin 22), and switch the output on.
//!    - For a crystal (`CRYSTAL` true): replace Q1 with a 4-MHz crystal, and C2 and C3 with the capacitors
//!      its data sheet asks for. TI characterized XT1 with the Abracon AB-4.000MHZ-B2 (SLASEC4D Table 5-4,
//!      notes 4 and 8, p. 36).
//! 2. Flash this example. Expected: LED2 blinks, on for 0.5 s and off for 0.5 s. Without the signal or the
//!    crystal, `freeze()` keeps waiting for XT1, and LED2 stays off.
//! 3. Scope with a 10X probe on MCLK, P3.0 (J2 pin 11), ground clip on GND (J3 pin 22): its counter shows
//!    4 MHz. On ACLK, P1.1 (J3 pin 28): 31.25 kHz. LED1 is off.
//! 4. With the clock signal only: switch the generator output off. A static XIN sets the fault flag
//!    (SLASEC4D Table 5-4, note 9, p. 36). Expected: within 2 s LED1 lights, and LED2 blinks about 3.8 times
//!    slower, on for 1.9 s and off for 1.9 s. MCLK on P3.0 is now 1.05 MHz, from the DCO, and ACLK on P1.1
//!    32.8 kHz, from REFO.
//! 5. Switch the generator output back on. Expected: within 2 s LED1 turns off, MCLK is 4 MHz again, ACLK
//!    31.25 kHz, and LED2 blinks every 0.5 s again. If LED1 turns off but MCLK doesn't return to 4 MHz,
//!    it didn't switch back to XT1: please report it.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// XT1's frequency, in XT1HFFREQ's lowest range, 1 MHz to 4 MHz (SLAU445I Table 3-10, p. 120)
const XT1_FREQ_HZ: u32 = 4_000_000;
/// A crystal on XIN and XOUT instead of a clock signal on XIN
const CRYSTAL: bool = false;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    let p6 = Batch::new(periph.p6).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut led2 = p6.pin6.to_output_low();

    // MCLK on P3.0 with P3SEL = 01 and P3DIR = 1 (SLASEC4D Table 6-65, p. 100), ACLK on P1.1 with
    // P1SEL = 10 and P1DIR = 1 (SLASEC4D Table 6-63, p. 96), and XIN on P2.7 with P2SEL = 10 (SLASEC4D
    // Table 6-64, p. 98)
    let _mclk_out = p3.pin0.to_output().to_alternate1();
    let _aclk_out = p1.pin1.to_output().to_alternate2();
    let xin = p2.pin7.to_alternate2();

    // XT1 in high-frequency mode (XTS = 1) at 4 MHz (XT1HFFREQ = 00b), with a crystal (XT1BYPASS = 0) or a
    // clock signal (XT1BYPASS = 1) (SLAU445I Table 3-10, p. 119 to p. 120). MCLK and SMCLK run from XT1CLK,
    // undivided (SELMS = 010b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118), and
    // ACLK from XT1CLK divided by 128 (SELA = 00b: SLAU445I Table 3-8, p. 117; DIVA = 0100b: SLAU445I
    // Table 3-10, p. 119). `freeze()` returns once XT1 runs without a fault. It references the FLL to REFO
    // (SELREF = 01b: SLAU445I Table 3-7, p. 116) for the DCO that MCLK and SMCLK fall back to.
    let (_smclk, _aclk, mut xt1clk, mut delay) = if CRYSTAL {
        // XOUT on P2.6 with P2SEL = 10 (SLASEC4D Table 6-64, p. 98)
        let xout = p2.pin6.to_alternate2();
        ClockConfig::new(periph.cs)
            .xt1clk_on(Xt1Config::crystal_hf(XT1_FREQ_HZ, xin, xout))
            .mclk_xt1clk(MclkDiv::_1)
            .smclk_on(SmclkDiv::_1)
            .aclk_xt1clk()
            .freeze(&mut fram)
    } else {
        ClockConfig::new(periph.cs)
            .xt1clk_on(Xt1Config::bypass_hf(XT1_FREQ_HZ, xin))
            .mclk_xt1clk(MclkDiv::_1)
            .smclk_on(SmclkDiv::_1)
            .aclk_xt1clk()
            .freeze(&mut fram)
    };

    loop {
        // Clear the fault flags: with XT1 running again, the clocks switch back to it, and with XT1 still
        // failing, XT1OFFG is set again straight away (SLAU445I 3.2.13, p. 109 to p. 110; XT1OFFG: SLAU445I
        // Table 3-11, p. 122)
        xt1clk.clear_fault();
        led1.set_state(xt1clk.is_faulted().into()).ok();
        led2.toggle().ok();
        delay.delay_ms(500);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
