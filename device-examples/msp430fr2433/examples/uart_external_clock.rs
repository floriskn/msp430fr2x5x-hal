//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The backchannel UART clocked from its UCA0CLK pin, by the function generator: twice a second it prints
//! a line at 9600 baud, a baud rate worked out from the 1 MHz the clock is said to have. With another
//! frequency on the pin, the baud rate is off by as much, and the text comes out garbled.
//!
//! UCSSELx = 00b selects the clock on the UCA0CLK pin, P1.6. The eUSCI takes up to 16 MHz there, with a duty
//! cycle of 50 % ± 10 %.
//! (UCA0CLK pin: SLASE59F Table 6-7, p. 46; P1.6 is UCA0CLK and P1.4 UCA0TXD with P1SELx = 01: SLASE59F
//! Table 6-17, p. 55; external clock: SLASE59F Table 5-14, p. 30. The backchannel UART is eUSCI_A0:
//! SLAU739 2.2.4, p. 9.)
//!
//! How to test (function generator, and optionally the scope):
//! 1. Generator: square wave, 1 MHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), duty cycle 50 %, output load
//!    High-Z. Check the levels on the scope before connecting: a negative or >3.6 V signal can damage the
//!    pin.
//! 2. Connect the generator to P1.6 (J1 pin 5), its ground to GND (J3 pin 22), and switch it on.
//! 3. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 4. Expected, twice a second: `9600 baud from 1 MHz on UCA0CLK`.
//! 5. Set the generator to 1.1 MHz: the lines come garbled, as the baud rate is now 10 % too high.
//! 6. Set it to 2 MHz, and open the COM port at 19200 baud instead: the lines are readable again, as the
//!    baud rate doubled with the clock.
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The frequency of the generator's clock on UCA0CLK
const UCLK_HZ: u32 = 1_000_000;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The UART doesn't use them.
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART, eUSCI_A0, with TXD on P1.4, 8N1, clocked from UCA0CLK on P1.6 (UCSSELx = 00b:
    // SLAU445I Table 22-8, p. 593). Both pins have P1SELx = 01 (SLASE59F Table 6-17, p. 55). The baud-rate
    // divider is worked out from UCLK_HZ (SLAU445I 22.3.10, p. 586).
    let mut tx = SerialConfig::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_uclk(p1.pin6.to_alternate1(), UCLK_HZ)
    .tx_only(p1.pin4.to_alternate1());

    loop {
        writeln!(tx, "9600 baud from 1 MHz on UCA0CLK\r").ok();
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
