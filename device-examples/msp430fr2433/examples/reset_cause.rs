//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Why did the device reset? `Pmm::take_reset_cause` reads the reasons, highest priority first, and the
//! backchannel UART prints the first one, once a second. S1 and S2 reset the device by software when they
//! are released.
//!
//! | Printed                          | Reset                                                         |
//! |----------------------------------|---------------------------------------------------------------|
//! | `Reset cause: Some(Brownout)`    | power-up: plug in the USB cable (brownout reset)              |
//! | `Reset cause: Some(ResetPin)`    | the reset button S3                                           |
//! | `Reset cause: Some(SoftwareBor)` | button S1, which calls `Pmm::software_bor()`                  |
//! | `Reset cause: Some(SoftwarePor)` | button S2, which calls `Pmm::software_por()`                  |
//! | `Reset cause: None`              | no reason: the debugger started the program after flashing it |
//!
//! The board's two single-color LEDs can't show these apart, so the example prints them. Any other reason
//! prints its `ResetCause` name, such as `Some(WatchdogTimeout)`.
//! (SYSRSTIV: SLAU445I 1.3.7, p. 36. The resets, with their priorities: brownout 02h, RST/NMI pin 04h,
//! software BOR 06h, software POR 14h: SLASE59F Table 6-9, p. 48. S1 (P2.3) and S2 (P2.7) pull their pins
//! low and have no pull-ups on the board, and S3 is the reset button on RST: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9). Expected: `Reset cause: None`, once a second.
//! 2. Press S3: `Reset cause: Some(ResetPin)`.
//! 3. Press and release S1: `Reset cause: Some(SoftwareBor)`. Press and release S2:
//!    `Reset cause: Some(SoftwarePor)`.
//! 4. Unplug the USB cable and plug it back in, and open the COM port again: `Reset cause: Some(Brownout)`.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
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

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    // After a BOR the pins stay locked in their reset state until LOCKLPM5 is cleared, and the data
    // sheet asks for the ports to be configured first (SLASE59F 6.10.3, p. 46: "the ports must be
    // configured first and then the LOCKLPM5 bit must be cleared"), so the pins are set below before
    // they are released. Measured on an MSP430FR2476: a software BOR sets LOCKLPM5 again, a software
    // POR and a watchdog PUC leave it clear, and then unlock_lpm5() changes nothing.
    let (mut pmm, _) = Pmm::new_locked(periph.pmm, periph.sys);

    // Read the first reason, then the rest, which also clears them for the next reset
    // (reading SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    // S1 and S2 inputs with their internal pullups (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1,
    // p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .config_pin7(|p| p.pullup())
        .split(&pmm);
    let mut s1 = p2.pin3;
    let mut s2 = p2.pin7;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SELx = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
    // Table 6-17, p. 55; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::new(
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

    // The ports are configured: release them
    pmm.unlock_lpm5();

    // PMMSWBOR triggers a BOR and PMMSWPOR a POR (SLAU445I Table 2-2, p. 91)
    loop {
        // Once a second, so that a terminal opened after the reset shows it too
        writeln!(tx, "Reset cause: {:?}\r", first).ok();
        for _ in 0..100 {
            if s1.is_low().unwrap() {
                while s1.is_low().unwrap() {}
                pmm.software_bor();
            }
            if s2.is_low().unwrap() {
                while s2.is_low().unwrap() {}
                pmm.software_por();
            }
            delay.delay_ms(10);
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
