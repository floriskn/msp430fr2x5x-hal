//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Why did the device reset? `Pmm::take_reset_cause` reads the reasons, highest priority first, and eUSCI_A0
//! prints the first one at start-up. Two buttons reset the device by software when they are released.
//!
//! | Printed                    | Reset                                                         |
//! |----------------------------|---------------------------------------------------------------|
//! | `Reset cause: Brownout`    | power-up: switch the power off and on (brownout reset)        |
//! | `Reset cause: ResetPin`    | a low level on RST/NMI                                        |
//! | `Reset cause: SoftwareBor` | the button on P2.2, which calls `Pmm::software_bor()`         |
//! | `Reset cause: SoftwarePor` | the button on P2.3, which calls `Pmm::software_por()`         |
//! | `Reset cause: other`       | any other reason                                              |
//! | `Reset cause: none`        | no reason: the debugger started the program after flashing it |
//!
//! (SYSRSTIV: SLAU445I 1.3.7, p. 36. The resets, with their priorities: brownout 02h, RST/NMI pin 04h,
//! software BOR 06h, software POR 14h: SLASEE4C Table 6-10, p. 52. P2.2 and P2.3 are GPIO inputs with their
//! pullups: SLASEE4C Table 6-16, p. 60; SLAU445I Table 8-1, p. 313. P2.3 only exists on the 20-pin RHL
//! package: SLASEE4C Table 4-2, p. 14. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document
//! covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (two push buttons and a 3.3-V USB-to-UART adapter):
//! 1. Connect a push button from P2.2 to GND and another from P2.3 to GND (the internal pullups are on).
//!    Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at 9600 baud. Power
//!    the MSP430FR2522 from its own supply, not from the adapter, so that the terminal stays open while the
//!    MSP430FR2522's power is off.
//! 2. Flash this example. Expected: `Reset cause: none`.
//! 3. Reset the device, with RST/NMI low for a moment: `Reset cause: ResetPin`.
//! 4. Press and release the button on P2.2: `Reset cause: SoftwareBor`. Press and release the one on
//!    P2.3: `Reset cause: SoftwarePor`.
//! 5. Switch the power off and on again: `Reset cause: Brownout`.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::{Pmm, ResetCause},
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    // After a BOR the pins stay locked in their reset state until LOCKLPM5 is cleared, and the data
    // sheet asks for the ports to be configured first (SLASEE4C 6.10.3, p. 51: "the ports must be
    // configured first and then the LOCKLPM5 bit must be cleared"), so the pins are set up below before
    // they are released. Measured on an MSP430FR2476: a software BOR sets LOCKLPM5 again, a software POR
    // and a watchdog PUC leave it clear, and then unlock_lpm5() changes nothing.
    let (mut pmm, _) = Pmm::new_locked(periph.pmm, periph.sys);

    // Read the first reason, then the rest, which also clears them for the next reset
    // (reading SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    // The buttons pull P2.2 and P2.3 low, against the internal pullups (PxDIR = 0, PxREN = 1, PxOUT = 1:
    // SLAU445I Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.pullup())
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut bor_button = p2.pin2;
    let mut por_button = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let mut fram = Fram::new(periph.frctl);
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0, 8N1
    // (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58; SLAU445I Table 22-8, p. 593)
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

    // The ports are configured: release them
    pmm.unlock_lpm5();

    let cause = match first {
        Some(ResetCause::Brownout) => "Brownout",
        Some(ResetCause::ResetPin) => "ResetPin",
        Some(ResetCause::SoftwareBor) => "SoftwareBor",
        Some(ResetCause::SoftwarePor) => "SoftwarePor",
        Some(_) => "other",
        None => "none",
    };
    print(&mut tx, "\r\nReset cause: ");
    print(&mut tx, cause);
    print(&mut tx, "\r\n");

    // PMMSWBOR triggers a BOR and PMMSWPOR a POR (SLAU445I Table 2-2, p. 91)
    loop {
        if bor_button.is_low().unwrap() {
            while bor_button.is_low().unwrap() {}
            pmm.software_bor();
        }
        if por_button.is_low().unwrap() {
            while por_button.is_low().unwrap() {}
            pmm.software_por();
        }
    }
}

fn print(tx: &mut impl Write, text: &str) {
    tx.write_all(text.as_bytes()).ok();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
