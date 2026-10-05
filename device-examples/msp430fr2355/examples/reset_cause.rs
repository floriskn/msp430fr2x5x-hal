//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Why did the device reset? `Pmm::take_reset_cause` reads the reasons, highest priority first, and the
//! backchannel UART prints the first one, once a second, so a terminal opened late still sees it. S1 and
//! S2 reset the device by software when they are released.
//!
//! | Printed                    | Reset                                                         |
//! |----------------------------|---------------------------------------------------------------|
//! | `Reset cause: Brownout`    | power-up: plug in the USB cable (brownout reset)              |
//! | `Reset cause: ResetPin`    | the reset button S3                                           |
//! | `Reset cause: SoftwareBor` | button S1, which calls `Pmm::software_bor()`                  |
//! | `Reset cause: SoftwarePor` | button S2, which calls `Pmm::software_por()`                  |
//! | `Reset cause: none`        | no reason: the debugger started the program after flashing it |
//!
//! Other reasons print their `ResetCause` name. This LaunchPad's two LEDs have one color each, too few
//! to tell these reasons apart, so the example prints them.
//! (SYSRSTIV: SLAU445I 1.3.7, p. 36. The resets, with their priorities: brownout 02h, RST/NMI pin 04h,
//! software BOR 06h, software POR 14h: SLASEC4D Table 6-12, p. 70. LED1 is red and LED2 green; S1 (P4.1)
//! and S2 (P2.3) pull their pins low, and S3 is the reset button on RST: SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11). Expected: `Reset cause: none`.
//! 2. Press S3: `Reset cause: ResetPin`.
//! 3. Press and release S1: `Reset cause: SoftwareBor`. Press and release S2: `Reset cause: SoftwarePor`.
//! 4. Unplug the USB cable and plug it back in, then open the COM port again: `Reset cause: Brownout`.
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
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    // After a BOR the pins stay locked in their reset state until LOCKLPM5 is cleared, and the data
    // sheet asks for the ports to be configured first (SLASEC4D 6.10.3, p. 69: "To enable the I/O
    // functions after a BOR reset, first configure the ports and then clear the LOCKLPM5 bit"), so the
    // pins are set up below before they are released. Measured on an MSP430FR2476: a software BOR sets
    // LOCKLPM5 again, a software POR and a watchdog PUC leave it clear, and then unlock_lpm5() changes
    // nothing.
    let (mut pmm, _) = Pmm::new_locked(periph.pmm, periph.sys);

    // Read the first reason, then the rest, which also clears them for the next reset
    // (reading SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let first = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    // S1 and S2 inputs with their internal pullups (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1,
    // p. 313), as the board has none (SLAU680 Figure 18, p. 26)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let p4 = Batch::new(periph.p4)
        .config_pin1(|p| p.pullup())
        .split(&pmm);
    let mut s1 = p4.pin1;
    let mut s2 = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SELx = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p4.pin3.to_alternate1());

    // The ports are configured: release them
    pmm.unlock_lpm5();

    // PMMSWBOR triggers a BOR and PMMSWPOR a POR (SLAU445I Table 2-2, p. 91)
    loop {
        match first {
            Some(cause) => writeln!(tx, "Reset cause: {:?}\r", cause).ok(),
            None => writeln!(tx, "Reset cause: none\r").ok(),
        };

        // About a second, watching S1 and S2
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
