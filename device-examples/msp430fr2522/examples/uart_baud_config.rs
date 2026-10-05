//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A UART with the user's guide's recommended baud-rate settings instead of calculated ones: an echo, as
//! in `echo`, at 9600 baud from SMCLK at 8 MHz.
//!
//! Given a baud rate, the clock selection of `SerialConfig` calculates the baud-rate settings from it and
//! the clock frequency (SLAU445I 22.3.10, p. 586). Given a `BaudConfig`, it uses the settings in it as
//! they are. These come from the user's guide's table of recommended settings, the row for 9600 baud
//! from 8 MHz: UCOS16 = 1, UCBRx = 52, UCBRFx = 1 and UCBRSx = 49h (SLAU445I Table 22-5, p. 589), whose
//! UCBRSx settings come from a search for the lowest error (its note 1). The calculation looks UCBRSx up
//! from the fractional part of N in a shorter table instead, and gets 25h for 8 MHz (SLAU445I Table 22-4,
//! p. 586). `BaudConfig::new()` makes that calculation in a constant, so the compiler works it out.
//!
//! SMCLK runs from the DCO in the 8 MHz range, at 7995392 Hz, 0.06 % below the 8 MHz of the table's row.
//! That adds 0.06 % to the row's errors of at most 0.14 % (its note 2: "Any frequency variation or jitter
//! of the clock source will make the errors worse"). A character that arrives with an error comes back as
//! `!` (parity), `}` (overrun), `?` (framing) or `#` (break). An LED on P1.0 lights once the UART is
//! set up. There's no LaunchPad for the MSP430FR25x2, so the PC needs a USB-to-UART adapter.
//! (eUSCI_A0: SLASEE4C 6.10.7, p. 53. UCA0TXD is P1.4 and UCA0RXD P1.5: SLASEE4C Table 6-11, p. 53;
//! SLASEE4C Table 6-15, p. 58.)
//!
//! How to test (a 3.3-V USB-to-UART adapter, an LED and a resistor):
//! 1. Connect the LED with a series resistor (about 1 kΩ) from P1.0 to GND, and the adapter: its RX to
//!    P1.4, its TX to P1.5 and its GND to GND.
//! 2. Flash this example, and open the adapter's COM port at 9600 baud.
//! 3. Expected: the LED lights, and the terminal shows `HELLO`. Each character you type comes back (with
//!    the terminal's local echo off).
//! 4. Change `BAUD` to `BaudConfig::new(DcoclkFreqSel::_8MHz.freq(), 9600)`, the settings calculated for
//!    SMCLK's 7995392 Hz (UCBRx = 52, UCBRFx = 0, UCBRSx = DFh), and repeat steps 2 and 3: the same.
#![no_main]
#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_nb::serial::{Read, Write};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};

use nb::block;
#[cfg(debug_assertions)]
use panic_msp430 as _;

#[cfg(not(debug_assertions))]
use panic_never as _;

/// 9600 baud from 8 MHz, the user's guide's recommended settings: oversampling (UCOS16 = 1), UCBRx = 52,
/// UCBRFx = 1, UCBRSx = 49h (SLAU445I Table 22-5, p. 589)
const BAUD: BaudConfig = BaudConfig::from_fields(true, 52, 1, 0x49);

#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr25x2::Peripherals::take() {
        let mut fram = Fram::new(periph.frctl);
        // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
        let _wdt = Wdt::constrain(periph.wdt_a);

        // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, and ACLK from REFO (SELMS = 000b, SELA = 01b:
        // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
        let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
            .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
            .smclk_on(SmclkDiv::_1)
            .aclk_refoclk()
            .freeze(&mut fram);

        let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
        let p1 = Batch::new(periph.p1).split(&pmm);

        // No board document covers an LED on P1.0: there is none for the MSP430FR25x2. P1.0 is a GPIO
        // output, P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58).
        let mut led = p1.pin0.to_output();

        led.set_low().ok();

        // TXD on P1.4 and RXD on P1.5: UCA0TXD and UCA0RXD with P1SELx = 01 in the default mapping,
        // USCIARMP = 0 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58)
        // (8N1, LSB first: UCMSB, UC7BIT, UCSPB, UCPEN in SLAU445I Table 22-8, p. 593; SMCLK is UCSSEL =
        // 10b: SLAU445I Table 22-8, p. 593)
        let (mut tx, mut rx) = SerialConfig::<_, _, DefaultMapping>::new(
            periph.e_usci_a0,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            Parity::NoParity,
            Loopback::NoLoop,
            BAUD,
        )
        .use_smclk(&smclk)
        .split(p1.pin4.to_alternate1(), p1.pin5.to_alternate1());

        led.set_high().ok();
        embedded_io::Write::write_all(&mut tx, b"HELLO\n").ok();
        loop {
            // (The receive errors are the UCPE, UCOE, UCFE and UCBRK flags: SLAU445I Table 22-1, p. 582)
            let ch: u8 = match block!(rx.read()) {
                Ok(c) => c,
                Err(RecvError::Parity)      => b'!',
                Err(RecvError::Overrun(_))  => b'}',
                Err(RecvError::Framing)     => b'?',
                Err(RecvError::Break)       => b'#',
            };
            block!(tx.write(ch)).unwrap();
        }
    } else {
        loop {}
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
