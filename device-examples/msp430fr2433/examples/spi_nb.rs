//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master with the non-blocking `FullDuplex` interface of embedded-hal-nb and a GPIO as the chip
//! select: once a second it sends two bytes, AAh and FFh, with the chip select low.
//!
//! eUSCI_A1 is a 3-pin master in SPI mode 0 (the clock idles low, and data is captured on its rising
//! edges), MSB first, at 500 kHz from SMCLK. Each `write` of a byte is followed by a `read` of the byte
//! that came in meanwhile. With more than one slave, "the software needs to use general-purpose I/O pins
//! instead to generate STE signals", so P1.3 is the chip select.
//! (eUSCI_A1's SPI pins: SLASE59F Table 6-10, p. 49. SPI mode 0: SLAU445I Table 23-3, p. 613. GPIO chip
//! selects: SLAU445I 23.3.3.2, p. 608.)
//!
//! How to test (the scope):
//! 1. Flash this example.
//! 2. Scope, ground on GND (J3 pin 22), 20 µs/div, trigger on CH3 falling: CH1 on SCLK, P2.4 (J1 pin 7),
//!    CH2 on MOSI, P2.6 (J2 pin 15), CH3 on the chip select, P1.3 (J1 pin 9). Expected once a second:
//!    the chip select low for 16 clock pulses, while MOSI sends AAh and FFh (10101010 and 11111111),
//!    MSB first. The scope's SPI decoder (Analysis > Decode) shows them as bytes.
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::MODE_0};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pmm::Pmm, spi::SpiConfig, watchdog::Wdt
};
use nb::block;
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    let _wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // eUSCI_A1: P2.6 = UCA1SIMO, P2.5 = UCA1SOMI and P2.4 = UCA1CLK with P2SELx = 01 (SLASE59F
    // Table 6-10, p. 49; SLASE59F Table 6-19, p. 58), on J2 pins 15 and 14 and J1 pin 7; CS is P1.3,
    // J1 pin 9 (SLAU739 Figure 18, p. 23).
    let mosi   = p2.pin6.to_alternate1();
    let miso   = p2.pin5.to_alternate1();
    let sck    = p2.pin4.to_alternate1();
    let mut cs = p1.pin3.to_output();
    cs.set_high().ok();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // MODE_0 captures data on the first clock edge with the clock idle low (UCCKPH = 1, UCCKPL = 0),
    // `true` sends the MSB first (UCMSB = 1), and UCMST = 1 makes it the master (SLAU445I Table 23-3,
    // p. 613). SMCLK is UCSSEL = 10b (SLASE59F Table 6-7, p. 46), and fBitClock = fBRCLK / UCBRx
    // (SLAU445I 23.3.6, p. 609).
    let mut spi = SpiConfig::new(periph.e_usci_a1, MODE_0, true)
        .to_master_using_smclk(&smclk, 16) // 8MHz / 16 = 500kHz
        .single_master_bus(miso, mosi, sck);

    loop {
        // Non-blocking interface available through embedded-hal-nb
        use embedded_hal_nb::spi::FullDuplex;

        cs.set_low().ok();

        // Blocking send. Sends out data on the MOSI line
        // Sending is infallible, besides `nb::WouldBlock` when the bus is busy.
        block!(spi.write(0b10101010)).unwrap();

        // Writing on MOSI also shifts in data on MISO - read from the hardware buffer with `.read()`.
        // (Receive and transmit run concurrently: SLAU445I 23.3.3, p. 607)
        // Every successful `.write()` call should be followed by a `.read()`.
        // You should handle errors here rather than unwrapping
        let _ = block!(spi.read()).unwrap();

        // This concludes the first byte of an SPI transaction.

        // Multi-byte transactions are performed by calling the methods repeatedly:
        block!(spi.write(0xFF)).unwrap();
        let _ = block!(spi.read()).unwrap();

        cs.set_high().ok();

        delay.delay_ms(1000);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
