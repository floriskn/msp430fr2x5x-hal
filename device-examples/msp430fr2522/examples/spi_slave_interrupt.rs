//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI slave that answers in its receive interrupt, with a master on the same chip: once a second
//! eUSCI_B0, the master, sends 12, 14 and 255, and eUSCI_A0, the slave, answers each byte with that byte
//! plus one, during the next byte. An LED on P1.0 lights while the answers are right.
//!
//! The master clocks four bytes (the fourth is 0) and reads four back; the last three must be 13, 15 and
//! 0. P2.2, a GPIO, drives the slave's STE, which is low during the transfer: only then does the slave
//! drive MISO. Both use SPI mode 0, MSB first, and the master runs at 10 kHz from SMCLK. The master is set
//! up first, so that its clock is idle when the slave leaves reset, which erratum USCI47 needs.
//! (The SPI pins: SLASEE4C Table 6-11, p. 53. STE in 4-pin slave mode: SLAU445I 23.3.4.1, p. 609. The
//! erratum: SLAZ705H USCI47, p. 10 to p. 11. No board document covers the LED: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (four jumper wires, an LED and a resistor):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND.
//! 2. Connect MOSI, P1.2, to P1.4; MISO, P1.3, to P1.5; SCLK, P1.1, to P1.6; and STE, P2.2, to P1.7.
//! 3. Flash this example.
//! 4. Expected: the LED lights and stays on. Without the wires it stays off.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{SpiBus, MODE_0};
use msp430_rt::entry;
use msp430_hal::pin_mapping::DefaultMapping;
use msp430fr25x2::{interrupt, EUsciA0};
use msp430_hal::spi::{Spi, SpiConfig, SpiErr, SpiSlave, StePolarity};
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pmm::Pmm, watchdog::Wdt
};

use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    // Slave, eUSCI_A0 in the default mapping, USCIARMP = 0: UCA0SIMO on P1.4, UCA0SOMI on P1.5, UCA0CLK on
    // P1.6 and UCA0STE on P1.7, with P1SELx = 01 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58)
    let sl_mosi = p1.pin4.to_alternate1();
    let sl_miso = p1.pin5.to_alternate1();
    let sl_sclk = p1.pin6.to_alternate1();
    let sl_ste  = p1.pin7.to_alternate1();

    // Master, eUSCI_B0 in the default mapping, USCIBRMP = 0: UCB0SIMO on P1.2, UCB0SOMI on P1.3 and
    // UCB0CLK on P1.1, with P1SELx = 01 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). P2.2
    // stays a GPIO that drives the slave's STE (P2SELx = 00 and P2DIR = 1: SLASEE4C Table 6-16, p. 60).
    let mosi = p1.pin2.to_alternate1();
    let miso = p1.pin3.to_alternate1();
    let sclk = p1.pin1.to_alternate1();
    let mut ste = p2.pin2.to_output();
    ste.set_high().ok();

    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let mut led = p1.pin0.to_output();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM,
    // DIVS: SLAU445I Table 3-9, p. 118). ACLK from REFO (SELA = 01b, same table): the MSP430FR25x2 has
    // no ACLK from the VLO (SLASEE4C 6.10.2, p. 49).
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // Configure another peripheral as an SPI master to drive the bus. It is configured before the slave:
    // MODE_0 is UCCKPH = 1, and an SPI slave with UCCKPH = 1 goes wrong if its clock pin isn't at the idle
    // level when it leaves reset (SLAZ705H USCI47, p. 10 to p. 11). The erratum's workaround: "The SPI
    // master must set the clock pin at the appropriate idle level (low for UCCKPL = 0, high for
    // UCCKPL = 1) before SPI slave is reset (UCSWRST bit is cleared)". With the master running, SCLK idles
    // low (UCCKPL = 0 in MODE_0).
    // (UCMST = 1, same clock phase and polarity: SLAU445I Table 23-12, p. 620. SMCLK is UCSSEL = 10b:
    // SLASEE4C Table 6-8, p. 49; fBitClock = fBRCLK / UCBRx: SLAU445I 23.3.6, p. 609)
    let mut spi: Spi<_, DefaultMapping> = SpiConfig::new(periph.e_usci_b0, MODE_0, true)
        .to_master_using_smclk(&smclk, 800) // 8MHz / 800 = 10kHz
        .single_master_bus(miso, mosi, sclk);

    // Configure a peripheral as an SPI slave.
    // It can be configured for either a shared or exclusive bus depending on whether
    // there are other slaves on the bus. On an exclusive bus MISO is always an output.
    // On a shared bus the STE pin is used to control whether this slave's MISO is an output or high impedance pin.
    // (While STE is inactive, "UCxSOMI is set to the input direction": SLAU445I 23.3.4.1, p. 609)
    // UCMST = 0 makes it a slave, and UCMODE = 10b is "4-pin SPI with UCxSTE active low"; MODE_0 and
    // MSB first are UCCKPH = 1, UCCKPL = 0, UCMSB = 1 (SLAU445I Table 23-3, p. 613).
    let mut spi_slave: SpiSlave<_, DefaultMapping> = SpiConfig::new(periph.e_usci_a0, MODE_0, true)
        .to_slave()
        .shared_bus(sl_miso, sl_mosi, sl_sclk, sl_ste, StePolarity::EnabledWhenLow);

    // UCRXIE enables the receive interrupt (SLAU445I Table 23-8, p. 617)
    critical_section::with(|cs| {
        spi_slave.set_rx_interrupt();
        SPI_SLAVE.replace(cs, Some(spi_slave));
    });

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { msp430::interrupt::enable() };

    loop {
        let mut recv_buf = [0; 4];
        let send_buf = [12, 14, 0xFF];

        ste.set_low().ok(); // Enable slave MISO

        // Can return Err, but both error types aren't relevant here.
        let _ = spi.transfer(&mut recv_buf, &send_buf);
        let _ = spi.flush();

        ste.set_high().ok(); // Make slave MISO high impedance

        // LED on if result matches expected
        led.set_state( (recv_buf[1..] == [13, 15, 00]).into() ).ok();

        delay.delay_ms(1000);
    }
}

static SPI_SLAVE: Mutex<RefCell<Option<SpiSlave<EUsciA0, DefaultMapping>>>> = Mutex::new(RefCell::new(None));

// The eUSCI_A0 vector, UCRXIFG and UCTXIFG in SPI mode through UCA0IV (FFECh: SLASEE4C Table 6-2,
// p. 46). A byte that arrives before the previous one was read sets UCOE (SLAU445I 23.4.3, p. 615),
// reported as `SpiErr::Overrun`.
#[interrupt]
fn EUSCI_A0() {
    critical_section::with(|cs| {
        let Some(ref mut spi_slave) = *SPI_SLAVE.borrow_ref_mut(cs) else {return};
        // If you have multiple interrupts enabled you can use .interrupt_source() to determine which one caused this interrupt
        let byte = match unsafe{spi_slave.read_unchecked()} { // Only Rx interrupts are enabled, so Rx buffer must be ready
            Ok(b) => b,
            Err(SpiErr::Overrun(b)) => b,
            // Only a master on a multi-master bus gets this (UCFE: SLAU445I Table 23-5, p. 615)
            Err(SpiErr::BusConflict) => return,
        };
        nb::block!( spi_slave.write(byte.wrapping_add(1)) ).unwrap(); // Infallible, safe to unwrap after blocking
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
