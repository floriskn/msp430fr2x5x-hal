//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI slave that answers in its receive interrupt, with a master on the same chip: once a second
//! eUSCI_B0, the master, sends 12, 14 and 255, and eUSCI_A1, the slave, answers each byte with that byte
//! plus one, during the next byte. LED1 lights while the answers are right.
//!
//! The master clocks four bytes (the fourth is 0) and reads four back; the last three must be 13, 15 and
//! 0. P3.2, a GPIO, drives the slave's STE, which is low during the transfer: only then does the slave
//! drive MISO. Both use SPI mode 0, MSB first, and the master runs at 10 kHz from SMCLK.
//! (The SPI pins: SLASE59F Table 6-10, p. 49. STE in 4-pin slave mode: SLAU445I 23.3.4.1, p. 609. LED1
//! on P1.0 is red, and P1.1 drives LED2 through J11: SLAU739 Figure 18, p. 23.)
//!
//! How to test (four jumper wires):
//! 1. Take the J11 jumper off, so that LED2 doesn't load the master's SCLK.
//! 2. Connect MOSI, P1.2 (J1 pin 10), to P2.6 (J2 pin 15); MISO, P1.3 (J1 pin 9), to P2.5 (J2 pin 14);
//!    SCLK, P1.1 (J2 pin 19), to P2.4 (J1 pin 7); and STE, P3.2 (J2 pin 17), to P3.1 (J2 pin 13).
//!    (Header pins: SLAU739 Figure 18, p. 23.)
//! 3. Flash this example.
//! 4. Expected: LED1 lights and stays on. Without the wires it stays off.
//! 5. Put the J11 jumper back.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{SpiBus, MODE_0};
use msp430_rt::entry;
use msp430fr2433::{interrupt, EUsciA1};
use msp430_hal::spi::{SpiConfig, SpiErr, SpiSlave, StePolarity};
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pmm::Pmm, watchdog::Wdt
};

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
    let p3 = Batch::new(periph.p3).split(&pmm);
    // Slave, eUSCI_A1: UCA1SIMO, UCA1SOMI and UCA1CLK with P2SELx = 01 (SLASE59F Table 6-19, p. 58) and
    // UCA1STE with P3SELx = 01 (SLASE59F Table 6-20, p. 59)
    let sl_mosi = p2.pin6.to_alternate1();
    let sl_miso = p2.pin5.to_alternate1();
    let sl_sclk = p2.pin4.to_alternate1();
    let sl_ste  = p3.pin1.to_alternate1();

    // Master, eUSCI_B0: UCB0SIMO, UCB0SOMI and UCB0CLK with P1SELx = 01 (SLASE59F Table 6-17, p. 55).
    // P3.2 stays a GPIO that drives the slave's STE.
    let mosi = p1.pin2.to_alternate1();
    let miso = p1.pin3.to_alternate1();
    let sclk = p1.pin1.to_alternate1();
    let mut ste = p3.pin2.to_output();
    ste.set_high().ok();

    // LED1 on P1.0, which is red (SLAU739 Figure 18, p. 23)
    let mut led1 = p1.pin0.to_output();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // Configure a peripheral as an SPI master to drive the bus. It goes first: then SCLK is at its idle
    // level, low, when the slave leaves reset, which a slave with UCCKPH = 1 needs on this device
    // (SLAZ664S USCI47, p. 14).
    // (UCMST = 1, same clock phase and polarity: SLAU445I Table 23-12, p. 620. SMCLK is UCSSEL = 10b:
    // SLASE59F Table 6-7, p. 46; fBitClock = fBRCLK / UCBRx: SLAU445I 23.3.6, p. 609)
    let mut spi = SpiConfig::new(periph.e_usci_b0, MODE_0, true)
        .to_master_using_smclk(&smclk, 800) // 8MHz / 800 = 10kHz
        .single_master_bus(miso, mosi, sclk);

    // Configure another as an SPI slave.
    // It can be configured for either a shared or exclusive bus depending on whether
    // there are other slaves on the bus. On an exclusive bus MISO is always an output.
    // On a shared bus the STE pin is used to control whether this slave's MISO is an output or high impedance pin.
    // (While STE is inactive, "UCxSOMI is set to the input direction": SLAU445I 23.3.4.1, p. 609)
    // UCMST = 0 makes it a slave, and UCMODE = 10b is "4-pin SPI with UCxSTE active low"; MODE_0 and
    // MSB first are UCCKPH = 1, UCCKPL = 0, UCMSB = 1 (SLAU445I Table 23-3, p. 613).
    let mut spi_slave = SpiConfig::new(periph.e_usci_a1, MODE_0, true)
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

        // Red LED on if result matches expected (LED1: SLAU739 Figure 18, p. 23)
        led1.set_state( (recv_buf[1..] == [13, 15, 00]).into() ).ok();

        delay.delay_ms(1000);
    }
}

static SPI_SLAVE: Mutex<RefCell<Option<SpiSlave<EUsciA1>>>> = Mutex::new(RefCell::new(None));

// The eUSCI_A1 vector, UCRXIFG and UCTXIFG in SPI mode through UCA1IV (FFE2h: SLASE59F Table 6-2,
// p. 42). A byte that arrives before the previous one was read sets UCOE (SLAU445I 23.4.3, p. 615),
// reported as `SpiErr::Overrun`.
#[interrupt]
fn USCI_A1() {
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
