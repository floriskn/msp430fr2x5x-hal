//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A non-blocking I2C master and a polled I2C slave, on two MSP430FR2522: ten times a second the master
//! writes the byte 10 to the slave and reads one byte back, and the slave answers with the byte it got.
//! The master's LED on P1.0 lights while the answer is right, and the slave's LED on P1.0 toggles after
//! each exchange.
//!
//! eUSCI_B0, the device's only eUSCI_B, is the master on one board and the slave, at address 0x1A, on the
//! other: `SLAVE` picks which. The master runs at 100 kHz from SMCLK, and both have the internal pull-ups
//! on their pins. The non-blocking master interface is lower-level than the blocking embedded-hal `I2c`
//! trait and needs more care: the code sends each start, schedules the stop and moves each byte itself.
//! With the slave on another chip, the master waits until its byte has left the Tx buffer before the
//! repeated start, and watches the byte counter to send the stop while the byte it reads comes in.
//! (One eUSCI_B: SLASEE4C 1.1, p. 1. eUSCI_B0's I2C pins: SLASEE4C Table 6-11, p. 53. The byte counter:
//! SLAU445I 24.3.8, p. 643. No board document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (two MSP430FR2522, two LEDs and resistors, three jumper wires):
//! 1. Power both MSP430FR2522 from 3.3 V, and on each connect an LED with a series resistor (about 1 kΩ)
//!    from P1.0 to GND.
//! 2. Connect SDA, P1.2, of one to P1.2 of the other, SCL, P1.3, to P1.3, and GND to GND. The internal
//!    pull-ups may be too weak: if it doesn't work, add resistors from SDA and SCL to 3.3 V.
//! 3. Set `SLAVE` to true and flash this example to the slave first. Set it back to false and flash it to
//!    the master.
//! 4. Expected: the slave's LED toggles every 100 ms, and the master's LED lights. Without the wires, or if
//!    the master started before the slave, nothing acknowledges the master, the master waits forever, and
//!    the LEDs don't change: reset the master, with its RST/NMI low for a moment.
//! (A low level on RST/NMI resets the device: SLAU445I 1.2, p. 30.)
#![no_main]
#![no_std]

use embedded_hal::{digital::{OutputPin, StatefulOutputPin}, delay::DelayNs};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cEvent, I2cSingleMaster, I2cSlave, TransmissionMode},
    pin_mapping::DefaultMapping, pmm::Pmm, prelude::*, watchdog::Wdt
};
use panic_msp430 as _;

/// Which board this is: the slave, or the master
const SLAVE: bool = false;
/// The slave's own address (UCBxI2COA0: SLAU445I Table 24-11, p. 656)
const SLAVE_ADDR: u8 = 0x1A;
/// The byte the master sends, and expects back
const ECHO_TX: u8 = 10;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let mut led = p1.pin0.to_output();
    // UCB0SCL on P1.3 and UCB0SDA on P1.2, P1SELx = 01 in the default mapping, USCIBRMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). The internal pull-ups are 20 kΩ to 50 kΩ (SLASEE4C
    // Table 5-10, p. 29), and I2C needs pull-ups on SDA and SCL (SLAU445I 24.3, p. 629).
    let scl = p1.pin3.pullup().to_alternate1(); // You may need stronger external pullup resistors
    let sda = p1.pin2.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    if SLAVE {
        // The slave answers to its own address (UCMST = 0, the own address in UCBxI2COA0: SLAU445I
        // 24.3.5.1, p. 633)
        let mut i2c_slave: I2cSlave<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_slave(SLAVE_ADDR)
                .configure(scl, sda);

        loop {
            // The slave echoes the master's byte back.

            // Slave receive
            // (The eUSCI_B slave "automatically acknowledges the received data": SLAU445I 24.3.5.1.2, p. 634)
            loop {
                if i2c_slave.poll() == Ok(I2cEvent::WriteStart) { break }
            }
            let byte = unsafe { i2c_slave.read_rx_buf_unchecked() }; // Safe since `poll` returned a Write event

            // Slave transmit
            loop {
                if i2c_slave.poll() == Ok(I2cEvent::ReadStart) { break }
            }
            let _ = nb::block!(i2c_slave.write_tx_buf(byte)); // Safe, infallible

            loop {
                if i2c_slave.poll() == Ok(I2cEvent::Stop) { break }
            }
            led.toggle().ok();
        }
    } else {
        // UCGLITx = 00b filters pulses of up to 50 ns (SLAU445I 24.3.6, p. 642). SMCLK is UCSSEL = 10b
        // (SLASEE4C Table 6-8, p. 49), and fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642).
        let mut i2c_master: I2cSingleMaster<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_single_master()
                .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
                .configure(scl, sda);

        loop {
            // The master sends a byte then receives a byte.

            // Master transmit
            i2c_master.send_start(SLAVE_ADDR, TransmissionMode::Transmit);
            let _ = nb::block!(i2c_master.write_tx_buf(ECHO_TX)); // Safe, slave doesn't send NACKs
            // The slave is on another chip, so wait until the byte has left the Tx buffer before the repeated
            // start: data is only sent while "The UCTXSTT bit is not set" (SLAU445I 24.3.5.2.1, p. 637)
            let _ = nb::block!(i2c_master.tx_buf_empty()); // Safe, slave doesn't send NACKs

            // Master swaps mode
            i2c_master.send_start(SLAVE_ADDR, TransmissionMode::Receive);

            // Master receive
            // A stop must be scheduled while the byte is being received (rather than *after* reading the Rx
            // buffer), as otherwise the bus will start the next byte then stall waiting for more data
            // (SLAU445I 24.3.5.2.2, p. 639). The repeated start sets the byte counter back to zero, and it
            // counts the byte from its second bit (SLAU445I 24.3.8, p. 643).
            while i2c_master.byte_count() != 0 {}
            while i2c_master.byte_count() == 0 {}
            i2c_master.schedule_stop();
            let echo_rx = nb::block!(i2c_master.read_rx_buf()).unwrap_or(0); // Safe, slave doesn't send NACKs

            // Enable the LED if the echoed value matches what was sent.
            led.set_state((echo_rx == ECHO_TX).into()).ok();
            delay.delay_ms(100);
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
