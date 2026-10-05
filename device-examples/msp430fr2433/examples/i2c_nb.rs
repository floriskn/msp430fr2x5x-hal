//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A non-blocking I2C master on one board and a polled I2C slave on another: ten times a second the master
//! writes the byte 10 to the slave and reads one byte back, and the slave answers with the byte it got. On
//! the master board LED2 lights while the answer is right, and on both boards LED1 toggles after each
//! exchange.
//!
//! The MSP430FR2433 has one eUSCI_B, so the master and the slave need a board each, and `MASTER` picks the
//! part. On the master board eUSCI_B0 is the master, at 100 kHz from SMCLK; on the other it is the slave,
//! at address 0x1A. Both boards have the internal pull-ups of SDA and SCL on. The non-blocking master
//! interface is lower-level than the blocking embedded-hal `I2c` trait and needs more care: the code sends
//! each start, schedules the stop and moves each byte itself. Before the repeated start it waits until its
//! byte has left the Tx buffer, and it follows the byte it reads with the byte counter, to set the stop
//! while that byte comes in.
//! (One eUSCI_B: SLASE59F Table 3-1, p. 7. Its I2C pins: SLASE59F Table 6-10, p. 49. The internal pull-ups
//! are 20 kΩ to 50 kΩ: SLASE59F Table 5-10, p. 27. LED1 on P1.0 is red and LED2 on P1.1 green: SLAU739
//! Figure 18, p. 23.)
//!
//! How to test (a second MSP-EXP430FR2433, three jumper wires):
//! 1. Connect the two boards: SDA, P1.2 (J1 pin 10), to P1.2 (J1 pin 10); SCL, P1.3 (J1 pin 9), to P1.3
//!    (J1 pin 9); and GND (J2 pin 20) to GND (J2 pin 20). Each board keeps its own USB cable.
//!    (Header pins: SLAU739 Figure 18, p. 23.)
//! 2. Set `MASTER` to false and flash this example to one board, the slave. Then set it to true and flash
//!    it to the other board, the master.
//! 3. Expected: on both boards LED1 toggles every 100 ms, and on the master board LED2 lights. Without the
//!    slave the master stops at its first exchange, and its LEDs don't change; start it again with its
//!    reset button S3 once the slave runs.
#![no_main]
#![no_std]

use embedded_hal::{digital::{OutputPin, StatefulOutputPin}, delay::DelayNs};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cEvent, I2cSingleMaster, TransmissionMode},
    pmm::Pmm,
    prelude::*,
    watchdog::Wdt,
};
use msp430fr2433::EUsciB0;
use panic_msp430 as _;

/// Flash one board with `true`, the master, and the other with `false`, the slave
const MASTER: bool = true;
/// The slave's own address, in UCBxI2COA0 (SLAU445I Table 24-11, p. 656)
const SLAVE_ADDR: u8 = 0x1A;
/// The byte the master sends, and expects back
const ECHO_TX: u8 = 10;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    let _wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut red_led = p1.pin0.to_output();
    let mut green_led = p1.pin1.to_output();

    // P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SELx = 01 (SLASE59F Table 6-17, p. 55). The internal
    // pullups are 20 kΩ to 50 kΩ (SLASE59F Table 5-10, p. 27).
    let scl = p1.pin3.pullup().to_alternate1(); // You may need stronger external pullup resistors
    let sda = p1.pin2.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    if MASTER {
        // UCGLITx = 00b filters pulses of up to 50 ns (SLAU445I 24.3.6, p. 642). SMCLK is UCSSEL = 10b
        // (SLASE59F Table 6-7, p. 46), and fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642).
        let mut i2c_master = I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_single_master()
            .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
            .configure(scl, sda);

        loop {
            // The master sends a byte then receives a byte.
            // The slave echoes the master's byte back.

            // Master transmit
            // (The eUSCI_B slave "automatically acknowledges the received data": SLAU445I 24.3.5.1.2, p. 634)
            i2c_master.send_start(SLAVE_ADDR, TransmissionMode::Transmit);
            let _ = nb::block!(i2c_master.write_tx_buf(ECHO_TX)); // Safe, slave doesn't send NACKs
            // The repeated start must wait until the byte has left the Tx buffer: data is only sent "as long
            // as ... The UCTXSTT bit is not set" (SLAU445I 24.3.5.2.1, p. 637)
            let _ = nb::block!(i2c_master.tx_buf_empty()); // Safe, slave doesn't send NACKs

            // Master swaps mode
            i2c_master.send_start(SLAVE_ADDR, TransmissionMode::Receive);

            // Master receive
            // A stop must be scheduled while the slave's byte comes in (rather than *after* reading the Rx
            // buffer), as otherwise the bus will start the next byte then stall waiting for more data
            // (SLAU445I 24.3.5.2.2, p. 639)
            wait_for_next_byte(&mut i2c_master);
            i2c_master.schedule_stop();
            let echo_rx = nb::block!(i2c_master.read_rx_buf()).unwrap_or(0); // Safe, slave doesn't send NACKs

            // Enable the LED if the echoed value matches what was sent.
            green_led.set_state((echo_rx == ECHO_TX).into()).ok();
            red_led.toggle().ok();
            delay.delay_ms(100);
        }
    } else {
        // The slave answers to its own address in UCBxI2COA0 (SLAU445I Table 24-11, p. 656)
        let mut i2c_slave = I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_slave(SLAVE_ADDR)
            .configure(scl, sda);

        loop {
            // Slave receive
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
            red_led.toggle().ok();
        }
    }
}

/// Wait until the first data byte after the next START or repeated start is on the bus. The byte counter
/// goes back to 0 with each START and repeated start, skips address bytes, and counts a byte from its
/// second bit (UCBCNTx: SLAU445I 24.3.8, p. 643).
fn wait_for_next_byte(i2c: &mut I2cSingleMaster<EUsciB0>) {
    while i2c.byte_count() != 0 {}
    while i2c.byte_count() == 0 {}
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
