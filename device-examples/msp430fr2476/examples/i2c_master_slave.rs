//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Two I2C master-slaves on one chip take turns as master: once a second eUSCI_B0 writes a count to
//! eUSCI_B1 and reads its answer, and then eUSCI_B1 does the same to eUSCI_B0. The backchannel UART prints
//! both answers.
//!
//! Both are `I2cMasterSlave`: masters on a bus with other masters that also answer their own address, 0x1A
//! for eUSCI_B0 and 0x1B for eUSCI_B1. Addressed by the other master while it's idle in master mode, an
//! eUSCI loses arbitration and works as a slave, and `return_to_master()` makes it a master again after the
//! STOP. Both use the non-blocking interface: as master `send_start()`, `write_tx_buf_as_master()`,
//! `tx_buf_empty()`, `schedule_stop()`, `read_rx_buf_as_master()` and `stop_sent()`, and as slave `poll()`,
//! `read_rx_buf_as_slave()` and `write_tx_buf_as_slave()`. The slave answers the byte it got plus one. Both
//! run at 100 kHz from SMCLK.
//! The LaunchPad has no pull-ups on these pins, so all four have their internal pull-up on: two on each
//! line.
//! (Masters with an own address, UCMM = 1: SLAU445I 24.3.5.2, p. 636. A master addressed as a slave loses
//! arbitration and becomes a slave: UCALIFG, SLAU445I Table 24-2, p. 646. The I2C pins: SLASEO7C
//! Table 9-11, p. 54. The internal pull-ups are 20 kΩ to 50 kΩ: SLASEO7C 8.12.4.1, p. 31. The LaunchPad
//! has no pull-ups on these pins: SLAU802 Figure 18, p. 24.)
//!
//! How to test (two jumper wires):
//! 1. Connect SDA, P1.2 (J1 pin 10), to P3.2 (J2 pin 15), and SCL, P1.3 (J1 pin 9), to P3.6 (J2 pin 14).
//!    (Header pins: SLAU802 Figure 10, p. 13.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 3. Expected, once a second: `B0 to B1: Ok(1), B1 to B0: Ok(1)`, then `Ok(2)` and so on, counting up.
//!    Without the wires nothing is printed: the slave waits forever.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cEvent, I2cMasterSlave, I2cMasterSlaveErr, I2cUsci, TransmissionMode},
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

const ADDRESS_B0: u8 = 0x1A;
const ADDRESS_B1: u8 = 0x1B;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);

    // eUSCI_B0: P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SEL = 01 (SLASEO7C Table 9-23, p. 65). eUSCI_B1:
    // P3.6 = UCB1SCL and P3.2 = UCB1SDA with P3SEL = 01 (SLASEO7C Table 9-25, p. 67). SDA and SCL need
    // pull-ups (SLAU445I 24.3, p. 629): the internal ones here, two on each line.
    let b0_scl = p1.pin3.pullup().to_alternate1();
    let b0_sda = p1.pin2.pullup().to_alternate1();
    let b1_scl = p3.pin6.pullup().to_alternate1();
    let b1_sda = p3.pin2.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9; SLASEO7C
    // Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
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

    // Masters with other masters on the bus (UCMM = 1) and an own address in UCBxI2COA0 (SLAU445I 24.3.5.2,
    // p. 636), clocked by SMCLK. With several masters fBitClock = fBRCLK/UCBRx may be at most fBRCLK/8
    // (SLAU445I 24.3.7, p. 642).
    let mut b0 = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
        .as_master_slave(ADDRESS_B0)
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(b0_scl, b0_sda);
    let mut b1 = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b1, GlitchFilter::Max50ns)
        .as_master_slave(ADDRESS_B1)
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(b1_scl, b1_sda);

    let mut count: u8 = 0;
    loop {
        let to_b1 = exchange(&mut b0, &mut b1, ADDRESS_B1, count);
        let to_b0 = exchange(&mut b1, &mut b0, ADDRESS_B0, count);
        writeln!(tx, "B0 to B1: {:?}, B1 to B0: {:?}\r", to_b1, to_b0).ok();
        count = count.wrapping_add(1);
        delay.delay_ms(1000);
    }
}

/// `master` writes `count` to `slave`, the master-slave at `address`, and reads its answer after a
/// repeated start. Both are on this chip, so they take their steps in turn.
fn exchange<M: I2cUsci, S: I2cUsci>(
    master: &mut I2cMasterSlave<M>,
    slave: &mut I2cMasterSlave<S>,
    address: u8,
    count: u8,
) -> Result<u8, I2cMasterSlaveErr> {
    // Master: START, the address with the write bit, and the byte (SLAU445I 24.3.5.2.1, p. 637). The
    // repeated start has to wait until the byte has left the Tx buffer: data is only sent while UCTXSTT is
    // clear (SLAU445I 24.3.5.2.1, p. 637).
    master.send_start(address, TransmissionMode::Transmit)?;
    nb::block!(master.write_tx_buf_as_master(count))?;
    nb::block!(master.tx_buf_empty())?;

    // Slave: the byte arrives (SLAU445I 24.3.5.1.2, p. 634)
    wait_for(slave, I2cEvent::WriteStart);
    let got = nb::block!(slave.read_rx_buf_as_slave()).unwrap(); // Infallible

    // Master: a repeated start, with the read bit (SLAU445I 24.3.5.2.1, p. 637)
    master.send_start(address, TransmissionMode::Receive)?;

    // Slave: the answer (SLAU445I 24.3.5.1.1, p. 633)
    wait_for(slave, I2cEvent::ReadStart);
    nb::block!(slave.write_tx_buf_as_slave(got.wrapping_add(1))).ok();

    // Master: the next byte received is followed by a NACK and the STOP, so the STOP is scheduled before it
    // arrives (SLAU445I 24.3.5.2.2, p. 639). The transaction ends once the STOP is on the bus.
    master.schedule_stop();
    let answer = nb::block!(master.read_rx_buf_as_master())?;
    nb::block!(master.stop_sent())?;

    // Slave: after the STOP, a master again (UCMST: SLAU445I Table 24-4, p. 649)
    wait_for(slave, I2cEvent::Stop);
    slave.return_to_master();
    Ok(answer)
}

/// Poll `i2c` until `event`
fn wait_for<U: I2cUsci>(i2c: &mut I2cMasterSlave<U>, event: I2cEvent) {
    while i2c.poll() != Ok(event) {}
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
