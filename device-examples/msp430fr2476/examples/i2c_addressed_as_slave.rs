//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An I2C master-slave addressed by another master in the middle of its own blocking transfer. Once a
//! second eUSCI_B0, a multi-master, starts reading a byte from eUSCI_B1, and right after that eUSCI_B1, a
//! master-slave at 0x1A, starts a blocking transfer of its own: `is_slave_present(0x09)`. Its START waits
//! for the bus, and eUSCI_B0 addresses it first, so it loses arbitration and works as a slave: the transfer
//! returns `AddressedAsSlave`, and the flags of the slave transaction stay set. eUSCI_B1 answers the read
//! with `poll()` and `write_tx_buf_as_slave()`, becomes a master again with `return_to_master()` after the
//! STOP, and then tries `is_slave_present(0x09)` again. The backchannel UART prints the three results. No
//! device answers at 0x09.
//!
//! The next read addresses eUSCI_B1 right after the STOP of its own transfer, which it flags as well. The
//! bus runs at 10 kHz from SMCLK, slow enough for eUSCI_B1 to start its transfer while eUSCI_B0 still sends
//! the address. The LaunchPad has no pull-ups on these pins, so all four have their internal pull-up on: two
//! on each line.
//! (A master's START waits "until the bus is available": SLAU445I 24.3.5.2.1, p. 637. A master addressed as
//! a slave loses arbitration and becomes a slave: UCALIFG, SLAU445I Table 24-2, p. 646. A STOP sets
//! UCSTPIFG in slave and master mode: SLAU445I Table 24-2, p. 646. The I2C pins: SLASEO7C Table 9-11, p. 54.
//! The internal pull-ups are 20 kΩ to 50 kΩ: SLASEO7C 8.12.4.1, p. 31. The LaunchPad has no pull-ups on
//! these pins: SLAU802 Figure 18, p. 24.)
//!
//! How to test (two jumper wires):
//! 1. Connect SDA, P1.2 (J1 pin 10), to P3.2 (J2 pin 15), and SCL, P1.3 (J1 pin 9), to P3.6 (J2 pin 14).
//!    (Header pins: SLAU802 Figure 10, p. 13.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 3. Expected, once a second: `first: Err(AddressedAsSlave), read: Ok(0), again: Ok(false)`, then
//!    `read: Ok(1)` and so on, counting up. Without the wires nothing is printed: eUSCI_B1 waits for the
//!    read forever.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cEvent, I2cMasterSlave, I2cUsci, TransmissionMode},
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// eUSCI_B1's own address
const ADDRESS_B1: u8 = 0x1A;
/// No device answers at this address
const ABSENT: u8 = 0x09;

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

    // Masters with other masters on the bus (UCMM = 1), eUSCI_B1 with an own address in UCBxI2COA0
    // (SLAU445I 24.3.5.2, p. 636), clocked by SMCLK. fBitClock = fBRCLK/UCBRx, at most fBRCLK/8 with several
    // masters (SLAU445I 24.3.7, p. 642).
    let mut b0 = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
        .as_multi_master()
        .use_smclk(&smclk, 800) // 8MHz / 800 = 10kHz
        .configure(b0_scl, b0_sda);
    let mut b1 = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b1, GlitchFilter::Max50ns)
        .as_master_slave(ADDRESS_B1)
        .use_smclk(&smclk, 800) // 8MHz / 800 = 10kHz
        .configure(b1_scl, b1_sda);

    let mut count: u8 = 0;
    loop {
        // eUSCI_B0: START, then the address with the read bit (SLAU445I 24.3.5.2.2, p. 639). The bus is free,
        // so the START comes at once, and the address byte with its acknowledge takes 9 SCL periods, 900 µs:
        // 100 µs later eUSCI_B0 still sends it.
        b0.send_start(ADDRESS_B1, TransmissionMode::Receive).ok();
        delay.delay_us(100);

        // eUSCI_B1: a blocking transfer of its own, whose START waits for the bus. eUSCI_B0 addresses it
        // meanwhile, so it ends with `AddressedAsSlave`.
        let first = b1.is_slave_present(ABSENT);

        // eUSCI_B1 as a slave transmitter (SLAU445I 24.3.5.1.1, p. 633)
        wait_for(&mut b1, I2cEvent::ReadStart);
        nb::block!(b1.write_tx_buf_as_slave(count)).ok();

        // eUSCI_B0: the byte is followed by a NACK and the STOP, so the STOP is scheduled before it arrives
        // (SLAU445I 24.3.5.2.2, p. 639). The transaction ends once the STOP is on the bus.
        b0.schedule_stop();
        let read = nb::block!(b0.read_rx_buf());
        nb::block!(b0.stop_sent()).ok();

        // eUSCI_B1: after the STOP, a master again (UCMST: SLAU445I Table 24-4, p. 649), and its transfer
        // once more
        wait_for(&mut b1, I2cEvent::Stop);
        b1.return_to_master();
        let again = b1.is_slave_present(ABSENT);

        writeln!(tx, "first: {:?}, read: {:?}, again: {:?}\r", first, read, again).ok();
        count = count.wrapping_add(1);
        delay.delay_ms(1000);
    }
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
