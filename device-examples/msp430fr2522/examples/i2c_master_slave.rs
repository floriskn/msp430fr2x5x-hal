//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Two I2C master-slaves take turns as master, tested with a second MSP430FR2522 board: the MSP430FR25x2
//! has only one eUSCI_B. Each board in turn writes a count to the other and reads its answer, and prints
//! the answer.
//!
//! Both boards run this example, as an `I2cMasterSlave`: a master on a bus with other masters that also
//! answers its own address. `FIRST` picks the board that starts, at address 0x1A; the other one is 0x1B.
//! Addressed by the other board while it's idle in master mode, the eUSCI loses arbitration and works as a
//! slave, and `return_to_master()` makes it a master again after the STOP. As master each board uses
//! `send_start()`, `write_tx_buf_as_master()`, `tx_buf_empty()`, `schedule_stop()`, `read_rx_buf_as_master()`
//! and `stop_sent()`, and as slave `poll()`, `read_rx_buf_as_slave()` and `write_tx_buf_as_slave()`. The
//! slave answers the byte it got plus one. Both run at 100 kHz from SMCLK. Both boards turn their internal
//! pull-ups on: two on each line.
//! There's no LaunchPad for the MSP430FR25x2.
//! (Masters with an own address, UCMM = 1: SLAU445I 24.3.5.2, p. 636. A master addressed as a slave loses
//! arbitration and becomes a slave: UCALIFG, SLAU445I Table 24-2, p. 646. One eUSCI_B: SLASEE4C Table 3-1,
//! p. 8. Its I2C pins, with USCIBRMP = 0, and UCA0TXD on P1.4: SLASEE4C Table 6-11, p. 53. The internal
//! pull-ups are 20 kΩ to 50 kΩ: SLASEE4C Table 5-10, p. 29. No board document covers the parts to connect:
//! there is none for the MSP430FR25x2.)
//!
//! How to test (a second MSP430FR2522 board, three wires, and a 3.3-V USB-to-UART adapter):
//! 1. Connect the two boards: P1.2 (SDA) to P1.2, P1.3 (SCL) to P1.3, and GND to GND.
//! 2. On either board, connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at
//!    9600 baud.
//! 3. Flash this example to one board. Set `FIRST` to true and flash it to the other board. Reset the
//!    `FIRST` board, so that it starts after the other one.
//! 4. Expected, once a second: `to the other board: Ok(1)`, then `Ok(2)` and so on, counting up.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cEvent, I2cMasterSlave, I2cMasterSlaveErr, TransmissionMode},
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use msp430fr25x2::EUsciB0;
use panic_msp430 as _;

/// This board starts as master; false on the other board
const FIRST: bool = false;
const OWN_ADDRESS: u8 = if FIRST { 0x1A } else { 0x1B };
const OTHER_ADDRESS: u8 = if FIRST { 0x1B } else { 0x1A };

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SELx = 01 in the default mapping, USCIBRMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). SDA and SCL need pull-ups (SLAU445I 24.3, p. 629): the
    // internal ones of both boards here, two on each line.
    let scl = p1.pin3.pullup().to_alternate1();
    let sda = p1.pin2.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58), 8N1 (SLAU445I Table 22-8, p. 593)
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

    // A master with other masters on the bus (UCMM = 1) and an own address in UCBxI2COA0 (SLAU445I
    // 24.3.5.2, p. 636), clocked by SMCLK. With several masters fBitClock = fBRCLK/UCBRx may be at most
    // fBRCLK/8 (SLAU445I 24.3.7, p. 642).
    let mut i2c = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
        .as_master_slave(OWN_ADDRESS)
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(scl, sda);

    let mut count: u8 = 0;
    let mut master = FIRST;
    loop {
        if master {
            delay.delay_ms(500);
            let answer = master_turn(&mut i2c, count);
            tx.write_all(b"to the other board: ").ok();
            match answer {
                Ok(byte) => {
                    tx.write_all(b"Ok(").ok();
                    print_decimal(&mut tx, byte);
                    tx.write_all(b")").ok();
                }
                Err(error) => {
                    let name: &[u8] = match error {
                        I2cMasterSlaveErr::GotNACK(_) => b"Err(GotNACK)",
                        I2cMasterSlaveErr::ArbitrationLost => b"Err(ArbitrationLost)",
                        I2cMasterSlaveErr::AddressedAsSlave => b"Err(AddressedAsSlave)",
                        _ => b"Err",
                    };
                    tx.write_all(name).ok();
                }
            }
            tx.write_all(b"\r\n").ok();
            count = count.wrapping_add(1);
        } else {
            slave_turn(&mut i2c);
        }
        master = !master;
    }
}

/// Write `count` to the other board, and read its answer after a repeated start
fn master_turn(i2c: &mut I2cMasterSlave<EUsciB0>, count: u8) -> Result<u8, I2cMasterSlaveErr> {
    // START, the address with the write bit, and the byte (SLAU445I 24.3.5.2.1, p. 637)
    i2c.send_start(OTHER_ADDRESS, TransmissionMode::Transmit)?;
    nb::block!(i2c.write_tx_buf_as_master(count))?;
    // UCTXIFG0 is set again when the byte moves into the shift register, and data is only sent while
    // UCTXSTT is clear (SLAU445I 24.3.5.2.1, p. 637), so the repeated start waits for it
    nb::block!(i2c.tx_buf_empty())?;

    // A repeated start, with the read bit. The next byte received is followed by a NACK and the STOP, so the
    // STOP is scheduled before it arrives (SLAU445I 24.3.5.2.2, p. 639). The transaction ends once the STOP
    // is on the bus.
    i2c.send_start(OTHER_ADDRESS, TransmissionMode::Receive)?;
    i2c.schedule_stop();
    let answer = nb::block!(i2c.read_rx_buf_as_master())?;
    nb::block!(i2c.stop_sent())?;
    Ok(answer)
}

/// Answer the other board: its byte plus one
fn slave_turn(i2c: &mut I2cMasterSlave<EUsciB0>) {
    // The byte arrives (SLAU445I 24.3.5.1.2, p. 634)
    wait_for(i2c, I2cEvent::WriteStart);
    let got = nb::block!(i2c.read_rx_buf_as_slave()).unwrap(); // Infallible
    // The answer (SLAU445I 24.3.5.1.1, p. 633)
    wait_for(i2c, I2cEvent::ReadStart);
    nb::block!(i2c.write_tx_buf_as_slave(got.wrapping_add(1))).ok();
    // After the STOP, a master again (UCMST: SLAU445I Table 24-4, p. 649)
    wait_for(i2c, I2cEvent::Stop);
    i2c.return_to_master();
}

/// Poll `i2c` until `event`
fn wait_for(i2c: &mut I2cMasterSlave<EUsciB0>, event: I2cEvent) {
    while i2c.poll() != Ok(event) {}
}

/// Print `n` in decimal. The text is put together by hand: `core::fmt` doesn't fit in the 7.25 KB of
/// program FRAM (SLASEE4C Table 6-19, p. 62) in a build with the oldest supported Rust.
fn print_decimal(tx: &mut impl Write, n: u8) {
    if n >= 100 {
        tx.write_all(&[b'0' + n / 100]).ok();
    }
    if n >= 10 {
        tx.write_all(&[b'0' + n / 10 % 10]).ok();
    }
    tx.write_all(&[b'0' + n % 10]).ok();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
