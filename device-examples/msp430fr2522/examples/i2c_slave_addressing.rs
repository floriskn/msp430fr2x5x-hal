//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An I2C slave that answers several addresses, tested with a second MSP430FR2522 board as the master: the
//! MSP430FR25x2 has only one eUSCI_B. Once a second the master tries each address in turn: it writes a byte
//! and, after a repeated start, reads one back. It prints which own address register of the slave
//! answered, or `NACK`.
//!
//! Both boards run this example, and `MASTER` picks the role. The slave's first own address, 0x40, has an
//! address mask that ignores the two lowest bits, so 0x40 to 0x43 match it. Those wait for the software to
//! acknowledge them, and it refuses 0x43. Its other own addresses are 0x50, 0x60 and 0x70, each with its
//! own receive and transmit flags. The slave answers a read with the number of the register that matched:
//! 0 for UCBxI2COA0 to 3 for UCBxI2COA3. It reads its flags through the interrupt vector, with the
//! interrupts enabled but GIE off. The master runs at 100 kHz from SMCLK. Both boards turn their internal
//! pull-ups on: two on each line. There's no LaunchPad for the MSP430FR25x2.
//! With `EARLY_TX` set to true, the slave has the early transmit interrupt and answers the general call
//! instead, and the eUSCI acknowledges the four addresses of the mask itself. The early transmit interrupt
//! needs the other own addresses off. Its transmit flag comes at each START, before the address is known,
//! and the slave's LED on P1.0 lights when it does.
//! (Several own addresses: SLAU445I 24.3.9.1, p. 644. The address mask and the software acknowledge,
//! UCSWACK: SLAU445I 24.3.9.2, p. 644. The early transmit interrupt: SLAU445I 24.3.11.2, p. 645. The
//! general call: UCGCEN, SLAU445I Table 24-11, p. 656; SLAU445I Figure 24-10, p. 635. One eUSCI_B:
//! SLASEE4C Table 3-1, p. 8. Its I2C pins, with USCIBRMP = 0, and UCA0TXD on P1.4: SLASEE4C Table 6-11,
//! p. 53. The internal pull-ups are 20 kΩ to 50 kΩ: SLASEE4C Table 5-10, p. 29. No board document covers
//! the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (a second MSP430FR2522 board, three wires, a 3.3-V USB-to-UART adapter, an LED and a
//! resistor):
//! 1. Connect the two boards: P1.2 (SDA) to P1.2, P1.3 (SCL) to P1.3, and GND to GND.
//! 2. On the slave board, connect the LED with a series resistor (about 1 kΩ) from P1.0 to GND. On the
//!    master board, connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port at
//!    9600 baud.
//! 3. Flash this example to the slave board. Set `MASTER` to true and flash it to the master board.
//! 4. Expected, once a second: `0x40: I2COA0, 0x42: I2COA0, 0x43: NACK, 0x44: NACK, 0x50: I2COA1,
//!    0x60: I2COA2, 0x70: I2COA3, general call: NACK`, on one line. 0x43 matches the mask but is refused,
//!    and 0x44 doesn't match it. The LED stays off.
//! 5. Set `EARLY_TX` to true, `MASTER` back to false, and flash the slave board again. Expected: `0x40:
//!    I2COA0, 0x42: I2COA0, 0x43: I2COA0, 0x44: NACK, 0x50: NACK, 0x60: NACK, 0x70: NACK, general call: ack`,
//!    and the LED lights.
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*, i2c::I2c};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cInterruptFlags as Flags, I2cVector, OwnAddressSlot},
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// This board is the master, which prints; false for the slave
const MASTER: bool = false;
/// The slave uses the early transmit interrupt and the general call, instead of the other own addresses and
/// the software acknowledge
const EARLY_TX: bool = false;
/// The first own address, and the mask that makes its two lowest bits don't care: 0x40 to 0x43 match
const OWN_ADDRESS_0: u8 = 0x40;
const MASK: u16 = 0x3FC;
/// Matches through the mask, but the software doesn't acknowledge it
const REFUSED: u8 = 0x43;
/// The other own addresses, UCBxI2COA1 to UCBxI2COA3
const OWN_ADDRESS_1: u8 = 0x50;
const OWN_ADDRESS_2: u8 = 0x60;
const OWN_ADDRESS_3: u8 = 0x70;
/// The addresses the master tries, before the general call
const ADDRESSES: [u8; 7] = [0x40, 0x42, 0x43, 0x44, 0x50, 0x60, 0x70];
const GENERAL_CALL: u8 = 0x00;

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

    if MASTER {
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

        // The only master on the bus (UCMM = 0), clocked by SMCLK, UCSSELx = 10b (SLAU445I Table 24-4,
        // p. 649), with fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642)
        let mut master = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_single_master()
            .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
            .configure(scl, sda);

        loop {
            for &address in ADDRESSES.iter() {
                // A byte written, then one read after a repeated start (SLAU445I 24.3.5.2.1, p. 637). A NACK
                // ends the write with a STOP, and the read isn't tried.
                let mut reply = [0];
                tx.write_all(b"0x").ok();
                print_hex(&mut tx, address);
                match master.write_read(address, &[address], &mut reply) {
                    Ok(()) => {
                        tx.write_all(b": I2COA").ok();
                        tx.write_all(&[digit(reply[0])]).ok();
                    }
                    Err(_) => {
                        tx.write_all(b": NACK").ok();
                    }
                }
                tx.write_all(b", ").ok();
            }
            // The general call: a write to address 0, which a slave with UCGCEN receives (SLAU445I
            // Figure 24-10, p. 635). The byte is any: no other device is on this bus.
            let answered = master.write(GENERAL_CALL, &[0x5A]).is_ok();
            let line: &[u8] = if answered { b"general call: ack\r\n" } else { b"general call: NACK\r\n" };
            tx.write_all(line).ok();
            delay.delay_ms(1000);
        }
    } else {
        // No board document covers the LED on P1.0: there is none for the MSP430FR25x2. P1.0 is a GPIO
        // output, P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58).
        let mut led = p1.pin0.to_output_low();

        // The slave: UCBxI2COA0, and UCBxADDMASK with the bits to compare (SLAU445I Table 24-16, p. 659)
        let slave = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_slave(OWN_ADDRESS_0)
            .address_mask(MASK);
        let slave = if EARLY_TX {
            // UCTXIFG0 at each START, which needs UCBxI2COA1 to UCBxI2COA3 off (UCETXINT: SLAU445I
            // Table 24-5, p. 651), and the general call (UCGCEN)
            slave.early_tx_interrupt().general_call()
        } else {
            // The software acknowledges UCBxI2COA0's matches (UCSWACK: SLAU445I Table 24-5, p. 651), and the
            // eUSCI acknowledges UCBxI2COA1 to UCBxI2COA3 itself (SLAU445I 24.3.9.2, p. 644)
            slave
                .software_address_ack()
                .own_address(OwnAddressSlot::_1, OWN_ADDRESS_1)
                .own_address(OwnAddressSlot::_2, OWN_ADDRESS_2)
                .own_address(OwnAddressSlot::_3, OWN_ADDRESS_3)
        };
        let mut slave = slave.configure(scl, sda);

        // UCSTTIE, UCSTPIE, and the receive and transmit interrupts of the four own addresses (SLAU445I
        // Table 24-18, p. 660 to p. 661), so that the vector reports them (SLAU445I 24.3.11.5, p. 646). GIE
        // stays clear, so they request no interrupt (SLAU445I 1.3.3, p. 33).
        slave.set_interrupts(
            Flags::StartReceived
                | Flags::StopReceived
                | Flags::RxBufFull
                | Flags::TxBufEmpty
                | Flags::Slave1RxBufFull
                | Flags::Slave1TxBufEmpty
                | Flags::Slave2RxBufFull
                | Flags::Slave2TxBufEmpty
                | Flags::Slave3RxBufFull
                | Flags::Slave3TxBufEmpty,
        );

        // From a START with an own address to the STOP
        let mut addressed = false;
        loop {
            // Reading UCB0IV clears the flag it reports (SLAU445I 24.3.11.5, p. 646)
            match slave.interrupt_source() {
                // A START with an own address (SLAU445I Table 24-2, p. 646). With UCSWACK, a match of
                // UCBxI2COA0 holds SCL low until the software decides, after reading the address from
                // UCBxADDRX (SLAU445I 24.3.9.2, p. 644; SLAU445I 24.3.7.2, p. 643).
                I2cVector::StartReceived => {
                    addressed = true;
                    let address = slave.received_address();
                    if !EARLY_TX && (address & MASK) == (OWN_ADDRESS_0 as u16 & MASK) {
                        slave.acknowledge_address(address != REFUSED as u16);
                    }
                }
                // A read: answer with the number of the own address register. Each one has its own transmit
                // flag (SLAU445I 24.3.11.1, p. 645). With UCETXINT, UCTXIFG0 comes at the START, before the
                // address (SLAU445I 24.3.11.2, p. 645).
                I2cVector::TxBufEmpty => {
                    if !addressed {
                        led.set_high().ok();
                    }
                    unsafe { slave.write_tx_buf_unchecked(0) };
                }
                I2cVector::Slave1TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(1) },
                I2cVector::Slave2TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(2) },
                I2cVector::Slave3TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(3) },
                // The written byte isn't used, but reading it frees the receive buffer (SLAU445I
                // 24.3.5.1.2, p. 634)
                I2cVector::RxBufFull
                | I2cVector::Slave1RxBufFull
                | I2cVector::Slave2RxBufFull
                | I2cVector::Slave3RxBufFull => {
                    let _ = unsafe { slave.read_rx_buf_unchecked() };
                }
                // UCSTPIFG is set by every STOP on the bus (SLAU445I Table 24-2, p. 646)
                I2cVector::StopReceived => addressed = false,
                _ => {}
            }
        }
    }
}

/// Print `byte` as two hex digits. The text is put together by hand: `core::fmt` doesn't fit in the
/// 7.25 KB of program FRAM (SLASEE4C Table 6-19, p. 62) in a build with the oldest supported Rust.
fn print_hex(tx: &mut impl Write, byte: u8) {
    tx.write_all(&[digit(byte >> 4), digit(byte)]).ok();
}

/// The hex digit of the low 4 bits of `n`
fn digit(n: u8) -> u8 {
    b"0123456789abcdef"[(n & 0x0F) as usize]
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
