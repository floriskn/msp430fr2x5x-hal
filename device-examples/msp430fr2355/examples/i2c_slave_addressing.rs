//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An I2C slave that answers several addresses. Once a second a master on the same chip tries each address
//! in turn: it writes a byte and, after a repeated start, reads one back. The backchannel UART prints which
//! own address register of the slave answered, or `NACK`.
//!
//! eUSCI_B1 is the slave. Its first own address, 0x40, has an address mask that ignores the two lowest
//! bits, so 0x40 to 0x43 match it. Those wait for the software to acknowledge them, and it refuses 0x43.
//! Its other own addresses are 0x50, 0x60 and 0x70, each with its own receive and transmit flags. The slave
//! answers a read with the number of the register that matched: 0 for UCBxI2COA0 to 3 for UCBxI2COA3.
//! eUSCI_B0 is the master, at 100 kHz from SMCLK. The LaunchPad has no pull-ups on these pins, so all four
//! have their internal pull-up on: two on each line.
//! With `EARLY_TX` set to true, the slave has the early transmit interrupt and answers the general call
//! instead, and the eUSCI acknowledges the four addresses of the mask itself. The early transmit interrupt
//! needs the other own addresses off. Its transmit flag comes at each START, before the address is known,
//! and LED1 lights when it does.
//! (Several own addresses: SLAU445I 24.3.9.1, p. 644. The address mask and the software acknowledge,
//! UCSWACK: SLAU445I 24.3.9.2, p. 644. The early transmit interrupt: SLAU445I 24.3.11.2, p. 645. The
//! general call: UCGCEN, SLAU445I Table 24-11, p. 656; SLAU445I Figure 24-10, p. 635. The I2C pins:
//! SLASEC4D Table 6-14, p. 72. The internal pull-ups are 20 kΩ to 50 kΩ: SLASEC4D Table 5-11, p. 43. The
//! LaunchPad has no pull-ups on these pins, and LED1 on P1.0 is red: SLAU680 Figure 18, p. 26.)
//!
//! How to test (two jumper wires):
//! 1. Connect SDA, P1.2 (J1 pin 10), to P4.6 (J2 pin 15), and SCL, P1.3 (J1 pin 9), to P4.7 (J2 pin 14).
//!    (Header pins: SLAU680 Figure 10, p. 15.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 3. Expected, once a second: `0x40: I2COA0, 0x42: I2COA0, 0x43: NACK, 0x44: NACK, 0x50: I2COA1,
//!    0x60: I2COA2, 0x70: I2COA3, general call: NACK`, on one line. 0x43 matches the mask but is refused,
//!    and 0x44 doesn't match it. LED1 stays off.
//! 4. Set `EARLY_TX` to true and flash again. Expected: `0x40: I2COA0, 0x42: I2COA0, 0x43: I2COA0,
//!    0x44: NACK, 0x50: NACK, 0x60: NACK, 0x70: NACK, general call: ack`, and LED1 lights.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::{Cell, RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*, i2c::I2c};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cInterruptFlags as Flags, I2cSlave, I2cVector, OwnAddressSlot},
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use msp430fr2355::{interrupt, EUsciB1};
use panic_msp430 as _;

/// Use the early transmit interrupt and the general call, instead of the other own addresses and the
/// software acknowledge
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

static SLAVE: Mutex<RefCell<Option<I2cSlave<EUsciB1>>>> = Mutex::new(RefCell::new(None));
/// From a START with an own address to the STOP
static ADDRESSED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
/// Set when the transmit flag came before the slave was addressed
static EARLY: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // The master, eUSCI_B0: P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SELx = 01 (SLASEC4D Table 6-63, p. 96).
    // The slave, eUSCI_B1: P4.7 = UCB1SCL and P4.6 = UCB1SDA with P4SELx = 01 (SLASEC4D Table 6-66, p. 102).
    // SDA and SCL need pull-ups (SLAU445I 24.3, p. 629): the internal ones here, two on each line.
    let m_scl = p1.pin3.pullup().to_alternate1();
    let m_sda = p1.pin2.pullup().to_alternate1();
    let sl_scl = p4.pin7.pullup().to_alternate1();
    let sl_sda = p4.pin6.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SELx = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::new(
        periph.e_usci_a1,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p4.pin3.to_alternate1());

    // The only master on the bus (UCMM = 0), clocked by SMCLK, UCSSELx = 10b (SLAU445I Table 24-4,
    // p. 649), with fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642)
    let mut master = I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
        .as_single_master()
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(m_scl, m_sda);

    // The slave: UCBxI2COA0, and UCBxADDMASK with the bits to compare (SLAU445I Table 24-16, p. 659)
    let slave = I2cConfig::new(periph.e_usci_b1, GlitchFilter::Max50ns)
        .as_slave(OWN_ADDRESS_0)
        .address_mask(MASK);
    let slave = if EARLY_TX {
        // UCTXIFG0 at each START, which needs UCBxI2COA1 to UCBxI2COA3 off (UCETXINT: SLAU445I Table 24-5,
        // p. 651), and the general call (UCGCEN)
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
    let mut slave = slave.configure(sl_scl, sl_sda);

    // UCSTTIE, UCSTPIE, and the receive and transmit interrupts of the four own addresses (SLAU445I
    // Table 24-18, p. 660 to p. 661). Set GIE, which masks every maskable interrupt while clear (SLAU445I
    // 1.3.3, p. 33).
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
    with(|cs| SLAVE.borrow_ref_mut(cs).replace(slave));
    unsafe { enable_interrupts() };

    loop {
        for &address in ADDRESSES.iter() {
            // A byte written, then one read after a repeated start (SLAU445I 24.3.5.2.1, p. 637). A NACK
            // ends the write with a STOP, and the read isn't tried.
            let mut reply = [0];
            match master.write_read(address, &[address], &mut reply) {
                Ok(()) => write!(tx, "0x{:02x}: I2COA{}, ", address, reply[0]).ok(),
                Err(_) => write!(tx, "0x{:02x}: NACK, ", address).ok(),
            };
        }
        // The general call: a write to address 0, which a slave with UCGCEN receives (SLAU445I Figure 24-10,
        // p. 635). The byte is any: no other device is on this bus.
        let answered = master.write(GENERAL_CALL, &[0x5A]).is_ok();
        writeln!(tx, "general call: {}\r", if answered { "ack" } else { "NACK" }).ok();

        led1.set_state(with(|cs| EARLY.borrow(cs).get()).into()).ok();
        delay.delay_ms(1000);
    }
}

// The eUSCI_B1 vector (FFDEh: SLASEC4D Table 6-2, p. 64). `interrupt_source()` reads UCB1IV, which clears
// the flag it reports (SLAU445I 24.3.11.5, p. 646).
#[interrupt]
fn EUSCI_B1() {
    with(|cs| {
        let mut slave = SLAVE.borrow_ref_mut(cs);
        let Some(slave) = slave.as_mut() else { return };
        let addressed = ADDRESSED.borrow(cs);
        match slave.interrupt_source() {
            // A START with an own address (SLAU445I Table 24-2, p. 646). With UCSWACK, a match of
            // UCBxI2COA0 holds SCL low until the software decides, after reading the address from UCBxADDRX
            // (SLAU445I 24.3.9.2, p. 644; SLAU445I 24.3.7.2, p. 643).
            I2cVector::StartReceived => {
                addressed.set(true);
                let address = slave.received_address();
                if !EARLY_TX && (address & MASK) == (OWN_ADDRESS_0 as u16 & MASK) {
                    slave.acknowledge_address(address != REFUSED as u16);
                }
            }
            // A read: answer with the number of the own address register. Each one has its own transmit
            // flag (SLAU445I 24.3.11.1, p. 645). With UCETXINT, UCTXIFG0 comes at the START, before the
            // address (SLAU445I 24.3.11.2, p. 645).
            I2cVector::TxBufEmpty => {
                if !addressed.get() {
                    EARLY.borrow(cs).set(true);
                }
                unsafe { slave.write_tx_buf_unchecked(0) };
            }
            I2cVector::Slave1TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(1) },
            I2cVector::Slave2TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(2) },
            I2cVector::Slave3TxBufEmpty => unsafe { slave.write_tx_buf_unchecked(3) },
            // The written byte isn't used, but reading it frees the receive buffer (SLAU445I 24.3.5.1.2,
            // p. 634)
            I2cVector::RxBufFull
            | I2cVector::Slave1RxBufFull
            | I2cVector::Slave2RxBufFull
            | I2cVector::Slave3RxBufFull => {
                let _ = unsafe { slave.read_rx_buf_unchecked() };
            }
            // UCSTPIFG is set by every STOP on the bus (SLAU445I Table 24-2, p. 646)
            I2cVector::StopReceived => addressed.set(false),
            _ => {}
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
