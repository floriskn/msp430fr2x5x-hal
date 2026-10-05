//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The I2C byte counter, with the automatic STOP, and the clock low timeout, tested with a second
//! MSP430FR2522 board as the slave: the MSP430FR25x2 has only one eUSCI_B. Once a second the master reads
//! 3 bytes from the slave, and its eUSCI sends the STOP by itself after the third. It prints the bytes,
//! whether the byte counter flag was set, and whether the clock low timeout went off: every other time the
//! slave waits 100 ms before its second byte, holding SCL low.
//!
//! Both boards run this example, and `MASTER` picks the role. The master runs at 100 kHz. With
//! `ack_last_byte()` it acknowledges the third byte too, where the I2C standard has a NACK before the STOP.
//! That's only for slaves that release SDA after a fixed number of bytes, so the slave, at address 0x1A,
//! sends 0xFF after its 3 bytes: all bits high, SDA released. The master reads its flags through the
//! interrupt vector, with the interrupts enabled but GIE off. Both boards turn their internal pull-ups on:
//! two on each line. There's no LaunchPad for the MSP430FR25x2.
//! The master's clock input, `use_uclk()`, isn't shown: no pin can feed it a clock from this device without
//! more parts. SMCLK and MCLK come out on P1.2 and P1.3, the I2C pins, and ACLK on P1.1, the clock input
//! itself.
//! (The byte counter and the automatic STOP: SLAU445I 24.3.8, p. 643 to p. 644. UCSTPNACK: SLAU445I
//! Table 24-5, p. 651. The clock low timeout: SLAU445I 24.3.7.3, p. 643. A slave transmitter holds SCL low
//! until its next byte is written: SLAU445I 24.3.5.1.1, p. 633. One eUSCI_B: SLASEE4C Table 3-1, p. 8. Its
//! I2C pins, with USCIBRMP = 0, and UCA0TXD on P1.4: SLASEE4C Table 6-11, p. 53. The clock outputs:
//! SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16, p. 60. The internal pull-ups are 20 kΩ to 50 kΩ:
//! SLASEE4C Table 5-10, p. 29. No board document covers the parts to connect: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (a second MSP430FR2522 board, three wires, a 3.3-V USB-to-UART adapter, and optionally the
//! scope):
//! 1. Connect the two boards: P1.2 (SDA) to P1.2, P1.3 (SCL) to P1.3, and GND to GND.
//! 2. On the master board, connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open its COM port
//!    at 9600 baud.
//! 3. Flash this example to the slave board. Set `MASTER` to true and flash it to the master board. If
//!    nothing is printed, reset the master.
//! 4. Expected, once a second, in turn: `read 01 02 03, byte counter: yes, clock low timeout: no` and
//!    `read 01 02 03, byte counter: yes, clock low timeout: yes`.
//! 5. Scope, ground on GND, 50 µs/div, trigger on CH1 falling: CH1 on SDA, P1.2, and CH2 on SCL, P1.3. The
//!    scope's I2C decoder (Analysis > Decode) shows a read of 0x1A with the bytes 01, 02 and 03, each
//!    followed by an ACK, the third one too, and then the STOP.
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{
        ClockLowTimeout, GlitchFilter, I2cConfig, I2cEvent, I2cInterruptFlags as Flags, I2cVector,
        TransmissionMode,
    },
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// This board is the master, which prints; false for the slave
const MASTER: bool = false;
const SLAVE_ADDRESS: u8 = 0x1A;
/// The bytes per read, which the byte counter counts
const COUNT: u8 = 3;
/// The slave's bytes. After them it sends 0xFF, which leaves SDA released.
const DATA: [u8; COUNT as usize] = [1, 2, 3];

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

        // The only master on the bus (UCMM = 0), clocked by SMCLK (UCSSELx = 10b: SLAU445I Table 24-4,
        // p. 649), with fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642). After COUNT bytes the eUSCI sets
        // UCBCNTIFG and sends the STOP (UCASTPx = 10b, UCBxTBCNT), it acknowledges the last byte too
        // (UCSTPNACK), and it sets UCCLTOIFG when SCL stays low for 135000 MODCLK cycles (UCCLTO = 01b)
        // (SLAU445I Table 24-5, p. 651; SLAU445I Table 24-8, p. 654).
        let mut master = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_single_master()
            .byte_counter(COUNT, true)
            .ack_last_byte()
            .clock_low_timeout(ClockLowTimeout::_28ms)
            .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
            .configure(scl, sda);
        // UCBCNTIE, UCCLTOIE and UCSTPIE, so that the vector reports these flags (SLAU445I 24.3.11.5,
        // p. 646). GIE stays clear, so they request no interrupt (SLAU445I 1.3.3, p. 33).
        master.set_interrupts(Flags::ByteCounterZero | Flags::ClockLowTimeout | Flags::StopReceived);

        loop {
            // START, then the address with the read bit (SLAU445I 24.3.5.2.2, p. 639)
            master.send_start(SLAVE_ADDRESS, TransmissionMode::Receive);
            let mut data = [0; COUNT as usize];
            let mut received = 0;
            let (mut counted, mut timeout, mut stopped) = (false, false, false);
            while !(stopped && received == data.len()) {
                // Each received byte, and the flags. Reading UCB0IV clears the flag it reports (SLAU445I
                // 24.3.11.5, p. 646).
                if let Ok(byte) = master.read_rx_buf() {
                    if let Some(slot) = data.get_mut(received) {
                        *slot = byte;
                    }
                    received += 1;
                }
                match master.interrupt_source() {
                    I2cVector::ByteCounterZero => counted = true,
                    I2cVector::ClockLowTimeout => timeout = true,
                    I2cVector::StopReceived => stopped = true,
                    _ => {}
                }
            }
            tx.write_all(b"read").ok();
            for byte in data {
                tx.write_all(b" ").ok();
                print_hex(&mut tx, byte);
            }
            tx.write_all(b", byte counter: ").ok();
            tx.write_all(yes_no(counted).as_bytes()).ok();
            tx.write_all(b", clock low timeout: ").ok();
            tx.write_all(yes_no(timeout).as_bytes()).ok();
            tx.write_all(b"\r\n").ok();
            delay.delay_ms(1000);
        }
    } else {
        let mut slave = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_slave(SLAVE_ADDRESS)
            .configure(scl, sda);

        // DATA, then 0xFF. A slave transmitter holds SCL low until its next byte is written (SLAU445I
        // 24.3.5.1.1, p. 633): every other read the slave waits 100 ms before the second byte, longer than
        // 135000 cycles of the slowest MODCLK, 3.8 MHz: 36 ms (SLASEE4C Table 5-9, p. 28).
        let (mut stall, mut sent) = (false, 0);
        loop {
            match slave.poll() {
                Ok(I2cEvent::ReadStart) => {
                    stall = !stall;
                    nb::block!(slave.write_tx_buf(DATA[0])).ok();
                    sent = 1;
                }
                Ok(I2cEvent::Read) => {
                    if stall && sent == 1 {
                        delay.delay_ms(100);
                    }
                    nb::block!(slave.write_tx_buf(DATA.get(sent).copied().unwrap_or(0xFF))).ok();
                    sent += 1;
                }
                _ => {}
            }
        }
    }
}

fn yes_no(flag: bool) -> &'static str {
    if flag {
        "yes"
    } else {
        "no"
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
