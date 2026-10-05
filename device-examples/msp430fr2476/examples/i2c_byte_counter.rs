//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The I2C byte counter, with the automatic STOP, and the clock low timeout: once a second a master reads
//! 3 bytes from a slave on the same chip, and its eUSCI sends the STOP by itself after the third. The
//! backchannel UART prints the bytes, whether the byte counter flag was set, and whether the clock low
//! timeout went off: every other time the slave waits 100 ms before its second byte, holding SCL low.
//!
//! eUSCI_B1 is the master, at 100 kHz. With `ack_last_byte()` it acknowledges the third byte too, where the
//! I2C standard has a NACK before the STOP. That's only for slaves that release SDA after a fixed number of
//! bytes, so the slave, eUSCI_B0 at address 0x1A, sends 0xFF after its 3 bytes: all bits high, SDA released.
//! The master reads its flags through the interrupt vector, with the interrupts enabled but GIE off. The
//! LaunchPad has no pull-ups on these pins, so all four have their internal pull-up on: two on each line.
//! With `EXTERNAL_CLOCK` set to true, the master's bit clock comes from its clock input pin, UCB1CLK, instead
//! of SMCLK: SMCLK comes out on P1.7, and a wire takes it to P3.5.
//! (The byte counter and the automatic STOP: SLAU445I 24.3.8, p. 643 to p. 644. UCSTPNACK: SLAU445I
//! Table 24-5, p. 651. The clock low timeout: SLAU445I 24.3.7.3, p. 643. A slave transmitter holds SCL low
//! until its next byte is written: SLAU445I 24.3.5.1.1, p. 633. UCLKI, the clock on the SPI clock pin:
//! SLAU445I Figure 24-1, p. 628; SLAU445I Table 24-4, p. 649. The I2C pins, and UCB1CLK in the SPI column:
//! SLASEO7C Table 9-11, p. 54. SMCLK on P1.7: SLASEO7C Table 9-23, p. 65. The internal pull-ups are 20 kΩ
//! to 50 kΩ: SLASEO7C 8.12.4.1, p. 31. The LaunchPad has no pull-ups on these pins: SLAU802 Figure 18,
//! p. 24.)
//!
//! How to test (two jumper wires, and optionally the scope and a third wire):
//! 1. Connect SDA, P1.2 (J1 pin 10), to P3.2 (J2 pin 15), and SCL, P1.3 (J1 pin 9), to P3.6 (J2 pin 14).
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 3. Expected, once a second, in turn: `read 01 02 03, byte counter: yes, clock low timeout: no` and
//!    `read 01 02 03, byte counter: yes, clock low timeout: yes`.
//! 4. Scope, ground on GND (J3 pin 22), 50 µs/div, trigger on CH1 falling: CH1 on SDA, P3.2, and CH2 on
//!    SCL, P3.6. The scope's I2C decoder (Analysis > Decode) shows a read of 0x1A with the bytes 01, 02 and
//!    03, each followed by an ACK, the third one too, and then the STOP.
//! 5. Set `EXTERNAL_CLOCK` to true, connect P1.7 (J3 pin 23) to P3.5 (J1 pin 7), and flash again. Expected:
//!    the same lines. Without that wire the master has no clock, and nothing is printed.
//! (Header pins: SLAU802 Figure 10, p. 13.)
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

/// Clock the master from SMCLK through a wire to its clock input pin, instead of from SMCLK directly
const EXTERNAL_CLOCK: bool = false;
const SLAVE_ADDRESS: u8 = 0x1A;
/// The bytes per read, which the byte counter counts
const COUNT: u8 = 3;
/// The slave's bytes. After them it sends 0xFF, which leaves SDA released.
const DATA: [u8; COUNT as usize] = [1, 2, 3];

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);

    // The master, eUSCI_B1: P3.6 = UCB1SCL and P3.2 = UCB1SDA with P3SEL = 01 (SLASEO7C Table 9-25, p. 67).
    // The slave, eUSCI_B0: P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SEL = 01 (SLASEO7C Table 9-23, p. 65).
    // SDA and SCL need pull-ups (SLAU445I 24.3, p. 629): the internal ones here, two on each line.
    let m_scl = p3.pin6.pullup().to_alternate1();
    let m_sda = p3.pin2.pullup().to_alternate1();
    let sl_scl = p1.pin3.pullup().to_alternate1();
    let sl_sda = p1.pin2.pullup().to_alternate1();
    // SMCLK on P1.7, P1SEL = 10 and P1DIR = 1 (SLASEO7C Table 9-23, p. 65), for `EXTERNAL_CLOCK`
    let _smclk_out = p1.pin7.to_output().to_alternate2();

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

    // The only master on the bus (UCMM = 0). After COUNT bytes the eUSCI sets UCBCNTIFG and sends the STOP
    // (UCASTPx = 10b, UCBxTBCNT), it acknowledges the last byte too (UCSTPNACK), and it sets UCCLTOIFG when
    // SCL stays low for 135000 MODCLK cycles (UCCLTO = 01b) (SLAU445I Table 24-5, p. 651; SLAU445I
    // Table 24-8, p. 654).
    let master = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b1, GlitchFilter::Max50ns)
        .as_single_master()
        .byte_counter(COUNT, true)
        .ack_last_byte()
        .clock_low_timeout(ClockLowTimeout::_28ms);
    let master = if EXTERNAL_CLOCK {
        // UCSSELx = 00b: UCLKI, the clock on the SPI clock pin, UCB1CLK: P3.5 with P3SEL = 01 (SLAU445I
        // Table 24-4, p. 649; SLAU445I Figure 24-1, p. 628; SLASEO7C Table 9-25, p. 67). Its pull-down keeps
        // it low without the wire. fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642).
        master.use_uclk(p3.pin5.pulldown().to_alternate1(), 80) // 8MHz / 80 = 100kHz
    } else {
        // UCSSELx = 10b: SMCLK (SLAU445I Table 24-4, p. 649)
        master.use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
    };
    let mut master = master.configure(m_scl, m_sda);
    // UCBCNTIE, UCCLTOIE and UCSTPIE, so that the vector reports these flags (SLAU445I 24.3.11.5, p. 646).
    // GIE stays clear, so they request no interrupt (SLAU445I 1.3.3, p. 33).
    master.set_interrupts(Flags::ByteCounterZero | Flags::ClockLowTimeout | Flags::StopReceived);

    let mut slave = I2cConfig::<_, _, _, DefaultMapping>::new(periph.e_usci_b0, GlitchFilter::Max50ns)
        .as_slave(SLAVE_ADDRESS)
        .configure(sl_scl, sl_sda);

    let mut stall = false;
    loop {
        // The master: START, then the address with the read bit (SLAU445I 24.3.5.2.2, p. 639)
        master.send_start(SLAVE_ADDRESS, TransmissionMode::Receive);
        let mut data = [0; COUNT as usize];
        let (mut received, mut sent) = (0, 0);
        let (mut counted, mut timeout, mut stopped) = (false, false, false);
        while !(stopped && received == data.len()) {
            // The slave: DATA, then 0xFF. Every other read it waits 100 ms before the second byte, longer
            // than 135000 cycles of the slowest MODCLK, 3.0 MHz: 45 ms (SLASEO7C 8.12.3.6, p. 30).
            match slave.poll() {
                Ok(I2cEvent::ReadStart) => {
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
            // The master: each received byte, and the flags. Reading UCB1IV clears the flag it reports
            // (SLAU445I 24.3.11.5, p. 646).
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
        writeln!(
            tx,
            "read {:02x} {:02x} {:02x}, byte counter: {}, clock low timeout: {}\r",
            data[0],
            data[1],
            data[2],
            yes_no(counted),
            yes_no(timeout)
        )
        .ok();
        stall = !stall;
        delay.delay_ms(1000);
    }
}

fn yes_no(flag: bool) -> &'static str {
    if flag {
        "yes"
    } else {
        "no"
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
