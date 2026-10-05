//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The I2C byte counter, with the automatic STOP, and the clock low timeout, tested with a second
//! MSP-EXP430FR2433 LaunchPad as the slave: the MSP430FR2433 has only one eUSCI_B. Once a second the master
//! reads 3 bytes from the slave, and its eUSCI sends the STOP by itself after the third. Its backchannel UART
//! prints the bytes, whether the byte counter flag was set, and whether the clock low timeout went off: every
//! other time the slave waits 100 ms before its second byte, holding SCL low.
//!
//! Both boards run this example, and `MASTER` picks the role. The master runs at 100 kHz. With
//! `ack_last_byte()` it acknowledges the third byte too, where the I2C standard has a NACK before the STOP.
//! That's only for slaves that release SDA after a fixed number of bytes, so the slave, at address 0x1A,
//! sends 0xFF after its 3 bytes: all bits high, SDA released. The master reads its flags through the
//! interrupt vector, with the interrupts enabled but GIE off. The LaunchPads have no pull-ups on these pins,
//! so both boards turn their internal ones on: two on each line.
//! With `EXTERNAL_CLOCK` set to true, the master's bit clock comes from its clock input pin, UCB0CLK on P1.1,
//! instead of SMCLK: SMCLK comes out on P1.7, and a wire takes it to P1.1.
//! (The byte counter and the automatic STOP: SLAU445I 24.3.8, p. 643 to p. 644. UCSTPNACK: SLAU445I
//! Table 24-5, p. 651. The clock low timeout: SLAU445I 24.3.7.3, p. 643. A slave transmitter holds SCL low
//! until its next byte is written: SLAU445I 24.3.5.1.1, p. 633. UCLKI, the clock on the SPI clock pin:
//! SLAU445I Figure 24-1, p. 628; SLAU445I Table 24-4, p. 649. One eUSCI_B: SLASE59F Table 3-1, p. 7. Its
//! I2C pins, and UCB0CLK in the SPI column: SLASE59F Table 6-10, p. 49. SMCLK on P1.7: SLASE59F Table 6-17,
//! p. 55. The internal pull-ups are 20 kΩ to 50 kΩ: SLASE59F Table 5-10, p. 27. The LaunchPad has no
//! pull-ups on these pins, and LED2 is on P1.1 through J11: SLAU739 Figure 18, p. 23.)
//!
//! How to test (a second MSP-EXP430FR2433 LaunchPad, three jumper wires, and optionally the scope and a
//! fourth wire):
//! 1. Connect the two LaunchPads: P1.2 (SDA, J1 pin 10) to P1.2, P1.3 (SCL, J1 pin 9) to P1.3, and GND
//!    (J3 pin 22) to GND.
//! 2. Flash this example to one board, the slave, with only that board plugged in. Set `MASTER` to true and
//!    flash it to the other board, the master, alone too. Then plug both in, put the TXD jumper of the
//!    master's J101 on, and open the COM port of its "MSP Application UART1" at 9600 baud (SLAU739 2.2.4,
//!    p. 9). If nothing is printed, press S3 (reset) on the master.
//! 3. Expected, once a second, in turn: `read 01 02 03, byte counter: yes, clock low timeout: no` and
//!    `read 01 02 03, byte counter: yes, clock low timeout: yes`.
//! 4. Scope, ground on GND (J3 pin 22), 50 µs/div, trigger on CH1 falling: CH1 on SDA, P1.2, and CH2 on
//!    SCL, P1.3. The scope's I2C decoder (Analysis > Decode) shows a read of 0x1A with the bytes 01, 02 and
//!    03, each followed by an ACK, the third one too, and then the STOP.
//! 5. Set `EXTERNAL_CLOCK` to true and flash the master again. Take off its jumper J11 (LED2), and connect
//!    its P1.7 (J1 pin 6) to P1.1 (J2 pin 19). Expected: the same lines. Without that wire the master has no
//!    clock, and nothing is printed.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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
    pmm::Pmm,
    prelude::*,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// This board is the master, which prints; false for the slave
const MASTER: bool = false;
/// Clock the master from SMCLK through a wire to its clock input pin, instead of from SMCLK directly
const EXTERNAL_CLOCK: bool = false;
const SLAVE_ADDRESS: u8 = 0x1A;
/// The bytes per read, which the byte counter counts
const COUNT: u8 = 3;
/// The slave's bytes. After them it sends 0xFF, which leaves SDA released.
const DATA: [u8; COUNT as usize] = [1, 2, 3];

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Hold the watchdog (WDTHOLD, SLAU445I Table 12-2, p. 366: after a PUC the WDT runs, SLAU445I 12.2.2,
    // p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5 (SLAU445I Table 2-7, p. 97). SLASE59F 6.10.3, p. 46 sets the ports up before
    // that; clearing it first leaves the pins inputs until they are set up (SLAU445I 8.3.1, p. 316).
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // P1.3 = UCB0SCL and P1.2 = UCB0SDA with P1SELx = 01 (SLASE59F Table 6-17, p. 55). SDA and SCL need
    // pull-ups (SLAU445I 24.3, p. 629): the internal ones of both boards here, two on each line.
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
        // SMCLK on P1.7, P1SELx = 10 and P1DIR = 1 (SLASE59F Table 6-17, p. 55), for `EXTERNAL_CLOCK`
        let _smclk_out = p1.pin7.to_output().to_alternate2();

        // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SELx = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
        // Table 6-17, p. 55; SLAU445I Table 22-8, p. 593)
        let mut tx = SerialConfig::new(
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

        // The only master on the bus (UCMM = 0). After COUNT bytes the eUSCI sets UCBCNTIFG and sends the
        // STOP (UCASTPx = 10b, UCBxTBCNT), it acknowledges the last byte too (UCSTPNACK), and it sets
        // UCCLTOIFG when SCL stays low for 135000 MODCLK cycles (UCCLTO = 01b) (SLAU445I Table 24-5, p. 651;
        // SLAU445I Table 24-8, p. 654).
        let master = I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_single_master()
            .byte_counter(COUNT, true)
            .ack_last_byte()
            .clock_low_timeout(ClockLowTimeout::_28ms);
        let master = if EXTERNAL_CLOCK {
            // UCSSELx = 00b: UCLKI, the clock on the SPI clock pin, UCB0CLK: P1.1 with P1SELx = 01 (SLAU445I
            // Table 24-4, p. 649; SLAU445I Figure 24-1, p. 628; SLASE59F Table 6-17, p. 55). Its pull-down
            // keeps it low without the wire. fBitClock = fBRCLK/UCBRx (SLAU445I 24.3.7, p. 642).
            master.use_uclk(p1.pin1.pulldown().to_alternate1(), 80) // 8MHz / 80 = 100kHz
        } else {
            // UCSSELx = 10b: SMCLK (SLAU445I Table 24-4, p. 649)
            master.use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        };
        let mut master = master.configure(scl, sda);
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
            delay.delay_ms(1000);
        }
    } else {
        let mut slave = I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
            .as_slave(SLAVE_ADDRESS)
            .configure(scl, sda);

        // DATA, then 0xFF. A slave transmitter holds SCL low until its next byte is written (SLAU445I
        // 24.3.5.1.1, p. 633): every other read the slave waits 100 ms before the second byte, longer than
        // 135000 cycles of the slowest MODCLK, 3.8 MHz: 36 ms (SLASE59F Table 5-9, p. 26).
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

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
