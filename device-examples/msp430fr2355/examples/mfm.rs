//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The Manchester Function Module (MFM) in a loopback: once a second the MFM sends a packet of 32 bytes,
//! Manchester-coded, on P5.1, a jumper wire brings it back to P5.0, and the MFM decodes it. The backchannel
//! UART prints what came back, and whether it matches what was sent.
//!
//! The MFM is the SPI master of eUSCI_B1, set up as its 4-wire slave with `SpiConfig::mfm_slave()`.
//! eUSCI_B1's interrupt hands the MFM each byte to send through the Tx buffer, and takes each byte it
//! received from the Rx buffer. A rising edge of TB2's CCR0 output starts a packet: TB2 runs as PWM with a
//! period of 0.5 s, and the CCR0 output toggles at the end of each period, so it rises once a second. The
//! MFM sends at SMCLK / 8, about 125 kbit/s with SMCLK at about 1 MHz, and MCLK runs at about 8 MHz so the
//! interrupt reads each byte in time. The first bit of a packet must be 1, so the first byte is A5h.
//!
//! The user's guide doesn't say which SPI clock mode, bit order and STE polarity the MFM uses. This example
//! uses mode 1 (UCCKPH = 0, which erratum USCI47 recommends for SPI slaves), MSB first, and STE active low.
//! If the bytes come back wrong, try the other settings with the three constants below.
//! (The MFM: SLAU445I 25.2, p. 666 to p. 667; SLAU445I 25.6, p. 668 to p. 669; SLASEC4D 6.10.14, p. 79.
//! MFM.RX on P5.0 and MFM.TX on P5.1, P5SELx = 10: SLASEC4D Table 6-67, p. 104. TB2's CCR0 output is the MFM
//! start trigger: SLASEC4D Table 6-18, p. 74. The toggle output mode: SLAU445I Table 14-4, p. 401. USCI47:
//! SLAZ695J USCI47, p. 12 to p. 13.)
//!
//! How to test (a jumper wire):
//! 1. Connect P5.1 (J3 pin 25) to P5.0 (J3 pin 26). (Header pins: SLAU680 Figure 10, p. 15.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 3. Expected, once a second: `sent 32, received 32, overruns 0, match: A5 01 02 03 ... 1E 1F`. Without
//!    the wire nothing comes back: `received 0`.
//! 4. A line with `differ` lists the bytes that came back: bits shifted by one point to the wrong clock
//!    mode, bits in reverse order to the wrong bit order. Overruns mean the interrupt was too slow.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;
use critical_section::with;
use embedded_hal::{
    delay::DelayNs,
    spi::{Mode, MODE_1},
};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    mfm::Mfm,
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    serial::*,
    spi::{SpiConfig, SpiErr, SpiVector, StePolarity},
    watchdog::Wdt,
};
use msp430fr2355::interrupt;
use panic_msp430 as _;

/// The SPI clock mode, bit order and STE polarity of eUSCI_B1, which must match the MFM's SPI master
const SPI_MODE: Mode = MODE_1;
const MSB_FIRST: bool = true;
const STE_POLARITY: StePolarity = StePolarity::EnabledWhenLow;

/// The packet: 256 data bits, as 32 bytes (SLAU445I 25.2, p. 666), the first bit 1 (SLAU445I Figure 25-2,
/// p. 667)
const PACKET: [u8; 32] = [
    0xA5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
    28, 29, 30, 31,
];

/// One packet: how many bytes went to the Tx buffer, how many came back and what they were, and how often a
/// received byte was overwritten before it was read
#[derive(Clone)]
struct Transfer {
    sent: usize,
    received: usize,
    overruns: u16,
    data: [u8; 32],
}

impl Transfer {
    const fn new() -> Self { Transfer { sent: 0, received: 0, overruns: 0, data: [0; 32] } }
}

static MFM: Mutex<RefCell<Option<Mfm>>> = Mutex::new(RefCell::new(None));
static TRANSFER: Mutex<RefCell<Transfer>> = Mutex::new(RefCell::new(Transfer::new()));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let p5 = Batch::new(periph.p5).split(&pmm);

    // MCLK = DCOCLKDIV in the 8 MHz range, SMCLK = MCLK / 8, and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). SMCLK clocks the MFM (SLAU445I
    // 25.3, p. 667).
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_8)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SEL = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
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

    // eUSCI_B1 as a 4-wire SPI slave, which the MFM needs (SLASEC4D 6.10.14, p. 79). Putting P5.0 and P5.1
    // in their MFM function enables the MFM (SLAU445I 25.2, p. 666). UCRXIE requests eUSCI_B1's interrupt
    // for each received byte (SLAU445I 23.3.8.2, p. 611).
    let spi = SpiConfig::new(periph.e_usci_b1, SPI_MODE, MSB_FIRST).to_slave().mfm_slave(STE_POLARITY);
    let mut mfm = Mfm::new(spi, p5.pin0.to_alternate2(), p5.pin1.to_alternate2());
    mfm.spi().set_rx_interrupt();
    with(|cs| MFM.borrow_ref_mut(cs).replace(mfm));

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };
    prepare_packet();

    // TB2 counts ACLK, 32768 Hz, from 0 to 16383 in up mode: a period of 0.5 s. The PWM setup puts CCR0's
    // output in toggle mode (OUTMOD = 100b), so it rises at the end of every second period.
    let _pwm = PwmParts3::new(periph.tb2, TimerConfig::aclk(&aclk), 16383);

    loop {
        // Wait for a whole packet, or 1.5 s
        for _ in 0..150 {
            if with(|cs| TRANSFER.borrow_ref(cs).received >= PACKET.len()) {
                break;
            }
            delay.delay_ms(10);
        }
        let transfer = with(|cs| TRANSFER.replace(cs, Transfer::new()));
        prepare_packet();
        report(&mut tx, &transfer);
    }
}

/// Put the first byte of the packet in the Tx buffer, which must happen before the trigger (SLAU445I 25.2,
/// p. 666), and request the interrupt for the next ones (UCTXIE: SLAU445I 23.3.8.1, p. 611)
fn prepare_packet() {
    with(|cs| {
        if let Some(mfm) = MFM.borrow_ref_mut(cs).as_mut() {
            if mfm.write(PACKET[0]).is_ok() {
                TRANSFER.borrow_ref_mut(cs).sent = 1;
            }
            mfm.spi().set_tx_interrupt();
        }
    });
}

/// Print the counts, whether the bytes that came back match the packet, and the bytes
fn report(tx: &mut impl Write, transfer: &Transfer) {
    let matches = transfer.received == PACKET.len() && transfer.data == PACKET;
    write!(
        tx,
        "sent {}, received {}, overruns {}, {}:",
        transfer.sent,
        transfer.received,
        transfer.overruns,
        if matches { "match" } else { "differ" }
    )
    .ok();
    for byte in &transfer.data[..transfer.received.min(PACKET.len())] {
        write!(tx, " {:02X}", byte).ok();
    }
    writeln!(tx, "\r").ok();
}

// The eUSCI_B1 vector at FFDEh (SLASEC4D Table 6-2, p. 64). `interrupt_source()` reports UCRXIFG before
// UCTXIFG, as UCB1IV would (SLAU445I Table 23-19, p. 625). The MFM has no interrupt of its own (SLAU445I
// 25.6.3, p. 669).
#[interrupt]
fn EUSCI_B1() {
    with(|cs| {
        let mut mfm = MFM.borrow_ref_mut(cs);
        let Some(mfm) = mfm.as_mut() else { return };
        let mut transfer = TRANSFER.borrow_ref_mut(cs);
        loop {
            match mfm.spi().interrupt_source() {
                SpiVector::RxBufferFull => {
                    let byte = match mfm.read() {
                        Ok(byte) => byte,
                        Err(nb::Error::Other(SpiErr::Overrun(byte))) => {
                            transfer.overruns += 1;
                            byte
                        }
                        Err(_) => break,
                    };
                    let i = transfer.received;
                    if i < PACKET.len() {
                        transfer.data[i] = byte;
                    }
                    transfer.received += 1;
                }
                SpiVector::TxBufferEmpty => {
                    let i = transfer.sent;
                    if i < PACKET.len() {
                        mfm.write(PACKET[i]).ok();
                        transfer.sent += 1;
                    } else {
                        // The whole packet is in: stop the Tx interrupt, which fires while the buffer
                        // is empty
                        mfm.spi().clear_tx_interrupt();
                    }
                }
                SpiVector::None => break,
            }
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
