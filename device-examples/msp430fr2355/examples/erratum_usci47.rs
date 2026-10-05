//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum USCI47, and of `SpiSlave::reset()`, which works around it: an SPI slave with
//! UCCKPH = 1 that leaves reset while its clock pin isn't at the idle level operates incorrectly, and an
//! eUSCI_A slave then receives nothing. eUSCI_A0, the slave, leaves reset while P4.5, the clock pin of
//! eUSCI_B1, the master, is held high, and the master sends three bytes. Then `reset()` resets the slave,
//! with the clock idle, and once a second the master sends the bytes again. The backchannel UART prints how
//! many bytes the slave received after the start, and what it received after the reset.
//!
//! The erratum: "The eUSCI SPI operates incorrectly" when "The eUSCI_A or eUSCI_B module is configured as a
//! SPI slave with clock phase mode" UCCKPH = 1, and "The SPI clock pin is not at the appropriate idle level
//! (low for UCCKPL = 0, high for UCCKPL = 1) when the UCSWRST bit in the UCxxCTLW0 register is cleared";
//! an eUSCI_A slave then "will not be able to receive a byte". One workaround: "If UCTXIFG is set twice but
//! UCRXIFG is not set, reset the MSP SPI slave by setting and then clearing the UCSWRST bit, and inform the
//! SPI master to resend the data" (SLAZ695J USCI47, p. 12 to p. 13). Here the master is on the same chip and
//! knows what it sent, so the example simply checks what arrived.
//!
//! Both use SPI mode 0, MSB first: UCCKPH = 1, and SCLK idles low (UCCKPL = 0). The master runs at 10 kHz
//! from SMCLK. P4.5 is a GPIO output until the slave is set up, and then becomes UCB1CLK.
//! (UCCKPH and UCCKPL: SLAU445I Table 23-3, p. 613. The SPI pins: SLASEC4D Table 6-14, p. 72. The
//! backchannel UART is eUSCI_A1, TXD on P4.3: SLAU680 2.2.4, p. 11.)
//!
//! How to test (three jumper wires):
//! 1. Connect MOSI, P1.7 (J1 pin 4), to P4.6 (J2 pin 15); MISO, P1.6 (J1 pin 3), to P4.7 (J2 pin 14); and
//!    SCLK, P1.5 (J1 pin 2), to P4.5 (J1 pin 7). (Header pins: SLAU680 Figure 10, p. 15.)
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 3. Expected, once a second: `after the start with SCLK high: 0 of 3 bytes; after reset(): 12 14 255`.
//!    A count above 0 means the erratum didn't show: please report what was printed.
//! 4. To try the erratum's other workaround, set `SCLK_HIGH_AT_START` to false and flash again. SCLK is
//!    then at its idle level when the slave leaves reset, and the first bytes arrive too:
//!    `after the start with SCLK low: 3 of 3 bytes; after reset(): 12 14 255`.
#![no_main]
#![no_std]

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{SpiBus, MODE_0},
};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, StopBits},
    spi::{Spi, SpiConfig, SpiSlave},
    watchdog::Wdt,
};
use msp430fr2355::{EUsciA0, EUsciB1};
use panic_msp430 as _;

/// `false` holds the master's clock pin low, its idle level, while the slave leaves reset: the erratum's
/// other workaround
const SCLK_HIGH_AT_START: bool = true;

/// The bytes the master sends
const SENT: [u8; 3] = [12, 14, 255];

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SELx = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
    let mut console = SerialConfig::new(
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

    // P4.5, the master's clock pin, as a GPIO output (P4SELx = 00, P4DIR = 1: SLASEC4D Table 6-66, p. 102):
    // high, or low, the idle level of mode 0 (UCCKPL = 0: SLAU445I Table 23-3, p. 613)
    let mut sclk = p4.pin5.to_output();
    sclk.set_state(SCLK_HIGH_AT_START.into()).ok();

    // The slave, eUSCI_A0, in 3-pin mode: UCA0SOMI on P1.6, UCA0SIMO on P1.7 and UCA0CLK on P1.5, with
    // P1SELx = 01 (SLASEC4D Table 6-63, p. 96). `exclusive_bus` releases it from reset (UCSWRST = 0) while
    // P4.5 holds its clock pin. MODE_0 and MSB first are UCCKPH = 1, UCCKPL = 0 and UCMSB = 1 (SLAU445I
    // Table 23-3, p. 613).
    let mut slave = SpiConfig::new(periph.e_usci_a0, MODE_0, true)
        .to_slave()
        .exclusive_bus(p1.pin6.to_alternate1(), p1.pin7.to_alternate1(), p1.pin5.to_alternate1());

    // The master, eUSCI_B1, in 3-pin mode: UCB1SOMI on P4.7, UCB1SIMO on P4.6 and UCB1CLK on P4.5, with
    // P4SELx = 01 (SLASEC4D Table 6-66, p. 102). From here SCLK idles low. SCLK = SMCLK / 800 = 10 kHz
    // (UCSSEL = 10b: SLAU445I Table 23-12, p. 620; fBitClock = fBRCLK / UCBRx: SLAU445I 23.3.6, p. 609).
    let mut master = SpiConfig::new(periph.e_usci_b1, MODE_0, true)
        .to_master_using_smclk(&smclk, 800)
        .single_master_bus(p4.pin7.to_alternate1(), p4.pin6.to_alternate1(), sclk.to_alternate1());

    let (_, count_at_start) = exchange(&mut master, &mut slave, &mut delay);
    // The workaround: set and clear UCSWRST, now that SCLK is idle (SLAZ695J USCI47, p. 12 to p. 13)
    slave.reset();

    let level = if SCLK_HIGH_AT_START { "high" } else { "low" };
    loop {
        let (received, count) = exchange(&mut master, &mut slave, &mut delay);
        write!(console, "after the start with SCLK {}: {} of 3 bytes; after reset():", level, count_at_start)
            .ok();
        for byte in &received[..count] {
            write!(console, " {}", byte).ok();
        }
        write!(console, "\r\n").ok();
        delay.delay_ms(1000);
    }
}

/// The master sends `SENT`, a byte at a time. Returns the bytes the slave received, and how many.
fn exchange(
    master: &mut Spi<EUsciB1>,
    slave: &mut SpiSlave<EUsciA0>,
    delay: &mut impl DelayNs,
) -> ([u8; 3], usize) {
    let mut received = [0; 3];
    let mut count = 0;
    for &byte in SENT.iter() {
        // Returns once the byte has gone out
        master.write(&[byte]).ok();
        // Look at the slave 1 ms, ten bit times, later: by then it has had the whole byte, and UCRXIFG is
        // set if the byte was moved to UCA0RXBUF (SLAU445I 23.3.4, p. 608)
        delay.delay_ms(1);
        if let Ok(byte) = slave.read() {
            received[count] = byte;
            count += 1;
        }
    }
    (received, count)
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
