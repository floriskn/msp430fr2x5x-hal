//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum USCI47, and of `SpiSlave::reset()`, which works around it: an SPI slave with
//! UCCKPH = 1 that leaves reset while its clock pin isn't at the idle level operates incorrectly, and an
//! eUSCI_A slave then receives nothing. eUSCI_A0, the slave, leaves reset while P1.1, the clock pin of
//! eUSCI_B0, the master, is held high, and the master sends three bytes. Then `reset()` resets the slave,
//! with the clock idle, and once a second the master sends the bytes again. eUSCI_A0 is the device's only
//! UART, so LEDs show the results: the one on P1.0 lights if the slave received fewer than three bytes
//! after the start, and the one on P2.2 while it receives all three, unchanged, after the reset.
//!
//! The erratum: "The eUSCI SPI operates incorrectly" when "The eUSCI_A or eUSCI_B module is configured as a
//! SPI slave with clock phase mode" UCCKPH = 1, and "The SPI clock pin is not at the appropriate idle level
//! (low for UCCKPL = 0, high for UCCKPL = 1) when the UCSWRST bit in the UCxxCTLW0 register is cleared";
//! an eUSCI_A slave then "will not be able to receive a byte". One workaround: "If UCTXIFG is set twice but
//! UCRXIFG is not set, reset the MSP SPI slave by setting and then clearing the UCSWRST bit, and inform the
//! SPI master to resend the data" (SLAZ705H USCI47, p. 10 to p. 11). Here the master is on the same chip and
//! knows what it sent, so the example simply checks what arrived.
//!
//! Both use SPI mode 0, MSB first: UCCKPH = 1, and SCLK idles low (UCCKPL = 0). The master runs at 10 kHz
//! from SMCLK. P1.1 is a GPIO output until the slave is set up, and then becomes UCB0CLK.
//! (UCCKPH and UCCKPL: SLAU445I Table 23-3, p. 613. The SPI pins: SLASEE4C Table 6-11, p. 53. "One
//! eUSCI_A supports UART, IrDA, and SPI": SLASEE4C 1.1, p. 1. No board document covers the LEDs: there is
//! none for the MSP430FR25x2.)
//!
//! How to test (three jumper wires, two LEDs and resistors):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and another from P2.2 to GND.
//! 2. Connect MOSI, P1.2, to P1.4; MISO, P1.3, to P1.5; and SCLK, P1.1, to P1.6.
//! 3. Flash this example.
//! 4. Expected: both LEDs light and stay on. If the LED on P1.0 stays off, the erratum didn't show: please
//!    report it.
//! 5. To try the erratum's other workaround, set `SCLK_HIGH_AT_START` to false and flash again. SCLK is
//!    then at its idle level when the slave leaves reset, the first bytes arrive too, and only the LED on
//!    P2.2 lights.
#![no_main]
#![no_std]

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{SpiBus, MODE_0},
};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    spi::{Spi, SpiConfig, SpiSlave},
    watchdog::Wdt,
};
use msp430fr25x2::{EUsciA0, EUsciB0};
use panic_msp430 as _;

/// `false` holds the master's clock pin low, its idle level, while the slave leaves reset: the erratum's
/// other workaround
const SCLK_HIGH_AT_START: bool = true;

/// The bytes the master sends
const SENT: [u8; 3] = [12, 14, 255];

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);

    // The LEDs on P1.0 and P2.2, GPIO outputs: PxSELx = 00 and PxDIR = 1 (SLASEE4C Table 6-15, p. 58;
    // SLASEE4C Table 6-16, p. 60)
    let mut led_erratum = p1.pin0.to_output();
    let mut led_received = p2.pin2.to_output();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM,
    // DIVS: SLAU445I Table 3-9, p. 118). ACLK from REFO (SELA = 01b, same table): the MSP430FR25x2 has
    // no ACLK from the VLO (SLASEE4C 6.10.2, p. 49).
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // P1.1, the master's clock pin, as a GPIO output (P1SELx = 00, P1DIR = 1: SLASEE4C Table 6-15, p. 58):
    // high, or low, the idle level of mode 0 (UCCKPL = 0: SLAU445I Table 23-3, p. 613)
    let mut sclk = p1.pin1.to_output();
    sclk.set_state(SCLK_HIGH_AT_START.into()).ok();

    // The slave, eUSCI_A0 in the default mapping, USCIARMP = 0, in 3-pin mode: UCA0SOMI on P1.5, UCA0SIMO
    // on P1.4 and UCA0CLK on P1.6, with P1SELx = 01 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15,
    // p. 58). `exclusive_bus` releases it from reset (UCSWRST = 0) while P1.1 holds its clock pin. MODE_0
    // and MSB first are UCCKPH = 1, UCCKPL = 0 and UCMSB = 1 (SLAU445I Table 23-3, p. 613).
    let mut slave: SpiSlave<_, DefaultMapping> = SpiConfig::new(periph.e_usci_a0, MODE_0, true)
        .to_slave()
        .exclusive_bus(p1.pin5.to_alternate1(), p1.pin4.to_alternate1(), p1.pin6.to_alternate1());

    // The master, eUSCI_B0 in the default mapping, USCIBRMP = 0, in 3-pin mode: UCB0SOMI on P1.3, UCB0SIMO
    // on P1.2 and UCB0CLK on P1.1, with P1SELx = 01 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15,
    // p. 58). From here SCLK idles low. SCLK = SMCLK / 800 = 10 kHz (UCSSEL = 10b: SLAU445I Table 23-12,
    // p. 620; fBitClock = fBRCLK / UCBRx: SLAU445I 23.3.6, p. 609).
    let mut master: Spi<_, DefaultMapping> = SpiConfig::new(periph.e_usci_b0, MODE_0, true)
        .to_master_using_smclk(&smclk, 800)
        .single_master_bus(p1.pin3.to_alternate1(), p1.pin2.to_alternate1(), sclk.to_alternate1());

    let (_, count_at_start) = exchange(&mut master, &mut slave, &mut delay);
    led_erratum.set_state((count_at_start < SENT.len()).into()).ok();
    // The workaround: set and clear UCSWRST, now that SCLK is idle (SLAZ705H USCI47, p. 10 to p. 11)
    slave.reset();

    loop {
        let (received, count) = exchange(&mut master, &mut slave, &mut delay);
        led_received.set_state((count == SENT.len() && received == SENT).into()).ok();
        delay.delay_ms(1000);
    }
}

/// The master sends `SENT`, a byte at a time. Returns the bytes the slave received, and how many.
fn exchange(
    master: &mut Spi<EUsciB0>,
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
