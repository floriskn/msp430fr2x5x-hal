//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The infrared modulator with eUSCI_A0's UART as its data: every 20 ms the UART sends `U`, and P1.7 sends
//! a burst of a 38 kHz carrier for each 0 bit of it, the start bit included, and nothing for the 1 bits.
//!
//! TB0's CCR2 output is the carrier, and TB1's CCR2 output, the envelope, stays high. In ASK mode the
//! modulator then outputs the carrier while the UART's data is 0, and stays low while it's 1, as the idle
//! line is. The UART runs at 2400 baud, so each bit is 417 µs long, about 16 periods of the carrier. `U` is
//! 55h: LSB first, the start bit is followed by 1, 0, 1, 0, 1, 0, 1, 0 and the stop bit.
//! (Carrier and coding inputs: SLASEC4D Table 6-16, p. 73 and SLASEC4D Table 6-17, p. 74. The ASK logic,
//! with the data "From UCA0TXD/UCA0SIMO" and the output on P1.7: SLAU445I Figure 1-13, p. 54. The
//! modulator drives "the eUSCI_A pin of UCA0TXD/UCA0SIMO": SLASEC4D 6.10.9, p. 75.)
//!
//! How to test (the scope):
//! 1. Flash this example.
//! 2. Scope on P1.7 (J1 pin 4), ground clip on GND (J3 pin 22): 1 V/div, 1 ms/div, trigger on a rising
//!    edge at 1.5 V in normal mode. Expected every 20 ms: five bursts, each 417 µs long and starting
//!    833 µs after the one before: the start bit and the four 0 bits of 55h. At 20 µs/div a burst shows
//!    the carrier, a square wave with a period of 26 µs.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal_nb::serial::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    ir::{IrMapping, IrMode, IrModulator},
    pmm::Pmm,
    pwm::{PwmParts3, TimerConfig},
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

/// 38 kHz carrier period, in cycles of the 8 MHz SMCLK
const CARRIER_PERIOD: u16 = 210;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The timers count SMCLK (TBSSEL = 10b:
    // SLASEC4D Table 6-9, p. 68).
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The carrier, high for half of each period
    // (TB0's CCR2 output is the "IR carrier input": SLASEC4D Table 6-16, p. 73)
    let carrier = PwmParts3::new(periph.tb0, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD / 2);
    // The envelope, high for the whole period, so the data alone switches the carrier on and off
    // (TB1's CCR2 output is the "IR coding input": SLASEC4D Table 6-17, p. 74)
    let envelope = PwmParts3::new(periph.tb1, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD);

    // eUSCI_A0 at 2400 baud, 8N1, with TXD on P1.7, P1SELx = 01 (SLASEC4D Table 6-63, p. 96), in the pin
    // mapping whose TXD pin carries the modulator's output
    let tx = SerialConfig::<_, _, IrMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        2400,
    )
    .use_smclk(&smclk)
    .tx_only(p1.pin7.to_alternate1());
    // In SYSCFG1: IREN = 1, IRMSEL = 0 for ASK, IRPSEL = 0 for normal polarity, IRDSSEL = 0 for the data
    // from eUSCI_A0 (SLAU445I Table 1-25, p. 76)
    // The modulator holds `tx` and stands in for it, so the loop sends through it as through `tx`
    let mut tx = IrModulator::with_uart_data(&carrier, &envelope, IrMode::Ask, false, tx);

    loop {
        block!(tx.write(b'U')).ok();
        delay.delay_ms(20);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
