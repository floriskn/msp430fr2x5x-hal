//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The infrared modulator with eUSCI_A0's UART as its data: every 20 ms the UART sends `U`, and P1.4 sends
//! a burst of a 38 kHz carrier for each 0 bit of it, the start bit included, and nothing for the 1 bits.
//!
//! TA0's CCR2 output is the carrier, and TA1's CCR2 output, the envelope, stays high. In ASK mode the
//! modulator then outputs the carrier while the UART's data is 0, and stays low while it's 1, as the idle
//! line is. The UART runs at 2400 baud, so each bit is 417 µs long, about 16 periods of the carrier. `U` is
//! 55h: LSB first, the start bit is followed by 1, 0, 1, 0, 1, 0, 1, 0 and the stop bit.
//! (Carrier and coding inputs: SLASEO7C Table 9-12, p. 55 and SLASEO7C Table 9-13, p. 56. The UART's TXD
//! signal is the modulator's data: SLASEO7C Figure 9-2, p. 57. The ASK logic is drawn for the
//! MSP430FR2433 in SLAU445I Figure 1-8, p. 50. The modulator drives "the eUSCI_A pin of UCA0TXD/
//! UCA0SIMO": SLASEO7C 9.10.8, p. 60. SLASEO7C Figure 9-2, p. 57 labels that pin P2.0, but measured on an
//! MSP430FR2476 the output is on P1.4, with eUSCI_A0 in its default mapping.)
//!
//! How to test (the scope):
//! 1. P1.4 isn't on the BoosterPack headers: it goes to the debug probe's backchannel UART through the
//!    TXD jumper of J101. Pull that jumper off, connect the probe tip to the TXD pin on the MSP430 side,
//!    away from the USB connector, and the ground clip to GND (J3 pin 22). (J101: SLAU802 Table 2, p. 8;
//!    its target-side TXD pin is P1.4_UART_TX: SLAU802 Figure 16, p. 22; board layout: SLAU802 Figure 1,
//!    p. 1; header pins: SLAU802 Figure 10, p. 13.)
//! 2. Flash this example.
//! 3. Scope: 1 V/div, 1 ms/div, trigger on a rising edge at 1.5 V in normal mode. Expected every 20 ms:
//!    five bursts, each 417 µs long and starting 833 µs after the one before: the start bit and the four
//!    0 bits of 55h. At 20 µs/div a burst shows the carrier, a square wave with a period of 26 µs.
//! 4. Put the TXD jumper back for the examples that use the backchannel UART.
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
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The timers count SMCLK (TASSEL = 10b:
    // SLASEO7C Table 9-8, p. 50).
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The carrier, high for half of each period
    // (TA0's CCR2 output is the "IR carrier input": SLASEO7C Table 9-12, p. 55)
    let carrier = PwmParts3::new(periph.ta0, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD / 2);
    // The envelope, high for the whole period, so the data alone switches the carrier on and off
    // (TA1's CCR2 output is the "IR coding input": SLASEO7C Table 9-13, p. 56)
    let envelope = PwmParts3::new(periph.ta1, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD);

    // eUSCI_A0 at 2400 baud, 8N1, with TXD on P1.4, P1SEL = 01 (SLASEO7C Table 9-23, p. 65), in the pin
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
    .tx_only(p1.pin4.to_alternate1());
    // In SYSCFG1: IREN = 1, IRMSEL = 0 for ASK, IRPSEL = 0 for normal polarity, IRDSSEL = 0 for the data
    // from eUSCI_A0 (SLAU445I Table 1-30, p. 81)
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
