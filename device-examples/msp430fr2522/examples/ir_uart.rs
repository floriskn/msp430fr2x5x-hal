//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The infrared modulator with eUSCI_A0's UART as its data: every 20 ms the UART sends `U`, and P2.0 sends
//! a burst of a 38 kHz carrier for each 0 bit of it, the start bit included, and nothing for the 1 bits.
//! There's no LaunchPad for the MSP430FR25x2.
//!
//! TA0's CCR2 output is the carrier, and TA1's CCR2 output, the envelope, stays high. In ASK mode the
//! modulator then outputs the carrier while the UART's data is 0, and stays low while it's 1, as the idle
//! line is. The UART runs at 2400 baud, so each bit is 417 µs long, about 16 periods of the carrier. `U` is
//! 55h: LSB first, the start bit is followed by 1, 0, 1, 0, 1, 0, 1, 0 and the stop bit. The output is
//! eUSCI_A0's TXD pin in its remapped mapping, P2.0, as the data sheet's figure shows it. On the
//! MSP430FR2476 the same figure turned out wrong when measured (see the HAL's `ir` module), so if P2.0
//! stays low, please report it.
//! (Carrier and coding inputs, the UART's TXD as the data, and the output on "P2.0/UCA0TXD/UCA0SIMO":
//! SLASEE4C Figure 6-2, p. 54. The ASK logic is drawn for the MSP430FR2433 in SLAU445I Figure 1-8, p. 50.
//! The modulator drives "the eUSCI_A pin of UCA0TXD/UCA0SIMO": SLASEE4C 6.10.8, p. 54. P2.0 is UCA0TXD
//! with P2SELx = 01 and USCIARMP = 1, and XOUT with P2SELx = 10: SLASEE4C Table 6-11, p. 53; SLASEE4C
//! Table 6-16, p. 60.)
//!
//! How to test (the scope):
//! 1. P2.0 must have no crystal on it. Flash this example.
//! 2. Scope on P2.0, ground clip on GND: 1 V/div, 1 ms/div, trigger on a rising edge at 1.5 V in normal
//!    mode. Expected every 20 ms: five bursts, each 417 µs long and starting 833 µs after the one before:
//!    the start bit and the four 0 bits of 55h. At 20 µs/div a burst shows the carrier, a square wave with
//!    a period of 26 µs.
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
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p2 = Batch::new(periph.p2).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range, ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The timers count SMCLK (TASSEL = 10b:
    // SLASEE4C Table 6-8, p. 49).
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The carrier, high for half of each period
    // (TA0's CCR2 output is the modulator's "Carrier": SLASEE4C Figure 6-2, p. 54)
    let carrier = PwmParts3::new(periph.ta0, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD / 2);
    // The envelope, high for the whole period, so the data alone switches the carrier on and off
    // (TA1's CCR2 output is the modulator's "Coding" input: SLASEE4C Figure 6-2, p. 54)
    let envelope = PwmParts3::new(periph.ta1, TimerConfig::smclk(&smclk), CARRIER_PERIOD - 1)
        .pwm2
        .into_ir_input(CARRIER_PERIOD);

    // eUSCI_A0 at 2400 baud, 8N1, with TXD on P2.0, P2SELx = 01, in the pin mapping whose TXD pin carries
    // the modulator's output: the remapped one, USCIARMP = 1 (SLASEE4C Table 6-11, p. 53; SLASEE4C
    // Table 6-16, p. 60)
    let mut tx = SerialConfig::<_, _, IrMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        2400,
    )
    .use_smclk(&smclk)
    .tx_only(p2.pin0.to_alternate1());
    // In SYSCFG1: IREN = 1, IRMSEL = 0 for ASK, IRPSEL = 0 for normal polarity, IRDSSEL = 0 for the data
    // from eUSCI_A0 (SLAU445I Table 1-30, p. 81)
    let _ir = IrModulator::with_uart_data(&carrier, &envelope, IrMode::Ask, false, &tx);

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
