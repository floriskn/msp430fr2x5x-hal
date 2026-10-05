//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master clocked from ACLK, whose SPI mode changes at run time: every 10 ms eUSCI_A1 sends A5h with
//! SCLK at 32.768 kHz, ACLK from REFO, and every second it switches between SPI mode 0 and mode 3.
//!
//! In mode 0 SCLK idles low, and in mode 3 it idles high. In both, MOSI changes on the falling edges of
//! SCLK, and the data is captured on the rising edges. `change_mode` resets the eUSCI to change the mode.
//! On this device UCSSELx = 01b selects ACLK; MODCLK can't clock the eUSCI here.
//! (ACLK: SLASEO7C Table 9-8, p. 50. SCLK = ACLK / UCBRx: SLAU445I 23.3.6, p. 609. REFO is 32768 Hz
//! ± 3.5 %: SLASEO7C 8.12.3.4, p. 30. UCCKPL and UCCKPH: SLAU445I Table 23-3, p. 613; SLAU445I
//! Figure 23-4, p. 610. eUSCI_A1's pins: SLASEO7C Table 9-11, p. 54.)
//!
//! How to test (the scope):
//! 1. Flash this example.
//! 2. Scope, ground on GND (J3 pin 22), 50 µs/div, trigger on CH1 rising, in normal mode: CH1 on SCLK,
//!    P2.4 (J2 pin 11), CH2 on MOSI, P2.6 (J1 pin 4). Expected: 8 clock pulses at 32.8 kHz (a period of
//!    30.5 µs, give or take REFO's tolerance) while MOSI sends 10100101, MSB first.
//! 3. Watch SCLK between the bytes: for a second it idles low (mode 0), for the next it idles high
//!    (mode 3), and so on.
//! (Header pins: SLAU802 Figure 10, p. 13.)
#![no_main]
#![no_std]

use embedded_hal::{
    delay::DelayNs,
    spi::{SpiBus, MODE_0, MODE_3},
};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    spi::SpiConfig,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p2 = Batch::new(periph.p2).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A1 as a 3-pin SPI master: SCLK on P2.4, MOSI on P2.6 and MISO on P2.5, with P2SEL = 01
    // (SLASEO7C Table 9-24, p. 66). MODE_0 captures data on the first clock edge with the clock idle low
    // (UCCKPH = 1, UCCKPL = 0), `true` sends the MSB first (UCMSB = 1) (SLAU445I Table 23-3, p. 613).
    // ACLK / 1 is SCLK (UCSSELx = 01b: SLASEO7C Table 9-8, p. 50; UCBRx: SLAU445I 23.3.6, p. 609).
    let mut spi = SpiConfig::new(periph.e_usci_a1, MODE_0, true)
        .to_master_using_aclk(&aclk, 1)
        .single_master_bus(p2.pin5.to_alternate1(), p2.pin6.to_alternate1(), p2.pin4.to_alternate1());

    let mut mode_3 = false;
    loop {
        for _ in 0..100 {
            // Returns once the byte has gone out
            spi.write(&[0xA5]).ok();
            delay.delay_ms(10);
        }
        // MODE_3 captures data on the second clock edge with the clock idle high (UCCKPH = 0, UCCKPL = 1:
        // SLAU445I Table 23-3, p. 613)
        mode_3 = !mode_3;
        spi.change_mode(if mode_3 { MODE_3 } else { MODE_0 });
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
