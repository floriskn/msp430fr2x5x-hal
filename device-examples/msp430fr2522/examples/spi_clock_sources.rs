//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master clocked from ACLK, whose SPI mode changes at run time: every 10 ms eUSCI_A0 sends A5h with
//! SCLK at 32.768 kHz, ACLK from REFO, and every second it switches between SPI mode 0 and mode 3. There's
//! no LaunchPad for the MSP430FR25x2.
//!
//! In mode 0 SCLK idles low, and in mode 3 it idles high. In both, MOSI changes on the falling edges of
//! SCLK, and the data is captured on the rising edges. `change_mode` resets the eUSCI to change the mode.
//! On this device UCSSELx = 01b selects ACLK; MODCLK can't clock the eUSCI here.
//! (ACLK: SLASEE4C Table 6-8, p. 49. SCLK = ACLK / UCBRx: SLAU445I 23.3.6, p. 609. REFO is 32768 Hz
//! ± 3.5 %: SLASEE4C Table 5-7, p. 27. UCCKPL and UCCKPH: SLAU445I Table 23-3, p. 613; SLAU445I
//! Figure 23-4, p. 610. eUSCI_A0's pins: SLASEE4C Table 6-11, p. 53.)
//!
//! How to test (the scope):
//! 1. Flash this example.
//! 2. Scope, ground on GND, 50 µs/div, trigger on CH1 rising, in normal mode: CH1 on SCLK, P1.6, CH2 on
//!    MOSI, P1.4. Expected: 8 clock pulses at 32.8 kHz (a period of 30.5 µs, give or take REFO's
//!    tolerance) while MOSI sends 10100101, MSB first.
//! 3. Watch SCLK between the bytes: for a second it idles low (mode 0), for the next it idles high
//!    (mode 3), and so on.
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
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    spi::SpiConfig,
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (_smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0 as a 3-pin SPI master in its default mapping (USCIARMP = 0): SCLK on P1.6, MOSI on P1.4 and
    // MISO on P1.5, with P1SELx = 01 (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). MODE_0
    // captures data on the first clock edge with the clock idle low (UCCKPH = 1, UCCKPL = 0), `true` sends
    // the MSB first (UCMSB = 1) (SLAU445I Table 23-3, p. 613). ACLK / 1 is SCLK (UCSSELx = 01b: SLASEE4C
    // Table 6-8, p. 49; UCBRx: SLAU445I 23.3.6, p. 609).
    let mut spi = SpiConfig::<_, _, DefaultMapping>::new(periph.e_usci_a0, MODE_0, true)
        .to_master_using_aclk(&aclk, 1)
        .single_master_bus(p1.pin5.to_alternate1(), p1.pin4.to_alternate1(), p1.pin6.to_alternate1());

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
