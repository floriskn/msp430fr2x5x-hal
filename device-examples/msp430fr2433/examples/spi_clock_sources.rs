//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An SPI master clocked from MODCLK, whose SPI mode changes at run time: every 10 ms eUSCI_A1 sends A5h with
//! SCLK at MODCLK / 100, about 48 kHz, and every second it switches between SPI mode 0 and mode 3.
//!
//! In mode 0 SCLK idles low, and in mode 3 it idles high. In both, MOSI changes on the falling edges of
//! SCLK, and the data is captured on the rising edges. `change_mode` resets the eUSCI to change the mode.
//! On this device UCSSELx = 01b selects MODCLK; ACLK can't clock the eUSCI here. MODCLK is 3.8 MHz to
//! 5.8 MHz, typically 4.8 MHz, so SCLK shows its frequency divided by 100. Its oscillator, MODOSC, only runs
//! while a module requests it, so `to_master_using_modclk` also enables the conditional requests
//! (MODOSCREQEN = 1). Erratum USCI45: MODCLK is asynchronous to MCLK, so the clock's high phase in the
//! first data bit can in rare cases be stretched a lot; no data is lost.
//! (MODCLK: SLASE59F Table 6-7, p. 46; SLASE59F Table 5-9, p. 26. MODOSC requests: SLAU445I 3.2.15.1,
//! p. 111; MODOSCREQEN: SLAU445I Table 3-12, p. 123. SCLK = MODCLK / UCBRx: SLAU445I 23.3.6, p. 609.
//! UCCKPL and UCCKPH: SLAU445I Table 23-3, p. 613; SLAU445I Figure 23-4, p. 610. eUSCI_A1's pins: SLASE59F
//! Table 6-10, p. 49. The erratum: SLAZ664S USCI45, p. 13 to p. 14.)
//!
//! How to test (the scope):
//! 1. Flash this example.
//! 2. Scope, ground on GND (J3 pin 22), 50 µs/div, trigger on CH1 rising, in normal mode: CH1 on SCLK,
//!    P2.4 (J1 pin 7), CH2 on MOSI, P2.6 (J2 pin 15). Expected: 8 clock pulses at about 48 kHz (38 kHz to
//!    58 kHz) while MOSI sends 10100101, MSB first. Now and then the first pulse can be much longer
//!    (erratum USCI45).
//! 3. Watch SCLK between the bytes: for a second it idles low (mode 0), for the next it idles high
//!    (mode 3), and so on.
//! (Header pins: SLAU739 Figure 18, p. 23.)
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
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p2 = Batch::new(periph.p2).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118). The SPI doesn't use them.
    let (_smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A1 as a 3-pin SPI master: SCLK on P2.4, MOSI on P2.6 and MISO on P2.5, with P2SELx = 01
    // (SLASE59F Table 6-19, p. 58). MODE_0 captures data on the first clock edge with the clock idle low
    // (UCCKPH = 1, UCCKPL = 0), `true` sends the MSB first (UCMSB = 1) (SLAU445I Table 23-3, p. 613).
    // MODCLK / 100 is SCLK (UCSSELx = 01b: SLASE59F Table 6-7, p. 46; UCBRx: SLAU445I 23.3.6, p. 609).
    let mut spi = SpiConfig::new(periph.e_usci_a1, MODE_0, true)
        .to_master_using_modclk(100)
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
