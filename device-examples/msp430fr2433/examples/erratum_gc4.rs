//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum GC4: with MCLK at 16 MHz from the DCO, the FRAM's error detection can report an
//! uncorrectable bit error that doesn't exist, and so reset the device, with no reset cause. With resets
//! for uncorrectable bit errors on, the program reads 2 KiB of FRAM over and over; every 2048 passes LED1
//! toggles and the backchannel UART prints the count. Each start prints the reset cause, so a reset from
//! the erratum shows up as a new `start` line with no cause, `00h`.
//!
//! The erratum: "During execution from FRAM a non-existent uncorrectable bit error can be detected and
//! trigger a PUC if the uncorrectable bit error detection flag is set (GCCTL0.UBDRSTEN = 1)", if "MCLK is
//! sourced from DCO frequency of 16 MHz" (or from an external clock or crystal above 12 MHz, which this
//! device doesn't have), and "This PUC will not be recognized by the SYSRSTIV register (SYSRSTIV = 0x00)"
//! (SLAZ664S GC4, p. 10). The HAL doesn't work around it; its `fram` module lists the erratum's three
//! workarounds: "Check the reset source for SYSRSTIV = 0 and ignore the reset", leave UBDRSTEN at 0, or
//! "Set the MCLK to maximum 12MHz" (SLAZ664S GC4, p. 10). `Pmm::take_reset_cause()` returns `None` for
//! SYSRSTIV = 0, printed as `00h`.
//! (UBDRSTEN: SLAU445I Table 6-3, p. 307. The RST pin, S3, resets with a BOR, SYSRSTIV 04h: SLASE59F
//! Table 6-9, p. 48; SLAU739 Figure 18, p. 23. LED1 on P1.0 is red: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 2. Press S3, so the run begins with a known reset: `start, reset cause: 04h`, then `passes: 2048`,
//!    `passes: 4096` and so on, with LED1 toggling at each line.
//! 3. Leave it running for at least an hour: the errata sheet gives no rate. Each new
//!    `start, reset cause: 00h` line is a reset from the erratum, and the count starts again. Note how
//!    many come per hour.
//! 4. Set `MCLK_FREQ` to `DcoclkFreqSel::_12MHz`, the erratum's third workaround, flash again and repeat
//!    steps 2 and 3: no such reset should come.
//! 5. Nothing needs undoing afterwards: only a BOR resets UBDRSTEN, but the next example's `Fram::new()`
//!    clears it (SLAU445I Figure 6-4, p. 307; see the HAL's `fram` module).
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::{Fram, UncorrectableBitError},
    gpio::Batch,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// MCLK: 16 MHz meets the erratum's condition, and `DcoclkFreqSel::_12MHz` is its third workaround
const MCLK_FREQ: DcoclkFreqSel = DcoclkFreqSel::_16MHz;

/// 2 KiB of constants, in FRAM with the program, read over and over
static DATA: [u16; 1024] = [0x5AA5; 1024];

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next reset (reading
    // SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let cause = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the MCLK_FREQ range, and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(MCLK_FREQ, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // A PUC for an uncorrectable bit error (UBDRSTEN: SLAU445I Table 6-3, p. 307)
    fram.set_uncorrectable_bit_error_action(UncorrectableBitError::Reset);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU739 2.2.4, p. 9; SLASE59F
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
    tx.write_all(b"start, reset cause: ").ok();
    print_hex(&mut tx, cause.map_or(0, u16::from));
    tx.write_all(b"\r\n").ok();

    let mut passes: u32 = 0;
    loop {
        // Read every word, so the reads go to the FRAM instead of being optimised away
        let mut sum: u16 = 0;
        for word in DATA.iter() {
            sum = sum.wrapping_add(unsafe { core::ptr::read_volatile(word) });
        }
        core::hint::black_box(sum);

        passes += 1;
        if passes % 2048 == 0 {
            led1.toggle().ok();
            tx.write_all(b"passes: ").ok();
            print_decimal(&mut tx, passes);
            tx.write_all(b"\r\n").ok();
        }
    }
}

/// Print `n` in decimal. `write!` would do it too, but `core::fmt` takes several KiB of FRAM, more than the
/// MSP430FR2522 versions of the other errata tests have to spare, and this prints the same way.
fn print_decimal(tx: &mut impl Write, n: u32) {
    let mut divisor: u32 = 1_000_000_000;
    while divisor > 1 && n < divisor {
        divisor /= 10;
    }
    while divisor > 0 {
        tx.write_all(&[b'0' + (n / divisor % 10) as u8]).ok();
        divisor /= 10;
    }
}

/// Print the low byte of `n` as two hexadecimal digits and an `h`, as the data sheet writes SYSRSTIV values
fn print_hex(tx: &mut impl Write, n: u16) {
    const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[DIGITS[(n >> 4 & 0xF) as usize], DIGITS[(n & 0xF) as usize], b'h']).ok();
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
