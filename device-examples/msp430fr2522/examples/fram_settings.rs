//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The FRAM controller's settings: its wait states, and what it does when it finds a bit error. eUSCI_A0
//! prints the reset causes, then how many MCLK cycles 32 reads from FRAM take with 0 to 7 wait states, and
//! after that a line for each bit error the FRAM corrects.
//!
//! A wait state adds an MCLK cycle to every FRAM access that the cache can't serve. The 32 reads are 8
//! bytes apart, so each one is in a different cache line, and TA0 times them, counting SMCLK, which runs
//! at the MCLK frequency. MCLK runs at about 1 MHz, which needs no wait state, so all eight settings work,
//! and the program goes back to none afterwards. It selects a reset for bit errors the FRAM can't correct,
//! and the system NMI for the ones it corrects. Bit errors can't be caused on purpose: on a sound device
//! the reset causes don't include `FramBitError`, and no corrected bit error is printed. The bit error
//! handling lasts until a BOR, so a run that starts without one finds it as the run before left it, and
//! `Fram::new()` switches it off.
//! (Wait states, NWAITS: SLAU445I 6.5, p. 302; SLAU445I Table 6-2, p. 306. Cache lines of four words:
//! SLAU445I 6.5.1, p. 303. No wait state up to 8 MHz: SLASEE4C 5.3, p. 17. TASSEL = 10b, SMCLK: SLASEE4C
//! Table 6-8, p. 49. Bit errors: SLAU445I 6.6, p. 303; UBDRSTEN and CBDIE: SLAU445I Table 6-3, p. 307.
//! Uncorrectable FRAM bit error, SYSRSTIV 1Ch, correctable, SYSSNIV 18h, and RST/NMI, SYSRSTIV 04h:
//! SLASEE4C Table 6-10, p. 52. A low level on RST/NMI resets the device: SLAU445I 1.2, p. 30. UCA0TXD is
//! P1.4: SLASEE4C Table 6-11, p. 53. No board document covers the adapter: there is none for the
//! MSP430FR25x2.)
//!
//! How to test (a 3.3-V USB-to-UART adapter):
//! 1. Connect the adapter: its RX to P1.4 (UCA0TXD), GND to GND. Open its COM port at 9600 baud.
//! 2. Flash this example, then reset the device, with RST/NMI low for a moment.
//! 3. Expected: `Reset causes: ResetPin`, then `MCLK cycles for 32 FRAM reads with 0 to 7 wait states:` and
//!    eight numbers, each larger than the one before, and then nothing more.
//! 4. A reset cause `FramBitError`, or a line `Corrected FRAM bit errors: 1`, would mean that the FRAM
//!    found a bit error.
//! 5. The bit error settings last until a BOR, but the next example switches them off again when it calls
//!    `Fram::new()`. (UBDRSTEN and CBDIE are `rw-[0]`: SLAU445I Figure 6-4, p. 307, with the key in
//!    SLAU445I Table 0-1, p. 28. RST/NMI low causes a BOR: SLAU445I 1.2, p. 30.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_io::Write;
use msp430_atomic::AtomicU16;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::{Fram, UncorrectableBitError, WaitStates},
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    sys::{self, SystemNmi},
    timer::{TimerConfig, TimerParts3},
    watchdog::Wdt,
};
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// 256 bytes to read, in FRAM: the linker puts constants in `.rodata`, in the ROM region of `memory.x`
static TABLE: [u16; 128] = [0; 128];

/// Corrected bit errors the `SYSNMI` handler has seen. The system NMI is non-maskable, so it can't share
/// data through a critical section, and this uses an atomic counter from `msp430-atomic`.
/// (NMIs "are not masked by the general interrupt enable (GIE) bit": SLAU445I 1.3.1, p. 33)
static CORRECTED: AtomicU16 = AtomicU16::new(0);

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58), 8N1 (SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::<_, _, DefaultMapping>::new(
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

    // Every reset cause, highest priority first, which also clears them (SLAU445I 1.3.7, p. 36)
    write!(tx, "\r\nReset causes:").ok();
    while let Some(cause) = pmm.take_reset_cause() {
        write!(tx, " {:?}", cause).ok();
    }
    writeln!(tx, "\r").ok();

    // A PUC for a bit error the FRAM can't correct (UBDRSTEN = 1), and the system NMI for one it corrects
    // (CBDIE = 1) (SLAU445I Table 6-3, p. 307)
    fram.set_uncorrectable_bit_error_action(UncorrectableBitError::Reset);
    fram.enable_correctable_bit_error_interrupts();

    // TA0 counts SMCLK (TASSEL = 10b: SLASEE4C Table 6-8, p. 49) from 0 to FFFFh, again and again (up mode:
    // SLAU445I 13.2.3.1, p. 371)
    let mut timer = TimerParts3::new(periph.ta0, TimerConfig::smclk(&smclk)).timer;
    timer.start(0xFFFF);

    // NWAITS = 0 to 7 (SLAU445I Table 6-2, p. 306). Up to 8 MHz MCLK needs no wait state, so each of them
    // works (fSYSTEM: SLASEE4C 5.3, p. 17).
    write!(tx, "MCLK cycles for 32 FRAM reads with 0 to 7 wait states:").ok();
    for wait_states in [
        WaitStates::Wait0,
        WaitStates::Wait1,
        WaitStates::Wait2,
        WaitStates::Wait3,
        WaitStates::Wait4,
        WaitStates::Wait5,
        WaitStates::Wait6,
        WaitStates::Wait7,
    ] {
        unsafe { fram.set_wait_states(wait_states) };
        let start = timer.count();
        // Every fourth word, so each read is in a different cache line of four words and is a cache miss
        // (SLAU445I 6.5.1, p. 303)
        for word in TABLE.iter().step_by(4) {
            unsafe { core::ptr::read_volatile(word) };
        }
        let cycles = timer.count().wrapping_sub(start);
        write!(tx, " {}", cycles).ok();
    }
    writeln!(tx, "\r").ok();
    unsafe { fram.set_wait_states(WaitStates::Wait0) };

    let mut seen = 0;
    loop {
        let corrected = CORRECTED.load();
        if corrected != seen {
            seen = corrected;
            writeln!(tx, "Corrected FRAM bit errors: {}\r", corrected).ok();
        }
    }
}

// The system NMI vector: vacant memory accesses, the JTAG mailbox and FRAM bit errors (FFFCh:
// SLASEE4C Table 6-2, p. 46). Reading SYSSNIV returns the highest-priority source and clears its flag
// (SLAU445I 1.3.7, p. 36).
#[interrupt]
fn SYSNMI() {
    while let Some(nmi) = sys::take_system_nmi() {
        if nmi == SystemNmi::FramCorrectableBitError {
            CORRECTED.add(1);
        }
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
