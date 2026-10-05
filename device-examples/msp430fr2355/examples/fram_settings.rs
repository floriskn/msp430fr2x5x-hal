//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The FRAM controller's settings: its wait states, and what it does when it finds a bit error. The
//! backchannel UART prints the bit error handling the program started with and what `Fram::new()` left of
//! it, the reset causes, then how many MCLK cycles 32 reads from FRAM take with 0 to 7 wait states, and
//! after that a line for each bit error the FRAM corrects.
//!
//! A wait state adds an MCLK cycle to every FRAM access that the cache can't serve. The 32 reads are 8
//! bytes apart, so each one is in a different cache line, and TB0 times them, counting SMCLK, which runs
//! at the MCLK frequency. MCLK runs at about 1 MHz, which needs no wait state, so all eight settings work,
//! and the program goes back to none afterwards. It selects a reset for bit errors the FRAM can't correct,
//! and the system NMI for the ones it corrects. Bit errors can't be caused on purpose: on a sound device
//! the reset causes don't include `FramBitError`, and no corrected bit error is printed. The bit error
//! handling lasts until a BOR, so a run that starts without one finds it as the run before left it, and
//! `Fram::new()` switches it off.
//! (Wait states, NWAITS: SLAU445I 6.5, p. 302; SLAU445I Table 6-2, p. 306. Cache lines of four words:
//! SLAU445I 6.5.1, p. 303. No wait state up to 8 MHz: SLASEC4D 5.3, p. 27. TBSSEL = 10b, SMCLK: SLASEC4D
//! Table 6-9, p. 68. Bit errors: SLAU445I 6.6, p. 303; UBDRSTEN and CBDIE: SLAU445I Table 6-3, p. 307.
//! Uncorrectable FRAM bit error, SYSRSTIV 1Ch, and correctable, SYSSNIV 18h: SLASEC4D Table 6-12, p. 70. S3
//! is the reset button: SLAU680 Figure 18, p. 26.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11). Then press the reset button S3.
//! 2. Expected: `Bit error handling (UBDRSTEN, UBDIE, CBDIE) at start: 000, after Fram::new(): 000`,
//!    `Reset causes: ResetPin`, then `MCLK cycles for 32 FRAM reads with 0 to 7 wait states:` and eight
//!    numbers, each larger than the one before, and then nothing more.
//! 3. A reset cause `FramBitError`, or a line `Corrected FRAM bit errors: 1`, would mean that the FRAM
//!    found a bit error.
//! 4. Flash the example again, without pressing S3. If the debugger restarts the device without a BOR, as
//!    mspdebug did on an MSP430FR2476, the first line shows the bit error handling of the run before, and
//!    that `Fram::new()` switched it off: `Bit error handling (UBDRSTEN, UBDIE, CBDIE) at start: 101,
//!    after Fram::new(): 000`. (UBDRSTEN, UBDIE and CBDIE are `rw-[0]`, reset by a BOR only: SLAU445I
//!    Figure 6-5, p. 307, with the key in SLAU445I Table 0-1, p. 28. The RST pin causes a BOR: SLAU445I
//!    1.2, p. 30.)
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
    pmm::Pmm,
    serial::*,
    sys::{self, SystemNmi},
    timer::{TimerConfig, TimerParts3},
    watchdog::Wdt,
};
use msp430fr2355::interrupt;
use panic_msp430 as _;

/// 256 bytes to read, in FRAM: the linker puts constants in `.rodata`, in the ROM region of `memory.x`
static TABLE: [u16; 128] = [0; 128];

/// Corrected bit errors the `SYSNMI` handler has seen. The system NMI is non-maskable, so it can't share
/// data through a critical section, and this uses an atomic counter from `msp430-atomic`.
/// (NMIs "are not masked by the general interrupt enable (GIE) bit": SLAU445I 1.3.1, p. 33)
static CORRECTED: AtomicU16 = AtomicU16::new(0);

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    // The bit error handling this run starts with, and what `Fram::new()` leaves of it
    let at_start = bit_error_handling();
    let mut fram = Fram::new(periph.frctl);
    let after_new = bit_error_handling();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p4 = Batch::new(periph.p4).split(&pmm);

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SELx = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
    // Table 6-66, p. 102; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::new(
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

    tx.write_all(b"\r\nBit error handling (UBDRSTEN, UBDIE, CBDIE) at start: ").ok();
    tx.write_all(&at_start).ok();
    tx.write_all(b", after Fram::new(): ").ok();
    tx.write_all(&after_new).ok();
    tx.write_all(b"\r\n").ok();

    // Every reset cause, highest priority first, which also clears them (SLAU445I 1.3.7, p. 36)
    write!(tx, "Reset causes:").ok();
    while let Some(cause) = pmm.take_reset_cause() {
        write!(tx, " {:?}", cause).ok();
    }
    writeln!(tx, "\r").ok();

    // A PUC for a bit error the FRAM can't correct (UBDRSTEN = 1), and the system NMI for one it corrects
    // (CBDIE = 1) (SLAU445I Table 6-3, p. 307)
    fram.set_uncorrectable_bit_error_action(UncorrectableBitError::Reset);
    fram.enable_correctable_bit_error_interrupts();

    // TB0 counts SMCLK (TBSSEL = 10b: SLASEC4D Table 6-9, p. 68) from 0 to FFFFh, again and again (up mode:
    // SLAU445I 14.2.3.1, p. 394)
    let mut timer = TimerParts3::new(periph.tb0, TimerConfig::smclk(&smclk)).timer;
    timer.start(0xFFFF);

    // NWAITS = 0 to 7 (SLAU445I Table 6-2, p. 306). Up to 8 MHz MCLK needs no wait state, so each of them
    // works (fSYSTEM: SLASEC4D 5.3, p. 27).
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

/// The bit error handling, UBDRSTEN, UBDIE and CBDIE in GCCTL0 (SLAU445I Table 6-3, p. 307), as the
/// characters `0` and `1`
fn bit_error_handling() -> [u8; 3] {
    let gcctl0 = unsafe { &*msp430fr2355::Frctl::ptr() }.gcctl0().read();
    [gcctl0.ubdrsten().bit(), gcctl0.ubdie().bit(), gcctl0.cbdie().bit()].map(|bit| b'0' + bit as u8)
}

// The system NMI vector: vacant memory accesses, the JTAG mailbox and FRAM bit errors (FFFCh:
// SLASEC4D Table 6-2, p. 63). Reading SYSSNIV returns the highest-priority source and clears its flag
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
