//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The bootloader (BSL) settings. At the start the backchannel UART prints why the device reset and
//! whether a BSL entry sequence was detected, then the first word of the BSL memory, as it is and with
//! the BSL memory switched off. Each press of S1 takes one more step: protect the BSL and read the word
//! again, then assign the lowest 16 bytes of RAM to the protected BSL and read one of them. That read is
//! a security violation, which resets the device, so the next start prints `reset: SecurityViolation`.
//! (SYSBSLC: SLAU445I Table 1-14, p. 67. SYSBSLIND: SLAU445I Table 1-13, p. 66. "When the BSL memory is
//! protected, access to these RAM locations is only possible from within the protected BSL memory
//! segments": SLAU445I 1.9.4, p. 45. BSL memory from 1000h and RAM from 2000h: SLASEO7C Table 9-31,
//! p. 73. Security violation (BOR), SYSRSTIV 0Ah: SLASEO7C Table 9-10, p. 52. S1 is P4.0: SLAU802
//! Figure 19, p. 25.)
//!
//! How to test:
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9). Press S3 to start the example again with the terminal
//!    open.
//! 2. Expected: `reset: ResetPin`, `no BSL entry sequence detected`, then `BSL memory at 1000h:` and
//!    a word, and `switched off: 3FFF`: the BSL memory then reads like vacant memory (SYSBSLOFF: SLAU445I
//!    Table 1-14, p. 67).
//! 3. Press S1. Expected: `protected:` and the same word as in step 2. (Measured on an MSP430FR2476: the
//!    program could still read the BSL memory with the protection on, see the HAL's
//!    `Bsl::set_protection`.)
//! 4. Press S1 again. Expected: `reading 2000h`, then the device resets and starts with
//!    `reset: SecurityViolation`. (Measured on an MSP430FR2476, see the HAL's `Bsl::set_ram_assigned`.)
//!    If the read were allowed, the program would print `read`, the value and `no reset` instead. The
//!    reset is a BOR, which also clears the BSL settings (`rw-[0]`: SLAU445I Table 1-14, p. 67, with the
//!    key in SLAU445I Table 0-1, p. 28).
//!
//! The BSL entry sequence is a pattern on the TEST and RST pins that starts the BSL (SLASEO7C 9.5, p. 47).
//! This test doesn't apply one, so expect `no BSL entry sequence detected`.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    sys::SysParts,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The first word of the BSL memory, 1000h to 17FFh (SLASEO7C Table 9-31, p. 73)
const BSL_MEMORY: usize = 0x1000;
/// The first word of RAM (SLASEO7C Table 9-31, p. 73), in the lowest 16 bytes that SYSBSLR assigns to the
/// BSL (SLAU445I Table 1-14, p. 67)
const BSL_RAM: usize = 0x2000;

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // S1 pulls P4.0 low, with the internal pullup on (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313) besides R9 (SLAU802 Figure 19, p. 25)
    let p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.pullup())
        .split(&pmm);
    let mut s1 = p4.pin0;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9; SLASEO7C
    // Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
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

    // Every reason for the reset, highest priority first. Reading them clears them (SLAU445I 1.3.7, p. 36).
    write!(tx, "\r\nreset:").ok();
    while let Some(cause) = pmm.take_reset_cause() {
        write!(tx, " {:?}", cause).ok();
    }
    writeln!(tx, "\r").ok();

    let mut bsl = SysParts::new(periph.sfr).bsl;
    // SYSBSLIND (SLAU445I Table 1-13, p. 66)
    if bsl.entry_detected() {
        writeln!(tx, "BSL entry sequence detected\r").ok();
    } else {
        writeln!(tx, "no BSL entry sequence detected\r").ok();
    }

    writeln!(tx, "BSL memory at {:04X}h: {:04X}\r", BSL_MEMORY, read(BSL_MEMORY)).ok();
    // SYSBSLOFF = 1, then 0 again (SLAU445I Table 1-14, p. 67)
    bsl.set_memory_off(true);
    writeln!(tx, "switched off: {:04X}\r", read(BSL_MEMORY)).ok();
    bsl.set_memory_off(false);

    writeln!(tx, "press S1 to protect the BSL\r").ok();
    wait_for_s1(&mut s1);
    // SYSBSLPE = 1 (SLAU445I Table 1-14, p. 67)
    bsl.set_protection(true);
    writeln!(tx, "protected: {:04X}\r", read(BSL_MEMORY)).ok();

    writeln!(tx, "press S1 to assign RAM to the BSL and read it\r").ok();
    wait_for_s1(&mut s1);
    writeln!(tx, "reading {:04X}h\r", BSL_RAM).ok();
    // Send the text before the reset
    tx.flush().ok();
    // SYSBSLR = 1 (SLAU445I Table 1-14, p. 67). Safety: the program's RAM starts at 2000h (memory.x), but
    // from here on nothing uses 2000h to 200Fh but the read that follows, which is the test: no
    // interrupts are enabled, and the stack is at the top of RAM.
    unsafe { bsl.set_ram_assigned(true) };
    let value = read(BSL_RAM);
    writeln!(tx, "read {:04X}, no reset\r", value).ok();

    loop {
        msp430::asm::nop();
    }
}

/// The word at `address`
fn read(address: usize) -> u16 {
    unsafe { core::ptr::read_volatile(address as *const u16) }
}

/// Wait until S1 is pressed and released, and for the bouncing to stop
fn wait_for_s1(s1: &mut impl InputPin) {
    while s1.is_high().unwrap() {}
    while s1.is_low().unwrap() {}
    for _ in 0..10_000 {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
