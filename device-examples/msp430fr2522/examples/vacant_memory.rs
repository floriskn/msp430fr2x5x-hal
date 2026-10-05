//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The vacant memory access interrupt: reading an address where no memory exists requests the system
//! NMI. Each press of a button on P2.3 reads address 2800h, just past the RAM, and the `SYSNMI` handler
//! reports the access. An LED on P1.0 toggles for each access the handler sees, and eUSCI_A0 prints what
//! was read.
//! (Vacant memory: SLAU445I 1.9.2, p. 45. The MSP430FR2522 has RAM from 2000h to 27FFh, and nothing from
//! 2800h up to the CapTIvate library ROM at 4000h: SLASEE4C Table 6-19, p. 62. P1.0 and P2.3 are GPIO with
//! PxSELx = 00: SLASEE4C Table 6-15, p. 58; SLASEE4C Table 6-16, p. 60. P2.3 only exists on the 20-pin RHL
//! package: SLASEE4C Table 4-2, p. 14. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53. No board document
//! covers the parts to connect: there is none for the MSP430FR25x2.)
//!
//! How to test (an LED and a resistor, a push button, and a 3.3-V USB-to-UART adapter):
//! 1. Connect an LED with a series resistor (about 1 kΩ) from P1.0 to GND, and a push button from P2.3 to
//!    GND (the internal pullup is on). Connect the adapter: its RX to P1.4 (UCA0TXD), its GND to GND. Open
//!    its COM port at 9600 baud.
//! 2. Flash this example. Expected: `Press the button to read vacant memory`.
//! 3. Press and release the button.
//!
//! Expected for each press: `Read 3FFF from 2800h, vacant memory accesses: 1` (then 2, 3, ...), and the
//! LED toggles.
//! A read from vacant memory gives 3FFFh (SLAU445I 1.9.2, p. 45: "Reads from vacant memory result in
//! the value 3FFFh").
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

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
    sys::{self, SysParts, SystemNmi},
    watchdog::Wdt,
};
use msp430_atomic::AtomicU16;
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// The first address past the RAM (2000h to 27FFh), below the CapTIvate library ROM (from 4000h)
/// (SLASEE4C Table 6-19, p. 62)
const VACANT_ADDRESS: usize = 0x2800;

/// Vacant memory accesses the `SYSNMI` handler has seen. The system NMI is non-maskable, so it can't
/// share data through a critical section, and this uses an atomic counter from `msp430-atomic`.
/// (NMIs "are not masked by the general interrupt enable (GIE) bit": SLAU445I 1.3.1, p. 33)
static VACANT_ACCESSES: AtomicU16 = AtomicU16::new(0);

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The button pulls P2.3 low, against the internal pullup (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0.to_output_low();
    let mut button = p2.pin3;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4: UCA0TXD with P1SELx = 01 in the default mapping, USCIARMP = 0, 8N1
    // (SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58; SLAU445I Table 22-8, p. 593)
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

    // VMAIE requests the system NMI on vacant memory accesses (SLAU445I 1.9.2, p. 45; VMAIE: SLAU445I
    // Table 1-9, p. 62)
    let mut sys = SysParts::new(periph.sfr);
    sys.vacant_memory.enable_interrupts();

    print(&mut tx, "\r\nPress the button to read vacant memory\r\n");
    let mut seen = 0;
    loop {
        if button.is_low().unwrap() {
            // Wait for the release, and for the bouncing to stop
            while button.is_low().unwrap() {}
            for _ in 0..10_000 {
                msp430::asm::nop();
            }
            let value = unsafe { core::ptr::read_volatile(VACANT_ADDRESS as *const u16) };
            print(&mut tx, "Read ");
            print_hex(&mut tx, value as u32, 4);
            print(&mut tx, " from ");
            print_hex(&mut tx, VACANT_ADDRESS as u32, 4);
            print(&mut tx, "h, ");
        }

        // The NMI arrives a few instructions after the access (measured on an MSP430FR2476), so the
        // count is checked here rather than right after the read
        let accesses = VACANT_ACCESSES.load();
        if accesses != seen {
            seen = accesses;
            led.toggle().ok();
            print(&mut tx, "vacant memory accesses: ");
            print_num(&mut tx, accesses as u32);
            print(&mut tx, "\r\n");
        }
    }
}

// Numbers are printed by hand: the formatting code of `write!` takes several KB, and this device has 7.25 KB
// of program FRAM (SLASEE4C Table 6-19, p. 62).

fn print(tx: &mut impl Write, text: &str) {
    tx.write_all(text.as_bytes()).ok();
}

/// Print `value` in decimal
fn print_num(tx: &mut impl Write, value: u32) {
    let mut digits = [0u8; 10];
    let mut pos = digits.len();
    let mut rest = value;
    loop {
        pos -= 1;
        digits[pos] = b'0' + (rest % 10) as u8;
        rest /= 10;
        if rest == 0 {
            break;
        }
    }
    tx.write_all(&digits[pos..]).ok();
}

/// Print the low `digits` hexadecimal digits of `value`
fn print_hex(tx: &mut impl Write, value: u32, digits: u32) {
    for digit in (0..digits).rev() {
        let nibble = (value >> (4 * digit)) as u8 & 0xF;
        let c = if nibble < 10 { b'0' + nibble } else { b'A' + nibble - 10 };
        tx.write_all(&[c]).ok();
    }
}

// The system NMI vector: vacant memory accesses, the JTAG mailbox and FRAM bit errors (FFFCh:
// SLASEE4C Table 6-2, p. 46). Reading SYSSNIV returns the highest-priority source and clears its flag
// (SLAU445I 1.3.7, p. 36).
#[interrupt]
fn SYSNMI() {
    while let Some(nmi) = sys::take_system_nmi() {
        if nmi == SystemNmi::VacantMemoryAccess {
            VACANT_ACCESSES.add(1);
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
