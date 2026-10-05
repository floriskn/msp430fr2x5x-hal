//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The JTAG mailbox: 16-bit and 32-bit messages between the program and a debugger tool, through the
//! debug connection. eUSCI_A0 prints each message the tool writes, the program answers it with the
//! message plus 1, and eUSCI_A0 reports each time the tool has read the outgoing mailbox. A button on
//! P2.2 switches between 16-bit and 32-bit messages.
//!
//! Both mailbox interrupts request the system NMI: one when a message has arrived, one when the tool has
//! read the outgoing message. A tool reaches the mailbox through 4-wire JTAG or Spy-Bi-Wire (SBW), with
//! the JTAG command JMB_EXCHANGE. That command works even on a device locked against JTAG access. The
//! user's guide names a password for device lock or unlock protection, and run-time data exchange, as
//! the mailbox's uses.
//! (JTAG mailbox: SLAU445I 1.10, p. 46 to p. 47; "a data exchange mechanism through SBW": SLASEE4C
//! 6.10.5, p. 52. JMB_EXCHANGE: SLAU445I 1.11.1, p. 47. The system NMI, FFFCh: SLASEE4C Table 6-2,
//! p. 46. UCA0TXD is P1.4: SLASEE4C Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58. P2.2 is a GPIO in both
//! packages: SLASEE4C Table 4-2, p. 14; SLASEE4C Table 6-16, p. 60. No board document covers the button:
//! there is none for the MSP430FR25x2.)
//!
//! How to test (a 3.3-V USB-to-UART adapter, a button or a jumper wire, and a debugger tool that reads
//! and writes the JTAG mailbox):
//! 1. Connect the adapter: its RX to P1.4 and its GND to GND. Connect a button from P2.2 to GND (the
//!    internal pullup is on); a wire from P2.2 that you touch to GND works as the button too.
//! 2. Flash this example, and open the adapter's COM port at 9600 baud. Reset the device if the terminal
//!    wasn't open yet.
//! 3. Expected, without a tool: `16-bit messages`, then `outgoing mailbox free`. The outgoing mailbox
//!    starts out empty, so its interrupt comes at once (JMBOUTIFG resets to 1: SLAU445I Table 1-10,
//!    p. 63).
//! 4. Connect the tool by Spy-Bi-Wire, on TEST and RST: 4-wire JTAG would take P1.4, the UART's TXD, as
//!    TCK (SLASEE4C Table 6-6, p. 47). Write the 16-bit message 1234h to the mailbox. Expected:
//!    `received 1234h`.
//! 5. Read the outgoing mailbox with the tool: the answer, 1235h. Expected: `outgoing mailbox free`. An
//!    answer can only go out once the tool has read the one before (JMBOUT0FG: SLAU445I Table 1-15,
//!    p. 68), so read each answer before writing the next message.
//! 6. Press the button: `32-bit messages`. Write the 32-bit message 12345678h: `received 12345678h`, and
//!    the answer is 12345679h. Press the button again for 16-bit messages. Switch only while no message
//!    is waiting in either direction: the user's guide warns that a partial message can be lost
//!    (JMBMODE: SLAU445I Table 1-15, p. 68).
//!
//! Steps 4 to 6 need a tool that we don't have: mspdebug (version 0.25) has no mailbox command in its
//! `help` list. Without one, only step 3 can be checked.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use embedded_hal::digital::*;
use embedded_io::Write;
use msp430_atomic::AtomicU16;
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
use msp430fr25x2::interrupt;
use panic_msp430 as _;

/// Messages that arrived, and outgoing messages the tool read, as the `SYSNMI` handler counts them. The
/// system NMI is non-maskable, so it can't share data through a critical section, and these are atomic
/// counters from `msp430-atomic`. (NMIs "are not masked by the general interrupt enable (GIE) bit":
/// SLAU445I 1.3.1, p. 33)
static ARRIVED: AtomicU16 = AtomicU16::new(0);
static READ_BY_TOOL: AtomicU16 = AtomicU16::new(0);

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The button pulls P2.2 low, with the internal pullup on (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I
    // Table 8-1, p. 313)
    let p2 = Batch::new(periph.p2)
        .config_pin2(|p| p.pullup())
        .split(&pmm);
    let mut button = p2.pin2;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // eUSCI_A0's TXD on P1.4, P1SEL = 01 in the default mapping, 8N1 (SLASEE4C Table 6-11, p. 53; SLASEE4C
    // Table 6-15, p. 58; SLAU445I Table 22-8, p. 593)
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

    // The mailbox starts in 16-bit mode (JMBMODE = 0 after a reset: SLAU445I Table 1-15, p. 68). JMBINIE
    // and JMBOUTIE request the system NMI (SLAU445I Table 1-9, p. 62; SLAU445I 1.10.4, p. 46 to p. 47).
    let sys = SysParts::new(periph.sfr);
    let mut mailbox16 = sys.jtag_mailbox;
    mailbox16.enable_rx_interrupts();
    mailbox16.enable_tx_interrupts();

    let mut arrived = 0;
    let mut read_by_tool = 0;
    loop {
        writeln!(tx, "\r\n16-bit messages\r").ok();
        while !pressed(&mut button) {
            if changed(&ARRIVED, &mut arrived) {
                if let Ok(msg) = mailbox16.read() {
                    writeln!(tx, "received {:04X}h\r", msg).ok();
                    // The answer. WouldBlock if the tool hasn't read the last one yet: then it's dropped.
                    mailbox16.write(msg.wrapping_add(1)).ok();
                }
            }
            if changed(&READ_BY_TOOL, &mut read_by_tool) {
                writeln!(tx, "outgoing mailbox free\r").ok();
            }
        }

        // JMBMODE = 1 (SLAU445I Table 1-15, p. 68)
        let mut mailbox32 = mailbox16.into_32bit();
        writeln!(tx, "32-bit messages\r").ok();
        while !pressed(&mut button) {
            if changed(&ARRIVED, &mut arrived) {
                if let Ok(msg) = mailbox32.read() {
                    writeln!(tx, "received {:08X}h\r", msg).ok();
                    mailbox32.write(msg.wrapping_add(1)).ok();
                }
            }
            if changed(&READ_BY_TOOL, &mut read_by_tool) {
                writeln!(tx, "outgoing mailbox free\r").ok();
            }
        }
        mailbox16 = mailbox32.into_16bit();
    }
}

/// Whether `counter` has changed since `seen`, which then follows it
fn changed(counter: &AtomicU16, seen: &mut u16) -> bool {
    let now = counter.load();
    let changed = now != *seen;
    *seen = now;
    changed
}

/// Whether the button is pressed. Then waits for its release, and for the bouncing to stop.
fn pressed(button: &mut impl InputPin) -> bool {
    if button.is_high().unwrap() {
        return false;
    }
    while button.is_low().unwrap() {}
    for _ in 0..10_000 {
        msp430::asm::nop();
    }
    true
}

// The system NMI vector: vacant memory accesses, the JTAG mailbox and FRAM bit errors (FFFCh: SLASEE4C
// Table 6-2, p. 46). Reading SYSSNIV returns the highest-priority source and clears its flag (SLAU445I
// 1.3.7, p. 36), JMBINIFG or JMBOUTIFG here. A message stays in the mailbox for main to read: JMBIN0FG,
// which `read()` checks, is cleared by reading SYSJMBI0 (SLAU445I Table 1-15, p. 68).
#[interrupt]
fn SYSNMI() {
    while let Some(nmi) = sys::take_system_nmi() {
        match nmi {
            SystemNmi::JtagMailboxIn => ARRIVED.add(1),
            SystemNmi::JtagMailboxOut => READ_BY_TOOL.add(1),
            _ => {}
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
