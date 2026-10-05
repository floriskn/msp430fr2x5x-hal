//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A stress test for erratum GC5: after a wake-up from LPM3 or LPM4, the FRAM's error detection can report
//! bit errors the FRAM doesn't have. With both kinds of bit error reported by the system NMI, the program
//! goes to sleep and is woken up again and again, and counts the NMIs. Every 2048 wake-ups LED1 toggles and
//! the backchannel UART prints the counts, which must stay at 0.
//!
//! The erratum: "The FRAM bit error detection may indicate bit errors, even the memory has no failure,
//! after wakeup from LPM1/2/3/4": a PUC with UBDRSTEN set, or an NMI with UBDIE or CBDIE set (SLAZ664S GC5,
//! p. 10 to p. 11). `request_lpm3()` and `request_lpm4()` apply its workaround (SLAZ664S GC5, p. 11): they
//! "Disable PUC (GCCTL0.UBDRSTEN=0), UBDIE and CBDIE interrupts ... prior to entering LPM", and after the
//! wake-up they read the FRAM where its cache can't answer, then "clear GCCTL1.UBDIFG and GCCTL1.CBDIFG,
//! and then reinitialize the GCCTL0 register after the first valid FRAM access has been completed" (see the
//! HAL's `lpm` module).
//!
//! In LPM3, TA0 counts ACLK from REFO and interrupts every 16 cycles, about every 0.5 ms, and its handler
//! wakes the CPU. LPM4 stops ACLK, and only an I/O wakes the device from it, so with `LPM4_FROM_GENERATOR`
//! the program sleeps in LPM4 instead, woken by each rising edge of a square wave from a function generator
//! on P2.2. MCLK runs at 1 MHz and SMCLK at the same frequency, and nothing uses MODCLK, so the conditions
//! of errata CS13, PMM32 and GC4 aren't met (SLAZ664S CS13, p. 9; SLAZ664S PMM32, p. 11 to p. 12; SLAZ664S
//! GC4, p. 10): a bit error here is GC5.
//! (Uncorrectable and correctable bit errors request the system NMI with UBDIE and CBDIE: SLAU445I
//! Table 6-3, p. 307; SYSSNIV 04h and 18h: SLASE59F Table 6-9, p. 48. REFO: 32768 Hz, SLASE59F Table 5-7,
//! p. 25. REFO, ACLK and TA0 run in LPM3, the FRAM is off in LPM3 and LPM4, and only I/O wakes the device
//! from LPM4: SLASE59F Table 6-1, p. 40 to p. 41. LED1 on P1.0 is red, S3 is the reset button, and P2.2
//! goes to the header only: SLAU739 Figure 18, p. 23.)
//!
//! How to test (for LPM4 a function generator, and the scope to check its signal):
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU739 2.2.4, p. 9).
//! 2. Expected: `start, reset cause: 00h`, then
//!    `wake-ups: 2048, FRAM bit errors: 0 correctable, 0 uncorrectable`, and so on, with LED1 toggling at
//!    each line. The cause is the value of SYSRSTIV (SLASE59F Table 6-9, p. 48): 00h, none, straight after
//!    flashing, and 04h after S3.
//! 3. Leave it running for at least an hour, overnight to be sure: the errata sheet gives no failure rate.
//!    Pass: both counts stay at 0. Fail: a count goes up.
//! 4. To check that the test can catch the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the
//!    loop then goes to sleep with a plain write to the status register, with the error reporting on. If
//!    errors show up that way but not with the HAL, the workaround works. If they don't show up either way,
//!    the test can't tell.
//! 5. LPM4: set the generator to a square wave, 2 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z, and check the levels on the scope first: a negative or >3.6 V signal can damage the pin.
//!    Connect it to P2.2 (J2 pin 18), its ground to GND (J2 pin 20), set `LPM4_FROM_GENERATOR` to true, and
//!    repeat steps 1 to 4. Keep the generator on: nothing else wakes the device, so without it the count
//!    stops.
//! 6. TA0 keeps running, interrupt and all, through a reset that is only a PUC, so the next example you
//!    flash could hang in the default interrupt handler once it enables interrupts. After flashing it,
//!    press S3 once. (Timer_A registers reset at a POR, "rw-(0)": SLAU445I Figure 13-16, p. 384; SLAU445I
//!    Table 0-1, p. 28. S3, the RST pin, resets with a BOR, which includes a POR: SLAU445I 1.2, p. 30.)
//!    UBDIE and CBDIE need nothing: only a BOR resets them, but the next example's `Fram::new()` clears
//!    them (SLAU445I Figure 6-4, p. 307; see the HAL's `fram` module).
//! (Header pins: SLAU739 Figure 18, p. 23.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::{arch::asm, cell::RefCell};
use critical_section::with;
use embedded_hal::digital::*;
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_atomic::AtomicU16;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::{Fram, UncorrectableBitError},
    gpio::{Batch, GpioVector, PxIV},
    lpm::{request_lpm3, request_lpm4},
    pmm::Pmm,
    serial::*,
    sys::{take_system_nmi, SystemNmi},
    timer::{TBxIV, TimerConfig, TimerParts3, TimerVector},
    watchdog::Wdt,
};
use msp430fr2433::{interrupt, Ta0, P2};
use panic_msp430 as _;

/// `false` goes to sleep without the HAL's workaround, to check that this test can catch the erratum
const USE_HAL_WORKAROUND: bool = true;
/// `true` sleeps in LPM4, woken by a function generator on P2.2, instead of in LPM3, woken by TA0
const LPM4_FROM_GENERATOR: bool = false;

/// The status register bits of LPM3, SCG1, SCG0 and CPUOFF, and of LPM4, with OSCOFF as well (SLAU445I
/// Table 1-2, p. 39; SLAU445I Figure 4-9, p. 130)
const LPM3_BITS: u16 = 0x00D0;
const LPM4_BITS: u16 = 0x00F0;
/// TA0's period in ACLK cycles: about 0.5 ms, with ACLK from REFO at 32768 Hz (SLASE59F Table 5-7, p. 25)
const TA0_PERIOD: u16 = 16;

static TA0IV: Mutex<RefCell<Option<TBxIV<Ta0>>>> = Mutex::new(RefCell::new(None));
static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));
/// The FRAM bit errors the system NMI reported. The NMI can't be masked, so it can't share data through a
/// critical section: it counts with atomics.
static CORRECTABLE: AtomicU16 = AtomicU16::new(0);
static UNCORRECTABLE: AtomicU16 = AtomicU16::new(0);

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

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range, and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The system NMI for uncorrectable bit errors (UBDIE) and for corrected ones (CBDIE) (SLAU445I
    // Table 6-3, p. 307)
    fram.set_uncorrectable_bit_error_action(UncorrectableBitError::Interrupt);
    fram.enable_correctable_bit_error_interrupts();

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

    // TA0 counts ACLK (TASSEL = 01b: SLASE59F Table 6-7, p. 46) from 0 to TA0_PERIOD - 1 in up mode ("The
    // number of timer counts in the period is TAxCCR0 + 1": SLAU445I 13.2.3.1, p. 371), and TAIE requests
    // the interrupt for TAIFG, which is set when it wraps around to 0 (SLAU445I Table 13-4, p. 384). Setting
    // it up stops it, MC = 0 (SLAU445I Table 13-4, p. 384): in LPM4 mode it stays stopped, without its
    // interrupt, even if a run in LPM3 mode left it running. Stopped, it requests no ACLK, which would
    // turn LPM4 into LPM3: a timer does when it "selects ACLK as its clock source and the timer is
    // enabled" (SLAU445I 3.2.12, p. 108).
    let parts = TimerParts3::new(periph.ta0, TimerConfig::aclk(&aclk));
    let mut timer = parts.timer;
    if LPM4_FROM_GENERATOR {
        timer.disable_interrupts();

        // P2.2 with its pulldown, so it doesn't float without the generator (PxDIR = 0, PxREN = 1, PxOUT = 0:
        // SLAU445I Table 8-1, p. 313), and a rising edge requests the port 2 interrupt (PxIES = 0: SLAU445I
        // Table 8-16, p. 336; PxIE: SLAU445I Table 8-17, p. 336)
        let p2 = Batch::new(periph.p2).config_pin2(|p| p.pulldown()).split(&pmm);
        let mut input = p2.pin2;
        input.select_rising_edge_trigger().enable_interrupts();
        with(|cs| P2IV.borrow_ref_mut(cs).replace(p2.pxiv));
    } else {
        timer.start(TA0_PERIOD - 1);
        timer.enable_interrupts();
        with(|cs| TA0IV.borrow_ref_mut(cs).replace(parts.tbxiv));
    }

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    let mut wake_ups: u32 = 0;
    loop {
        if USE_HAL_WORKAROUND {
            if LPM4_FROM_GENERATOR {
                request_lpm4();
            } else {
                request_lpm3();
            }
        } else if LPM4_FROM_GENERATOR {
            // A plain entry, as the errata sheet's own code does it, with `__bis_SR_register()` (SLAZ664S
            // PMM32, p. 12)
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM4_BITS, options(nostack)) };
        } else {
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM3_BITS, options(nostack)) };
        }
        wake_ups += 1;

        if wake_ups % 2048 == 0 {
            led1.toggle().ok();
            tx.write_all(b"wake-ups: ").ok();
            print_decimal(&mut tx, wake_ups);
            tx.write_all(b", FRAM bit errors: ").ok();
            print_decimal(&mut tx, CORRECTABLE.load().into());
            tx.write_all(b" correctable, ").ok();
            print_decimal(&mut tx, UNCORRECTABLE.load().into());
            tx.write_all(b" uncorrectable\r\n").ok();
            // Finish sending before the next sleep: a busy eUSCI keeps its clock, SMCLK, running, which
            // turns LPM3 and LPM4 into LPM0 (SLAU445I 22.3.14, p. 590; SLAU445I Table 1-3, p. 39)
            tx.flush().ok();
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

// The TA0 vector of CCR1, CCR2 and TAIFG, decoded with TA0IV (FFF6h: SLASE59F Table 6-2, p. 41). Reading
// TA0IV clears the flag it reports (SLAU445I 13.2.6.2, p. 380). `wake_cpu` returns the CPU to active mode,
// so the main loop carries on after the sleep (the SR saved on the stack: SLAU445I 1.4.2, p. 40).
#[interrupt(wake_cpu)]
fn TIMER0_A1() {
    with(|cs| {
        if let Some(ta0iv) = TA0IV.borrow_ref_mut(cs).as_mut() {
            let _: TimerVector = ta0iv.interrupt_vector();
        }
    });
}

// The port 2 vector, P2IFG.0 to P2IFG.7 through P2IV (FFDAh: SLASE59F Table 6-2, p. 42), with `wake_cpu` as
// above. Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
#[interrupt(wake_cpu)]
fn PORT2() {
    with(|cs| {
        if let Some(p2iv) = P2IV.borrow_ref_mut(cs).as_mut() {
            let _: GpioVector = p2iv.get_interrupt_vector();
        }
    });
}

// The system NMI vector: vacant memory, the JTAG mailbox and FRAM bit errors (FFFCh: SLASE59F Table 6-2,
// p. 41). Reading SYSSNIV gives the highest pending source and clears its flag, UBDIFG or CBDIFG here
// (SLAU445I 1.3.7, p. 36; SLAU445I Table 6-4, p. 308).
#[interrupt]
fn SYSNMI() {
    while let Some(nmi) = take_system_nmi() {
        match nmi {
            SystemNmi::FramCorrectableBitError => CORRECTABLE.add(1),
            SystemNmi::FramUncorrectableBitError => UNCORRECTABLE.add(1),
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
