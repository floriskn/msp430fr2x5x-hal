//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A stress test for erratum CS13: the device can lock up when it enters LPM3 or LPM4 with the DCO above
//! 2 MHz while an interrupt arrives. With MCLK at 16 MHz, the program goes to sleep and is woken up again
//! and again; every 2048 sleeps LED1 toggles and the backchannel UART prints the count. A lock-up stops
//! both.
//!
//! The erratum: "The device might enter lockup state if DCO frequency is above 2 MHz and two events happen
//! at the same time: 1) The device transitions from AM to LPM3/4 ... 2) An interrupt is requested", and
//! only a BOR or a power cycle ends it (SLAZ695J CS13, p. 8). `request_lpm3()` and `request_lpm4()` apply
//! its workaround 4: "Set DCOCLK to 2MHz or lower before entering LPM3/4, then restore DCOCLK after
//! wake-up" (SLAZ695J CS13, p. 9). They slow the DCO to below 1 MHz, with the FLL off, until they return,
//! so the interrupt handlers run slowly too (see the HAL's `lpm` module).
//!
//! In LPM3, TB0 counts ACLK from REFO and interrupts every 16 cycles, about every 0.5 ms, and its handler
//! wakes the CPU. Before each sleep the main loop waits a random 0 to 511 µs, so the sleeps start at every
//! point between two interrupts, some of them just as an interrupt arrives. LPM4 stops ACLK, and with it
//! TB0, so with `LPM4_FROM_GENERATOR` the program sleeps in LPM4 instead, woken by an I/O interrupt at each
//! rising edge of a square wave from a function generator on P2.2. SMCLK runs at MCLK's frequency and
//! nothing uses MODCLK, so the conditions of erratum PMM32 aren't met (SLAZ695J PMM32, p. 9 to p. 10): a
//! lock-up here is CS13.
//! (REFO: 32768 Hz, SLASEC4D Table 5-7, p. 40. REFO, ACLK and TB0 run in LPM3, ACLK stops in LPM4, and I/O
//! wakes the device from LPM4: SLASEC4D Table 6-1, p. 61 to p. 62. LED1 on P1.0 is red, S3 is the reset
//! button, and P2.2 goes to the header only: SLAU680 Figure 18, p. 26.)
//!
//! How to test (for LPM4 a function generator, and the scope to check its signal):
//! 1. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11).
//! 2. Expected: `start, reset cause: 00h`, then `sleeps: 2048`, `sleeps: 4096` and so on, with LED1
//!    toggling at each line. The cause is the value of SYSRSTIV (SLASEC4D Table 6-12, p. 70): 00h, none,
//!    straight after flashing, and 04h after S3.
//! 3. Leave it running for at least an hour, overnight to be sure: the errata sheet gives no failure rate.
//!    Pass: the count keeps going up. Fail: LED1 and the count stop (a lock-up: press S3 to recover), or a
//!    new `start` line appears (a reset).
//! 4. To check that the test can catch the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the
//!    loop then goes to sleep at 16 MHz with a plain write to the status register. If it locks up that way
//!    but not with the HAL, the workaround works. If it doesn't lock up either way, the test can't tell.
//! 5. LPM4: set the generator to a square wave, 2 kHz, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output load
//!    High-Z, and check the levels on the scope first: a negative or >3.6 V signal can damage the pin.
//!    Connect it to P2.2 (J2 pin 18), its ground to GND (J2 pin 20), set `LPM4_FROM_GENERATOR` to true, and
//!    repeat steps 1 to 4. Keep the generator on: nothing else wakes the device, so without it the count
//!    stops.
//! 6. TB0 keeps running, interrupt and all, through a reset that is only a PUC, so the next example you
//!    flash could hang in the default interrupt handler once it enables interrupts. After flashing it,
//!    press S3 once. (Timer_B registers reset at a POR, "rw-(0)": SLAU445I Figure 14-16, p. 409; SLAU445I
//!    Table 0-1, p. 28. S3, the RST pin, resets with a BOR, which includes a POR: SLAU445I 1.2, p. 30.)
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::{arch::asm, cell::RefCell};
use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, Mutex};
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV},
    lpm::{request_lpm3, request_lpm4},
    pmm::Pmm,
    serial::*,
    timer::{TBxIV, TimerConfig, TimerParts3, TimerVector},
    watchdog::Wdt,
};
use msp430fr2355::{interrupt, Tb0, P2};
use panic_msp430 as _;

/// `false` goes to sleep without the HAL's workaround, to check that this test can catch the lock-up
const USE_HAL_WORKAROUND: bool = true;
/// `true` sleeps in LPM4, woken by a function generator on P2.2, instead of in LPM3, woken by TB0
const LPM4_FROM_GENERATOR: bool = false;

/// The status register bits of LPM3, SCG1, SCG0 and CPUOFF, and of LPM4, with OSCOFF as well (SLAU445I
/// Table 1-2, p. 39; SLAU445I Figure 4-9, p. 130)
const LPM3_BITS: u16 = 0x00D0;
const LPM4_BITS: u16 = 0x00F0;
/// TB0's period in ACLK cycles: about 0.5 ms, with ACLK from REFO at 32768 Hz (SLASEC4D Table 5-7, p. 40)
const TB0_PERIOD: u16 = 16;

static TB0IV: Mutex<RefCell<Option<TBxIV<Tb0>>>> = Mutex::new(RefCell::new(None));
static P2IV: Mutex<RefCell<Option<PxIV<P2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);
    // Read the first reset cause, then the rest, which also clears them for the next reset (reading
    // SYSRSTIV clears the highest pending flag: SLAU445I 1.3.7, p. 36)
    let cause = pmm.take_reset_cause();
    while pmm.take_reset_cause().is_some() {}

    let p1 = Batch::new(periph.p1).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();

    // MCLK = SMCLK = DCOCLKDIV in the 16 MHz range, above the erratum's 2 MHz, and ACLK from REFO (SELMS =
    // 000b, SELA = 01b: SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A1's TXD on P4.3, P4SEL = 01, 8N1 (SLAU680 2.2.4, p. 11; SLASEC4D
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
    tx.write_all(b"start, reset cause: ").ok();
    print_hex(&mut tx, cause.map_or(0, u16::from));
    tx.write_all(b"\r\n").ok();

    // TB0 counts ACLK (TBSSEL = 01b: SLASEC4D Table 6-9, p. 68) from 0 to TB0_PERIOD - 1 in up mode ("The
    // number of timer counts in the period is TBxCL0 + 1": SLAU445I 14.2.3.1, p. 394), and TBIE requests
    // the interrupt for TBIFG, which is set when it wraps around to 0 (SLAU445I Table 14-6, p. 410). Setting
    // it up stops it, MC = 0 (SLAU445I Table 14-6, p. 409): in LPM4 mode it stays stopped, without its
    // interrupt, even if a run in LPM3 mode left it running. Stopped, it requests no ACLK, which would
    // turn LPM4 into LPM3: a timer does when it "selects ACLK as its clock source and the timer is
    // enabled" (SLAU445I 3.2.12, p. 108).
    let parts = TimerParts3::new(periph.tb0, TimerConfig::aclk(&aclk));
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
        timer.start(TB0_PERIOD - 1);
        timer.enable_interrupts();
        with(|cs| TB0IV.borrow_ref_mut(cs).replace(parts.tbxiv));
    }

    // Set GIE, which masks every maskable interrupt while clear (SLAU445I 1.3.3, p. 33)
    unsafe { enable_interrupts() };

    let mut random: u16 = 1;
    let mut sleeps: u32 = 0;
    loop {
        // Wait a random 0 to 511 µs, so the sleep starts at another point between two interrupts
        random = next_random(random);
        delay.delay_us((random & 0x1FF) as u32);

        if USE_HAL_WORKAROUND {
            if LPM4_FROM_GENERATOR {
                request_lpm4();
            } else {
                request_lpm3();
            }
        } else if LPM4_FROM_GENERATOR {
            // A plain entry, as the errata sheet's own code does it, with `__bis_SR_register()` (SLAZ695J
            // PMM32, p. 11)
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM4_BITS, options(nostack)) };
        } else {
            unsafe { asm!("bis.w #{bits}, SR", "nop", bits = const LPM3_BITS, options(nostack)) };
        }
        sleeps += 1;

        if sleeps % 2048 == 0 {
            led1.toggle().ok();
            tx.write_all(b"sleeps: ").ok();
            print_decimal(&mut tx, sleeps);
            tx.write_all(b"\r\n").ok();
            // Finish sending before the next sleep: a busy eUSCI keeps its clock, SMCLK, running, which
            // turns LPM3 and LPM4 into LPM0 (SLAU445I 22.3.14, p. 590; SLAU445I Table 1-3, p. 39)
            tx.flush().ok();
        }
    }
}

/// The next number of a xorshift generator, for the random waits
fn next_random(x: u16) -> u16 {
    let x = x ^ (x << 7);
    let x = x ^ (x >> 9);
    x ^ (x << 8)
}

/// Print `n` in decimal. `write!` would do it too, but `core::fmt` takes several KiB of FRAM, more than the
/// MSP430FR2522 version of this test has to spare.
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

// The TB0 vector of CCR1, CCR2 and TBIFG, decoded with TB0IV (FFF6h: SLASEC4D Table 6-2, p. 63). Reading
// TB0IV clears the flag it reports (SLAU445I 14.2.6.2, p. 405). `wake_cpu` returns the CPU to active mode,
// so the main loop carries on after the sleep (the SR saved on the stack: SLAU445I 1.4.2, p. 40).
#[interrupt(wake_cpu)]
fn TIMER0_B1() {
    with(|cs| {
        if let Some(tb0iv) = TB0IV.borrow_ref_mut(cs).as_mut() {
            let _: TimerVector = tb0iv.interrupt_vector();
        }
    });
}

// The port 2 vector, P2IFG.0 to P2IFG.7 through P2IV (FFD2h: SLASEC4D Table 6-2, p. 64), with `wake_cpu` as
// above. Reading P2IV "automatically resets the highest pending interrupt flag" (SLAU445I 8.2.6, p. 315).
#[interrupt(wake_cpu)]
fn PORT2() {
    with(|cs| {
        if let Some(p2iv) = P2IV.borrow_ref_mut(cs).as_mut() {
            let _: GpioVector = p2iv.get_interrupt_vector();
        }
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
