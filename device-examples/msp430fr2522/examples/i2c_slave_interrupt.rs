//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! An I2C slave that works in its interrupt, with the master on a second MSP430FR2522: ten times a second
//! the master writes a value from 0 to 9 into the first byte of the slave's array. The master's LED on
//! P1.0 toggles after each write, and the slave's LED on P1.0 lights while that byte is 0: for 100 ms once
//! a second.
//!
//! The slave, eUSCI_B0 at address 0x1A, holds an 8-byte array. The first byte a master writes sets the
//! index; the bytes written or read after it go to, or come from, that index and the ones after it. A
//! transaction that starts with a read uses the index of the one before (0 at first). The master runs at
//! 100 kHz from SMCLK. eUSCI_B0 is the device's only eUSCI_B, so the master is a second MSP430FR2522,
//! running this example with `SLAVE` set to false. Both have the internal pull-ups on their pins.
//! (One eUSCI_B: SLASEE4C 1.1, p. 1. eUSCI_B0's I2C pins: SLASEE4C Table 6-11, p. 53. SDA and SCL need
//! pull-ups: SLAU445I 24.3, p. 629. No board document covers the LEDs: there is none for the MSP430FR25x2.)
//!
//! How to test (two MSP430FR2522, two LEDs and resistors, three jumper wires):
//! 1. Power both MSP430FR2522 from 3.3 V, and on each connect an LED with a series resistor (about 1 kΩ)
//!    from P1.0 to GND.
//! 2. Connect SDA, P1.2, of one to P1.2 of the other, SCL, P1.3, to P1.3, and GND to GND. The internal
//!    pull-ups may be too weak: if it doesn't work, add resistors from SDA and SCL to 3.3 V.
//! 3. Set `SLAVE` to true and flash this example to the slave. Set it back to false and flash it to the
//!    master.
//! 4. Expected: the master's LED toggles every 100 ms, and the slave's LED flashes once a second. Without
//!    the wires no write arrives, the byte stays 0, and the slave's LED stays on.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

// This example is quite big (particularly debug builds on old compiler versions), so we use a couple of tricks here to shrink binary size:
// Anything that can panic (like RefCell) will pull in some string formatting, which bloats the binary! Instead use UnsafeCell (carefully).
// Likewise avoid panics from array bounds checks by using .get_unchecked(), etc.

use core::cell::UnsafeCell;

use critical_section::Mutex;
use msp430::interrupt::enable as enable_interrupts;
use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}, i2c::I2c};
use msp430_atomic::AtomicU8;
use msp430_rt::entry;
use msp430fr25x2::{interrupt, EUsciB0};
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cInterruptFlags as Flags, I2cSingleMaster, I2cSlave, I2cVector},
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    prelude::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// Which board this is: the slave, or the master that writes to it
const SLAVE: bool = false;
/// The slave's own address (UCBxI2COA0: SLAU445I Table 24-11, p. 656)
const SLAVE_ADDR: u8 = 0x1A;

static I2C_SLAVE: Mutex<UnsafeCell<Option< I2cSlave<EUsciB0> >>> = Mutex::new(UnsafeCell::new(None));

// Store the exposed 'registers' as Atomic values, so they can be easily read/written to between the interrupt and main fn
const ARR_LEN: usize = 8;
static ARR: [AtomicU8; ARR_LEN] = [const { AtomicU8::new(0) }; ARR_LEN];

#[entry]
fn main() -> ! {
    let periph = msp430fr25x2::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Halt the watchdog, which runs from every PUC (SLAU445I 12.2.2, p. 363)
    let _wdt = Wdt::constrain(periph.wdt_a);

    // Pmm::new clears LOCKLPM5, so the pins take on their configuration (SLAU445I 8.3.1, p. 316)
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    // The LED on P1.0, a GPIO output: P1SELx = 00 and P1DIR = 1 (SLASEE4C Table 6-15, p. 58)
    let mut led = p1.pin0.to_output();
    // UCB0SCL on P1.3 and UCB0SDA on P1.2, P1SELx = 01 in the default mapping, USCIBRMP = 0 (SLASEE4C
    // Table 6-11, p. 53; SLASEE4C Table 6-15, p. 58). The internal pull-ups are 20 kΩ to 50 kΩ (SLASEE4C
    // Table 5-10, p. 29), and I2C needs pull-ups on SDA and SCL (SLAU445I 24.3, p. 629).
    let scl = p1.pin3.pullup().to_alternate1(); // You may need stronger external pullup resistors
    let sda = p1.pin2.pullup().to_alternate1();

    // MCLK = SMCLK = DCOCLKDIV in the 8 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b: SLAU445I
    // Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    if SLAVE {
        // The slave answers to its own address (UCMST = 0, the own address in UCBxI2COA0: SLAU445I
        // 24.3.5.1, p. 633)
        let mut i2c_slave: I2cSlave<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_slave(SLAVE_ADDR)
                .configure(scl, sda);

        i2c_slave.set_interrupts(Flags::StopReceived | Flags::RxBufFull | Flags::TxBufEmpty);

        critical_section::with(|cs| {
            unsafe { *I2C_SLAVE.borrow(cs).get() = Some(i2c_slave) }
        });

        unsafe { enable_interrupts(); }

        loop {
            // Enable the LED if the value at index 0 is 0.
            led.set_state((ARR[0].load() == 0).into()).ok();
        }
    } else {
        let mut i2c_master: I2cSingleMaster<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_single_master()
                .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz (fBitClock = fBRCLK/UCBRx: SLAU445I 24.3.7, p. 642)
                .configure(scl, sda);

        let index = 0;
        let mut value = 0;
        loop {
            // Write a value between 0 and 9 to index 0.
            let _ = i2c_master.write(SLAVE_ADDR, &[index, value]);
            value = (value + 1) % 10;

            // Toggle the LED after each
            led.toggle().ok();
            delay.delay_ms(100);
        }
    }
}

// Static mut variables defined inside an interrupt handler are safe. See: https://docs.rust-embedded.org/book/start/interrupts.html
// The eUSCI_B0 receive or transmit vector at FFEAh (SLASEE4C Table 6-2, p. 46)
#[allow(static_mut_refs)]
#[interrupt]
fn EUSCI_B0() {
    static mut BYTE_COUNT: u8 = 0; // Bytes since the initial start condition
    static mut ARR_INDEX: usize = 0;

    critical_section::with(|cs| {
        let Some(i2c_slave) = unsafe { &mut *I2C_SLAVE.borrow(cs).get() }.as_mut() else { return; };
        match i2c_slave.interrupt_source() {
            I2cVector::RxBufFull => {
                // Safety: Rx interrupt triggered, so Rx buffer is ready.
                // ("After the first data byte is received, the receive interrupt flag UCRXIFG0 is set":
                // SLAU445I 24.3.5.1.2, p. 634)
                let val = unsafe { i2c_slave.read_rx_buf_unchecked() };
                // If this is the first byte treat the I2C byte as the array index
                if *BYTE_COUNT == 0 {
                    *ARR_INDEX = val as usize % ARR_LEN;
                } else {
                    // Otherwise treat the I2C byte as data to be stored
                    unsafe { ARR.get_unchecked(*ARR_INDEX) }.store(val);
                    *ARR_INDEX = (*ARR_INDEX + 1) % ARR_LEN; // Autoincrement index
                }
            }
            I2cVector::TxBufEmpty => {
                // Safety: ARR_INDEX is always less than ARR_LEN.
                let val = unsafe { ARR.get_unchecked(*ARR_INDEX) }.load();
                // Safety: Tx interrupt triggered, so Tx buffer is ready.
                // (When the master reads, "UCTR and UCTXIFG0 become set" and SCL is held low until data is
                // written to UCBxTXBUF: SLAU445I 24.3.5.1.1, p. 633)
                unsafe { i2c_slave.write_tx_buf_unchecked(val) };
                *ARR_INDEX = (*ARR_INDEX + 1) % ARR_LEN; // Autoincrement index
            }
            I2cVector::StopReceived => {
                *ARR_INDEX = ARR_INDEX.wrapping_sub(1).min(ARR_LEN - 1); // Undo last autoincrement
                *BYTE_COUNT = 0;
                return;
            }
            _ => (), // unreachable
        }
        *BYTE_COUNT += 1;
    })
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
