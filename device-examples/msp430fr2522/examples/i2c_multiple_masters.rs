//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! Two I2C masters on one bus, on two MSP430FR2522: ten times a second board A, a multi-master, writes a
//! byte to board B and reads it back. Board B, a master that is also a slave, answers in its interrupt,
//! and then, as master, checks whether a device answers at address 0x09. Board A's LED on P1.0 lights when
//! the byte came back, and board B's LED on P1.0 when no device answered at 0x09.
//!
//! When another master addresses it, board B stops being a master and works as a slave; after the STOP
//! its interrupt makes it a master again, and its main loop takes the bus while board A waits. Its own
//! address is 26 (0x1A). Both run at 100 kHz from SMCLK, with the internal pull-ups on their pins.
//! eUSCI_B0 is the device's only eUSCI_B, so each board plays one part: `MASTER_SLAVE` picks which.
//! (A master addressed as a slave "becomes a slave": SLAU445I Table 24-2, p. 646. One eUSCI_B: SLASEE4C
//! 1.1, p. 1. eUSCI_B0's I2C pins: SLASEE4C Table 6-11, p. 53. No board document covers the LEDs: there is
//! none for the MSP430FR25x2.)
//!
//! How to test (two MSP430FR2522, two LEDs and resistors, three jumper wires):
//! 1. Power both MSP430FR2522 from 3.3 V, and on each connect an LED with a series resistor (about 1 kΩ)
//!    from P1.0 to GND.
//! 2. Connect SDA, P1.2, of one to P1.2 of the other, SCL, P1.3, to P1.3, and GND to GND. The internal
//!    pull-ups may be too weak: if it doesn't work, add resistors from SDA and SCL to 3.3 V.
//! 3. Set `MASTER_SLAVE` to true and flash this example to board B. Set it back to false and flash it to
//!    board A.
//! 4. Expected: both LEDs light and stay on.
#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

// We use UnsafeCell here over RefCell to minimise binary size. Binary size suffers when panics are possible, as they pull in lots of
// strings and formatting. Debug builds from old compiler versions suffer in particular.
use core::cell::UnsafeCell;

use critical_section::Mutex;
use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c};
use msp430::interrupt::enable as enable_interrupts;
use msp430_atomic::AtomicBool;
use msp430_rt::entry;
use msp430fr25x2::{interrupt, EUsciB0};
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch,
    i2c::{GlitchFilter, I2cConfig, I2cInterruptFlags as Flags, I2cMasterSlave, I2cMultiMaster, I2cVector},
    pin_mapping::DefaultMapping, pmm::Pmm, prelude::*, watchdog::Wdt
};
use panic_msp430 as _;

/// Which board this is: board B, the master-slave, or board A, the multi-master
const MASTER_SLAVE: bool = false;
/// Board B's own address
const MASTER_SLAVE_ADDR: u8 = 26;

static I2C_MULTI_MASTER: Mutex<UnsafeCell<Option< I2cMasterSlave<EUsciB0> >>> = Mutex::new(UnsafeCell::new(None));
/// Set by board B's interrupt handler when a transaction that addressed it has ended
static ADDRESSED: AtomicBool = AtomicBool::new(false);

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

    if MASTER_SLAVE {
        // Board B: an I2C device that is both master and slave. The device will automatically failover from
        // master to slave when addressed.
        // (SLAU445I Table 24-2, p. 646: arbitration is lost "when the eUSCI_B operates as master but is
        // addressed as a slave by another master in the system", and then "the UCMST bit is cleared and
        // the I2C controller becomes a slave".)
        // Attempting any master actions will fail until the slave event has been handled.
        let mut i2c_master_slave: I2cMasterSlave<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_master_slave(MASTER_SLAVE_ADDR)
                .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz (fBitClock = fBRCLK/UCBRx: SLAU445I 24.3.7, p. 642)
                .configure(scl, sda);

        critical_section::with(|cs| {
            i2c_master_slave.set_interrupts(Flags::StartReceived);
            unsafe { *I2C_MULTI_MASTER.borrow(cs).get() = Some(i2c_master_slave) }
        });
        unsafe { enable_interrupts() };

        loop {
            // The master-slave is set back into master mode in the StopReceived interrupt. Board A has just
            // finished its transaction then, and waits 100 ms before the next, so the bus is free: check
            // whether a device with address 0x9 is on the bus (aka a zero-byte write)
            if ADDRESSED.load() {
                ADDRESSED.store(false);
                critical_section::with(|cs| {
                    let Some(i2c_master_slave) = unsafe { &mut *I2C_MULTI_MASTER.borrow(cs).get() }.as_mut() else {
                        return;
                    };
                    if let Ok(false) = i2c_master_slave.is_slave_present(9u8) {
                        led.set_high().ok(); // Turn on the LED if address 0x9 is not on bus
                    }
                });
            }
        }
    } else {
        // Board A: since there are two masters on the bus this has to be a multi-master, rather than a
        // single-master.
        let mut i2c_master: I2cMultiMaster<_, DefaultMapping> =
            I2cConfig::new(periph.e_usci_b0, GlitchFilter::Max50ns)
                .as_multi_master()
                .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz (fBitClock = fBRCLK/UCBRx: SLAU445I 24.3.7, p. 642)
                .configure(scl, sda);

        loop {
            // The master sends a byte then receives a byte from the master-slave.
            // The master-slave echoes the master's byte back.
            let mut echo_rx = [0; 1];
            const ECHO_TX: [u8; 1] = [128; 1];

            // Start a transaction using the master. This will force the master-slave into slave mode.
            // We don't care about errors for this example, but should be handled in a real implementation.
            let _ = i2c_master.write_read(MASTER_SLAVE_ADDR, &ECHO_TX, &mut echo_rx);

            // If the I2C devices echoed correctly set the LED
            led.set_state((echo_rx == ECHO_TX).into()).ok();
            delay.delay_ms(100);
        }
    }
}

// Static mut variables defined inside an interrupt handler are safe. See: https://docs.rust-embedded.org/book/start/interrupts.html
// The eUSCI_B0 receive or transmit vector at FFEAh (SLASEE4C Table 6-2, p. 46)
#[allow(static_mut_refs)]
#[interrupt]
fn EUSCI_B0() {
    static mut TEMP_VAR: u8 = 0;
    critical_section::with(|cs| {
        let Some(i2c_master_slave) = unsafe { &mut *I2C_MULTI_MASTER.borrow(cs).get() }.as_mut() else { return; };
        match i2c_master_slave.interrupt_source() {
            I2cVector::StartReceived => {
                // We have been addressed as a slave. Enable Rx, Tx and Stop interrupts.
                // (UCSTTIFG is set when the module "detects a START condition together with its own address":
                // SLAU445I Table 24-2, p. 646)
                i2c_master_slave.set_interrupts(Flags::TxBufEmpty | Flags::RxBufFull | Flags::StopReceived);
            }
            I2cVector::RxBufFull => {
                // Store the received value so we can echo it back later when the master switches to read mode
                // (UCRXIFG0 is set when a data byte is received: SLAU445I 24.3.5.1.2, p. 634)
                *TEMP_VAR = unsafe { i2c_master_slave.read_rx_buf_as_slave_unchecked() };
            }
            I2cVector::TxBufEmpty => {
                // Echo back the stored value
                // (When the master reads, "UCTR and UCTXIFG0 become set": SLAU445I 24.3.5.1.1, p. 633)
                unsafe { i2c_master_slave.write_tx_buf_as_slave_unchecked(*TEMP_VAR) };
            }
            I2cVector::StopReceived => {
                // Slave addressing concluded. Disable Rx, Tx, and Stop interrupts. We don't want these to trigger when acting as a master.
                // (UCSTPIFG "is set when the I2C module detects a STOP condition on the bus" and is "used in
                // slave and master mode": SLAU445I Table 24-2, p. 646)
                i2c_master_slave.clear_interrupts(Flags::TxBufEmpty | Flags::RxBufFull | Flags::StopReceived);
                i2c_master_slave.return_to_master();
                ADDRESSED.store(true);
            }
            _ => (), // unreachable
        }
    })
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
