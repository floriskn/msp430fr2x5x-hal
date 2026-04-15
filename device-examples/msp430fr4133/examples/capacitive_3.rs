#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::{
    arch::{asm, naked_asm},
    cell::UnsafeCell,
};

use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use embedded_hal_nb::serial::Write;
use msp430::{
    asm::nop,
    interrupt::{disable, enable, Mutex},
};
use msp430_rt::entry;
use msp430fr2x5x_hal::{
    capacitive::*,
    capture::{CapTrigger, CaptureParts3, CapturePin},
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, Output, Pin, Pin0},
    lpm::enter_lpm0,
    pmm::Pmm,
    pwm::{TimerConfig, TimerDiv, TimerExDiv},
    serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, SerialUsci, StopBits, Tx},
    timer::TimerParts3,
    watchdog::{Wdt, WdtClkPeriods},
};
use msp430fr413x::{interrupt, timer_0_a3::ta0cctl1::Ccis, watchdog_timer::wdtctl::Wdtssel, P4};
use nb::block;
use panic_msp430 as _;

const KEY_LVL: i32 = 10000;

static mut BASE_CNT: i32 = 0;
static mut MEAS_CNT: i32 = 0;

static LED: Mutex<UnsafeCell<Option<Pin<P4, Pin0, Output>>>> = Mutex::new(UnsafeCell::new(None));

// Red onboard LED should blink at a steady period.
#[entry]
fn main() -> ! {
    let periph = msp430fr413x::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.fram);
    let mut wdt = Wdt::constrain(periph.watchdog_timer).to_interval();

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    let mut p1 = Batch::new(periph.p1)
        .config_pin3(|p| p.to_output())
        .split(&pmm);

    let mut p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.to_output())
        .split(&pmm);

    let mut tx = SerialConfig::new(
            periph.usci_a0_uart_mode,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            // Launchpad UART-to-USB converter doesn't handle parity, so we don't use it
            Parity::NoParity,
            Loopback::NoLoop,
            9600,
        )
        .use_smclk(&smclk)
        .tx_only(p1.pin0.to_alternate1());

    p1.pin3.set_low();
    p4.pin0.set_high();

    // with(|cs| unsafe { *LED.borrow(cs).get() = Some(p4.pin0) });

    // periph.capacitive_touch_io_0.captio0ctl().read().bits();

    // 1. Counting hardware
    let capacitive = CapacitiveParts3::new(periph.timer_0_a3);

    let el5 = CapacitiveElement { pin: p1.pin6, threshold: 100, max_response: 500 };
    let el6 = CapacitiveElement { pin: p1.pin7, threshold: 100, max_response: 500 };

    // FIX: Bind the WDT gate to a variable too
    let sw_gate = SwGate::new();

    let mut s3 = CapacitiveSensor::new_fro(
        &capacitive,
        [&el5, &el6],
        &sw_gate, // Use the named variable
        &periph.capacitive_touch_io_0,
        1000,
    );

    // --- Now the group works ---
    // let mut group = CapacitiveGroup::new([
    //     &mut s3
    // ]);

    s3.init();
    s3.update(5);
    
    loop {
        let button = s3.buttons();
        if let Some(button) = button {
            p1.pin3.set_high();

            match button {
                0 => {
                    embedded_io::Write::write_all(&mut tx, b"BUTTON 0\r\n").ok();
                },
                1 => {
                    embedded_io::Write::write_all(&mut tx, b"BUTTON 1\r\n").ok();
                }
                _ => {}
            }
        } else {
            p1.pin3.set_low();
        }
        p4.pin0.toggle();

        delay.delay_ms(100);
    }
}
