#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

use core::{arch::{asm, naked_asm}, cell::UnsafeCell};

use critical_section::with;
use embedded_hal::{delay::DelayNs, digital::*};
use embedded_hal_nb::serial::Write;
use msp430_rt::{entry};
use msp430fr2x5x_hal::{
    capture::{CapTrigger, CaptureParts3, CapturePin}, clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::{Batch, Output, Pin, Pin0}, lpm::enter_lpm0, pmm::Pmm, pwm::{TimerConfig, TimerDiv, TimerExDiv}, serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, SerialUsci, StopBits, Tx}, timer::TimerParts3, watchdog::{Wdt, WdtClkPeriods}
};
use msp430fr413x::{P4, interrupt, timer_0_a3::ta0cctl1::Ccis, watchdog_timer::wdtctl::Wdtssel};
use msp430::{asm::nop, interrupt::{Mutex, disable, enable}};
use nb::block;
use panic_msp430 as _;

const KEY_LVL: i32 = 10000;

static mut BASE_CNT: i32 = 0;
static mut MEAS_CNT: i32 = 0;


static LED: Mutex<UnsafeCell<Option<Pin<P4, Pin0, Output>>>> =
    Mutex::new(UnsafeCell::new(None));

// Red onboard LED should blink at a steady period.
#[entry]
fn main() -> ! {
    let periph = msp430fr413x::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.fram);
    
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    let mut p1 = Batch::new(periph.p1)
        .config_pin0(|p| p.to_output())
        .split(&pmm);

    let mut p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    
    periph.watchdog_timer.wdtctl().write(|w| unsafe { w.wdtpw().bits(0x5A).wdthold().set_bit() });
    
    periph.sfr.sfrie1().modify(|_, w| w.wdtie().set_bit());

    p1.pin0.set_low();
    p4.pin0.set_high();

    with(|cs| unsafe { *LED.borrow(cs).get() = Some(p4.pin0) });
    
    unsafe { asm!("bis.b #8, SR", options(nomem, nostack, preserves_flags)) };

    measure_count();
    unsafe { BASE_CNT = MEAS_CNT };

    for _ in 0..15 {
        measure_count();
        unsafe { BASE_CNT = (MEAS_CNT + BASE_CNT) / 2 };
    }

    loop {
        let mut j = KEY_LVL;
        let mut key_pressed = false;

        measure_count(); 

        let mut delta_cnt = unsafe { BASE_CNT - MEAS_CNT };

        if delta_cnt < 0 {
            unsafe { BASE_CNT = (BASE_CNT + MEAS_CNT) >> 1 };
            delta_cnt = 0;
        }
        if delta_cnt > j {
            j = delta_cnt;
            key_pressed = true;
        } else {
            key_pressed = false;
        }

        periph.watchdog_timer.wdtctl().write(|w| unsafe { w.wdtpw().bits(0x5A).wdttmsel().set_bit().wdtcntcl().set_bit().wdtis().variant(WdtClkPeriods::_512).wdtssel().variant(Wdtssel::Aclk) });

        if !key_pressed {
            unsafe { BASE_CNT = BASE_CNT - 150 };
        }
        // pulse_LED
        if key_pressed {
            p1.pin0.set_high();
        } else {
            p1.pin0.set_low();
        }

        with(|cs| {
            let Some(led) = unsafe { &mut *LED.borrow(cs).get() }.as_mut() else { return; };
            led.toggle();
        });

        unsafe { asm!("bis.b #208, SR", options(nomem, nostack)) };
    }
}

fn measure_count() {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };

    periph.timer_0_a3.ta0ctl().write(|w| unsafe { w.tassel().bits(3).mc().bits(2) });
    periph.timer_0_a3.ta0cctl1().write(|w| unsafe { w.cm().bits(3).ccis().bits(2).cap().set_bit() });

    periph.capacitive_touch_io_0.captio0ctl().modify(|_, w| w.captioen().set_bit().captioposel0().set_bit().captiopisel0().set_bit());

    periph.watchdog_timer.wdtctl().write(|w| unsafe { w.wdtpw().bits(0x5A).wdttmsel().set_bit().wdtcntcl().set_bit().wdtis().variant(WdtClkPeriods::_512k).wdtssel().variant(Wdtssel::Smclk) });
    periph.timer_0_a3.ta0ctl().modify(|_, w| w.taclr().set_bit());    
    unsafe { asm!("bis.b #24, SR", options(nomem, nostack)) };
    periph.timer_0_a3.ta0cctl1().modify(|r, w| unsafe { w.bits(r.bits() ^ 0x1000) });
    unsafe { MEAS_CNT = periph.timer_0_a3.ta0ccr1().read().bits() as i32 };
    periph.watchdog_timer.wdtctl().write(|w| unsafe { w.wdtpw().bits(0x5A).wdthold().set_bit() });
    periph.capacitive_touch_io_0.captio0ctl().write(|w| unsafe { w.bits(0) });
}

#[interrupt(wake_cpu)]
fn WDT() {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };

    periph.timer_0_a3.ta0cctl1().modify(|r, w| unsafe { w.bits(r.bits() ^ 0x1000) });
}

