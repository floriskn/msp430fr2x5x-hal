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
    capture::{CapTrigger, CaptureParts3, CapturePin}, clock::{ClockConfig, DcoclkFreqSel, MclkDiv, Smclk, SmclkDiv}, fram::Fram, gpio::{Batch, Output, Pin, Pin0}, lpm::{enter_lpm0, request_lpm3}, pmm::Pmm, pwm::{TimerConfig, TimerDiv, TimerExDiv}, serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, SerialUsci, StopBits, Tx}, timer::TimerParts3, watchdog::{IntervalMode, WatchdogSelect, Wdt, WdtClkPeriods}
};
use msp430fr413x::{P4, interrupt, timer_0_a3::ta0cctl1::Ccis, watchdog_timer::wdtctl::Wdtssel};
use msp430::{asm, interrupt::{Mutex, disable, enable as enable_interrupts}};
use nb::block;
use panic_msp430 as _;

const KEY_LVL: i32 = 20000;

static mut BASE_CNT: i32 = 0;
static mut MEAS_CNT: i32 = 0;


static LED: Mutex<UnsafeCell<Option<Pin<P4, Pin0, Output>>>> =
    Mutex::new(UnsafeCell::new(None));

// Red onboard LED should blink at a steady period.
#[entry]
fn main() -> ! {
    let periph = msp430fr413x::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.fram);
    let mut wdt = Wdt::constrain(periph.watchdog_timer).to_interval();
    wdt.enable_interrupts();
    
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
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


    // let captures = CaptureParts3::config(periph.timer_0_a3, TimerConfig::aclk(&aclk))
    //     .config_cap1_trigger(CapTrigger::BothEdges)
    //     .commit(); 
    
        
    p1.pin3.set_low();
    p4.pin0.set_high();

    embedded_io::Write::write_all(&mut tx, b"HELLO\r\n").ok();

    with(|cs| unsafe { *LED.borrow(cs).get() = Some(p4.pin0) });
    
    unsafe { enable_interrupts() };

    measure_count(&mut tx, &mut wdt, &smclk);
    unsafe { BASE_CNT = MEAS_CNT };

    for _ in 0..15 {
        measure_count(&mut tx, &mut wdt, &smclk);
        unsafe { BASE_CNT = (MEAS_CNT + BASE_CNT) / 2 };
    }

    loop {
        let mut j = KEY_LVL;
        let mut key_pressed = false;
        
        measure_count(&mut tx, &mut wdt, &smclk);

        let mut delta_cnt = unsafe { BASE_CNT - MEAS_CNT };
        // print_num(&mut tx, delta_cnt);
        // write(&mut tx, '\r');
        // write(&mut tx, '\n');

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

        wdt.set_aclk(&aclk).set_interval_and_start(WdtClkPeriods::_512);

        if !key_pressed {
            unsafe { BASE_CNT = BASE_CNT - 150 };
        }
        // pulse_LED
        if key_pressed {
            p1.pin3.set_high();
        } else {
            p1.pin3.set_low();
        }

        with(|cs| {
            let Some(led) = unsafe { &mut *LED.borrow(cs).get() }.as_mut() else { return; };
            led.toggle();
        });

        request_lpm3();
    }
}

fn measure_count<U: SerialUsci>(tx: &mut Tx<U>, wdt: &mut Wdt<IntervalMode>, smlk: &Smclk) {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };
    
    periph.timer_0_a3.ta0ctl().write(|w| unsafe { w.tassel().bits(3).mc().bits(2).taclr().set_bit() });
    periph.timer_0_a3.ta0cctl1().write(|w| unsafe { w.cm().bits(3).ccis().bits(2).cap().set_bit() });
    // periph.timer_0_a3.ta0cctl1().modify(|_, w| w.ccifg().clear_bit());

    periph.capacitive_touch_io_0.captio0ctl().modify(|_, w| w.captioen().set_bit().captioposel0().set_bit().captiopisel1().set_bit().captiopisel2().set_bit());
    
    wdt.set_smclk(&smlk).set_interval_and_start(WdtClkPeriods::_512k);
    enter_lpm0();
    periph.timer_0_a3.ta0cctl1().modify(|r, w| unsafe { w.ccis().bits(r.ccis().bits() ^ 0b01) });
    unsafe { MEAS_CNT = periph.timer_0_a3.ta0ccr1().read().bits() as i32 };
    wdt.pause();
    periph.capacitive_touch_io_0.captio0ctl().write(|w| unsafe { w.bits(0) });
}

fn print_bin_u16<U: SerialUsci>(tx: &mut Tx<U>, n: u16) {
    // Start with a mask for the most significant bit (2^15)
    // 0x8000 = 1000 0000 0000 0000 in binary
    let mut mask = 0x8000u16;

    // Iterate through all 16 bits
    while mask > 0 {
        if (n & mask) != 0 {
            write(tx, '1');
        } else {
            write(tx, '0');
        }
        
        // Shift mask one position to the right
        mask >>= 1;
    }
}

fn print_num<U: SerialUsci>(tx: &mut Tx<U>, n: i32) {
    let mut n = n;

    if n == 0 {
        write(tx, '0');
        return;
    }

    if n < 0 {
        write(tx, '-');
        n = n.wrapping_abs();
    }

    let mut val = n as u32;
    // Powers of 10 for u32
    let powers: [u32; 10] = [
        1_000_000_000, 100_000_000, 10_000_000, 1_000_000, 
        100_000, 10_000, 1_000, 100, 10, 1
    ];

    let mut printing = false;

    for &p in powers.iter() {
        let mut digit = 0u8;
        // Repeated subtraction instead of division/modulo
        while val >= p {
            val -= p;
            digit += 1;
        }
        
        if digit > 0 || printing {
            write(tx, (digit + b'0') as char);
            printing = true;
        }
    }
}


fn write<U: SerialUsci>(tx: &mut Tx<U>, ch: char) {
    // Instead of unwrap(), use a match or discard the result
    let _ = nb::block!(tx.write(ch as u8)); 
}

#[interrupt(wake_cpu)]
fn WDT() {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };
    
    periph.timer_0_a3.ta0cctl1().modify(|r, w| unsafe { w.ccis().bits(r.ccis().bits() ^ 0b01) });

    // periph.timer_0_a3.ta0cctl1().modify(|r, w| );
}

