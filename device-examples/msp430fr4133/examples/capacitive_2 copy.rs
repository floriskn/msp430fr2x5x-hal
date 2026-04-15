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
    capture::{CapTrigger, Capture, CaptureParts3, CapturePin, CaptureVector, TBxIV}, clock::{ClockConfig, DcoclkFreqSel, MclkDiv, Smclk, SmclkDiv}, fram::Fram, gpio::{Batch, Output, Pin, Pin0}, lpm::{enter_lpm0, request_lpm3}, pin_mapping::DefaultMapping, pmm::Pmm, pwm::{CCR1, TimerConfig, TimerDiv, TimerExDiv}, rtc::{Rtc, RtcDiv}, serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, SerialUsci, StopBits, Tx}, timer::TimerParts3, watchdog::{IntervalMode, WatchdogSelect, Wdt, WdtClkPeriods}
};
use msp430fr413x::{P4, UsciA0UartMode, interrupt, timer_0_a3::ta0cctl1::Ccis, watchdog_timer::wdtctl::Wdtssel};
use msp430::{asm, interrupt::{Mutex, disable, enable as enable_interrupts}};
use nb::block;
use panic_msp430 as _;

const KEY_LVL: i32 = 30000;

static mut BASE_CNT: i32 = 0;
static mut MEAS_CNT: i32 = 0;


static LED: Mutex<UnsafeCell<Option<Pin<P4, Pin0, Output>>>> =
    Mutex::new(UnsafeCell::new(None));
static TX: Mutex<UnsafeCell<Option<Tx<UsciA0UartMode>>>> =
    Mutex::new(UnsafeCell::new(None));
static RTC: Mutex<UnsafeCell<Option<Rtc<msp430fr2x5x_hal::rtc::RtcXt1clk>>>> =
    Mutex::new(UnsafeCell::new(None));
static CAPTURE: Mutex<UnsafeCell<Option<Capture<msp430fr413x::Timer0A3, CCR1>>>> =
    Mutex::new(UnsafeCell::new(None));
static VECTOR: Mutex<UnsafeCell<Option<TBxIV<msp430fr413x::Timer0A3, DefaultMapping>>>> =
    Mutex::new(UnsafeCell::new(None));

// Red onboard LED should blink at a steady period.
#[entry]
fn main() -> ! {
    let periph = msp430fr413x::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.fram);
    let mut wdt = Wdt::constrain(periph.watchdog_timer).to_interval();
    wdt.enable_interrupts();
    
    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    let mut p4 = Batch::new(periph.p4)
        .config_pin0(|p| p.to_output())
        .split(&pmm);   

    let (smclk, aclk, _delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_32)
        .smclk_on(SmclkDiv::_1)
        .aclk_xt1clk(32768, p4.pin1.to_alternate1(), p4.pin2.to_alternate1())
        .freeze(&mut fram);

    let mut p1 = Batch::new(periph.p1)
        .config_pin3(|p| p.to_output())
        .split(&pmm);

    let mut tx: Tx<msp430fr413x::generic::Periph<msp430fr413x::usci_a0_uart_mode::RegisterBlock, 1280>> = SerialConfig::new(
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

    let mut rtc = Rtc::new(periph.real_time_clock).use_xt1clk(&aclk);
    rtc.set_clk_div(RtcDiv::_10);
    // rtc.enable_interrupts();

    let captures = CaptureParts3::config(periph.timer_0_a3, TimerConfig::inclk())
        .config_cap1_trigger(CapTrigger::BothEdges)
        .config_cap1_input_B()
        .commit(); 
    let mut capture = captures.cap1;
    let vectors = captures.tbxiv;
    capture.enable_interrupts();

    // captures.cap1.
        
    p1.pin3.set_low();
    p4.pin0.set_high();

    embedded_io::Write::write_all(&mut tx, b"HELLO\n").ok();

    rtc.start(1000);

    with(|cs| unsafe { 
        *LED.borrow(cs).get() = Some(p4.pin0);
        *TX.borrow(cs).get() = Some(tx);
        *RTC.borrow(cs).get() = Some(rtc);
        *CAPTURE.borrow(cs).get() = Some(capture);
        *VECTOR.borrow(cs).get() = Some(vectors);
    });

    unsafe { enable_interrupts() };
    
    measure_count_2();
    unsafe { BASE_CNT = MEAS_CNT };

    for _ in 0..15 {
        measure_count_2();
        unsafe { BASE_CNT = (MEAS_CNT + BASE_CNT) / 2 };
    }

    loop {
        let mut j = KEY_LVL;
        let mut key_pressed = false;

        measure_count_2();

        let mut delta_cnt = unsafe { BASE_CNT - MEAS_CNT };

        with(|cs| {
            let Some(tx) = unsafe { &mut *TX.borrow(cs).get() }.as_mut() else { return; };
            print_num(tx, unsafe { MEAS_CNT });
            write(tx, '\r');
            write(tx, '\n');
        });

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

        if !key_pressed {
            unsafe { BASE_CNT = BASE_CNT - 150 };
        }
        // pulse_LED
        if key_pressed {
            p1.pin3.set_high();
        } else {
            p1.pin3.set_low();
        }
    }
    
    unsafe { enable_interrupts() };

    measure_count(&mut wdt, &smclk);
    unsafe { BASE_CNT = MEAS_CNT };

    for _ in 0..15 {
        measure_count(&mut wdt, &smclk);
        unsafe { BASE_CNT = (MEAS_CNT + BASE_CNT) / 2 };
    }

    loop {
        let mut j = KEY_LVL;
        let mut key_pressed = false;
        
        measure_count(&mut wdt, &smclk);

        let mut delta_cnt = unsafe { BASE_CNT - MEAS_CNT };
        print_num(&mut tx, delta_cnt);
        write(&mut tx, '\r');
        write(&mut tx, '\n');

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


// #[interrupt(wake_cpu)]
// fn RTC() {
//     with(|cs| {
//         let Some(tx) = unsafe { &mut *TX.borrow(cs).get() }.as_mut() else { return; };
//         let Some(rtc) = unsafe { &mut *RTC.borrow(cs).get() }.as_mut() else { return; };
//         embedded_io::Write::write_all(tx, b"y\n").ok();
//         rtc.start(2000);
//     });
// }


#[interrupt(wake_cpu)]
fn TIMER0_A1() {
    with(|cs| {
        let Some(tx) = unsafe { &mut *TX.borrow(cs).get() }.as_mut() else { return; };
        let Some(vector) = unsafe { &mut *VECTOR.borrow(cs).get() }.as_mut() else { return; };
        let Some(capture) = unsafe { &mut *CAPTURE.borrow(cs).get() }.as_mut() else { return; };
        let Some(led) = unsafe { &mut *LED.borrow(cs).get() }.as_mut() else { return; };
        let Some(rtc) = unsafe { &mut *RTC.borrow(cs).get() }.as_mut() else { return; };

        let _ = vector.interrupt_vector();

        led.toggle();

        // print_num(tx, unsafe { BASE_CNT - MEAS_CNT });
        // write(tx, '\r');
        // write(tx, '\n');
        // write(tx, 'x');

        rtc.start(1000);
    });    
    // with(|cs| {
    //     let Some(led) = unsafe { &mut *LED.borrow(cs).get() }.as_mut() else { return; };
    //     led.set_low();
    //     let Some(tx) = unsafe { &mut *TX.borrow(cs).get() }.as_mut() else { return; };
    //     embedded_io::Write::write_all(tx, b"x\n").ok();
    // });
}

fn measure_count_2() {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };

    periph.capacitive_touch_io_0.captio0ctl().modify(|_, w| w.captioen().set_bit().captioposel0().set_bit().captiopisel1().set_bit().captiopisel2().set_bit());
    
    enter_lpm0();

    unsafe { MEAS_CNT = periph.timer_0_a3.ta0ccr1().read().bits() as i32 };

    periph.capacitive_touch_io_0.captio0ctl().write(|w| unsafe { w.bits(0) });
}

fn measure_count(wdt: &mut Wdt<IntervalMode>, smlk: &Smclk) {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };

    // TODO: impl inclk
    periph.timer_0_a3.ta0ctl().write(|w| unsafe { w.tassel().bits(3).mc().bits(2) });
    periph.timer_0_a3.ta0cctl1().write(|w| unsafe { w.cm().bits(3).ccis().bits(2).cap().set_bit() });

    periph.capacitive_touch_io_0.captio0ctl().modify(|_, w| w.captioen().set_bit().captioposel0().set_bit().captiopisel1().set_bit().captiopisel2().set_bit());
    
    wdt.set_smclk(&smlk).set_interval_and_start(WdtClkPeriods::_512k);
    periph.timer_0_a3.ta0ctl().modify(|_, w| w.taclr().set_bit());    
    enter_lpm0();
    periph.timer_0_a3.ta0cctl1().modify(|r, w| unsafe { w.ccis().bits(r.ccis().bits() ^ 0b01) });
    unsafe { MEAS_CNT = periph.timer_0_a3.ta0ccr1().read().bits() as i32 };
    wdt.pause();
    periph.capacitive_touch_io_0.captio0ctl().write(|w| unsafe { w.bits(0) });
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

