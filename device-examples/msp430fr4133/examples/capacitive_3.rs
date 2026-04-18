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
use msp430fr413x::{
    interrupt,
    real_time_clock::rtcctl::Rtcss,
    timer_0_a3::{ta0cctl1::Ccis, ta0ctl::Mc},
    watchdog_timer::wdtctl::Wdtssel,
    Timer0A3, UsciA0UartMode, P4,
};
use nb::block;
use panic_msp430 as _;

const KEY_LVL: i32 = 10000;

static mut BASE_CNT: i32 = 0;
static mut MEAS_CNT: i32 = 0;

static LED: Mutex<UnsafeCell<Option<Pin<P4, Pin0, Output>>>> = Mutex::new(UnsafeCell::new(None));

static TX: Mutex<UnsafeCell<Option<Tx<UsciA0UartMode>>>> = Mutex::new(UnsafeCell::new(None));

// Red onboard LED should blink at a steady period.
#[entry]
fn main() -> ! {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };
    let mut fram = Fram::new(periph.fram);
    let mut wdt = Wdt::constrain(periph.watchdog_timer).to_interval();

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);

    let (smclk, aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    let mut p1 = Batch::new(periph.p1)
        .config_pin3(|p| p.to_output())
        .split(&pmm);

    let p2 = Batch::new(periph.p2).split(&pmm);

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

    // wdt.set_aclk(&aclk);

    p1.pin3.set_low();
    p4.pin0.set_high();

    // with(|cs| unsafe { *LED.borrow(cs).get() = Some(p4.pin0) });

    // periph.capacitive_touch_io_0.captio0ctl().read().bits();

    // 1. Counting hardware
    let capacitive = CapacitiveParts3::new(unsafe { Timer0A3::steal() });

    // let el5 = CapacitiveElement::new::<_, 100, 500>(p1.pin6);
    let el6 = CapacitiveElement::new::<_, 100, 755>(p1.pin7.to_output());
    // let el7 = CapacitiveElement::new::<_, 100, 500>(p2.pin5);

    // FIX: Bind the WDT gate to a variable too

    // let mut s3 = CapacitiveSensor::new_fro(
    //     &capacitive,
    //     [el5],
    //     &sw_gate, // Use the named variable
    //     &periph.capacitive_touch_io_0,
    //     1000,
    //     TrackingRate::Fast,
    //     DriftRate::Slow,
    //     DirectionOfInterest::Increment,
    //     Button::new(),
    // );

    let timer_gate = RtcGate::new(unsafe { msp430fr413x::RealTimeClock::steal() });

    let mut s4 = CapacitiveSensor::new_ro(
        &capacitive,
        [el6],
        &timer_gate, // Use the named variable
        &periph.capacitive_touch_io_0,
        (Rtcss::Smclk, RtcDiv::_1, 1024),
        TrackingRate::Fast,
        DriftRate::Slow,
        DirectionOfInterest::Increment,
        Button::new(),
    );

    // let rtc_gate = RtcGate::new(unsafe { msp430fr413x::RealTimeClock::steal() });

    // let mut s5 = CapacitiveSensor::new_ro(
    //     &capacitive,
    //     [el7],
    //     &rtc_gate, // Use the named variable
    //     &periph.capacitive_touch_io_0,
    //     (Rtcss::Smclk, RtcDiv::_10, 1024),
    //     TrackingRate::Fast,
    //     DriftRate::Slow,
    //     DirectionOfInterest::Increment,
    //     Button::new(),
    // );

    // --- Now the group works ---
    // let mut group = CapacitiveGroup::new([
    //     &mut s3
    // ]);

    // s3.init();
    // s3.update(5);

    s4.init();
    s4.update(5);

    // s5.init();
    // s5.update(5);

    let mut last_button: Option<i16> = None;

    let mut any = false;

    embedded_io::Write::write_all(&mut tx, b"----------------------\r\n").ok();

    with(|cs| unsafe {
        *TX.borrow(cs).get() = Some(tx);
    });

    loop {
        // let button = s3.sensor();

        // if let Some(b) = button {
        //     any = true;

        //     // if button != last_button {
        //     //     match b {
        //     //         0 => {
        //     //             embedded_io::Write::write_all(&mut tx, b"BUTTON 0\r\n").ok();
        //     //         }
        //     //         1 => {
        //     //             embedded_io::Write::write_all(&mut tx, b"BUTTON 1\r\n").ok();
        //     //         }
        //     //         _ => {}
        //     //     }
        //     // }
        // }
        // last_button = button;

        let button = s4.sensor();

        if let Some(b) = button {
            // embedded_io::Write::write_all(&mut tx, b"raw=").ok();
            // print_num(&mut tx, b as i32);
            // write(&mut tx, '\r');
            // write(&mut tx, '\n');
            any = true;
            // embedded_io::Write::write_all(&mut tx, b"S4\r\n").ok();
        }

        // let button = s5.sensor();

        // if let Some(b) = button {
        //     any = true;
        //     // embedded_io::Write::write_all(&mut tx, b"S5\r\n").ok();
        // }

        if any {
            p1.pin3.set_high();
        } else {
            p1.pin3.set_low();
        }

        p4.pin0.toggle();

        any = false;

        delay.delay_ms(100);
    }
}

#[interrupt(wake_cpu)]
fn TIMER1_A0() {}

#[interrupt(wake_cpu)]
fn TIMER0_A0() {}

#[interrupt(wake_cpu)]
fn WDT() {
    let periph = unsafe { msp430fr413x::Peripherals::steal() };

    if periph.real_time_clock.rtciv().read().bits() == 0x02 {
        nop();
    }
}

#[inline(always)]
unsafe fn rtc_check() -> u16 {
    let periph = msp430fr413x::Peripherals::steal();

    if periph.real_time_clock.rtciv().read().bits() == 0x02 {
        1
    } else {
        0
    }
}

#[interrupt(wake_cpu)]
unsafe fn RTC() {
    let periph = msp430fr413x::Peripherals::steal();

    if periph.real_time_clock.rtciv().read().bits() == 0x02 {
    }
}

// #[export_name = "RTC"]
// #[unsafe(naked)]
// unsafe extern "msp430-interrupt" fn rtc_handler() {
//     core::arch::naked_asm!(
//         // Call Rust helper FIRST (allowed only if it does NOT return normally)
//         "call #{inner}",

//         // If condition true → clear bits
//         "cmp #0, r12",
//         "jeq 1f",
//         "bic.b #0xF0, 0(r1)",

//         "1:",
//         "reti",
//         inner = sym rtc_check,
//     );
// }

// #[export_name = "RTC"]
// #[unsafe(naked)]
// unsafe extern "msp430-interrupt" fn ot1o618qmf07394p() {
//     #[inline(always)]
//     extern "msp430-interrupt" fn ot1o618qmf07394p<'a>() {
//         interrupt::RTC;
//         let periph = unsafe { msp430fr413x::Peripherals::steal() };
//         if periph.real_time_clock.rtciv().read().bits() == 0x02 {

//         }
//     }
//     {
//         // Clear SCG1, SCG0, OSC_OFF, CPU_OFF in saved copy of SR register on stack
//         const MASK: u8 = (1 << 7) + (1 << 6) + (1 << 5) + (1 << 4);
//         core::arch::naked_asm!(
//             "bic.b #{mask}, 0(r1)",
//             "jmp {inner}",
//             inner = sym ot1o618qmf07394p,
//             mask = const MASK
//         );
//     }
// }

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
        1_000_000_000,
        100_000_000,
        10_000_000,
        1_000_000,
        100_000,
        10_000,
        1_000,
        100,
        10,
        1,
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

// #[interrupt(wake_cpu)]
// fn TIMER0_A1() {
//     let periph = unsafe { msp430fr413x::Peripherals::steal() };

//     periph
//         .timer_0_a3
//         .ta0ctl()
//         .modify(|_, w| w.mc().set(Mc::Mc0 as u8));

//     unsafe {
//         periph
//             .timer_0_a3
//             .ta0cctl0()
//             .clear_bits(|w| w.ccie().clear_bit());
//     }
// }
