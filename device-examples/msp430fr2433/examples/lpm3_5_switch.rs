//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The LPM3.5 switch, which connects the LPM3.5 domain, the RTC counter and the backup memory, to the
//! core supply. As in `lpm3_5.rs`, the device sleeps in LPM3.5 and the RTC wakes it about once a second;
//! each wake-up toggles LED1, whose state is kept in the backup memory. Before each sleep the program puts
//! the switch in manual mode, turned off and on in turn, and LED2 shows what
//! `Pmm::lpm3_5_switch_connected()` reads back: LED2 is lit whenever LED1 is off.
//!
//! In manual mode the user's guide recommends turning the switch off before LPM3.5, which `enter_lpm3_5()`
//! does, and the wake-up, a BOR, brings back automatic mode with the switch connected. With the switch off,
//! the domain takes clocks no faster than 40 kHz, so the program reads and writes the backup memory and the
//! RTC before it sets the switch, and the RTC counts VLOCLK. The VLO's frequency isn't exact, so neither is
//! the second.
//! (LPM5SM and LPM5SW: SLAU445I 2.2.7, p. 88; SLAU445I Table 2-7, p. 97. The LPM3.5 domain: SLASE59F
//! Figure 1-1, p. 3. The RTC can wake the device from LPM3.5, and "Any exit from LPMx.5 causes a BOR":
//! SLAU445I 1.4.3.2, p. 41 to p. 42. LPMx.5 wake-up, SYSRSTIV 08h: SLASE59F Table 6-9, p. 48. In LPM3.5 the
//! RTC can only count XT1CLK or VLOCLK: SLAU445I 15.2.2, p. 417. The VLO runs at 10 kHz ±50 %: SLASE59F
//! Table 6-7, p. 46. The backup memory keeps its data in LPM3.5: SLASE59F 6.10.10, p. 52. LED1 on P1.0 is
//! red and LED2 on P1.1 is green: SLAU739 Figure 18, p. 23.)
//!
//! How to test:
//! 1. Flash this example. After flashing with mspdebug, unplug the board's USB cable, wait a second, and plug
//!    it back in: the example only works after that. (Uniflash and Code Composer Studio need no replug.)
//! 2. Expected: LED1 toggles about once a second, and LED2 lights whenever LED1 is off.
#![no_main]
#![no_std]

use embedded_hal::digital::*;
use msp430_rt::entry;
use msp430fr2433::{P2, P3};
use msp430_hal::{
    bak_mem::BackupMemory,
    clock::VLOCLK_FREQ_HZ,
    gpio::Batch,
    lpm::{enter_lpm3_5, enter_lpm3_5_unchecked, SvsState},
    pmm::{Lpm3_5Switch, Pmm, ResetCause},
    rtc::{Rtc, RtcDiv},
    watchdog::Wdt,
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2433::Peripherals::take().unwrap();

    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366). A WDT in watchdog mode would keep
    // the device out of LPMx.5 (SLAU445I 1.4.3.1 step 7, p. 41).
    let wdt = Wdt::constrain(periph.wdt_a);
    // Pmm::new clears LOCKLPM5 here, before the pins are configured again. After a wake-up from LPM3.5,
    // SLAU445I 1.4.3.3 steps 1 to 4, p. 42 configures the pins and the RTC first and clears LOCKLPM5 after
    // that; this example keeps the simpler order, as lpm3_5.rs does. It also brings back automatic mode
    // (LPM5SM = 0: SLAU445I Table 2-7, p. 97).
    let (mut pmm, _) = Pmm::new(periph.pmm, periph.sys);

    // Read every reset cause, which also clears them for the next start, and note a wake-up from LPMx.5
    // (SYSRSTIV 08h: SLASE59F Table 6-9, p. 48; reading SYSRSTIV clears the highest pending flag: SLAU445I
    // 1.3.7, p. 36)
    let mut wake_up = false;
    while let Some(cause) = pmm.take_reset_cause() {
        if cause == ResetCause::Lpmx5WakeUp {
            wake_up = true;
        }
    }

    // Floating inputs draw extra current in LPMx.5, so every other pin gets its pulldown (SLAU445I 8.3.3,
    // p. 317: "It is critical that no inputs are left floating". Pulldowns, PxDIR = 0, PxREN = 1,
    // PxOUT = 0: SLAU445I Table 8-1, p. 313.)
    let p1 = Batch::new(periph.p1)
        .pulldown_all()
        .config_pin0(|p| p.to_output())
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut led1 = p1.pin0;
    let mut led2 = p1.pin1;
    init_unused_gpio(periph.p2, periph.p3, &pmm);

    if wake_up {
        // Show LED1's state from before the sleep, which the backup memory kept, and store the other one
        // for the next wake-up (in LPMx.5 "The register content of all modules and the CPU is lost":
        // SLAU445I 1.4.3, p. 40)
        let bak_mem = BackupMemory::as_u8s(periph.bakmem);
        let led1_on = bak_mem[0] == 1;
        led1.set_state(led1_on.into()).ok();
        bak_mem[0] = if led1_on { 0 } else { 1 };

        // Clear the RTC interrupt flag ("Reading RTCIV register clears the interrupt flag": SLAU445I
        // 15.2.4, p. 418)
        periph.rtc.rtciv().read();

        // The barrier keeps the compiler from moving the write to the backup memory after `set_switch()`,
        // which may turn the switch off
        msp430::asm::barrier();
        set_switch(&mut pmm, &mut led2, led1_on);
        // Enter LPM3.5 again. Like lpm3_5.rs, this relies on the RTC keeping the settings of the first run
        // through LPM3.5. SVSHE = 0 turns the high-side SVS off in LPM3.5 (SLAU445I Table 2-2, p. 91;
        // SLAU445I 1.4.3.1 step 9c, p. 41).
        unsafe { enter_lpm3_5_unchecked(wdt, SvsState::Disabled) };
    } else {
        // A fresh start. The RTC counts VLOCLK (RTCSS = 11b), divided by 1 (RTCPS = 000b), and requests its
        // interrupt (RTCIE) at each overflow, after VLOCLK_FREQ_HZ + 1 counts, about a second (SLAU445I
        // Table 15-2, p. 420; SLAU445I 15.2.1, p. 417)
        let mut rtc = Rtc::new(periph.rtc).use_vloclk();
        rtc.set_clk_div(RtcDiv::_1);
        rtc.start(VLOCLK_FREQ_HZ);
        rtc.enable_interrupts();

        set_switch(&mut pmm, &mut led2, false);
        // Interrupts were never enabled, so `enter_lpm3_5()` enters LPM3.5 with GIE clear, as SLAU445I
        // 1.4.3.1 step 8, p. 41 does. The RTC event still wakes the device (SLAU445I 1.4.3.2, p. 41).
        enter_lpm3_5(wdt, rtc, SvsState::Disabled);
    }
}

/// Put the LPM3.5 switch in manual mode, disconnected if LED1 is on and connected if it is off, and show
/// on LED2 whether it reads back as connected (LPM5SM = 1, LPM5SW: SLAU445I Table 2-7, p. 97)
fn set_switch(pmm: &mut Pmm, led2: &mut impl OutputPin, led1_on: bool) {
    pmm.set_lpm3_5_switch(if led1_on { Lpm3_5Switch::Disconnected } else { Lpm3_5Switch::Connected });
    led2.set_state(pmm.lpm3_5_switch_connected().into()).ok();
}

/// Enable pulldowns on the unused ports (SLAU445I 8.3.2, p. 317)
fn init_unused_gpio(p2: P2, p3: P3, pmm: &Pmm) {
    Batch::new(p2).pulldown_all().split(pmm);
    Batch::new(p3).pulldown_all().split(pmm);
}

// Note: In this case we don't need an ISR when waking from LPMx.5, since power on disables interrupts
// and this program never enables them; the RTC interrupt flag is cleared before LPM3.5 is entered again.
// (A BOR resets the SR, GIE included: SLAU445I 1.2.1, p. 32. The wake-up interrupt is serviced once
// interrupts are enabled: SLAU445I 1.4.3.3 step 7, p. 42.)

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
