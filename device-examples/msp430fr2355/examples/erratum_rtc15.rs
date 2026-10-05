//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! A test of erratum RTC15: the RTC hangs when it is moved off XT1CLK while XT1 is stopped. The RTC counts
//! XT1, which runs in bypass mode from a function generator, and LED1 toggles at each RTC overflow. Each
//! time you switch the generator off, the program moves the RTC to the VLO, as a fault handler would, and
//! the backchannel UART reports whether the RTC still counts.
//!
//! The erratum: "if the RTC Counter clock source is changed by user software (e.g. in the clock fault
//! handling ISR) from XT1CLK to a different clock source while XT1CLK is stopped the RTC Counter hangs",
//! until a reset from the RST pin, a power cycle, or XT1 running again (SLAZ695J RTC15, p. 11).
//! `Rtc::start()` applies its workaround when it moves the RTC off XT1CLK while XT1 is stopped: "Change the
//! RTC Counter clock source away from XT1CLK normally", then "Reconfigure the XIN pin as a GPIO output, then
//! toggle the GPIO twice with at least 2 rising or falling edges" (SLAZ695J RTC15, p. 11; see the HAL's
//! `rtc` module). Here it toggles XIN 4 times and gives the pin back to XT1. XT1 counts as stopped when its
//! fault flag XT1OFFG is set and comes back when cleared: the flag stays set after a fault has ended, until
//! it is cleared (SLAU445I 3.2.13, p. 109).
//!
//! When the signal stops, XT1OFFG is set, and the RTC stops too: the fail-safe doesn't cover the RTC's
//! XT1CLK input. Once the RTC count has stood still for 100 ms, the program moves the RTC to the VLO and
//! waits up to 2 s for an overflow. Then it waits for XT1 to run again, and moves the RTC back to XT1CLK.
//! The board's crystal on XIN could keep XT1 going, or give it edges that free a hung RTC, so the test needs
//! the crystal off and the generator alone on XIN.
//! (XT1 bypass mode: SLAU445I 3.2.4, p. 103. XIN is P2.7 and XOUT P2.6, with P2SEL = 10: SLASEC4D
//! Table 6-64, p. 98. The RTC counts XT1CLK with RTCSS = 10b and VLOCLK with 11b: SLASEC4D Table 6-10,
//! p. 68; SLASEC4D Table 6-9, p. 68. SLAU445I 3.2.13, p. 109 to p. 110 describes the switch to REFO for
//! MCLK, SMCLK, ACLK and the FLL reference only. XT1OFFG: SLAU445I Table 3-11, p. 122. VLO: 10 kHz typical,
//! SLASEC4D Table 5-8, p. 40; ±50 %, SLASEC4D Table 6-9, p. 68. LED1 on P1.0 is red: SLAU680 Figure 18,
//! p. 26.)
//!
//! How to test (soldering iron, multimeter, function generator, and the scope to check its signal):
//! 1. Take the crystal off: Q1 is soldered straight to XIN, P2.7, and XOUT, P2.6, with only its load
//!    capacitors C3 and C2 (12 pF) to GND, no resistor or jumper between, so only desoldering Q1
//!    disconnects it (SLAU680 2.5, p. 13; SLAU680 Figure 18, p. 26). P2.7 isn't on a header (SLAU680
//!    Figure 10, p. 15), so solder a wire to the pad of Q1's pin 1, the XIN side, which C3 also connects
//!    to, and check with the multimeter's continuity test that it reaches P2.7, pin 8 of the chip. Solder
//!    Q1 back afterwards: the xt1_ examples need it.
//! 2. Generator: square wave, 32.768 kHz, duty cycle 50 %, 0 V to 3.3 V (3.3 Vpp, 1.65 V offset), output
//!    load High-Z. Check the levels, and the frequency's unit (kHz, not Hz), on the scope before
//!    connecting: a negative or >3.6 V signal can damage the pin.
//! 3. Connect it to the wire on XIN, its ground to GND (J2 pin 20), and switch the output on. Do this
//!    before flashing: `freeze()` waits for XT1.
//! 4. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU680 2.2.4, p. 11). Expected: `RTC on XT1CLK. Switch the generator output
//!    off.`, and LED1 toggles every 0.25 s.
//! 5. Switch the generator output off. Pass: `The RTC counts VLOCLK. Switch the generator output on.`, and
//!    LED1 keeps toggling, every 0.25 s or so (the VLO is only accurate to ±50 %). Fail: `The RTC hangs.
//!    Switch the generator output on.`, and LED1 stops.
//! 6. Switch the output on: `RTC on XT1CLK. Switch the generator output off.` again. Repeat steps 5 and 6
//!    five times or more: every time the RTC must count VLOCLK. It takes a minute.
//! 7. To check that the test can catch the erratum, set `USE_HAL_WORKAROUND` to false and flash again: the
//!    RTC then moves to the VLO without the XIN toggles, and should hang at every switch.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, Xt1Config},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    rtc::{Rtc, RtcDiv},
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

/// `false` moves the RTC to the VLO without the HAL's workaround, to check that this test can catch the hang
const USE_HAL_WORKAROUND: bool = true;

/// Frequency the function generator is set to
const XT1_FREQ_HZ: u32 = 32_768;
/// RTC counts per toggle of LED1: 0.25 s of XT1CLK, and about 0.25 s of the VLO's typical 10 kHz
const XT1_TICKS: u16 = 8192;
const VLO_TICKS: u16 = 2500;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p4 = Batch::new(periph.p4).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let xin = p2.pin7.to_alternate2();

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range (SELMS = 000b: SLAU445I Table 3-8, p. 117; DIVM, DIVS:
    // SLAU445I Table 3-9, p. 118), and ACLK from XT1CLK (SELA = 00b: SLAU445I Table 3-8, p. 117), which keeps
    // XT1 on (SLAU445I 3.2.4, p. 103); XT1 in bypass mode (XT1BYPASS = 1: SLAU445I Table 3-10, p. 120)
    let (smclk, _aclk, mut xt1clk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .xt1clk_on(Xt1Config::bypass(XT1_FREQ_HZ, xin))
        .aclk_xt1clk()
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

    // The RTC counts XT1CLK undivided (RTCSS = 10b: SLASEC4D Table 6-10, p. 68; RTCPS = 000b: SLAU445I
    // Table 15-2, p. 420), and overflows every XT1_TICKS cycles (SLAU445I 15.2.1, p. 417)
    let mut rtc = Rtc::new(periph.rtc).use_xt1clk(&xt1clk);
    rtc.set_clk_div(RtcDiv::_1);
    loop {
        rtc.start(XT1_TICKS - 1);
        tx.write_all(b"RTC on XT1CLK. Switch the generator output off.\r\n").ok();

        // Wait for XT1 to stop: its fault flag is set, and the RTC count stands still for 100 ms
        loop {
            if rtc.wait().is_ok() {
                led1.toggle().ok();
            }
            if xt1clk.is_faulted() {
                let count = rtc.get_count();
                delay.delay_ms(100);
                if rtc.get_count() == count {
                    break;
                }
            }
        }

        // Move the RTC to the VLO (RTCSS = 11b: SLASEC4D Table 6-9, p. 68)
        let mut rtc_vlo = rtc.use_vloclk();
        if USE_HAL_WORKAROUND {
            rtc_vlo.start(VLO_TICKS - 1);
        } else {
            // The same steps as `Rtc::start()`, without the XIN toggles: RTCMOD, RTCSS, the counter reset
            // RTCSR, and a read of RTCIV to clear an old overflow (SLAU445I Table 15-2, p. 420; SLAU445I
            // Table 15-4, p. 422; SLAU445I 15.2.4, p. 418)
            let regs = unsafe { &*msp430fr2355::Rtc::ptr() };
            regs.rtcmod().write(|w| unsafe { w.bits(VLO_TICKS - 1) });
            regs.rtcctl().modify(|_, w| w.rtcss().vloclk());
            regs.rtcctl().modify(|_, w| w.rtcsr().set_bit());
            regs.rtciv().read();
        }

        // An overflow must come within 2 s: VLO_TICKS take 0.5 s at the VLO's slowest, 5 kHz
        let mut counts = false;
        for _ in 0..2000 {
            delay.delay_ms(1);
            if rtc_vlo.wait().is_ok() {
                counts = true;
                break;
            }
        }
        if counts {
            tx.write_all(b"The RTC counts VLOCLK. Switch the generator output on.\r\n").ok();
        } else {
            tx.write_all(b"The RTC hangs. Switch the generator output on.\r\n").ok();
        }

        // Wait for XT1 to run again: then the fault flag stays clear when it's cleared (SLAU445I 3.2.13,
        // p. 109)
        loop {
            if rtc_vlo.wait().is_ok() {
                led1.toggle().ok();
            }
            xt1clk.clear_fault();
            if !xt1clk.is_faulted() {
                break;
            }
        }
        rtc = rtc_vlo.use_xt1clk(&xt1clk);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
