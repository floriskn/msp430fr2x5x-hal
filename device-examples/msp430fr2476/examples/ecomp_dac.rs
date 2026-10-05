//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The comparator eCOMP0 against its own 6-bit DAC, with the DAC's two buffers: the voltage on P2.2 is
//! compared with a threshold of 2.01 V or 0.98 V, picked by software or by the comparator's output. LED1
//! shows the output, the backchannel UART prints it with the mode, and each press of S1 moves to the next
//! mode:
//! 1. Software mode, buffer 1 (`new_sw_dac()`): the threshold is buffer 1, 39/64 of 3.3 V, 2.01 V.
//! 2. Software mode, buffer 2 (`select_buffer()`): 19/64 of 3.3 V, 0.98 V.
//! 3. Hardware mode (`into_hw_buffer_mode()`): the output picks the buffer, buffer 1 while it's low and
//!    buffer 2 while it's high. So it rises as P2.2 passes 2.01 V, and falls only once P2.2 is below 0.98 V.
//! The next press goes back to the first mode (`into_sw_buffer_mode()`).
//!
//! The output also goes to its pin, P3.4 (`with_output_pin()`), which the LaunchPad doesn't connect to
//! anything. In the software modes the output can switch back and forth while the input is close to the
//! threshold, as a comparator's output does with a small difference at its inputs; in hardware mode the
//! threshold moves away as soon as the output switches.
//! (eCOMP0's inputs, COMP0.1 on P2.2 and the DAC: SLASEO7C Table 9-21, p. 63. The output pin: SLASEO7C
//! Table 9-22, p. 63; SLAU802 Figure 18, p. 24. The output is high while V+ is higher than V-: SLAU445I
//! 18.2.1, p. 505. Switching back and forth: SLAU445I 18.2.3, p. 505. The buffers: SLAU445I 18.2.4, p. 506;
//! CPDACBUFS and CPDACSW: SLAU445I Table 18-6, p. 512; the buffer the output picks: SLAU445I Figure 18-4,
//! p. 507. The DAC's voltage: SLAU445I Table 18-7, p. 513. LED1 on P1.0 is green, and S1 is P4.0: SLAU802
//! Figure 19, p. 25.)
//!
//! How to test (function generator):
//! 1. Generator: the DC waveform, Offset 1.500 V, output load High-Z. Check the voltage with the multimeter
//!    first: 0 V to 3.3 V only (the input range: SLASEO7C 8.12.9.1, p. 42). Connect it to P2.2 (J1 pin 5),
//!    its ground to GND (J3 pin 22).
//! 2. Flash this example, with the TXD jumper of J101 on, and open the COM port of "MSP Application
//!    UART1" at 9600 baud (SLAU802 2.2.4, p. 9).
//! 3. Expected, twice a second: `software mode, buffer 1, 2011 mV: output low`, and LED1 off, as 1.5 V is
//!    below 2.01 V. Set the offset to 2.500 V: `output high`, LED1 on. Back to 1.500 V.
//! 4. Press S1: `software mode, buffer 2, 980 mV: output high`, LED1 on. At 0.500 V: low.
//! 5. Press S1, with the offset at 1.500 V: `hardware mode, rising at 2011 mV, falling at 980 mV: output
//!    high`. Set the offset to 0.500 V: low. Back to 1.500 V: still low. 2.500 V: high. 1.500 V: still high.
//! 6. Press S1: back to step 3.
//! 7. Generator: the Ramp waveform, Symmetry 50 % (a triangle), 0.2 Hz, 2 Vpp, offset 1.5 V (0.5 V to
//!    2.5 V). In hardware mode LED1 turns on as the input rises past 2.01 V and off as it falls past 0.98 V,
//!    so it's on longer than in either software mode.
//! (Header pins: SLAU802 Figure 10, p. 13.)
#![no_main]
#![no_std]

use embedded_hal::{delay::DelayNs, digital::*};
use embedded_io::Write;
use msp430_rt::entry;
use msp430_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    delay::SysDelay,
    ecomp::{
        BufferSel, Comparator, DacVRef, ECompConfig, FilterStrength, Hysteresis, NegativeInput,
        OutputPolarity, PositiveInput, PowerMode,
    },
    fram::Fram,
    gpio::Batch,
    pin_mapping::DefaultMapping,
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use msp430fr247x::EComp0;
use panic_msp430 as _;

/// The DAC counts of the two thresholds. The DAC gives its reference, here DVCC, times count / 64 (SLAU445I
/// Table 18-7, p. 513), and the LaunchPad supplies 3.3 V (SLAU802 2.3.1, p. 10).
const UPPER: u8 = 39; // 2011 mV
const LOWER: u8 = 19; // 980 mV

#[entry]
fn main() -> ! {
    let periph = msp430fr247x::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.frctl);
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let p1 = Batch::new(periph.p1).split(&pmm);
    let p2 = Batch::new(periph.p2).split(&pmm);
    let p3 = Batch::new(periph.p3).split(&pmm);
    // S1 on P4.0 is an input with its pull-up (PxDIR = 0, PxREN = 1, PxOUT = 1: SLAU445I Table 8-1, p. 313)
    let p4 = Batch::new(periph.p4).config_pin0(|p| p.pullup()).split(&pmm);
    let mut led1 = p1.pin0.to_output_low();
    let mut s1 = p4.pin0;

    // MCLK = SMCLK = DCOCLKDIV in the 1 MHz range and ACLK from REFO (SELMS = 000b, SELA = 01b:
    // SLAU445I Table 3-8, p. 117; DIVM, DIVS: SLAU445I Table 3-9, p. 118)
    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.cs)
        .mclk_dcoclk(DcoclkFreqSel::_1MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk()
        .freeze(&mut fram);

    // The backchannel UART: eUSCI_A0's TXD on P1.4, P1SEL = 01, 8N1 (SLAU802 2.2.4, p. 9; SLASEO7C
    // Table 9-23, p. 65; SLAU445I Table 22-8, p. 593)
    let mut tx = SerialConfig::<_, _, DefaultMapping>::new(
        periph.e_usci_a0,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::OneStopBit,
        Parity::NoParity,
        Loopback::NoLoop,
        9600,
    )
    .use_smclk(&smclk)
    .tx_only(p1.pin4.to_alternate1());

    // The DAC, with DVCC as its reference (CPDACREFS = 0), in software mode with buffer 1 selected
    // (CPDACBUFS = 1, CPDACSW = 0) (SLAU445I Table 18-6, p. 512)
    let (dac_config, comparator_config) = ECompConfig::begin(periph.e_comp0);
    let mut dac = dac_config.new_sw_dac(DacVRef::Vcc, BufferSel::_1);
    dac.write_buffer_1(UPPER);
    dac.write_buffer_2(LOWER);

    // V+ is COMP0.1 on P2.2, with P2SEL = 11 (SLASEO7C Table 9-24, p. 66), and V- the DAC, so the output is
    // high while P2.2 is above the DAC's voltage. High-speed mode, no hysteresis and no filter (CPMSEL,
    // CPHSEL, CPFLT: SLAU445I Table 18-3, p. 510). The output pin is P3.4 with P3SEL = 10 and P3DIR = 1
    // (SLASEO7C Table 9-25, p. 67).
    let mut comparator = comparator_config
        .configure(
            PositiveInput::COMPx_1(p2.pin2.to_alternate3()),
            NegativeInput::Dac(&dac),
            OutputPolarity::Noninverted,
            PowerMode::HighSpeed,
            Hysteresis::Off,
            FilterStrength::Off,
        )
        .with_output_pin(p3.pin4.to_output().to_alternate2());

    loop {
        show("software mode, buffer 1, 2011 mV", &mut comparator, &mut led1, &mut s1, &mut tx, &mut delay);
        // CPDACSW = 1
        dac.select_buffer(BufferSel::_2);
        show("software mode, buffer 2, 980 mV", &mut comparator, &mut led1, &mut s1, &mut tx, &mut delay);
        // CPDACBUFS = 0: the output picks the buffer
        let hw_dac = dac.into_hw_buffer_mode();
        show(
            "hardware mode, rising at 2011 mV, falling at 980 mV",
            &mut comparator,
            &mut led1,
            &mut s1,
            &mut tx,
            &mut delay,
        );
        // CPDACBUFS = 1, and buffer 1 again
        dac = hw_dac.into_sw_buffer_mode();
        dac.select_buffer(BufferSel::_1);
    }
}

/// Copy the comparator's output, CPOUT (SLAU445I Table 18-3, p. 511), to LED1, and print it with `mode` twice
/// a second, until S1 is pressed and released. The 20 ms waits after the press and after the release keep a
/// bouncing contact from counting as more than one press.
fn show(
    mode: &str,
    comparator: &mut Comparator<EComp0>,
    led1: &mut impl OutputPin,
    s1: &mut impl InputPin,
    tx: &mut impl Write,
    delay: &mut SysDelay,
) {
    let mut ticks: u8 = 0;
    loop {
        let high = comparator.value();
        led1.set_state(high.into()).ok();
        if ticks == 0 {
            writeln!(tx, "{}: output {}\r", mode, if high { "high" } else { "low" }).ok();
        }
        ticks = (ticks + 1) % 50;

        if s1.is_low().unwrap() {
            delay.delay_ms(20);
            while s1.is_low().unwrap() {}
            delay.delay_ms(20);
            return;
        }
        delay.delay_ms(10);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
