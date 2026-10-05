//! UNTESTED ON HARDWARE: nobody has run this example on a board yet. If you test it, remove this note
//! and open a pull request.
//!
//! The Smart Analog Combo SAC0 as an inverting amplifier with a gain of 2, biased by its own DAC: the voltage
//! on P1.2 comes out on P1.1 turned around 1.65 V and doubled, 1.65 V - 2 × (input - 1.65 V).
//!
//! SAC0's op-amp gets the input on OA0-, P1.2, through its internal resistor ladder, and the bias on its +
//! input from its 12-bit DAC: 2048 of 4096 steps of the 3.3 V supply, 1.65 V. It drives its output OA0O,
//! P1.1. The LaunchPad's light sensor uses SAC2, not SAC0, so its jumpers J7 to J9 can stay on.
//! (SACs are on the MSP430FR235x only: SLASEC4D 6.10.15, p. 79. Inverting mode with the DAC as the bias:
//! SLAU445I 20.2.2.4, p. 525. Gain 2: SLAU445I Table 20-2, p. 523; 1.98 to 2.02: SLASEC4D Table 5-25,
//! p. 56. The DAC's voltage: SLAU445I Table 20-3, p. 529; its reference, DVCC: SLASEC4D Table 6-31, p. 80.
//! The supply: SLAU680 2.3.1, p. 12. SAC0's pins: SLASEC4D Table 6-27, p. 79; SLASEC4D Table 6-63, p. 96.
//! The light sensor, on SAC2's OA2O, OA2- and OA2+ (P3.1 to P3.3) through J7 to J9: SLAU680 2.2.5.1, p. 11;
//! SLAU680 Figure 18, p. 26.)
//!
//! How to test (function generator, and the multimeter or the scope):
//! 1. Generator: the DC waveform, Offset 1.400 V, output load High-Z. Check the voltage with the multimeter
//!    first: 0 V to 3.3 V only (the input range: SLASEC4D Table 5-25, p. 55). Connect it to P1.2 (J1 pin
//!    10), its ground to GND (J3 pin 22).
//! 2. Flash this example.
//! 3. Expected: the multimeter on P1.1 (J3 pin 28) reads about 2.15 V. At an offset of 1.650 V it reads
//!    1.65 V, and at 1.900 V 1.15 V: the output moves twice as far as the input, the other way.
//! 4. Generator: sine, 1 kHz, 0.5 Vpp, offset 1.65 V (1.4 V to 1.9 V). The scope, channel 1 on P1.2 and
//!    channel 2 on P1.1, shows the output as a sine of 1 Vpp, from 1.15 V to 2.15 V, upside down against the
//!    input.
//! (Header pins: SLAU680 Figure 10, p. 15.)
#![no_main]
#![no_std]

use msp430_rt::entry;
use msp430_hal::{
    gpio::Batch,
    pmm::Pmm,
    sac::{BiasInput, InvertingGain, LoadTrigger, NegativeInput, PowerMode, SacConfig, VRef},
    watchdog::Wdt,
};
use panic_msp430 as _;

/// The bias: the DAC gives its reference times count / 4096 (SLAU445I Table 20-3, p. 529), half of 3.3 V
const BIAS: u16 = 2048;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();
    // Stop the watchdog (WDTHOLD = 1: SLAU445I Table 12-2, p. 366)
    let _wdt = Wdt::constrain(periph.wdt_a);

    let (pmm, _) = Pmm::new(periph.pmm, periph.sys);
    let port1 = Batch::new(periph.p1).split(&pmm);

    // OA0- on P1.2 and OA0O on P1.1, P1SELx = 11 (SLASEC4D Table 6-63, p. 96)
    let p1_2 = port1.pin2.to_alternate3();
    let p1_1 = port1.pin1.to_alternate3();

    let (dac_config, amp_config) = SacConfig::begin(periph.sac0);

    // The DAC with DVCC as its reference (DACSREF = 0: SLASEC4D Table 6-31, p. 80), loading each value as
    // it's written (DACLSEL = 00b: SLAU445I Table 20-8, p. 534)
    let mut dac = dac_config.configure(VRef::Vcc, LoadTrigger::Immediate);
    dac.set_count(BIAS);

    // Inverting mode with a gain of 2: OA0- through the resistor ladder (MSEL = 00b, NSEL = 01b), GAIN =
    // 010b, and the DAC on the + input (PSEL = 01b) (SLAU445I 20.2.2.4, p. 525; SLAU445I Table 20-2,
    // p. 523; SLASEC4D Table 6-27, p. 79)
    let _amp = amp_config
        .inverting_amplifier(
            BiasInput::Dac(&dac),
            NegativeInput::ExtPin(p1_2),
            InvertingGain::_2,
            PowerMode::HighPerformance,
        )
        .output_pin(p1_1);

    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
