# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
- Add XT1 support, in crystal and bypass mode, through `Xt1Config` and `ClockConfig::xt1clk_on()`. XT1 can source ACLK, MCLK and SMCLK, and serve as the FLL reference. `Xt1Config::crystal()` and `bypass()` set up a 32 kHz XT1; on the MSP430FR2x5x, `crystal_hf()` and `bypass_hf()` set up a 1 MHz to 24 MHz XT1, typed as `HighFrequency`. The XT1 pins of each device are available as `clock::Xt1Xin` and `clock::Xt1Xout`.
- Add `ClockConfig::try_freeze()`, which gives up on XT1 after a timeout instead of blocking, and `ClockConfig::xt1clk_off()` to fall back to the internal oscillators.
- Add `Xt1clk::is_faulted()` and `Xt1clk::clear_fault()`. XT1 faults switch the clocks it sources to a fallback oscillator until the fault flags are cleared.
- Add `Xt1clk::enable_fault_interrupt()` and `Xt1clk::disable_fault_interrupt()`, which report oscillator faults through the user NMI, and `clock::take_fault_interrupt()` to handle it in the `UNMI` interrupt handler.
- Add `ClockConfig::refo_low_power()` on the MSP430FR2x5x (enhanced clock system).
- Add `TimerConfig::vloclk()`, for the timers that can be clocked from the VLO: TA0 and TA2 on the MSP430FR247x, and TA0 on the MSP430FR25x2.
- Add timer cascading: `TimerConfig::cascade()` clocks a timer from the CCR2 output of another timer, set up with `SubTimer::into_cascade_output()` or `PwmUninit::into_cascade_output()`, so that it counts the periods of that timer. Available for TA1 (from TA0) and TA3 (from TA2) on the MSP430FR247x, TA1 (from TA0) on the MSP430FR25x2, and TB1 (from TB0) on the MSP430FR2x5x.
- Add `TimerParts2` for timers with two capture/compare registers: TA2 and TA3 on the MSP430FR2433, which couldn't be used before. They aren't connected to any pins, so they work as timers only.
- The DCO is now trimmed in software for every frequency except the device's highest, as the user's guide recommends, so the FLL locks reliably.
- The 8 MHz and 16 MHz DCO settings now run at 7.995 MHz and 15.991 MHz. They previously ran slightly above 8 MHz and 16 MHz, which needed an extra FRAM wait state and, at 16 MHz, exceeded the maximum frequency of most devices.
- FRAM wait states now also cover MCLK while the DCO is being configured, which runs undivided before the MCLK divider is applied.
- The RTC can now be clocked from ACLK and XT1CLK. `Rtc::start()` now resets the counter after selecting the clock, as the user's guide recommends.
- `enter_lpm3_5()` now accepts an RTC clocked from XT1CLK. After a wake-up from LPM3.5, `Pmm::new_locked()` and `Pmm::unlock_lpm5()` allow XT1 to be reconfigured before the pins are unlocked, so it keeps clocking the RTC.
- PWM pins no longer specify their alternate function separately: it is derived from the pin type.
- Breaking: `ClockConfig::aclk_vloclk()` is no longer available on the MSP430FR25x2, which can't source ACLK from VLO.
- Fixed XT1 pins on the MSP430FR2x5x, which need alternate function 2.
- Fixed LPM3.5 entry stopping XT1 on devices other than the MSP430FR2x5x, and not resetting `P2SEL1`. LPMx.5 entry now also clears ACLKREQEN, as the user's guide requires.
- Fixed `delay_ns()` and `delay_us()` never waiting longer than 1 ms.
- Fixed the TB0 clock input pin (TB0CLK) on the MSP430FR247x, which is P6.1, not P2.7.
- Fixed `InfoMemory::write()` and `into_unprotected()` switching off the program FRAM write protection (PFWP) until the next reset. `write()` now runs with interrupts disabled, as the user's guide recommends.
- Fixed `Spi::change_mode()` disabling the SPI interrupts.
- Fixed pin remapping on the MSP430FR247x and MSP430FR25x2 resetting the rest of SYSCFG2/SYSCFG3: setting up eUSCI_B0 switched an RTC on ACLK to SMCLK (and cleared the ADC input enables on the MSP430FR25x2), and the eUSCI_A0, eUSCI_B1, TA2 and TA3 remaps undid each other.
- Breaking: the MSP430FR25x2 ADC is 10-bit with a 1.5 V reference only, so `Resolution::_12BIT` and `ReferenceVoltage::_2V0`/`_2V5` are no longer available there. Its analog inputs are now enabled with `to_adc_mode()` (SYSCFG2.ADCPCTLx), as on the MSP430FR2433; `Alternate3` selected CapTIvate.
- Fixed on the MSP430FR25x2: 256 bytes of information memory, remapped eUSCI_B0 pins on alternate function 2, and pin functions as in the data sheet (CAP1.x only on the MSP430FR2522).
- Fixed on the MSP430FR2x5x: eCOMP input pins on alternate function 3, pin directions as in the data sheet, and the SAC op-amp pins on P3 only on the MSP430FR235x.
- Fixed on the MSP430FR247x: PWM on TA2/TA3 with `RemappedMapping` uses the remapped pins (`PwmPeriph`, `PwmUninit` and `Pwm` take the pin mapping), the TB0 CCR6 capture input is P4.4, and the TA3 CCR2 PWM pin P3.7 uses alternate function 1.
- Fixed on the MSP430FR2433: TA1 capture inputs on P1.5/P1.4, ACLK output on P2.2 on alternate function 2, and the eUSCI_A1 SPI STE pin (P3.1) added.
- Fixed the MODCLK frequency constants, now the data sheets' typical values: 4.8 MHz on the MSP430FR2433 and MSP430FR25x2, 3.8 MHz on the MSP430FR247x.
- Add `Capture::interrupt_capture()` for CCR0, whose capture flag is cleared when its own interrupt is serviced. Captures arriving while a capture is read are now reported as overcaptures instead of being lost.
- GPIO batches now turn pin interrupts off while reconfiguring a port, and switch pins whose two function select bits both change through PxSELC. Pins a device doesn't have start out `Unavailable` and can't be used.
- Add `to_output_low()` and `to_output_high()` for GPIO pins, since PxOUT is undefined after a reset.
- PWM `max_duty_cycle()` and `get_max_duty()` now return the period (CCR0 + 1), so the maximum duty cycle is 100 %.
- `Timer::count()` takes the median of three reads, for timers clocked asynchronously to MCLK.
- The PMM is unlocked and locked again around each register write, and `enable_internal_reference()` waits until the reference has settled.
- ADC: `read_count()` no longer returns the result of a pending conversion of another channel, and `count_to_mv()` scales by the full-scale count (2^n - 1).
- UART: the eUSCI is configured while held in reset, and a baud rate above a third of the clock panics instead of being clamped.
- I2C: clock divisors below the user's guide minimum (4, or 8 with several masters) panic, and `zero_byte_write()` sets START and STOP together. Breaking: `send_nack()` is only available in slave roles.
- Breaking: `Wdt::wait()` is only available in interval mode.
- `WdtClkPeriods` and `SvsState` are HAL enums with the same variants on every device. `WdtClkPeriods::_2048m` and `SvsState::Svshe0`/`Svshe1` remain as aliases.
- Add `enter_lpm0_with_interrupts()`, `request_lpm3_with_interrupts()` and `request_lpm4_with_interrupts()`, which enable interrupts in the same instruction that starts the sleep. Entering a low-power mode is now a compiler barrier.
- Breaking: the SAC amplifier modes only accept the inputs the user's guide supports (SLAU445I Table 20-1). `PositiveInput`, for the open-loop and non-inverting modes, no longer has a `Dac` variant; the inverting amplifier takes a `BiasInput` (OA+ or the DAC) and the buffer a `BufferInput` (OA+, the DAC or the paired amplifier).
- Add `DacConfig::configure_with_interrupts()` and `Dac::data_loaded()`, for the SAC DAC interrupt that requests new data after a timer-triggered load.
- Add `Pwm::duty()`, `Pwm::enable()` and `Pwm::disable()`, which were only available through embedded-hal 0.2.
- The example projects now link `libmul_f5`, so multiplication uses the hardware multiplier (MPY32): 16-bit products are about 5 times and 32/64-bit products about 10 times faster than with `libmul_none`. The commented-out `libmul_32` used the register addresses of other devices, which are PM5CTL0 here.
- Add the `sys` module. `SysParts` gives the RST/NMI pin (reset or NMI mode, pull resistor, reset filter, NMI edge and interrupt), the vacant memory access interrupt and the JTAG mailbox (16 and 32-bit transfers). `sys::take_nmi_pin_interrupt()` and `sys::take_system_nmi()` serve the `UNMI` and `SYSNMI` interrupt handlers.
- Add `Pmm::take_reset_cause()`, which reports why the device reset, `Pmm::software_bor()` and `Pmm::software_por()`, and `Pmm::set_svsh()` to turn the high-side supply voltage supervisor off in LPM2 to LPM4.
- Add FRAM bit error handling: `Fram::set_uncorrectable_bit_error_action()` resets the device or requests the system NMI for errors the FRAM can't correct, and `Fram::enable_correctable_bit_error_interrupts()` reports corrected ones.
- Add `Fram::set_writable_program_fram()` on the MSP430FR2x5x and MSP430FR2522, which leaves the start of program FRAM writable (FRWPOA).
- Add `ClockConfig::reset_on_fll_unlock()`, which resets the device if the DCO runs too fast for the FLL.
- Fixed `Fram::set_wait_states()` leaving the FRAM controller registers unlocked.

## [v0.8.0] - 2026-08-14
- Changed name of project from `msp430fr2x5c-hal` to `msp430-hal` to better represent the scope of the project.
  - On the old `msp430fr2x5c-hal` crate, this added a build error telling users to switch to the new `msp430-hal` crate.
  - On the new crate `msp430-hal`, 0.8.0 is a no-change release from 0.7.0, just to keep version numbers in line.

## [v0.7.0] - 2026-08-11
- Add support for the MSP430FR25x2 subfamily.
- Add support for the MSP430FR247x subfamily.
- Add support for pin remapping on certain peripherals on certain devices (like the FR247x subfamily). For devices that don't support pin remapping this should be invisible for the most part.
- Add defmt support through `defmt` feature flag.
- Optimised GPIO toggling implementation
- Fixed a bug around enabling/disabling the PWM on the FR2433 Timer1 peripheral.

## [v0.6.1] - 2026-03-11
- Fix missing readme on crates.io

## [v0.6.0] - 2026-03-11
- The library now requires specifying a device feature (e.g. `msp430fr2355`) as part of the process to support more devices. Users must enable exactly one device feature that matches the device being targetted.
- The underlying MSP430FR2355 PAC version has been bumped to v0.6, which now uses svd2rust 0.37.1. This has changed the capitalisation of peripheral instances from `SCREAMING_CASE` to `snake_case`, e.g.`periph.WDT_A` is now `periph.wdt_a`. Peripheral type names have also changed case, and underscores have been omitted, e.g. `E_USCI_A0` is now `EUsciA0`. Both the HAL and PAC should be updated together to v0.6.
- Add initial support for the MSP430FR2433
- The `REFOCLK` and `VLOCLK` constants have been renamed to the more descriptive `REFOCLK_FREQ_HZ` and `VLOCLK_FREQ_HZ`.
- The frequency of MODCLK is now exported through the `MODCLK_FREQ_HZ` constant.
- Batch GPIO configuration now supports configuring pins to alternate modes
- Added Batch GPIO methods to set all pins in a port as inputs with either pullups or pulldowns. Useful for minimising power usage on unused pins, as leaving them in the default floating state can waste a lot of power through noise-induced schmitt trigger activations.
- The `pac::Sys` register block is now consumed by `Pmm::new()` and additionally returns an instance of `InfoMemory`. The `SYSCFGx` registers are touched internally by several peripherals in the HAL, but there was previously no mechanism to prevent users from accidentally modifying these registers afterwards. An instance of `pac::Sys` can be constructed unsafely using `pac::Sys::steal()` if it is required, but it is up to the user to ensure that control bits used by the HAL are not modified.
- Refactors to the Information Memory interface:
  - As above, rather than `InfoMemory::as_x()` methods consuming the `pac::Sys` register block, the `Pmm::new()` method returns an instance of `InfoMemory`.
  - `InfoMemory` now offers two modes of operation: The information memory is normally write protected, but `.write()` takes a closure where the info memory's write protection is temporarily disabled. Alternatively, `.into_unprotected()` disables the write protection entirely and returns a mutable reference to the underlying array.
- Fixed linker error in release mode in some examples: "undefined reference to `__mspabi_func_epilog`" by linking lgcc last in `.cargo/config.toml`. If you encounter this error in your project you can make the same fix.


## [v0.5.0] - 2025-10-30

### Additions
- Add support for low power modes
- Add SPI slave support. This includes modifying SPI configuration flow and `SpiErr`.
- Add support for I2C multi-master, slave, and master-slave roles.
- Add support for Smart Analog Combo and Enhanced Comparator modules (MSP430FR23xx only)
- Add support for reading from / writing to backup memory and information memory
- Add support for hardware CRC module
- Add implementations for embedded-hal 1.0 traits (including embedded-io and embedded-hal-nb).
- Support for delays when using sub-1MHz clock sources (e.g. ACLK, VLOCLK).
- Add methods to enable the internal voltage reference and temperature sensor.
- Add additional generic delay implementations for eh-0.2.7 DelayMs trait (u8, u32, i32).
- Derive `Debug` and `Copy` for `RecvError`.

### Changes
- MSRV updated to nightly-2023-09-01 (1.82) due to `msp430-rt` 0.4.1.
- Gate embedded-hal 0.2.7 implementations behind `embedded-hal-02` feature.
- Expose functionality from the traits dropped between eh-0.2.7 and eh-1.0 (ADC, timers, RTC, watchdog, etc.) as methods on structs instead.
- The SPI struct has been renamed from `SpiBus` to `Spi` to avoid naming conflicts with the new embedded-hal 1.0 trait `SpiBus`.
- Ensure crate builds successfully back to `nightly-2023-09-01`.
- Replace public references to `void::Void` with `core::convert::Infallible`.

### Bugfixes
- Fix SPI flushing bug.
- Fix GPIO pins labelled with incorrect SPI functionality for eUSCI A0 and A1.
- Fixed a bug that would sometimes cause an infinite loop during clock configuration (same bug mentioned in v0.4.0).
- Bring I2C implementation inline with the contracts mentioned in embedded-hal.

## [v0.4.1] - 2025-01-25

- Fix doc.rs build issue

## [v0.4.0] - 2025-01-22

- Add support for ADC interface
- Add support for SPI interface
- Add support for I2C interface
- Add `Delay` object, allowing for millisecond delays
- Change `ClockConfig::freeze` to return `Delay` in addition to its other return values
- Mitigate issue of device hanging after clock configuration by adding NOPs

## [v0.3.3] - 2022-12-24

- Bump `msp430fr2355` to v0.5.2 to ensure atomic PAC operations are single-instruction

## [v0.3.2] - 2022-10-26

- Bump `msp430fr2355` to v0.5.1
- Fix documentation generation on doc.rs

## [v0.3.1] - 2022-10-26

- Remove erroneous line in docs

## [v0.3.0] - 2022-10-26

- Bump `msp430fr2355` to v0.5
- Add CI pipeline
- Update dependencies to latest ecosystem versions
