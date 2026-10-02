//! Interrupt Compare Controller (ICC): interrupt priorities and nesting
//!
//! Only on the MSP430FR2x5x. Without the ICC, the interrupt vector table fixes the interrupt priorities, and an
//! interrupt handler runs to the end unless it enables interrupts itself, in which case any interrupt can interrupt
//! it. With the ICC enabled, each maskable interrupt source has one of four priorities, and a handler can only be
//! interrupted by a higher priority (SLAU445I chapter 5). Sources with the same priority are served in vector table
//! order.
//!
//! Nesting needs interrupts enabled in each handler: call `msp430::interrupt::enable()` at the start of every
//! interrupt handler that may be interrupted (data sheet: Interrupt Compare Controller).
//!
//! ```ignore
//! let mut icc = Icc::new(periph.icc);
//! icc.set_priority(IccSource::Timer0B0, Priority::Highest);
//! icc.enable();
//! ```

use crate::_pac;

/// The priority of an interrupt source (ILSRx). The ICC serves a higher priority first, and lets it interrupt
/// the handler of a lower one.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Priority {
    /// Level 0, the highest
    Highest = 0,
    /// Level 1
    High = 1,
    /// Level 2
    Low = 2,
    /// Level 3, the lowest, as after reset
    Lowest = 3,
}

impl Priority {
    #[inline(always)]
    fn from_bits(bits: u16) -> Self {
        match bits & 0b11 {
            0 => Priority::Highest,
            1 => Priority::High,
            2 => Priority::Low,
            _ => Priority::Lowest,
        }
    }
}

/// The maskable interrupt sources the ICC manages, by their level setting fields ILSR0 to ILSR21 (data sheet:
/// ICC interrupt source assignments)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum IccSource {
    /// Port 4 (`PORT4`)
    Port4 = 0,
    /// Port 3 (`PORT3`)
    Port3 = 1,
    /// Port 2 (`PORT2`)
    Port2 = 2,
    /// Port 1 (`PORT1`)
    Port1 = 3,
    /// The DACs of SAC1 and SAC3 (`SAC1_SAC3`), MSP430FR235x only
    #[cfg(feature = "sac")]
    Sac1Sac3 = 4,
    /// The DACs of SAC0 and SAC2 (`SAC0_SAC2`), MSP430FR235x only
    #[cfg(feature = "sac")]
    Sac0Sac2 = 5,
    /// eCOMP0 and eCOMP1 (`ECOMP0_ECOMP1`)
    EComp = 6,
    /// The ADC (`ADC`)
    Adc = 7,
    /// eUSCI_B1 (`EUSCI_B1`)
    EUsciB1 = 8,
    /// eUSCI_B0 (`EUSCI_B0`)
    EUsciB0 = 9,
    /// eUSCI_A1 (`EUSCI_A1`)
    EUsciA1 = 10,
    /// eUSCI_A0 (`EUSCI_A0`)
    EUsciA0 = 11,
    /// The watchdog in interval mode (`WDT`)
    Watchdog = 12,
    /// The RTC (`RTC`)
    Rtc = 13,
    /// TB3's CCR1 to CCR6 and overflow (`TIMER3_B1`)
    Timer3B1 = 14,
    /// TB3's CCR0 (`TIMER3_B0`)
    Timer3B0 = 15,
    /// TB2's CCR1, CCR2 and overflow (`TIMER2_B1`)
    Timer2B1 = 16,
    /// TB2's CCR0 (`TIMER2_B0`)
    Timer2B0 = 17,
    /// TB1's CCR1, CCR2 and overflow (`TIMER1_B1`)
    Timer1B1 = 18,
    /// TB1's CCR0 (`TIMER1_B0`)
    Timer1B0 = 19,
    /// TB0's CCR1, CCR2 and overflow (`TIMER0_B1`)
    Timer0B1 = 20,
    /// TB0's CCR0 (`TIMER0_B0`)
    Timer0B0 = 21,
}

// ICCSC bits
const ICCEN: u16 = 1 << 7;
const VSEFLG: u16 = 1 << 5;

/// The Interrupt Compare Controller
pub struct Icc(_pac::Icc);

impl Icc {
    /// Take the ICC. It stays disabled until [`Icc::enable`], with every source at the lowest priority.
    #[inline]
    pub fn new(icc: _pac::Icc) -> Self { Icc(icc) }

    #[inline(always)]
    fn ilsr_ptr(&self, source: IccSource) -> (*mut u16, u16) {
        // Eight 2-bit fields per register, ICCILSR0 at offset 4 (SLAU445I Table 5-1)
        let index = source as u16;
        let reg = unsafe { (self.0.iccsc().as_ptr() as *mut u16).add(2 + (index / 8) as usize) };
        (reg, (index % 8) * 2)
    }

    /// Set the priority of an interrupt source. Its priority can change at any time.
    #[inline]
    pub fn set_priority(&mut self, source: IccSource, priority: Priority) {
        let (reg, shift) = self.ilsr_ptr(source);
        critical_section::with(|_| unsafe {
            let value = reg.read_volatile() & !(0b11 << shift) | (priority as u16) << shift;
            reg.write_volatile(value);
        });
    }

    /// The priority of an interrupt source.
    #[inline]
    pub fn priority(&self, source: IccSource) -> Priority {
        let (reg, shift) = self.ilsr_ptr(source);
        Priority::from_bits(unsafe { reg.read_volatile() } >> shift)
    }

    /// Serve interrupts by priority, with nesting (ICCEN).
    #[inline]
    pub fn enable(&mut self) { unsafe { self.0.iccsc().set_bits(|w| w.bits(ICCEN)) } }

    /// Serve interrupts in vector table order again (ICCEN).
    #[inline]
    pub fn disable(&mut self) { unsafe { self.0.iccsc().clear_bits(|w| w.bits(!ICCEN)) } }

    /// The priority of the interrupt being served (ICMC), or `None` if no interrupt is (VSEFLG). In a nested
    /// handler, this is the priority of the innermost one.
    #[inline]
    pub fn current_priority(&self) -> Option<Priority> {
        let sc = self.0.iccsc().read().bits();
        if sc & VSEFLG != 0 {
            None
        } else {
            Some(Priority::from_bits(sc))
        }
    }

    /// How many interrupt handlers are nested at the moment, 0 to 4 (MVSSP).
    #[inline]
    pub fn nesting_depth(&self) -> u8 { ((self.0.iccmvs().read().bits() >> 8) & 0b111) as u8 }
}
