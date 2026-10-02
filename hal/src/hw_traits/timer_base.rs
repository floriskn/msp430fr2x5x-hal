// Functionality common to both TimerA and TimerB (Timer_B is identical to Timer_A apart from the
// differences in SLAU445I 14.1.1, p. 391)

use super::Steal;

// TASSEL/TBSSEL values (SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
pub enum Tbssel {
    Tbxclk,
    Aclk,
    Smclk,
    Inclk,
}

/// Timer clock divider (ID: SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
pub enum TimerDiv {
    /// No division
    _1,
    /// Divide by 2
    _2,
    /// Divide by 4
    _4,
    /// Divide by 8
    _8,
}

/// Timer expansion clock divider, applied on top of the normal clock divider (TAIDEX/TBIDEX: SLAU445I
/// 13.2.1.1, p. 370; SLAU445I Table 13-9, p. 389; SLAU445I Table 14-11, p. 414)
pub enum TimerExDiv {
    /// No division
    _1,
    /// Divide by 2
    _2,
    /// Divide by 3
    _3,
    /// Divide by 4
    _4,
    /// Divide by 5
    _5,
    /// Divide by 6
    _6,
    /// Divide by 7
    _7,
    /// Divide by 8
    _8,
}

// OUTMOD values (SLAU445I Table 13-2, p. 376; SLAU445I Table 14-4, p. 401)
pub enum Outmod {
    Out,
    Set,
    ToggleReset,
    SetReset,
    Toggle,
    Reset,
    ToggleSet,
    ResetSet,
}

// CM values (SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
pub enum Cm {
    NoCap,
    RisingEdge,
    FallingEdge,
    BothEdges,
}

// CCIS values (SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
pub enum Ccis {
    InputA,
    InputB,
    Gnd,
    Vcc,
}

pub trait TimerBase: Steal {
    /// Reset timer countdown (TBCLR: SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
    fn reset(&self);

    // The three mode functions write MC, set TBCLR and clear TBIFG (SLAU445I Table 13-4, p. 384; SLAU445I
    // Table 14-6, p. 409)
    /// Set to upmode, reset timer, and clear interrupts
    fn upmode(&self);
    /// Set to continuous mode, reset timer, and clear interrupts
    fn continuous(&self);
    /// Set to up/down mode, reset timer, and clear interrupts
    fn updown_mode(&self);
    /// The counting mode (MC: SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
    fn mode_rd(&self) -> u8;
    /// Set the counter length (Timer_B CNTL: 0 = 16-bit, 1 = 12-bit, 2 = 10-bit, 3 = 8-bit). Timer_A has none
    /// (SLAU445I Table 14-6, p. 409; SLAU445I 14.1.1, p. 391).
    fn set_cntl(&self, cntl: u8);

    /// Apply clock select settings (TBSSEL and ID: SLAU445I Table 13-4, p. 384; SLAU445I Table 14-6, p. 409)
    fn config_clock(&self, tbssel: Tbssel, div: TimerDiv);

    /// Check if timer is stopped
    fn is_stopped(&self) -> bool;

    /// Stop timer (MC = 0: SLAU445I Table 13-1, p. 371; SLAU445I Table 14-1, p. 394)
    fn stop(&self);

    /// Resume a *stopped* timer. Assumes the previous mode was 'stop'.
    /// Atomic, fast.
    fn resume(&self, mode: RunningMode);

    /// Change a timer's mode. Non-atomic, slower. To go from one mode to another the user's guide stops the
    /// timer first (MC = 0) (SLAU445I 13.2.3, p. 371; 14.2.3, p. 394).
    fn change_mode(&self, mode: Mode);

    /// Set expansion register clock divider settings (TBIDEX: SLAU445I Table 13-9, p. 389; SLAU445I
    /// Table 14-11, p. 414)
    fn set_tbidex(&self, tbidex: TimerExDiv);

    fn tbifg_rd(&self) -> bool;
    fn tbifg_clr(&self);

    fn tbie_set(&self);
    fn tbie_clr(&self);

    fn tbxiv_rd(&self) -> u16;

    /// Get the current timer value (TBxR: SLAU445I Table 13-5, p. 385; SLAU445I Table 14-7, p. 410).
    fn get_tbxr(&self) -> u16;
}

// MC values (SLAU445I Table 13-1, p. 371; SLAU445I Table 14-1, p. 394)
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum RunningMode {
    Up = 0b01,
    Continuous = 0b10,
    UpDown = 0b11,
}
#[repr(u8)]
pub enum Mode {
    Stop = 0b00,
    Up = 0b01,
    Continuous = 0b10,
    UpDown = 0b11,
}

pub trait CCRn<C>: Steal {
    fn set_ccrn(&self, count: u16);
    fn get_ccrn(&self) -> u16;

    fn config_outmod(&self, outmod: Outmod);
    fn config_cap_mode(&self, cm: Cm, ccis: Ccis);

    fn ccifg_rd(&self) -> bool;
    fn ccifg_clr(&self);

    fn ccie_set(&self);
    fn ccie_clr(&self);

    fn cov_ccifg_rd(&self) -> (bool, bool);
    fn cov_ccifg_clr(&self);
    fn cov_clr(&self);

    /// The output mode (OUTMOD: SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
    fn outmod_rd(&self) -> u8;
    /// Switch between an output mode and its inverse (set ↔ reset, toggle/reset ↔ toggle/set,
    /// set/reset ↔ reset/set: SLAU445I Table 13-2, p. 376) by setting or clearing the top OUTMOD bit. The
    /// other two bits stay, so this never passes through mode 0 (SLAU445I 13.2.5.1.3, p. 379; 14.2.5.1.3,
    /// p. 404, note "Switching between output modes").
    fn set_outmod_high_bit(&self, set: bool);
    /// Switch the capture input between GND and VCC (CCIS bit 0), for a software capture (SLAU445I
    /// 13.2.4.1.1, p. 376; 14.2.4.1.1, p. 399)
    fn toggle_ccis_low_bit(&self);
    /// Set when the compare latch loads (Timer_B CLLD: 0 = at once, 1 = when the timer counts to 0,
    /// 2 = also when it counts to the top in up/down mode). Timer_A has no compare latch (SLAU445I
    /// Table 14-2, p. 400; SLAU445I Table 14-8, p. 411; SLAU445I 14.1.1, p. 391). In up mode 1 and 2 load
    /// at once on the MSP430FR2x5x and MSP430FR247x (SLAZ695J TB25, p. 11; SLAZ726B TB25, p. 8).
    fn set_clld(&self, clld: u8);
}

/// Label for capture-compare register 0
pub struct CCR0;
/// Label for capture-compare register 1
pub struct CCR1;
/// Label for capture-compare register 2
pub struct CCR2;
/// Label for capture-compare register 3
pub struct CCR3;
/// Label for capture-compare register 4
pub struct CCR4;
/// Label for capture-compare register 5
pub struct CCR5;
/// Label for capture-compare register 6
pub struct CCR6;

// Write a Timer_B-only field, or nothing for Timer_A
macro_rules! timer_b_field {
    (A, $reg:expr, $field:ident, $value:expr) => { let _ = $value; };
    (B, $reg:expr, $field:ident, $value:expr) => {
        $reg.modify(|_, w| unsafe { w.$field().bits($value) });
    };
}
pub(crate) use timer_b_field;

// Mark Timer_B peripherals
macro_rules! timer_b_marker {
    (A, $TBx:ident) => {};
    (B, $TBx:ident) => { impl crate::timer::TimerB for $TBx {} };
}
pub(crate) use timer_b_marker;

macro_rules! ccrn_impl {
    ($kind:ident, $TBx:ident, $CCRn:ident, $tbxcctln:ident, $tbxccrn:ident) => {
        impl CCRn<$CCRn> for $TBx {
            #[inline(always)]
            fn set_ccrn(&self, count: u16) { self.$tbxccrn().write(|w| unsafe { w.bits(count) }); }

            #[inline(always)]
            fn get_ccrn(&self) -> u16 { self.$tbxccrn().read().bits() }

            // A write: OUTMOD, with CAP = 0 (compare mode) and every other field cleared (SLAU445I
            // Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
            #[inline(always)]
            fn config_outmod(&self, outmod: Outmod) {
                self.$tbxcctln().write(|w| unsafe { w.outmod().bits(outmod as u8) });
            }

            #[inline(always)]
            fn config_cap_mode(&self, cm: Cm, ccis: Ccis) {
                // CAP = 1 selects capture mode (SLAU445I 13.2.4.1, p. 374). SCS synchronizes the capture with
                // the timer clock, which the user's guide recommends (SLAU445I 13.2.4.1, p. 375; SLAU445I
                // 14.2.4.1, p. 398).
                self.$tbxcctln().write(|w| unsafe { w
                    .cap().set_bit()
                    .scs().set_bit()
                    .cm().bits(cm as u8)
                    .ccis().bits(ccis as u8)
                });
            }

            #[inline(always)]
            fn ccifg_rd(&self) -> bool { self.$tbxcctln().read().ccifg().bit() }

            #[inline(always)]
            fn ccifg_clr(&self) {
                unsafe { self.$tbxcctln().clear_bits(|w| w.ccifg().clear_bit()) };
            }

            #[inline(always)]
            fn ccie_set(&self) { unsafe { self.$tbxcctln().set_bits(|w| w.ccie().set_bit()) }; }

            #[inline(always)]
            fn ccie_clr(&self) { unsafe { self.$tbxcctln().clear_bits(|w| w.ccie().clear_bit()) }; }

            #[inline(always)]
            fn cov_ccifg_rd(&self) -> (bool, bool) {
                let cctl = self.$tbxcctln().read();
                (cctl.cov().bit(), cctl.ccifg().bit())
            }

            #[inline(always)]
            fn cov_clr(&self) {
                unsafe { self.$tbxcctln().clear_bits(|w| w.cov().clear_bit()) };
            }

            #[inline(always)]
            fn cov_ccifg_clr(&self) {
                unsafe {
                    self.$tbxcctln().clear_bits(|w| w
                        .ccifg().clear_bit()
                        .cov().clear_bit())
                };
            }

            // OUTMOD is bits 7..5 (SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
            #[inline(always)]
            fn outmod_rd(&self) -> u8 { (self.$tbxcctln().read().bits() >> 5) as u8 & 0b111 }

            #[inline(always)]
            fn set_outmod_high_bit(&self, set: bool) {
                // OUTMOD is bits 7..5 (SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
                if set {
                    unsafe { self.$tbxcctln().set_bits(|w| w.bits(1 << 7)) };
                } else {
                    unsafe { self.$tbxcctln().clear_bits(|w| w.bits(!(1 << 7))) };
                }
            }

            #[inline(always)]
            fn toggle_ccis_low_bit(&self) {
                // CCIS is bits 13..12 (SLAU445I Table 13-6, p. 386; SLAU445I Table 14-8, p. 411)
                self.$tbxcctln().modify(|r, w| unsafe { w.bits(r.bits() ^ (1 << 12)) });
            }

            // CLLD is a Timer_B field (SLAU445I Table 14-8, p. 411)
            #[inline(always)]
            fn set_clld(&self, clld: u8) {
                $crate::hw_traits::timer_base::timer_b_field!($kind, self.$tbxcctln(), clld, clld);
            }
        }
    };
}
pub(crate) use ccrn_impl;

macro_rules! timer_base_impl {
    (
        $kind:ident, // A for Timer_A, B for Timer_B
        $TBx:ident, $tbx:ident, $tbxctl:ident, $tbxex:ident, $tbxiv:ident, $tbxr:ident, // Timer registers
        $txclr:ident, $txifg:ident, $txidex:ident, $txie:ident, $txssel:ident, // Register field names (differ between TimerA and TimerB)
        $([$CCRn:ident, $tbxcctln:ident, $tbxccrn:ident]),* // CCR registers
    ) => {
        impl Steal for $TBx {
            #[inline(always)]
            unsafe fn steal() -> Self {
                $TBx::steal()
            }
        }

        impl TimerBase for $TBx {
            // TBCLR clears the count, the clock divider logic and the count direction (SLAU445I Table 13-4,
            // p. 384; SLAU445I Table 14-6, p. 409)
            #[inline(always)]
            fn reset(&self) {
                unsafe { self.$tbxctl().set_bits(|w| w.$txclr().set_bit()) };
            }

            // The three mode functions set TBCLR along with MC, which also restarts the divider logic as a
            // TBIDEX change needs (SLAU445I 13.3.6, p. 389; SLAU445I 14.3.6, p. 414). MC values: SLAU445I
            // Table 13-1, p. 371; SLAU445I Table 14-1, p. 394.
            #[inline(always)]
            fn upmode(&self) {
                self.$tbxctl().modify(|r, w| {
                    unsafe { w.bits(r.bits())
                        .$txclr().set_bit()
                        .$txifg().clear_bit()
                        .mc().bits(Mode::Up as u8)
                    }
                });
            }

            #[inline(always)]
            fn continuous(&self) {
                self.$tbxctl().modify(|r, w| {
                    unsafe { w.bits(r.bits())
                        .$txclr().set_bit()
                        .$txifg().clear_bit()
                        .mc().bits(Mode::Continuous as u8)
                    }
                });
            }

            #[inline(always)]
            fn updown_mode(&self) {
                self.$tbxctl().modify(|r, w| {
                    unsafe { w.bits(r.bits())
                        .$txclr().set_bit()
                        .$txifg().clear_bit()
                        .mc().bits(Mode::UpDown as u8)
                    }
                });
            }

            #[inline(always)]
            fn mode_rd(&self) -> u8 {
                self.$tbxctl().read().mc().bits()
            }

            #[inline(always)]
            fn set_cntl(&self, cntl: u8) {
                $crate::hw_traits::timer_base::timer_b_field!($kind, self.$tbxctl(), cntl, cntl);
            }

            // A write, so MC = 0 and the timer stops: the clock source and the dividers are only to be
            // changed while it's stopped (SLAU445I 13.2.1.1, p. 370, note "Timer_A dividers"; 13.2.7,
            // p. 382; 14.2.1.2, p. 393; 14.2.7, p. 407)
            #[inline(always)]
            fn config_clock(&self, tbssel: Tbssel, div: TimerDiv) {
                self.$tbxctl()
                    .write(|w| unsafe { w
                        .$txssel().bits(tbssel as u8)
                        .id().bits(div as u8)
                    });
            }

            #[inline(always)]
            fn is_stopped(&self) -> bool {
                self.$tbxctl().read().mc().bits() == (Mode::Stop as u8)
            }

            #[inline(always)]
            fn stop(&self) {
                unsafe { self.$tbxctl().clear_bits(|w| w.mc().bits(Mode::Stop as u8)) };
            }

            #[inline(always)]
            fn change_mode(&self, mode: Mode) {
                self.$tbxctl().modify(|_,w| unsafe{ w.mc().bits(mode as u8) });
            }

            #[inline(always)]
            fn resume(&self, mode: RunningMode) {
                unsafe { self.$tbxctl().set_bits(|w| w.mc().bits(mode as u8)) };
            }

            #[inline(always)]
            fn set_tbidex(&self, tbidex: TimerExDiv) {
                self.$tbxex().write(|w| unsafe { w.$txidex().bits(tbidex as u8) });
            }

            #[inline(always)]
            fn tbifg_rd(&self) -> bool {
                self.$tbxctl().read().$txifg().bit()
            }

            #[inline(always)]
            fn tbifg_clr(&self) {
                unsafe { self.$tbxctl().clear_bits(|w| w.$txifg().clear_bit()) };
            }

            #[inline(always)]
            fn tbie_set(&self) {
                unsafe { self.$tbxctl().set_bits(|w| w.$txie().set_bit()) };
            }

            #[inline(always)]
            fn tbie_clr(&self) {
                unsafe { self.$tbxctl().clear_bits(|w| w.$txie().clear_bit()) };
            }

            // Reading TBxIV clears the highest pending flag (SLAU445I 13.2.6.2, p. 380; 14.2.6.2, p. 405)
            #[inline(always)]
            fn tbxiv_rd(&self) -> u16 {
                self.$tbxiv().read().bits()
            }

            #[inline(always)]
            fn get_tbxr(&self) -> u16 {
                self.$tbxr().read().bits()
            }
        }

        $crate::hw_traits::timer_base::timer_b_marker!($kind, $TBx);

        $($crate::hw_traits::timer_base::ccrn_impl!($kind, $TBx, $CCRn, $tbxcctln, $tbxccrn);)*
    };
}
pub(crate) use timer_base_impl;
