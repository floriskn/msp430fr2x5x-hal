use super::Steal;

// The base address of a PAC peripheral: the `A` of its `Periph<RB, A>` type
pub trait BaseAddr {
    const BASE: usize;
}
impl<RB, const A: usize> BaseAddr for crate::_pac::generic::Periph<RB, A> {
    const BASE: usize = A;
}

// The address of register `$reg` of PAC peripheral `$P`, as a constant: the peripheral's base address plus
// the register's offset in the PAC's register block, which the register's accessor gives. The block is
// zeroed memory, not the peripheral: the accessor only takes the address of its field.
macro_rules! reg_addr {
    ($P:ty, $reg:ident) => {{
        let block = core::mem::MaybeUninit::<<$P as core::ops::Deref>::Target>::zeroed();
        let base = block.as_ptr();
        let reg = unsafe { (*base).$reg() } as *const _ as *const u8;
        <$P as $crate::hw_traits::gpio::BaseAddr>::BASE + unsafe { reg.offset_from(base as *const u8) } as usize
    }};
}
pub(crate) use reg_addr;

// Set, clear or toggle bits of a register at a constant address in one instruction: BIS, BIC or XOR with
// the address as the absolute destination (`&ADDR`). The instruction reads, changes and writes the
// register, and an interrupt waits for it to complete ("Any currently executing instruction is
// completed", SLAU445I 1.3.4.1, p. 33), so the change is atomic, as the PAC's `set_bits`, `clear_bits`
// and `toggle_bits` are, which take the address in a register. BIS and BIC leave the status bits as they
// are, XOR changes them (SLAU445I 4.6.2.6, p. 176; SLAU445I 4.6.2.5, p. 175; SLAU445I 4.6.2.51,
// p. 221). `$suffix` is `.b` for a byte register.
macro_rules! bits_op {
    (bis $suffix:literal, $addr:expr, $bits:expr) => {
        unsafe { core::arch::asm!(concat!("bis", $suffix, " {b}, &{a}"), b = in(reg) $bits, a = const $addr,
            options(nostack, preserves_flags)) }
    };
    (bic $suffix:literal, $addr:expr, $bits:expr) => {
        unsafe { core::arch::asm!(concat!("bic", $suffix, " {b}, &{a}"), b = in(reg) $bits, a = const $addr,
            options(nostack, preserves_flags)) }
    };
    (xor $suffix:literal, $addr:expr, $bits:expr) => {
        unsafe { core::arch::asm!(concat!("xor", $suffix, " {b}, &{a}"), b = in(reg) $bits, a = const $addr,
            options(nostack)) }
    };
}
pub(crate) use bits_op;

// The port registers: SLAU445I Table 8-4, p. 319 to p. 331
pub trait GpioPeriph: Steal {
    // PxIN, read only (SLAU445I Table 8-9, p. 334)
    fn pxin_rd(&self) -> u8;

    // PxOUT (SLAU445I Table 8-10, p. 334)
    fn pxout_rd(&self) -> u8;
    fn pxout_wr(&self, bits: u8);
    fn pxout_set(&self, bits: u8);
    fn pxout_clear(&self, bits: u8);
    fn pxout_toggle(&self, bits: u8);

    // PxDIR (SLAU445I Table 8-11, p. 334)
    fn pxdir_rd(&self) -> u8;
    fn pxdir_wr(&self, bits: u8);
    fn pxdir_set(&self, bits: u8);
    fn pxdir_clear(&self, bits: u8);

    // PxREN (SLAU445I Table 8-12, p. 335)
    fn pxren_rd(&self) -> u8;
    fn pxren_wr(&self, bits: u8);
    fn pxren_set(&self, bits: u8);
    fn pxren_clear(&self, bits: u8);

    // PxSELC, which "Always reads as 0" (SLAU445I Table 8-15, p. 336), so it is only written
    fn pxselc_wr(&self, bits: u8);

    // PxSEL0 (SLAU445I Table 8-13, p. 335)
    fn pxsel0_rd(&self) -> u8;
    fn pxsel0_wr(&self, bits: u8);
    fn pxsel0_set(&self, bits: u8);
    fn pxsel0_clear(&self, bits: u8);

    // PxSEL1 (SLAU445I Table 8-14, p. 335)
    fn pxsel1_rd(&self) -> u8;
    fn pxsel1_wr(&self, bits: u8);
    fn pxsel1_set(&self, bits: u8);
    fn pxsel1_clear(&self, bits: u8);

    // ADCPCTL0 to ADCPCTL7 are bits 0 to 7 of SYSCFG2 (SLAU445I Table 1-31, p. 82; SLASE59F Table 6-17,
    // p. 55; SLASEE4C Table 6-15, p. 58), set or cleared in one instruction. `mask` of `adcpctl_clr` holds
    // the bits to keep, as for the PAC's `clear_bits`.
    #[cfg(feature = "adcpctl")]
    #[inline(always)]
    fn adcpctl_set(&self, mask: u16) {
        const ADDR: usize = reg_addr!(crate::_pac::Sys, syscfg2);
        bits_op!(bis "", ADDR, mask);
    }
    #[cfg(feature = "adcpctl")]
    #[inline(always)]
    fn adcpctl_clr(&self, mask: u16) {
        const ADDR: usize = reg_addr!(crate::_pac::Sys, syscfg2);
        bits_op!(bic "", ADDR, !mask);
    }
}

// Ports with interrupts also have PxIES, PxIE and PxIFG (SLAU445I 8.2.6, p. 314) and a word-accessible
// PxIV (SLAU445I 8.2.6, p. 315)
pub trait IntrPeriph: GpioPeriph {
    // PxIES (SLAU445I Table 8-16, p. 336)
    fn pxies_rd(&self) -> u8;
    fn pxies_wr(&self, bits: u8);
    fn pxies_set(&self, bits: u8);
    fn pxies_clear(&self, bits: u8);

    // PxIE (SLAU445I Table 8-17, p. 336)
    fn pxie_rd(&self) -> u8;
    fn pxie_wr(&self, bits: u8);
    fn pxie_set(&self, bits: u8);
    fn pxie_clear(&self, bits: u8);

    // PxIFG (SLAU445I Table 8-18, p. 337)
    fn pxifg_rd(&self) -> u8;
    fn pxifg_wr(&self, bits: u8);
    fn pxifg_set(&self, bits: u8);
    fn pxifg_clear(&self, bits: u8);

    // PxIV, a 16-bit register (SLAU445I Tables 8-5 to 8-8, p. 332 to p. 333). Reading it clears the flag it
    // reports (SLAU445I 8.2.6, p. 315).
    fn pxiv_rd(&self) -> crate::gpio::GpioVector;
}

// Read, write, set and clear the bits of one 8-bit port register `$reg` of PAC peripheral `$P` (SLAU445I
// Table 8-4, p. 319 to p. 331). Set and clear are one instruction each, see `bits_op`.
macro_rules! reg_methods {
    ($P:ty, $reg:ident, $rd:ident, $wr:ident, $set:ident, $clear:ident) => {
        // One port register of SLAU445I Tables 8-9 to 8-18, p. 334 to p. 337
        #[inline(always)]
        fn $rd(&self) -> u8 { self.$reg().read().bits() }

        #[inline(always)]
        fn $wr(&self, bits: u8) { self.$reg().write(|w| unsafe { w.bits(bits) }); }

        #[inline(always)]
        fn $set(&self, bits: u8) {
            const ADDR: usize = $crate::hw_traits::gpio::reg_addr!($P, $reg);
            $crate::hw_traits::gpio::bits_op!(bis ".b", ADDR, bits);
        }

        // `bits` are the bits to keep, as for the PAC's `clear_bits`
        #[inline(always)]
        fn $clear(&self, bits: u8) {
            const ADDR: usize = $crate::hw_traits::gpio::reg_addr!($P, $reg);
            $crate::hw_traits::gpio::bits_op!(bic ".b", ADDR, !bits);
        }
    };
}
pub(crate) use reg_methods;

macro_rules! gpio_impl {
    ($px:ident: $Px:ident =>
     $pxin:ident, $pxout:ident, $pxdir:ident, $pxren:ident, $pxselc:ident, $pxsel0:ident, $pxsel1:ident
     $(, [$pxies:ident, $pxie:ident, $pxifg:ident, $pxiv:ident])?
    ) => {
        mod $px {
            use crate::{pac, gpio::*, hw_traits::{Steal, gpio::*}};

            impl Steal for pac::$Px {
                #[inline(always)]
                unsafe fn steal() -> Self {
                    $Px::steal()
                }
            }

            impl GpioPeriph for pac::$Px {
                // PxIN (SLAU445I Table 8-9, p. 334)
                #[inline(always)]
                fn pxin_rd(&self) -> u8 {
                    self.$pxin().read().bits()
                }

                // PxSELC (SLAU445I Table 8-15, p. 336)
                #[inline(always)]
                fn pxselc_wr(&self, bits: u8) {
                    self.$pxselc().write(|w| unsafe { w.bits(bits) });
                }

                // PxOUT (SLAU445I Table 8-10, p. 334), in one instruction, see `bits_op`
                #[inline(always)]
                fn pxout_toggle(&self, bits: u8) {
                    const ADDR: usize = reg_addr!(pac::$Px, $pxout);
                    bits_op!(xor ".b", ADDR, bits);
                }

                // PxOUT, PxDIR, PxREN, PxSEL0 and PxSEL1 (SLAU445I Tables 8-10 to 8-14, p. 334 to p. 335)
                reg_methods!(pac::$Px, $pxout, pxout_rd, pxout_wr, pxout_set, pxout_clear);
                reg_methods!(pac::$Px, $pxdir, pxdir_rd, pxdir_wr, pxdir_set, pxdir_clear);
                reg_methods!(pac::$Px, $pxren, pxren_rd, pxren_wr, pxren_set, pxren_clear);
                reg_methods!(pac::$Px, $pxsel0, pxsel0_rd, pxsel0_wr, pxsel0_set, pxsel0_clear);
                reg_methods!(pac::$Px, $pxsel1, pxsel1_rd, pxsel1_wr, pxsel1_set, pxsel1_clear);
            }

            $(
                impl IntrPeriph for pac::$Px {
                    // PxIES, PxIE and PxIFG (SLAU445I Tables 8-16 to 8-18, p. 336 to p. 337)
                    reg_methods!(pac::$Px, $pxies, pxies_rd, pxies_wr, pxies_set, pxies_clear);
                    reg_methods!(pac::$Px, $pxie, pxie_rd, pxie_wr, pxie_set, pxie_clear);
                    reg_methods!(pac::$Px, $pxifg, pxifg_rd, pxifg_wr, pxifg_set, pxifg_clear);

                    // PxIV (SLAU445I Tables 8-5 to 8-8, p. 332 to p. 333)
                    #[inline(always)]
                    fn pxiv_rd(&self) -> GpioVector {
                        let r = self.$pxiv().read();
                        let iv = r.$pxiv();
                        if iv.is_ifg0() { GpioVector::Pin0Isr }
                        else if iv.is_ifg1() { GpioVector::Pin1Isr }
                        else if iv.is_ifg2() { GpioVector::Pin2Isr }
                        else if iv.is_ifg3() { GpioVector::Pin3Isr }
                        else if iv.is_ifg4() { GpioVector::Pin4Isr }
                        else if iv.is_ifg5() { GpioVector::Pin5Isr }
                        else if iv.is_ifg6() { GpioVector::Pin6Isr }
                        else if iv.is_ifg7() { GpioVector::Pin7Isr }
                        else { GpioVector::NoIsr }
                    }
                }
            )?
        }
    };
}
pub(crate) use gpio_impl;
