//! Embedded hal delay implementation
//!
//! The delays count MCLK cycles in a loop, so they last at least as long as requested; interrupts
//! during a delay make it longer.

/// Delay provider struct
#[derive(Copy, Clone)]
pub struct SysDelay {
    /// Loop iterations per millisecond
    iters_per_ms: u16,
    /// Loop iterations per microsecond, in 1/4096ths
    iters_per_us_q12: u32,
}

/// MCLK cycles per iteration of the delay loop: `SUB #1, Rn` takes 1 cycle and `JNZ` 2 (SLAU445I 4.5.1.5)
const CYCLES_PER_ITER: u32 = 3;

impl SysDelay {
    /// Create a new delay object for an MCLK of `freq` Hz
    pub(crate) fn new(freq: u32) -> Self {
        // Round up, so delays don't fall short. The clock could be REFOCLK or VLOCLK, so be careful
        // of small frequencies.
        let iters_per_ms = freq.div_ceil(1000 * CYCLES_PER_ITER).max(1) as u16;
        let iters_per_us_q12 = (freq.div_ceil(1000) * 4096).div_ceil(1000 * CYCLES_PER_ITER);
        SysDelay { iters_per_ms, iters_per_us_q12 }
    }

    /// Spin for `iters` iterations of [`CYCLES_PER_ITER`] cycles
    #[inline(always)]
    fn spin(iters: u16) {
        if iters == 0 {
            return;
        }
        unsafe {
            core::arch::asm!(
                "1:",
                "sub #1, {n}",
                "jnz 1b",
                n = inout(reg) iters => _,
                options(nomem, nostack),
            );
        }
    }

    #[inline]
    fn ms(&self, ms: u32) {
        for _ in 0..ms {
            Self::spin(self.iters_per_ms);
        }
    }

    /// Spin for `us` microseconds, which is below 1000
    #[inline]
    fn short_us(&self, us: u16) {
        // At most 999 * 32768 for a 24 MHz MCLK, which fits
        Self::spin(((us as u32 * self.iters_per_us_q12 + 4095) >> 12) as u16);
    }

    #[inline]
    fn us(&self, us: u32) {
        // Divide only for long delays, where it takes a small part of the time
        if us < 1000 {
            self.short_us(us as u16);
        } else {
            self.ms(us / 1000);
            self.short_us((us % 1000) as u16);
        }
    }
}

mod ehal1 {
    use super::*;
    use embedded_hal::delay::DelayNs;

    impl DelayNs for SysDelay {
        /// Pauses execution for at least `ns` nanoseconds, rounded up to whole microseconds. At low MCLK
        /// frequencies the call itself takes a few microseconds.
        #[inline]
        fn delay_ns(&mut self, ns: u32) {
            if ns <= 1000 {
                self.us(1);
            } else {
                self.us(ns.div_ceil(1000));
            }
        }

        /// Pauses execution for at least `us` microseconds. At low MCLK frequencies the call itself
        /// takes a few microseconds.
        #[inline]
        fn delay_us(&mut self, us: u32) { self.us(us) }

        /// Pauses execution for at least `ms` milliseconds.
        #[inline]
        fn delay_ms(&mut self, ms: u32) { self.ms(ms) }
    }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use super::*;
    use embedded_hal_02::blocking::delay::DelayMs;

    macro_rules! impl_delay {
        ($typ: ty) => {
            impl DelayMs<$typ> for SysDelay {
                #[inline]
                fn delay_ms(&mut self, ms: $typ) {
                    for _ in 0..ms {
                        SysDelay::spin(self.iters_per_ms);
                    }
                }
            }
        };
    }

    impl_delay!(u8);
    impl_delay!(u16);
    impl_delay!(u32);

    // A delay implementation for the default literal type to allow calls like `delay_ms(100)`
    // Negative durations are treated as zero.
    impl_delay!(i32);
}
