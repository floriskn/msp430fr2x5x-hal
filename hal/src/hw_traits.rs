pub trait Steal {
    unsafe fn steal() -> Self;
}

#[cfg(feature = "ecomp")]
pub mod ecomp;
pub mod eusci;
pub mod gpio;
#[cfg(feature = "sac")]
pub mod sac;
pub mod timer_a;
// Timer_B only on the MSP430FR2x5x (SLASEC4D Table 3-1, p. 8) and the MSP430FR247x (SLASEO7C Table 6-1,
// p. 6); the MSP430FR2433 and MSP430FR25x2 have only Timer_A (SLASE59F Table 3-1, p. 7; SLASEE4C Table 3-1,
// p. 8)
#[cfg(any(feature = "2x5x", feature = "247x"))]
pub mod timer_b;
pub mod timer_base;
