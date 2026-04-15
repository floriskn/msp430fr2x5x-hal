pub trait Steal {
    unsafe fn steal() -> Self;
}

pub mod capacitive;
#[cfg(feature = "ecomp")]
pub mod ecomp;
pub mod eusci;
pub mod gpio;
pub mod sac;
pub mod timer_base;
pub mod timer_a;
pub mod timer_b;
