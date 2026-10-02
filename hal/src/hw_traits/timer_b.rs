#[allow(unused_imports)] // The device modules import the shared traits through this
pub use crate::hw_traits::timer_base::*;

// Timer_B adds the compare latches (CLLD) and the counter length (CNTL) to the shared features
macro_rules! timer_b_impl {
    ($($args:tt)*) => { $crate::hw_traits::timer_base::timer_base_impl!(B, $($args)*); };
}
pub(crate) use timer_b_impl;
