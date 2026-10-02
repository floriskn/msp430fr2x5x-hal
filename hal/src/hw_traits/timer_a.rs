#[allow(unused_imports)] // The device modules import the shared traits through this
pub use crate::hw_traits::timer_base::*;

// Timer_A has no features beyond the shared ones
#[allow(unused_macros)] // Not every device has a Timer_A
macro_rules! timer_a_impl {
    ($($args:tt)*) => { $crate::hw_traits::timer_base::timer_base_impl!(A, $($args)*); };
}
#[allow(unused_imports)]
pub(crate) use timer_a_impl;
