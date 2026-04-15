use super::Steal;

pub trait Capacitive: Steal {
  fn captioxctl_rd(&self) -> u16;
  fn captioxctl_set(&self, bits: u16);
  fn enable(&self);
  fn disable(&self);
}

macro_rules! capacitive_impl {
    (
    $CAPTIOx:ident, $captioxctl:ident
    ) => {
        impl Steal for $CAPTIOx {
            #[inline(always)]
            unsafe fn steal() -> Self {
                $CAPTIOx::steal()
            }
        }
        
        impl Capacitive for $CAPTIOx {
          #[inline(always)]
          fn captioxctl_rd(&self) -> u16 {
            self.$captioxctl().read().bits()
          }

          #[inline(always)]
          fn captioxctl_set(&self, bits: u16) {
            self.$captioxctl().write(|w| unsafe { w.bits(bits) });
          }

          #[inline(always)]
          fn enable(&self) {
            unsafe { self.$captioxctl().set_bits(|w| w.captioen().set_bit()) };
          }

          #[inline(always)]
          fn disable(&self) {
            unsafe { self.$captioxctl().clear_bits(|w| w.captioen().clear_bit()) };
          }
        }
    };
}
pub(crate) use capacitive_impl;
