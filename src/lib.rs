// Some overall notes:
// We generally don't use the named field methods provided by PACs, as these are inconsistently
// implemented among PACs. Ie f3's may have a `'`.enabled()` method, but `l4` does not;
// in these cases, writing `set_bit()` works for both.

// We use a combination of macros and feature-gating to handle differences in families, as appropriate.
// We leverage the `paste` and `cfg-if` crates to improve syntax.

// The main way we divide MCUs is by PAC modules. Note that there are sub-variants that may have differences
// that this doesn't take into account. (eg different USB memory sizes among f303 variants)

// We use `unsafe` blocks for most multi-fit field writes. This is required by some PACs, but not others.
// The rust embedded team removes requirement for `unsafe` on fields that are deemed sufficiently
// constrained as to not need these blocks.
// Using `unsafe` for all is cleaner than feature-gating, due to how many fields this affects. We've allowed
// these warnings; ie hidden during build.

// todo issues hidden in modules we need to fix:
// - RTC wakeup clearing WUTF flag on L5. Not sure how to do it; SR? (Can't modify) ICSR? (don't remember the problme there)
// - Timer can't set PSC on L5: getting alternating `field, not a method`, and the inverse errors.
// - timer on L5 is effectively broken until this is fixed.
// - EXTI / interrupts on L5 and H7. What are the steps for H7? We have it compiling on H5,
// - but I don't think the EXTICRn register writes are set up correctly.

#![no_std]
// Some reg modifications are marked `unsafe` in some PAC crates, but not others.
// Disable these warnings.
#![allow(unused_unsafe)]

#[cfg(not(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l552",
    feature = "l562",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm4",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
    feature = "f446"
)))]
compile_error!("This crate requires an MCU-specifying feature to be enabled. eg `l552`.");

// F3 PAC
#[cfg(feature = "f301")]
pub use stm32f3::stm32f301 as pac;

#[cfg(feature = "f302")]
pub use stm32f3::stm32f302 as pac;

#[cfg(feature = "f303")]
pub use stm32f3::stm32f303 as pac;

#[cfg(feature = "f373")]
pub use stm32f3::stm32f373 as pac;

#[cfg(feature = "f3x4")]
pub use stm32f3::stm32f3x4 as pac;

// L4 PAC
#[cfg(feature = "l4x1")]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(feature = "l4x2")]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(feature = "l4x3")]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(feature = "l4x5")]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(feature = "l4x6")]
pub use stm32l4::stm32l4x6 as pac;

// L5 PAC
#[cfg(feature = "l552")]
pub use stm32l5::stm32l552 as pac;

#[cfg(feature = "l562")]
pub use stm32l5::stm32l562 as pac;

// H7 PAC
#[cfg(feature = "h743")]
pub use stm32h7::stm32h743 as pac;

#[cfg(feature = "h743v")]
pub use stm32h7::stm32h743v as pac;

#[cfg(feature = "h747cm4")]
pub use stm32h7::stm32h747cm4 as pac;

#[cfg(feature = "h747cm7")]
pub use stm32h7::stm32h747cm7 as pac;

#[cfg(feature = "h753")]
pub use stm32h7::stm32h753 as pac;

#[cfg(feature = "h753v")]
pub use stm32h7::stm32h753v as pac;

#[cfg(feature = "h7b3")]
pub use stm32h7::stm32h7b3 as pac;
#[cfg(feature = "f446")]
pub use stm32f4::stm32f446 as pac;


// todo: U5 once SVD is out.

pub mod traits;

/// In the prelude, we export the `embedded-hal` traits we implement
pub mod prelude {
    // pub use crate::traits::*;
    pub use embedded_hal::{
        blocking::delay::{DelayMs, DelayUs},
        digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
    };
}

#[cfg(not(any(feature = "l5", feature = "h7")))] // todo
pub mod adc;
pub mod clocks;
pub mod dac;
pub mod delay;
#[cfg(any(feature = "l4"))] // todo
pub mod dma;
#[cfg(not(any(feature = "l5", feature = "h7")))] // todo
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod low_power;
pub mod rtc;
#[cfg(any(feature = "l4"))] // todo
pub mod serial;
pub mod spi;
pub mod timer;

// In the l4 series, only l4x2 and l4x3 have USB.
cfg_if::cfg_if! {
    if #[cfg(all(
        feature = "usb",
        not(any(feature = "l4x1", feature = "l4x5", feature = "l4x6", feature = "h7"))
    ))] {
        pub mod usb;
    } else if #[cfg(all(feature ="h7", feature = "usb"))] {
        pub mod usb_h7 as usb;
    }
}

// todo: should `access_global` be removed from this? It has nothing to do with STM32.
/// Reduce boilerplate for getting mutable static global variants. Eg for use
/// in interrupts.
#[macro_export]
macro_rules! access_global {
    ($NAME_GLOBAL:ident, $name_local:ident, $cs:expr) => {
        let mut part1 = $NAME_GLOBAL.borrow($cs).borrow_mut();
        let $name_local = part1.as_mut().unwrap();
    };
}
