#![no_std]

// Some reg modifications are marked `unsafe` in some PAC crates, but not others.
// Disable these warnings.
#[allow(unused_unsafe)]
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

// todo: U5 once SVD is out.

mod traits;

pub mod prelude {
    pub use crate::traits::*;
}

pub mod adc;
pub mod clocks;
pub mod dac;
pub mod delay;
pub mod flash;
pub mod i2c;
pub mod low_power;
pub mod rtc;
pub mod spi;
pub mod timer;
