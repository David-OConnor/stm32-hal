#![no_std]

#[cfg(not(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f3x4",
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
    feature = "stm32l552",
    feature = "stm32l562",
    feature = "stm32h743",
    feature = "stm32h743v",
    feature = "stm32h747cm4",
    feature = "stm32h747cm7",
    feature = "stm32h753",
    feature = "stm32h753v",
    feature = "stm32h7b3",
)))]
compile_error!("This crate requires an MCU-specifying feature to be enabled. eg `stm32l552`.");

// F3 PAC
#[cfg(feature = "stm32f301")]
pub use stm32f3::stm32f301 as pac;

#[cfg(feature = "stm32f302")]
pub use stm32f3::stm32f302 as pac;

#[cfg(feature = "stm32f303")]
pub use stm32f3::stm32f303 as pac;

#[cfg(feature = "stm32f373")]
pub use stm32f3::stm32f373 as pac;

#[cfg(feature = "stm32f3x4")]
pub use stm32f3::stm32f3x4 as pac;

// L4 PAC
#[cfg(feature = "stm32l4x1")]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(feature = "stm32l4x2")]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(feature = "stm32l4x3")]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(feature = "stm32l4x5")]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(feature = "stm32l4x6")]
pub use stm32l4::stm32l4x6 as pac;

// L5 PAC
#[cfg(feature = "stm32l552")]
pub use stm32l5::stm32l552 as pac;

#[cfg(feature = "stm32l562")]
pub use stm32l5::stm32l562 as pac;

// H7 PAC
#[cfg(feature = "stm32h743")]
pub use stm32h7::stm32h743 as pac;

#[cfg(feature = "stm32h743v")]
pub use stm32h7::stm32h743v as pac;

#[cfg(feature = "stm32h747cm4")]
pub use stm32h7::stm32h747cm4 as pac;

#[cfg(feature = "stm32h747cm7")]
pub use stm32h7::stm32h747cm7 as pac;

#[cfg(feature = "stm32h753")]
pub use stm32h7::stm32h753 as pac;

#[cfg(feature = "stm32h753v")]
pub use stm32h7::stm32h753v as pac;

#[cfg(feature = "stm32h7b3")]
pub use stm32h7::stm32h7b3 as pac;

// todo: U5

pub mod clocks;
pub mod dac;
pub mod delay;
pub mod flash;
pub mod i2c;
// pub mod low_power;
pub mod rtc;
pub mod timer;
