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
pub use stm32l4::stm32l552 as pac;

#[cfg(feature = "stm32l562")]
pub use stm32l4::stm32l562 as pac;

// todo: U5 and H7

pub mod clocks;
pub mod delay;
pub mod flash;
// pub mod i2c;
// pub mod low_power;
pub mod rtc;
pub mod timer;
