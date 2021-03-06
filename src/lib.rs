
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


pub mod delay;
pub mod clocks;
// pub mod rtc;
