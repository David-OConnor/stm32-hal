cfg_if::cfg_if! {
    if #[cfg(feature = "f3")] {
        mod f3;
        pub use f3::*;
    } else if #[cfg(feature = "l4")] {
        mod l4;
        pub use l4::*;
    } else if #[cfg(feature = "l5")] {
        mod l5;
        pub use l5::*;
    }
}

/// This trait allows you to return information about a common's speeds.
/// It's used for configuring peripherals.
pub trait ClockCfg {
    /// System clock speed, in Hz.
    fn sysclk(&self) -> u32;

    /// HCLK speed, in Hz. Ie AHB bus, core, memory, and DMA.
    fn hclk(&self) -> u32;

    /// Cortex System timer speed, in Hz.
    fn systick(&self) -> u32;

    /// USB speed, in Hz.
    fn usb(&self) -> u32;

    /// APB1 peripheral common speed, in Hz.
    fn apb1(&self) -> u32;

    /// APB1 timer common speed, in Hz.
    fn apb1_timer(&self) -> u32;

    /// APB2 timer common speed, in Hz.
    fn apb2(&self) -> u32;

    /// APB2 peripheral common speed, in Hz.
    fn apb2_timer(&self) -> u32;

    /// Validate that the clocks speeds are all within the acceptable range
    /// for the MCU
    fn validate_speeds(&self) -> Validation; // todo Separate USB validation?
}

// todo: Continue working through DRY between the clock modules.
/// Is a set of speeds valid?
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Validation {
    Valid,
    NotValid,
}

/// Speed out of limits.
pub struct SpeedError {}
