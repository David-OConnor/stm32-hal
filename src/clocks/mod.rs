/// This trait allows you to return information about a clocks's speeds.
/// It's used for configuring peripherals.
pub trait ClockCfg {
    /// System clock speed, in Hz.
    fn sysclk(&self) -> u32;

    /// HCLK speed, in Hz. Ie AHB bus, core, memory, and DMA.
    fn hclk(&self) -> u32;

    /// Cortex System timer speed, in Hz.
    fn systick(&self) -> u32;

    /// APB1 peripheral clocks speed, in Hz.
    fn apb1(&self) -> u32;

    /// APB1 timer clocks speed, in Hz.
    fn apb1_timer(&self) -> u32;

    /// APB2 timer clocks speed, in Hz.
    fn apb2(&self) -> u32;

    /// APB2 peripheral clocks speed, in Hz.
    fn apb2_timer(&self) -> u32;
}