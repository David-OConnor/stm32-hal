//! Hardware delays, using Cortex-m systick. Thin wrapper of `cortex-m::delay::Delay`.

// todo: If EH dependence on cortex_m::delay::Delay is removed, delete this module, and
// todo direct examples there. https://github.com/rust-embedded/cortex-m/issues/343
// todo PR: https://github.com/rust-embedded/cortex-m/pull/344

use cast::u32;
use cortex_m::{self, peripheral::SYST};

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use crate::traits::ClockCfg;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    cortex_m_delay: cortex_m::delay::Delay,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new<C: ClockCfg>(syst: SYST, clock_cfg: &C) -> Self {
        Self {
            cortex_m_delay: cortex_m::delay::Delay::new(syst, clock_cfg.systick()),
        }
    }

    /// Delay using the Cortex-M systick for a certain duration, µs. This is the core delay
    /// code all other functions, including the EH trait ones call indirectly.
    pub fn delay_us(&mut self, us: u32) {
        self.cortex_m_delay.delay_us(us);
    }

    /// Delay using the Cortex-M systick for a certain duration, ms.
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

/// Delay, with sSstem timer (SysTick) as the provider.
impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        Delay::delay_ms(self, u32(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        Delay::delay_ms(self, u32(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us);
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        Delay::delay_us(self, u32(us));
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        Delay::delay_us(self, u32(us));
    }
}
