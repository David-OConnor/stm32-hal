//! Support for the Random Number Generator (RNG) peripheral.

use cortex_m::interrupt::free;

use crate::{
    pac::{RCC, RNG},
    rcc_en_reset,
};

use cfg_if::cfg_if;

/// Represents a RNG peripheral.
pub struct Rng {
    pub regs: RNG,
}

impl Rng {
    /// Initialize a RNG peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: RNG) -> Self {
        free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };

            cfg_if! {
                if #[cfg(feature = "g0")] {
                    rcc_en_reset!(ahb1, rng, rcc);
                } else if #[cfg(any(feature = "wb", feature = "wl"))] {
                    rcc_en_reset!(ahb3, rng, rcc);
                } else {
                    rcc_en_reset!(ahb2, rng, rcc);
                }
            }
        });

        regs.cr.modify(|_, w| w.rngen().set_bit());

        Self { regs }
    }

    /// Load a random number from the data register
    pub fn read(&mut self) -> i32 {
        // When data is not ready (DRDY=”0”) RNG_DR returns zero.
        // It is recommended to always verify that RNG_DR is different from zero. Because when it is
        // the case a seed error occurred between RNG_SR polling and RND_DR output reading (rare
        // event).
        // todo: CHeck for 0? Check for DREDY?

        self.regs.dr.read().bits() as i32
    }

    /// Return true if a reading is available.
    pub fn reading_ready(&mut self) -> bool {
        self.regs.sr.read().drdy().bit_is_set()
    }

    /// Enable an interrupt. An interrupt isgenerated when a random number is ready or when an error
    /// occurs. Therefore at each interrupt, check that: No error occured (SEIS and CEIS bits should be set
    /// to 0 in the RNG_SR register. A random number is ready. The DRDY bit must be set to 1 in the
    /// RNG_SR register.
    pub fn enable_interrupt(&mut self) {
        self.regs.cr.modify(|_, w| w.ie().set_bit());
    }
}
