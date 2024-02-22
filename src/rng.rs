//! Support for the Random Number Generator (RNG) peripheral. This provides a simple
//! API for accessing hardware-generated 32-bit random numbers, where constructing a `Rng` peripheral enables its
//! peripheral clock, and provides some methods.
//! Once this struct is constructed, the freestanding functions `read()`, and `reading_ready()` may be
//! used to get a random number number, and check if a new one is available.

use cfg_if::cfg_if;

use crate::{
    pac::{RCC, RNG},
    util::rcc_en_reset,
};

/// Represents a RNG peripheral.
pub struct Rng {
    pub regs: RNG,
}

impl Rng {
    /// Initialize a RNG peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: RNG) -> Self {
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

/// Gets a random value without needing to pass the `Rng` struct. Assumes it has been initialized,
/// to enable it and its peripheral clock.
pub fn read() -> i32 {
    let regs = unsafe { &(*RNG::ptr()) };
    regs.dr.read().bits() as i32
}

/// Return true if a reading is available.
pub fn reading_ready() -> bool {
    let regs = unsafe { &(*RNG::ptr()) };
    regs.sr.read().drdy().bit_is_set()
}
