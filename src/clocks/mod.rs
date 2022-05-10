//! This module contains clock configurations for various MCUs. They tend to be significantly
//! different from one another, so we've feature-gated these files, rather than
//! code within the files, to differentiate families.
//!
//! See STM32CubeIDE for an interactive editor that's very useful for seeing what
//! settings are available, and validating them.
//!
//! See the Reference Manuals for non-interactive visualizations.

cfg_if::cfg_if! {
    if #[cfg(any(feature = "f3", feature = "f4"))] {
        mod f;
        pub use f::*;
    } else if #[cfg(any(feature = "l4", feature = "l5", feature = "g0", feature = "g4", feature = "wb", feature = "wl"))] {
        mod baseline;
        pub use baseline::*;
    } else if #[cfg(feature = "u5")] {
        // todo once SVD is out
    } else if #[cfg(feature = "h7")] {
        mod h7;
        pub use h7::*;
    }
}

// todo: Consider merging the modules into a single file: There's more similar than different.
// todo: You have a good deal of DRY atm between modules.

// Dat structures and functions that are shared between clock modules go here.

// todo: Continue working through DRY between the clock modules.

/// Speed out of limits.
#[derive(Debug)]
pub struct SpeedError {
    pub error_msg: &'static str,
}

impl SpeedError {
    pub(crate) fn new(error_msg: &'static str) -> Self {
        Self { error_msg }
    }
}

// #[derive(Clone, Copy)]
// #[repr(u8)]
// pub enum ClocksValid {
//     Valid,
//     NotValid,
// }
