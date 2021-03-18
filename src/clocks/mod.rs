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
        mod f3_4;
        pub use f3_4::*;
    } else if #[cfg(any(feature = "l4", feature = "l5"))] {
        mod l4_5;
        pub use l4_5::*;
    } else if #[cfg(feature = "u5")] {
        // todo once SVD is out
    } else if #[cfg(feature = "h7")] {
        mod h7;
        pub use h7::*;
    }
}

// Dat structures and functions that are shared between clock modules go here.

// todo: Continue working through DRY between the clock modules.

/// Speed out of limits.
pub struct SpeedError {}
