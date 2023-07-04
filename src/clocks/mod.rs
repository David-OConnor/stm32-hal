//! This module contains clock configurations for various MCUs. They tend to be significantly
//! different from one another, so we've feature-gated these files, rather than
//! code within the files, to differentiate families. This documentation is built for H723, and will
//! not be correct for other variants. For F series defaults, check out the [Default impl here](https://github.com/David-OConnor/stm32-hal/blob/main/src/clocks/f).
//! [Check here for other H7 variants](https://github.com/David-OConnor/stm32-hal/blob/main/src/clocks/h7.rs) For other
//! STM32 families, [look here](https://github.com/David-OConnor/stm32-hal/blob/main/src/clocks/baseline.rs).
//!
//! Alternatively, you can examine the `CLocks` structure to see which scalers are set, or generate docs locally
//! for your variant.//!
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
    } else if #[cfg(any(feature = "h5", feature = "h7"))] {
        mod h;
        pub use h::*;
    }
}

// todo: Consider merging the modules into a single file: There's more similar than different.
// todo: You have a good deal of DRY atm between modules.

// Dat structures and functions that are shared between clock modules go here.

// todo: Continue working through DRY between the clock modules.

/// Speed out of limits.
#[derive(Clone, Copy, Debug)]
pub enum RccError {
    Speed,
    Hardware,
}

// #[derive(Clone, Copy)]
// #[repr(u8)]
// pub enum ClocksValid {
//     Valid,
//     NotValid,
// }
