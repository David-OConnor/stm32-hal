//! Support for Controller Area Network (CAN) bus. Thinly wraps the [bxCAN](https://docs.rs/bxcan/0.5.0/bxcan/)
//! or [can-fd](https://crates.io/keywords/can-fd) libraries.
//!
//! Requires the `can_bx` or `can_fd_g[h]` features. F3, F4, and L4 use BX CAN. G0, G4, L5, and H7 use FD CAN.

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "f3")] {
        pub mod f3;
        pub use f3::*;
    } else if #[cfg(any(feature = "f4", feature = "l4"))] {
        pub mod f4l4;
        pub use f4l4::*;
    } else if #[cfg(feature = "g4")]{
        pub mod g4;
        pub use g4::*;
    } else { // eg G0, H7
        pub mod baseline;
        pub use baseline::*;
    }
}

// todo: H5 support.
