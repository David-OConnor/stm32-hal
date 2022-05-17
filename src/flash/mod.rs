//! Read and write onboard flash memory.

use cfg_if::cfg_if;

// Note that L5 code is simialr to other families like L4 and G4, but splits many options into
// 2 sets; one for secure, one for nonsecure.

cfg_if::cfg_if! {
    if #[cfg(feature = "l5")] {
        mod trustzone;
        pub use trustzone::*;
    } else {
        mod non_trustzone;
        pub use non_trustzone::*;
    }
}
