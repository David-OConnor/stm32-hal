//! Read and write onboard flash memory. Erase pages (sectors on H7), write data,
//! and read data.
//!
//! Before using this module, check the datasheet and/or RM for your specific
//! STM32 variant for page [sector] size, and number of pages [sectors] available.
//! Make sure not to write to a page your MCU doesn't have, or that includes your
//! program's memory.

use cfg_if::cfg_if;

// Note that L5 code is simialr to other families like L4 and G4, but splits many options into
// 2 sets; one for secure, one for nonsecure.

const BANK1_START_ADDR: usize = 0x0800_0000;

cfg_if! {
    if #[cfg(any(feature = "l5", feature = "g473", feature = "g474", feature = "g483", feature = "g484"))] {
        const PAGE_SIZE_SINGLE_BANK: usize = 4_096;
        const PAGE_SIZE_DUAL_BANK: usize = 2_048;
        const BANK2_START_ADDR: usize = 0x0804_0000;
    } else if #[cfg(feature = "h7")]{
        const SECTOR_SIZE: usize = 0x2_0000;
        const BANK2_START_ADDR: usize = 0x0810_0000;
    } else {
        const PAGE_SIZE: usize = 2_048;
        #[allow(dead_code)]  // bank arg on single-bank MCUs.
        const BANK2_START_ADDR: usize = 0x0804_0000;
    }
}

cfg_if! {
    if #[cfg(feature = "l5")] {
        mod trustzone;
        pub use trustzone::*;
    } else {
        mod non_trustzone;
        pub use non_trustzone::*;
    }
}
