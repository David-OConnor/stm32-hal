//! Read and write onboard flash memory. Erase pages (sectors on H7), write data,
//! and read data.
//!
//! Before using this module, check the datasheet and/or RM for your specific
//! STM32 variant for page [sector] size, and number of pages [sectors] available.
//! Make sure not to write to a page your MCU doesn't have, or that includes your
//! program's memory.

use cfg_if::cfg_if;

use crate::pac::FLASH;

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
    if #[cfg(any(feature = "l5", feature = "h5"))] {
        mod trustzone;
        pub use trustzone::*;
    } else {
        mod non_trustzone;
        pub use non_trustzone::*;
    }
}

pub struct Flash {
    pub regs: FLASH,
    #[cfg(any(
        feature = "g473",
        feature = "g474",
        feature = "g483",
        feature = "g484",
        feature = "l5"
    ))]
    pub dual_bank: DualBank,
}

// todo: Is H5 more like H7, or the others?

/// Contains code common to both modules.
impl Flash {
    /// Create a struct used to perform operations on Flash.
    pub fn new(regs: FLASH) -> Self {
        cfg_if! {
            if #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484", feature = "l5"))] {
                // Some G4 variants let you select dual or single-bank mode.
                // Self { regs, dual_bank: DualBank::Single }
                Self { regs, dual_bank: DualBank::Dual } // todo: Experimenting
            } else {
                Self { regs }
            }
        }
    }

    /// Read flash memory at a given page and offset into an 8-bit-dword buffer.
    #[allow(unused_variables)] // bank arg on single-bank MCUs.
    pub fn read(&self, bank: Bank, page: usize, offset: usize, buf: &mut [u8]) {
        // H742 RM, section 4.3.8:
        // Single read sequence
        // The recommended simple read sequence is the following:
        // 1. Freely perform read accesses to any AXI-mapped area.
        // 2. The embedded Flash memory effectively executes the read operation from the read
        // command queue buffer as soon as the non-volatile memory is ready and the previously
        // requested operations on this specific bank have been served.
        cfg_if! {
            if #[cfg(any(
                feature = "g473",
                feature = "g474",
                feature = "g483",
                feature = "g484",
                feature = "l5",
            ))] {
                let mut addr = page_to_address(self.dual_bank, bank, page) as *mut u32;
            } else if #[cfg(feature = "h7")]{
                let mut addr = page_to_address(bank, page) as *mut u32;
            } else {
                let mut addr = page_to_address(page) as *mut u32;
            }
        }

        unsafe {
            // Offset it by the start position
            addr = unsafe { addr.add(offset) };
            // Iterate on chunks of 32bits
            for chunk in buf.chunks_mut(4) {
                let word = unsafe { core::ptr::read_volatile(addr) };
                let bytes = word.to_le_bytes();

                let len = chunk.len();
                if len < 4 {
                    chunk[0..len].copy_from_slice(&bytes[0..len]);
                } else {
                    chunk[0..4].copy_from_slice(&bytes);
                };

                unsafe { addr = addr.add(1) };
            }
        }
    }
}

/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
#[cfg(not(any(
    feature = "g473",
    feature = "g474",
    feature = "g483",
    feature = "g484",
    feature = "h5",
    feature = "l5",
    feature = "h7"
)))]
fn page_to_address(page: usize) -> usize {
    BANK1_START_ADDR + page * PAGE_SIZE
}

#[cfg(any(
    feature = "g473",
    feature = "g474",
    feature = "g483",
    feature = "g484",
    feature = "h5",
    feature = "l5",
))]
fn page_to_address(dual_bank: DualBank, bank: Bank, page: usize) -> usize {
    if dual_bank == DualBank::Single {
        BANK1_START_ADDR + page * PAGE_SIZE_SINGLE_BANK
    } else {
        match bank {
            Bank::B1 => BANK1_START_ADDR + page * PAGE_SIZE_DUAL_BANK,
            Bank::B2 => BANK2_START_ADDR + page * PAGE_SIZE_DUAL_BANK,
        }
    }
}

#[cfg(feature = "h7")]
/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
fn page_to_address(bank: Bank, sector: usize) -> usize {
    // Note; Named sector on H7.
    let starting_pt = match bank {
        Bank::B1 => BANK1_START_ADDR,
        // todo: This isn't the same bank2 starting point for all H7 variants!
        #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
        Bank::B2 => BANK2_START_ADDR,
    };

    starting_pt + sector * SECTOR_SIZE
}
