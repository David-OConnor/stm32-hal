use cfg_if::cfg_if;

use crate::pac::{can, CAN};
use crate::{pac::RCC, util::rcc_en_reset};
use bxcan;

/// Interface to the CAN peripheral.
pub struct Can {
    pub regs: CAN,
}

impl Can {
    /// Initialize a CAN peripheral, including  enabling and resetting
    /// its RCC peripheral clock. This is not handled by the `bxcan` or `canfd` crates.
    pub fn new(regs: CAN) -> Self {
        let rcc = unsafe { &*RCC::ptr() };

        rcc_en_reset!(apb1, can, rcc);

        Self { regs }
    }

    /// Print the (raw) contents of the status register.
    pub fn read_status(&self) -> u32 {
        unsafe { self.regs.msr.read().bits() }
    }
}

// Implement the traits required for the `bxcan` or `fdcan` library.
cfg_if! {
    if #[cfg(feature = "can_bx")] {
        unsafe impl bxcan::Instance for Can {
            const REGISTERS: *mut bxcan::RegisterBlock = CAN::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can {
            const NUM_FILTER_BANKS: u8 = 28;
        }

        unsafe impl bxcan::MasterInstance for Can {}
    }
}
