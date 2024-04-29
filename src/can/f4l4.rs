use cfg_if::cfg_if;

use bxcan;
// todo: F4 has CAN2 as well.
use crate::pac::CAN1 as CAN;
use crate::{pac::RCC, util::rcc_en_reset};

/// Interface to the CAN peripheral.
pub struct Can {
    pub regs: CAN,
}

impl Can {
    /// Initialize a CAN peripheral, including  enabling and resetting
    /// its RCC peripheral clock. This is not handled by the `bxcan` or `canfd` crates.
    pub fn new(regs: CAN) -> Self {
        let rcc = unsafe { &*RCC::ptr() };

        rcc_en_reset!(apb1, can1, rcc);
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
            #[cfg(feature = "f4")]
            const NUM_FILTER_BANKS: u8 = 28;
            #[cfg(any(feature = "l4"))]
            const NUM_FILTER_BANKS: u8 = 14;
        }

        unsafe impl bxcan::MasterInstance for Can {}
    }
}
