//! Support for Controller Area Network (CAN) bus. Thinly wraps the [bxCAN](https://docs.rs/bxcan/0.5.0/bxcan/)
//! or [can-fd](https://crates.io/keywords/can-fd) libraries.
//!
//! Requires the `can_bx` or `can_fd_g[h]` features. F3, F4, and L4 use BX CAN. G0, G4, L5, and H7 use FD CAN.

use crate::{pac::RCC, util::rcc_en_reset};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "f3")] {
        use bxcan;
        use crate::pac::{can, CAN};

    } else if #[cfg(any(feature = "f4", feature = "l4"))] {
        use bxcan;
        // todo: F4 has CAN2 as well.
        use crate::pac::{can1 as can, CAN1 as CAN};
    } else if #[cfg(feature = "g4")]{
        use fdcan;
        use crate::pac::{fdcan as can, FDCAN1 as CAN};
    } else { // eg G0, H7
        use fdcan;
        // todo: CAN2 on H7.
        use crate::pac::{fdcan1 as can, FDCAN1 as CAN};
    }
}

/// Interface to the CAN peripheral.
pub struct Can {
    pub regs: CAN,
}

impl Can {
    /// Initialize a CAN peripheral, including  enabling and resetting
    /// its RCC peripheral clock. This is not handled by the `bxcan` or `canfd` crates.
    pub fn new(regs: CAN) -> Self {
        let rcc = unsafe { &*RCC::ptr() };

        cfg_if! {
            if #[cfg(feature = "f3")] {
                rcc_en_reset!(apb1, can, rcc);
            } else if #[cfg(any(feature = "f4", feature = "l4"))] {
                rcc_en_reset!(apb1, can1, rcc);
            } else if #[cfg(feature = "h7")]{
                // We don't yet have apb1h support in `rcc_en_reset`.
                rcc.apb1henr.modify(|_, w| w.fdcanen().set_bit());
                rcc.apb1hrstr.modify(|_, w| w.fdcanrst().set_bit());
                rcc.apb1hrstr.modify(|_, w| w.fdcanrst().clear_bit());

            } else {
                rcc_en_reset!(apb1, fdcan, rcc);
            }
        }

        Self { regs }
    }
}

// Implement the traits required for the `bxcan` or `fdcan` library.
cfg_if! {
    if #[cfg(feature = "bx_can")] {
        unsafe impl bxcan::Instance for Can {
            const REGISTERS: *mut bxcan::RegisterBlock = CAN::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can {
            #[cfg(any(feature = "f3", feature = "f4"))]
            const NUM_FILTER_BANKS: u8 = 28;
            #[cfg(any(feature = "f4", feature = "l4"))]
            const NUM_FILTER_BANKS: u8 = 14;
        }

        unsafe impl bxcan::MasterInstance for Can {}
    } else {
        unsafe impl fdcan::Instance for Can {
            const REGISTERS: *mut fdcan::RegisterBlock = CAN::ptr() as *mut _;
        }
        unsafe impl fdcan::message_ram::Instance for Can {
            #[cfg(feature = "g4")]
            const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_ac00 as *mut _);
            #[cfg(feature = "h7")]
            const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_ac00 as *mut _);
            // todo: (0x4000_ac00 + 0x1000) for H7, CAN2.
            // todo: (0x4000_a750 as *mut _) for G4, CAN2
            // todo: (0x4000_aaa0 as *mut _) fir G4m CAN3.
        }
    }
}
