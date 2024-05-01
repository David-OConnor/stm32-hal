//! Support for Controller Area Network (CAN) bus. Thinly wraps the [bxCAN](https://docs.rs/bxcan/0.5.0/bxcan/)
//! or [can-fd](https://crates.io/keywords/can-fd) libraries.
//!
//! Requires the `can_bx` or `can_fd_g[h]` features. F3, F4, and L4 use BX CAN. G0, G4, L5, and H7 use FD CAN.

use cfg_if::cfg_if;

use crate::{pac::RCC, util::rcc_en_reset};

// todo: H5 support.
cfg_if! {
    if #[cfg(feature = "f3")] {
        use bxcan;
        use crate::pac::{can, CAN};

    } else if #[cfg(any(feature = "f4", feature = "l4"))] {
        use bxcan;
        // todo: F4 has CAN2 as well.
        use crate::pac::{CAN1 as CAN};
    } else { // eg G0, H7
        use fdcan;
        // todo: CAN2 on H7.
        use crate::pac::{FDCAN1 as CAN};
    }
}

cfg_if! {
    if #[cfg(feature = "g4")] {
        pub mod g4;
        pub use g4::*;
    }else{
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

                        // set_message_ram_layout();

                    } else {
                        rcc_en_reset!(apb1, fdcan, rcc);
                    }
                }

                Self { regs }
            }

            /// Print the (raw) contents of the status register.
            pub fn read_status(&self) -> u32 {
                cfg_if! {
                    if #[cfg(any(feature = "h7", feature = "l5"))] {
                        unsafe { self.regs.psr.read().bits() }
                    } else {
                        unsafe { self.regs.msr.read().bits() }
                    }
                }
            }
        }

#[cfg(feature = "h7")]
        // todo: Troubleshooting. COpied from H7xx-hal
        /// Set the message RAM layout. This is flexible on H7. This function hard-sets it to the setting
        /// that is hard-set by hardware on G4.
        /// todo: Allow flexibility.
        ///
        /// Note: Perhaps due to a reset of message ram called by the FDCAN crate's `.into_config_mode()`,
        /// we run this in application firmware once in config mode. Although a better API would be in the constructor
        /// This must be done after initial setup (Enabling RCC clocks most-likely).
        pub fn set_message_ram_layout() {
            let regs = unsafe { &(*CAN::ptr()) };

            // RM, section 56.4.1: Operation modes: "Access to the FDCAN configuration registers is only
            // enabled when both INIT bit in FDCAN_CCCR register and CCE bit in FDCAN_CCCR register are set.
            // Note: we do this as 2 separate writes. RM: "CCE bit in FDCAN_CCCR register can only be set/cleared while INIT bit in FDCAN_CCCR
            // is set. CCE bit in FDCAN_CCCR register is automatically cleared when INIT bit in
            // FDCAN_CCCR is cleared."
            regs.cccr.modify(|_, w| w.init().set_bit());
            while regs.cccr.read().init().bit_is_clear() {}
            regs.cccr.modify(|_, w| w.cce().set_bit());
            while regs.cccr.read().cce().bit_is_clear() {}

            let mut word_addr = 0x000; // todo: 0x400 for FDCAN2?

            use fdcan::message_ram::*;

            // 11-bit filter
            regs.sidfc
                .modify(|_, w| unsafe { w.flssa().bits(word_addr) });
            word_addr += STANDARD_FILTER_MAX as u16;

            // 29-bit filter
            regs.xidfc
                .modify(|_, w| unsafe { w.flesa().bits(word_addr) });
            word_addr += 2 * EXTENDED_FILTER_MAX as u16;

            // Rx FIFO 0
            regs.rxf0c.modify(|_, w| unsafe {
                w.f0sa()
                    .bits(word_addr)
                    .f0s()
                    .bits(RX_FIFO_MAX)
                    .f0wm()
                    .bits(RX_FIFO_MAX)
            });
            word_addr += 18 * RX_FIFO_MAX as u16;

            // Rx FIFO 1
            regs.rxf1c.modify(|_, w| unsafe {
                w.f1sa()
                    .bits(word_addr)
                    .f1s()
                    .bits(RX_FIFO_MAX)
                    .f1wm()
                    .bits(RX_FIFO_MAX)
            });
            word_addr += 18 * RX_FIFO_MAX as u16;

            // Rx buffer - see below
            // Tx event FIFO
            regs.txefc.modify(|_, w| unsafe {
                w.efsa()
                    .bits(word_addr)
                    .efs()
                    .bits(TX_EVENT_MAX)
                    .efwm()
                    .bits(TX_EVENT_MAX)
            });
            word_addr += 2 * TX_EVENT_MAX as u16;

            // Tx buffers
            regs.txbc
                .modify(|_, w| unsafe { w.tbsa().bits(word_addr).tfqs().bits(TX_FIFO_MAX) });
            word_addr += 18 * TX_FIFO_MAX as u16;

            // Rx Buffer - not used
            regs.rxbc.modify(|_, w| unsafe { w.rbsa().bits(word_addr) });

            // TX event FIFO?
            // Trigger memory?

            // Set the element sizes to 16 bytes
            regs.rxesc
                .modify(|_, w| unsafe { w.rbds().bits(0b111).f1ds().bits(0b111).f0ds().bits(0b111) });
            regs.txesc.modify(|_, w| unsafe { w.tbds().bits(0b111) });
        }

        // Implement the traits required for the `bxcan` or `fdcan` library.
        cfg_if! {
            if #[cfg(feature = "can_bx")] {
                unsafe impl bxcan::Instance for Can {
                    const REGISTERS: *mut bxcan::RegisterBlock = CAN::ptr() as *mut _;
                }

                unsafe impl bxcan::FilterOwner for Can {
                    #[cfg(any(feature = "f3", feature = "f4"))]
                    const NUM_FILTER_BANKS: u8 = 28;
                    #[cfg(any(feature = "l4"))]
                    const NUM_FILTER_BANKS: u8 = 14;
                }

                unsafe impl bxcan::MasterInstance for Can {}
            } else {
                unsafe impl fdcan::Instance for Can {
                    const REGISTERS: *mut fdcan::RegisterBlock = CAN::ptr() as *mut _;
                }

                unsafe impl fdcan::message_ram::Instance for Can {
                    #[cfg(feature = "h7")]
                    // H743 RM, table 8. "Register boundary addresses". 0x4000_AC00 - 0x4000_D3FF". CAN message RAM.
                    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_ac00 as *mut _);
                    // todo: (0x4000_ac00 + 0x1000) for H7, CAN2.
                    // todo: (0x4000_a750 as *mut _) for G4, CAN2
                    // todo: (0x4000_aaa0 as *mut _) fir G4 CAN3.
                }
            }
        }
        // todo: H5 support.

    }
}
