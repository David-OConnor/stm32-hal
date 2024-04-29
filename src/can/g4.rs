use crate::pac::{FDCAN1, FDCAN2, FDCAN3};
use crate::{pac::RCC, util::rcc_en_reset};
use fdcan;

macro_rules! create_cans {
    ($can:ident, $fdcan:ident, $msg_ram_address:literal ) => {
        /// Interface to the CAN peripheral.
        pub struct $can {
            pub regs: $fdcan,
        }
        impl $can {
            /// Initialize a CAN peripheral, including  enabling and resetting
            /// its RCC peripheral clock. This is not handled by the `bxcan` or `canfd` crates.
            pub fn new(regs: $fdcan) -> Self {
                let rcc = unsafe { &*RCC::ptr() };

                rcc_en_reset!(apb1, fdcan, rcc);

                Self { regs }
            }

            /// Print the (raw) contents of the status register.
            pub fn read_status(&self) -> u32 {
                unsafe { self.regs.psr.read().bits() }
            }
        }
        unsafe impl fdcan::Instance for $can {
            const REGISTERS: *mut fdcan::RegisterBlock = $fdcan::ptr() as *mut _;
        }

        unsafe impl fdcan::message_ram::Instance for $can {
            // G4 RM, table 3. "Series memory map and peripheral register boundary
            // addresses". CAN message RAM: 3 listings, 1kb each:
            const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = ($msg_ram_address as *mut _);
        }
    };
}

create_cans!(Can1, FDCAN1, 0x4000_a400);
create_cans!(Can2, FDCAN2, 0x4000_a800);
create_cans!(Can3, FDCAN3, 0x4000_ac00);
