use fdcan;

#[cfg(not(feature = "g431"))]
use crate::pac::{FDCAN2, FDCAN3};
use crate::{
    pac::{FDCAN1, RCC},
    util::rcc_en_reset,
};

const MESSAGE_RAM_BASE_ADDRESS: u32 = 0x4000_a400;

// The RM is a bit contradictory. Table 3 implies that each FDCAN memory block is 0x400 in size.
// But section 44.3.3 says each block is 0x350 and that is what actually works.
const MESSAGE_RAM_SIZE: u32 = 0x350;

const FDCAN1_MESSAGE_RAM_ADDRESS: u32 = MESSAGE_RAM_BASE_ADDRESS;
const FDCAN2_MESSAGE_RAM_ADDRESS: u32 = FDCAN1_MESSAGE_RAM_ADDRESS + MESSAGE_RAM_SIZE;
const FDCAN3_MESSAGE_RAM_ADDRESS: u32 = FDCAN2_MESSAGE_RAM_ADDRESS + MESSAGE_RAM_SIZE;

macro_rules! create_cans {
    ($can:ident, $fdcan:ident, $msg_ram_address:ident ) => {
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
            const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = ($msg_ram_address as *mut _);
        }
    };
}

create_cans!(Can, FDCAN1, FDCAN1_MESSAGE_RAM_ADDRESS);
#[cfg(not(feature = "g431"))]
create_cans!(Can2, FDCAN2, FDCAN2_MESSAGE_RAM_ADDRESS);
#[cfg(not(feature = "g431"))]
create_cans!(Can3, FDCAN3, FDCAN3_MESSAGE_RAM_ADDRESS);
