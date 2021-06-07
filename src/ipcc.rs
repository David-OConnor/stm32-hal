//! Inter-processor communication controller (IPCC).
//! Used on STM32WB for communication between cores.

// todo: Finish this.

use crate::{
    pac::{self, IPCC, RCC},
    traits::ClockCfg,
};

#[derive(Clone, Copy)]
pub enum IpccChannel {
    C1,
    C2,
    C3,
    C4,
    C5,
    C6,
}

// todo: from wb HAL
// #[repr(C)]
// pub enum IpccChannel {
//     Channel1 = 0x00000001,
//     Channel2 = 0x00000002,
//     Channel3 = 0x00000004,
//     Channel4 = 0x00000008,
//     Channel5 = 0x00000010,
//     Channel6 = 0x00000020,
// }

/// Represents an Inter-Integrated Circuit (I2C) peripheral.
pub struct Ipcc {
    regs: IPCC,
}

impl Ipcc {
    /// Configures the I2C peripheral. `freq` is in Hz. Doesn't check pin config.
    pub fn new<C: ClockCfg>(regs: IPCC, rcc: &mut RCC) -> Self {
        rcc.ahb3enr.modify(|_, w| w.ipccen().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.ipccrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.ipccrst().clear_bit());

        // todo?
        // rcc.ahb4enr.modify(|_, w| w.ipccen().set_bit());
        // rcc.ahb4rstr.modify(|_, w| w.ipccrst().set_bit());
        // rcc.ahb4rstr.modify(|_, w| w.ipccrst().clear_bit());
        Self { regs }
    }
}
