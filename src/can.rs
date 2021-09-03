//! Support for Controller Area Network (CAN) bus. Thinly wraps the [bxCAN library](https://docs.rs/bxcan/0.5.0/bxcan/).
//! Note that this is currently for bxCAN only; different from the `fdCAN` used on newer families
//!
//! Requires the `can` feature.

// todo: Decouble from the `bxCAN` crate, remove the feature, and add fdCAN support.

use bxcan;
use core::ops::Deref;

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
};

#[cfg(feature = "f3")]
use crate::pac::can;
#[cfg(not(feature = "f3"))]
use crate::pac::can1 as can;

use cfg_if::cfg_if;

#[cfg(feature = "f4")]
#[derive(Clone, Copy)]
/// Specify the CAN device to use. Used internally for setting the appropriate APB.
pub enum CanDevice {
    One,
    Two,
}

/// Interface to the CAN peripheral.
pub struct Can<R> {
    pub regs: R,
}

impl<R> Can<R>
where
    R: Deref<Target = can::RegisterBlock>,
{
    #[cfg(not(feature = "f4"))]
    /// Creates a CAN interface.
    pub fn new<P>(regs: R, rcc: &mut RCC) -> Self {
        #[cfg(feature = "f3")]
        rcc_en_reset!(apb1, can, rcc);
        #[cfg(feature = "l4")]
        rcc_en_reset!(apb1, can1, rcc);

        Self { regs }
    }

    #[cfg(feature = "f4")]
    /// Creates a CAN interface.
    pub fn new(regs: R, device: CanDevice, rcc: &mut RCC) -> Self {
        match device {
            CanDevice::One => {
                rcc_en_reset!(apb1, can1, rcc);
            }
            CanDevice::Two => {
                rcc_en_reset!(apb1, can2, rcc);
            }
        }

        Self { regs }
    }
}

// todo: F3 calls it "CAN", and F4 has 2 CANs.

cfg_if! {
    if #[cfg(feature = "f3")] {
        unsafe impl bxcan::Instance for Can<pac::CAN> {
            const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can<pac::CAN> {
            const NUM_FILTER_BANKS: u8 = 14; // QC
        }

        unsafe impl bxcan::MasterInstance for Can<pac::CAN> {}
    } else if #[cfg(feature = "f4")] {
        unsafe impl bxcan::Instance for Can<pac::CAN1> {
            const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN1::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can<pac::CAN1> {
            const NUM_FILTER_BANKS: u8 = 14;  // QC
        }

        unsafe impl bxcan::MasterInstance for Can<pac::CAN1> {}

        unsafe impl bxcan::Instance for Can<pac::CAN2> {
            const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN2::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can<pac::CAN2> {
            const NUM_FILTER_BANKS: u8 = 14;  // QC
        }

        unsafe impl bxcan::MasterInstance for Can<pac::CAN2> {}
    } else { // L4
        unsafe impl bxcan::Instance for Can<pac::CAN1> {
            const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN1::ptr() as *mut _;
        }

        unsafe impl bxcan::FilterOwner for Can<pac::CAN1> {
            const NUM_FILTER_BANKS: u8 = 14;
        }

        unsafe impl bxcan::MasterInstance for Can<pac::CAN1> {}
    }
}
