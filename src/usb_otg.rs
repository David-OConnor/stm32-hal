//! USB support, including for simulated COM ports. This module is a thin wrapper required to work with
//! the `usbd` crate.
//!
//! Requires the `usbotg_fs` or `usbotg_hs` features.
//! Used on F4, L4x6, and H7. Others use the `usb` module.

// Based on `stm3h7xx-hal`

#[cfg(all(feature = "h7", feature = "usbotg_fs"))]
compile_error!("target only supports usbotg_hs feature");
#[cfg(all(not(feature = "h7"), feature = "usbotg_hs"))]
compile_error!("target only supports usbotg_fs feature");

use cfg_if::cfg_if;
pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

use crate::{
    gpio::Pin,
    pac::{self, PWR, RCC},
};

cfg_if! {
    if #[cfg(feature = "usbotg_hs")] {
        // On older H7s (H743 etc), OTG1 maps to pisn PB14 and PB15.
        type Usb1GlobalRegType = pac::OTG1_HS_GLOBAL;
        type Usb1DeviceRegType = pac::OTG1_HS_DEVICE;
        type Usb1PwrclkRegType = pac::OTG1_HS_PWRCLK;

        cfg_if!{
        // Note that on STM32H743 and related MCUs, OTG2 is known as "USB-FS", which
        // can be a bit confusing. This refers to the USB periphral on pins PA11 and PA12
        // for those MCUs. On newer ones like H723, use OTG1, which in that case, still
        // maps to PA11 and PA12.
            if #[cfg(not(any(feature = "h735", feature = "h7b3")))] {
                type Usb2RegGlobalType = pac::OTG2_HS_GLOBAL;
                type Usb2RegDeviceType = pac::OTG2_HS_DEVICE;
                type Usb2RegPwrclkType = pac::OTG2_HS_PWRCLK;
            }
        }
    } else if #[cfg(feature = "usbotg_fs")] {
        // Eg F4 and L4x6.
        type Usb1GlobalRegType = pac::OTG_FS_GLOBAL;
        type Usb1DeviceRegType = pac::OTG_FS_DEVICE;
        type Usb1PwrclkRegType = pac::OTG_FS_PWRCLK;
    }
}

pub struct Usb1 {
    pub usb_global: Usb1GlobalRegType,
    pub usb_device: Usb1DeviceRegType,
    pub usb_pwrclk: Usb1PwrclkRegType,
    pub hclk: u32,
}

impl Usb1 {
    pub fn new(
        usb_global: Usb1GlobalRegType,
        usb_device: Usb1DeviceRegType,
        usb_pwrclk: Usb1PwrclkRegType,
        hclk: u32,
    ) -> Self {
        Self {
            usb_global,
            usb_device,
            usb_pwrclk,
            hclk,
        }
    }
}

cfg_if! {
    if #[cfg(all(feature = "h7", not(any(feature = "h735", feature = "h7b3"))))] {
        pub struct Usb2 {
            pub usb_global: Usb2RegGlobalType,
            pub usb_device: Usb2RegDeviceType,
            pub usb_pwrclk: Usb2RegPwrclkType,
            pub hclk: u32,
        }

        impl Usb2 {
            pub fn new(
                usb_global: Usb2RegGlobalType,
                usb_device: Usb2RegDeviceType,
                usb_pwrclk: Usb2RegPwrclkType,
                hclk: u32,
            ) -> Self {
                Self {
                    usb_global,
                    usb_device,
                    usb_pwrclk,
                    hclk,
                }
            }
        }
    }
}

macro_rules! usb_peripheral {
    ($USB:ident, $GLOBAL:ident, $clock_enable_reg:ident, $reset_reg:ident, $en:ident, $rst:ident) => {
        unsafe impl Sync for $USB {}

        unsafe impl UsbPeripheral for $USB {
            const REGISTERS: *const () = $GLOBAL::ptr() as *const ();

            #[cfg(feature = "usbotg_fs")]
            const HIGH_SPEED: bool = false;
            #[cfg(feature = "usbotg_hs")]
            const HIGH_SPEED: bool = true;

            const FIFO_DEPTH_WORDS: usize = 1024; // <-- do something here maybe?
            const ENDPOINT_COUNT: usize = 9; // <--

            fn enable() {
                let pwr = unsafe { &*PWR::ptr() };
                let rcc = unsafe { &*RCC::ptr() };

                // USB Regulator in BYPASS mode
                #[cfg(feature = "h7")]
                // pwr.cr3.modify(|_, w| w.usbregen().set_bit()); // todo ?
                pwr.cr3.modify(|_, w| w.usb33den().set_bit());
                #[cfg(feature = "l4x6")] // this was present in the usb module
                pwr.cr2.modify(|_, w| w.usv().set_bit());
                // The f4 doesn't seem to have anything similar

                // Enable USB peripheral
                rcc.$clock_enable_reg.modify(|_, w| w.$en().set_bit());

                // Reset USB peripheral
                rcc.$reset_reg.modify(|_, w| w.$rst().set_bit());
                rcc.$reset_reg.modify(|_, w| w.$rst().clear_bit());
            }

            fn ahb_frequency_hz(&self) -> u32 {
                // For correct operation, the AHB frequency should be higher
                // than 30MHz. See RM0433 Rev 7. Section 57.4.4. This is checked
                // by the UsbBus implementation in synopsys-usb-otg.

                self.hclk
            }
        }
    };
}

cfg_if! {
    if #[cfg(any(feature = "f4", feature = "l4"))] {
        usb_peripheral! {
            Usb1, Usb1GlobalRegType, ahb2enr, ahb2rstr, otgfsen, otgfsrst
        }
    } else if #[cfg(feature = "h7")] {
        usb_peripheral! {
            Usb1, Usb1GlobalRegType, ahb1enr, ahb1rstr, usb1otgen, usb1otgrst
        }
    }
}

pub type Usb1BusType = UsbBus<Usb1>;

cfg_if! {
if #[cfg(all(feature = "h7", not(any(feature = "h735", feature = "h7b3"))))] {
    usb_peripheral! {
        Usb2, Usb2RegGlobalType, ahb1enr, ahb1rstr, usb2otgen, usb2otgrst
    }

    pub type Usb2BusType = UsbBus<Usb2>;
}
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub struct Usb1Ulpi {
            pub usb_global: Usb1GlobalRegType,
            pub usb_device: Usb1DeviceRegType,
            pub usb_pwrclk: Usb1PwrclkRegType,
            pub prec: u32, // todo: What should this be? Maybe d2ccip2 / cdccip2 ?
            pub hclk: u32,
            pub ulpi_clk: Pin,
            pub ulpi_dir: Pin,
            pub ulpi_nxt: Pin,
            pub ulpi_stp: Pin,
            pub ulpi_d0: Pin,
            pub ulpi_d1: Pin,
            pub ulpi_d2: Pin,
            pub ulpi_d3: Pin,
            pub ulpi_d4: Pin,
            pub ulpi_d5: Pin,
            pub ulpi_d6: Pin,
            pub ulpi_d7: Pin,
        }

        unsafe impl Sync for Usb1Ulpi {}


        unsafe impl UsbPeripheral for Usb1Ulpi {
            const REGISTERS: *const () = Usb1GlobalRegType::ptr() as *const ();

            const HIGH_SPEED: bool = true;
            const FIFO_DEPTH_WORDS: usize = 1024;
            const ENDPOINT_COUNT: usize = 9;

            fn enable() {
                let rcc = unsafe { &*pac::RCC::ptr() };

                // Enable USB peripheral
                rcc.ahb1enr.modify(|_, w| w.usb1otgen().enabled());

                // Enable ULPI Clock
                rcc.ahb1enr.modify(|_, w| w.usb1ulpien().enabled());

                // Reset USB peripheral
                rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().set_bit());
                rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().clear_bit());
            }

            fn ahb_frequency_hz(&self) -> u32 {
                self.hclk
            }

            fn phy_type(&self) -> synopsys_usb_otg::PhyType {
                synopsys_usb_otg::PhyType::ExternalHighSpeed
            }
        }
        pub type Usb1UlpiBusType = UsbBus<Usb1Ulpi>;
    }
}
