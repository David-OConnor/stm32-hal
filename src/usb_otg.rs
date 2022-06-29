//! USB support, including for simulated COM ports. This module is a thin wrapper required to work with
//! the `usbd` crate.
//!
//! Requires the `usbotg_fs` or `usbotg_hs` features.
//! Used on F4, L4x5, L4x6, and H7. Others use the `usb` module.

// Based on `stm3h7xx-hal`

use crate::{
    clocks::Clocks,
    gpio::Pin,
    pac::{self, PWR, RCC},
};

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub struct Usb1 {
    pub usb_global: pac::OTG1_HS_GLOBAL,
    pub usb_device: pac::OTG1_HS_DEVICE,
    pub usb_pwrclk: pac::OTG1_HS_PWRCLK,
    pub hclk: u32,
}

impl Usb1 {
    pub fn new(
        usb_global: pac::OTG1_HS_GLOBAL,
        usb_device: pac::OTG1_HS_DEVICE,
        usb_pwrclk: pac::OTG1_HS_PWRCLK,
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

#[cfg(not(feature = "h735"))]
pub struct Usb2 {
    pub usb_global: pac::OTG2_HS_GLOBAL,
    pub usb_device: pac::OTG2_HS_DEVICE,
    pub usb_pwrclk: pac::OTG2_HS_PWRCLK,
    pub hclk: u32,
}

#[cfg(not(feature = "h735"))]
impl Usb2 {
    pub fn new(
        usb_global: pac::OTG1_HS_GLOBAL,
        usb_device: pac::OTG1_HS_DEVICE,
        usb_pwrclk: pac::OTG1_HS_PWRCLK,
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

macro_rules! usb_peripheral {
    ($USB:ident, $GLOBAL:ident, $en:ident, $rst:ident) => {
        unsafe impl Sync for $USB {}

        unsafe impl UsbPeripheral for $USB {
            const REGISTERS: *const () = pac::$GLOBAL::ptr() as *const ();

            const HIGH_SPEED: bool = true;
            const FIFO_DEPTH_WORDS: usize = 1024;
            const ENDPOINT_COUNT: usize = 9;

            fn enable() {
                let pwr = unsafe { &*PWR::ptr() };
                let rcc = unsafe { &*RCC::ptr() };

                cortex_m::interrupt::free(|_| {
                    // USB Regulator in BYPASS mode
                    pwr.cr3.modify(|_, w| w.usb33den().set_bit());

                    // Enable USB peripheral
                    rcc.ahb1enr.modify(|_, w| w.$en().set_bit());

                    // Reset USB peripheral
                    rcc.ahb1rstr.modify(|_, w| w.$rst().set_bit());
                    rcc.ahb1rstr.modify(|_, w| w.$rst().clear_bit());
                });
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

usb_peripheral! {
    Usb1, OTG1_HS_GLOBAL, usb1otgen, usb1otgrst
}
pub type Usb1BusType = UsbBus<Usb1>;

#[cfg(not(feature = "h735"))]
usb_peripheral! {
    USB2, OTG2_HS_GLOBAL, usb2otgen, usb2otgrst
}
#[cfg(not(feature = "h735"))]
pub type Usb2BusType = UsbBus<USB2>;

pub struct Usb1Ulpi {
    pub usb_global: pac::OTG1_HS_GLOBAL,
    pub usb_device: pac::OTG1_HS_DEVICE,
    pub usb_pwrclk: pac::OTG1_HS_PWRCLK,
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
    const REGISTERS: *const () = pac::OTG1_HS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = true;
    const FIFO_DEPTH_WORDS: usize = 1024;
    const ENDPOINT_COUNT: usize = 9;

    fn enable() {
        let rcc = unsafe { &*pac::RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.ahb1enr.modify(|_, w| w.usb1otgen().enabled());

            // Enable ULPI Clock
            rcc.ahb1enr.modify(|_, w| w.usb1ulpien().enabled());

            // Reset USB peripheral
            rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().set_bit());
            rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().clear_bit());
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk
    }

    fn phy_type(&self) -> synopsys_usb_otg::PhyType {
        synopsys_usb_otg::PhyType::ExternalHighSpeed
    }
}
pub type Usb1UlpiBusType = UsbBus<Usb1Ulpi>;
