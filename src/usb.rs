//! Based on `stm32f3xx-hal`

//! USB peripheral
//!
//! Requires the `usb` feature.
//!
//! See [examples/usb_serial.rs] for a usage example.
//!
//! [examples/usb_serial.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.1/examples/usb_serial.rs

use crate::pac::{RCC, USB};
use stm32_usbd::UsbPeripheral;

pub use stm32_usbd::UsbBus;

/// USB Peripheral
///
/// Constructs the peripheral, which
/// than gets passed to the [`UsbBus`].
pub struct Peripheral {
    /// USB Register Block
    pub usb: USB,
    // Data Negativ Pin
    // pub pin_dm: PA11<AF14>,
    // /// Data Positiv Pin
    // pub pin_dp: PA12<AF14>,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();
    const DP_PULL_UP_FEATURE: bool = false; // todo: What should this be?
    const EP_MEMORY: *const () = 0x4000_6000 as _;
    // #[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
    // const EP_MEMORY_SIZE: usize = 512;
    // #[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]

    // f303 subvariants have diff mem sizes and bits/word scheme. :/
    #[cfg(feature = "f303")]
    const EP_MEMORY_SIZE: usize = 512;
    // todo: Feature-gate various memory sizes

    #[cfg(not(feature = "l4"))]
    const EP_MEMORY_SIZE: usize = 1024;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            cfg_if::cfg_if! {
                if #[cfg(feature = "f3")] {
                    // Enable USB peripheral
                    rcc.apb1enr.modify(|_, w| w.usben().enabled());

                    // Reset USB peripheral
                    rcc.apb1rstr.modify(|_, w| w.usbrst().reset());
                    rcc.apb1rstr.modify(|_, w| w.usbrst().clear_bit());
                } else if #[cfg(not(feature = "l4x3"))]{
                    rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());

                    // Reset USB peripheral
                    rcc.apb1rstr1.modify(|_, w| w.usbfsrst().set_bit());
                    rcc.apb1rstr1.modify(|_, w| w.usbfsrst().clear_bit());
                }
            }
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        // 72 Mhz is the highest frequency, so this ensures a minimum of 1µs wait time.
        cortex_m::asm::delay(72);
    }
}

/// Type of the UsbBus
///
/// As this MCU family has only USB peripheral,
/// this is the only possible concrete type construction.
pub type UsbBusType = UsbBus<Peripheral>;
