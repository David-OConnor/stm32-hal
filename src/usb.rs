//! USB support, including for simulated COM ports. This module is a thin wrapper required to work with
//! the `usbd` crate.
//! Requires the `usb` feature.

// Based on `stm32f3xx-hal`


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
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();

    // Embedded pull-up resistor on USB_DP line
    const DP_PULL_UP_FEATURE: bool = true; // todo: What should this be?

    // Pointer to the endpoint memory
    const EP_MEMORY: *const () = 0x4000_6000 as _;

    // Endpoint memory access scheme. Check RM.
    // Set to `true` if "2x16 bits/word" access scheme is used, otherwise set to `false`.
    const EP_MEMORY_ACCESS_2X16: bool = true;
    // #[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
    // const EP_MEMORY_SIZE: usize = 512;
    // #[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]

    // f303 subvariants have diff mem sizes and bits/word scheme. :/

    // todo: Is there a way to pass `EP_MEMORY_SIZE` as an arg?

    // Endpoint memory size in bytes
    #[cfg(feature = "f3")]
    const EP_MEMORY_SIZE: usize = 512;
    // todo: Feature-gate various memory sizes

    #[cfg(feature = "l4")]
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
                }  else if #[cfg(feature = "l4x3")] {  // todo: rstr fuekd missing in Pac for L4x3.
                    rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());

                    let rstr_val = rcc.apb1rstr1.read().bits();
                    rcc.apb1rstr1.modify(|_, w| unsafe { w.bits(rstr_val | (1 << 26)) }); // Set bit 26
                    rcc.apb1rstr1.modify(|_ ,w| unsafe { w.bits(rstr_val & !(1 << 26)) }); // Clear bit 26
                } else {
                    rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());
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
