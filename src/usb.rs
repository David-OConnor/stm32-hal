//! USB support, including for simulated COM ports. This module is a thin wrapper required to work with
//! the `usbd` crate.
//! Requires the `usb` feature.
//! Only used on F303, L4x2, and L4x3. Other families that use USB use the `usb_otg` module.

use crate::{
    pac::{PWR, RCC, USB},
    rcc_en_reset,
};
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
    const DP_PULL_UP_FEATURE: bool = true; // todo: What should this be? try false?
                                           // todo: L4xx hal has it true. f3xx has false.

    // Pointer to the endpoint memory
    // todo: This is the L4 setting. Is this right?
    // L4 Reference manual, Table 2. USB SRAM is on APB1, at this address:
    const EP_MEMORY: *const () = 0x4000_6c00 as _;

    // Endpoint memory access scheme. Check RM.
    // Set to `true` if "2x16 bits/word" access scheme is used, otherwise set to `false`.
    const EP_MEMORY_ACCESS_2X16: bool = true;

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
                    rcc_en_reset!(apb1, usb, rcc);
                } else if #[cfg(feature = "l4x3")] {
                    // todo: rstr absent or missing in Pac for L4x3. Present in RM.
                    rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());

                    let rstr_val = rcc.apb1rstr1.read().bits();
                    rcc.apb1rstr1.modify(|_, w| unsafe { w.bits(rstr_val | (1 << 26)) }); // Set bit 26
                    rcc.apb1rstr1.modify(|_ ,w| unsafe { w.bits(rstr_val & !(1 << 26)) }); // Clear bit 26
                } else if #[cfg(feature = "l4x2")] {
                    rcc_en_reset!(apb1, usbfs, rcc);
                }
                else { // G
                    rcc_en_reset!(apb1, usb, rcc);
                }
            }
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        cortex_m::asm::delay(120);
    }
}

/// Type of the UsbBus
///
/// As this MCU family has only USB peripheral,
/// this is the only possible concrete type construction.
pub type UsbBusType = UsbBus<Peripheral>;

/// Enables the Vdd USB power supply
pub fn enable_usb_pwr(pwr: &mut PWR, rcc: &mut RCC) {
    // Enable PWR peripheral
    // todo: This breaks the RTC. Why did you put it there? And isn't it pwren?
    // rcc_en_reset!(apb1, pwr, rcc);

    // Enable VddUSB
    pwr.cr2.modify(|_, w| w.usv().set_bit());
}
