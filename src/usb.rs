//! USB support, including for simulated COM ports. This module is a thin wrapper required to work with
//! the `stm32_usbd` crate.
//! Requires the `usb` feature.
//! Used on F303, L4x2, L4x3, L5, G0, and G4. F4, L4x5, L4x6 and H7 use USB use the `usb_otg` module.
//! For G0 series, only available on G0B0, G0B1, G0C1, which the PAC doesn't yet differentiate.

use crate::{
    pac::{PWR, RCC},
    rcc_en_reset,
};

#[cfg(not(feature = "g4"))]
use crate::pac::USB;
#[cfg(feature = "g4")]
use crate::pac::USB_FS_DEVICE as USB;

pub use stm32_usbd::UsbBus;
use stm32_usbd::UsbPeripheral;

use cfg_if::cfg_if;

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
    #[cfg(not(feature = "l4x3"))]
    const REGISTERS: *const () = USB::ptr() as *const ();

    // todo workaround due to an issue.
    #[cfg(feature = "l4x3")]
    const REGISTERS: *const () = 0x4000_6800 as *const ();

    // Embedded pull-up resistor on USB_DP line
    #[cfg(feature = "f3")]
    const DP_PULL_UP_FEATURE: bool = false;

    #[cfg(not(feature = "f3"))]
    const DP_PULL_UP_FEATURE: bool = true;

    // Pointer to the endpoint memory
    // todo: This is the L4 setting. Is this right?
    // L4 Reference manual, Table 2. USB SRAM is on APB1, at this address:
    #[cfg(feature = "l4")]
    const EP_MEMORY: *const () = 0x4000_6c00 as _;

    #[cfg(feature = "l5")]
    const EP_MEMORY: *const () = 0x5000_d800 as _;

    #[cfg(feature = "g0")]
    const EP_MEMORY: *const () = 0x4000_5c00 as _;

    #[cfg(feature = "g4")]
    const EP_MEMORY: *const () = 0x4000_6000 as _;

    // Endpoint memory size in bytes
    // F303 subvariants have diff mem sizes and bits/word scheme: 0B/C use 512 size and 1x16 bits/word.
    // F303D/E use 1024 bytes, and 2x16 bits/word.
    #[cfg(feature = "f3")]
    const EP_MEMORY_SIZE: usize = 512;
    // todo: Feature-gate various memory sizes

    #[cfg(any(feature = "l4", feature = "l5", feature = "g4"))]
    const EP_MEMORY_SIZE: usize = 1_024;

    #[cfg(feature = "g0")]
    const EP_MEMORY_SIZE: usize = 2_048;

    // Endpoint memory access scheme.
    // Set to `true` if "2x16 bits/word" access scheme is used, otherwise set to `false`.
    #[cfg(any(feature = "l4", feature = "l5", feature = "g4"))]
    const EP_MEMORY_ACCESS_2X16: bool = true;

    #[cfg(any(feature = "f3", feature = "g0"))]
    // F3 uses 1x16 or 2x16 depending on variant. G0 uses a 32-bit word.
    const EP_MEMORY_ACCESS_2X16: bool = false;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            cfg_if! {
                if #[cfg(feature = "l4x3")] {
                    // todo: rstr absent or missing in Pac for L4x3. Fixed in next PAC.
                    rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());

                    let rstr_val = rcc.apb1rstr1.read().bits();
                    rcc.apb1rstr1.modify(|_, w| unsafe { w.bits(rstr_val | (1 << 26)) }); // Set bit 26
                    rcc.apb1rstr1.modify(|_ , w| unsafe { w.bits(rstr_val & !(1 << 26)) }); // Clear bit 26
                } else if #[cfg(feature = "l4x2")] {
                    rcc_en_reset!(apb1, usbfs, rcc);
                } else if #[cfg(feature = "l5")] {
                    rcc.apb1enr2.modify(|_, w| w.usbfsen().set_bit());
                    rcc.apb1rstr2.modify(|_, w| w.usbfsrst().set_bit());
                    rcc.apb1rstr2.modify(|_ , w| w.usbfsrst().clear_bit());
                } else { // G0 and G4.
                    rcc_en_reset!(apb1, usb, rcc);
                }
            }
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay, around 1µs.
        // todo: Come back to this. This value current assumes 1µs
        // todo for an L4 running at full speed.
        cortex_m::asm::delay(80);
    }
}

/// Type of the UsbBus
pub type UsbBusType = UsbBus<Peripheral>;

#[cfg(any(feature = "l4", feature = "l5", feature = "g0"))]
/// Enables the Vdd USB power supply. Note that we also need to enable `PWREN` in APB1,
/// but we handle this using the RTC setup. Use a raw pointer if doing this without the RTC
/// already set up.
pub fn enable_usb_pwr(pwr: &mut PWR, rcc: &mut RCC) {
    // Enable VddUSB
    pwr.cr2.modify(|_, w| w.usv().set_bit());
}
