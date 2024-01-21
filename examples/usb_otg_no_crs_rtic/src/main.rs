#![no_std]
#![no_main]

// Not all stm32 devices have a Clock Recovery System (CRS) such as the stm32l476.
// For these device families, we can't use the CRS or clocks that depend on the
// CRS such as the HSI. See
// https://github.com/David-OConnor/stm32-hal/issues/64 for further details.
//
// The following example was run and tested on the STM32L476ZGTx varient.

use defmt_rtt as _;
use panic_probe as _;

pub mod usb {
    use hal::pac::{PWR, RCC};

    /// Enables VddUSB power supply
    pub fn enable_usb_pwr() {
        // Enable PWR peripheral
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());

        // Enable VddUSB
        let pwr = unsafe { &*PWR::ptr() };
        pwr.cr2.modify(|_, w| w.usv().set_bit());
    }
}

#[rtic::app(device = pac, dispatchers = [USART1])]
mod app {

    use cortex_m::asm;

    use hal::{
        self,
        clocks::Clocks,
        gpio::{Pin, PinMode, Port},
        pac,
    };

    use hal::clocks::Clk48Src;
    use hal::usb_otg::{Usb1, UsbBus};

    use usb_device::prelude::*;

    use usbd_serial::SerialPort;
    use usbd_serial::USB_CLASS_CDC;

    use usb_device::class_prelude::UsbBusAllocator;

    use super::*;

    pub struct PeripheralUsb {
        pub serial: SerialPort<'static, UsbBus<Usb1>>,
        pub device: UsbDevice<'static, UsbBus<Usb1>>,
    }

    #[shared]
    struct Shared {
        peripheral_usb: PeripheralUsb,
    }

    #[local]
    struct Local {}

    #[init(local = [
           usb_buf: [u32; 64] = [0; 64],
           usb_bus: Option<UsbBusAllocator<UsbBus<Usb1>>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Use the msi clock for the 48 Mhz input to the USB peripheral rather
        // than the hsi.
        let clock_cfg = Clocks {
            clk48_src: Clk48Src::Msi,
            ..Default::default()
        };
        clock_cfg.setup().unwrap();
        clock_cfg.enable_msi_48();

        usb::enable_usb_pwr();

        let _usb_dm = Pin::new(Port::A, 11, PinMode::Alt(10));
        let _usb_dp = Pin::new(Port::A, 12, PinMode::Alt(10));

        let usb1 = Usb1::new(
            cx.device.OTG_FS_GLOBAL,
            cx.device.OTG_FS_DEVICE,
            cx.device.OTG_FS_PWRCLK,
            clock_cfg.hclk(),
        );
        let buf: &'static mut _ = cx.local.usb_buf;
        let bus: &'static mut _ = cx.local.usb_bus.insert(UsbBus::new(usb1, buf));
        let serial = SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake Company")
            .product("Serial Port")
            .serial_number("SN")
            .device_class(USB_CLASS_CDC)
            .build();
        let peripheral_usb = PeripheralUsb { serial, device };

        (Shared { peripheral_usb }, Local {}, init::Monotonics())
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        usb_say_hello::spawn().unwrap();
        loop {
            asm::nop()
        }
    }

    #[task(shared = [peripheral_usb])]
    fn usb_say_hello(cx: usb_say_hello::Context) {
        defmt::println!("usb_say_hello");
        let mut peripheral_usb = cx.shared.peripheral_usb;

        peripheral_usb.lock(|PeripheralUsb { device, serial }| loop {
            if !device.poll(&mut [serial]) {
                continue;
            }

            // Something in the usb buffer. Process it.
            let mut buf = [0u8; 64];

            match serial.read(&mut buf[..]) {
                Ok(count) => {
                    // Echo back to the serial console.
                    serial.write(&buf[..count]).unwrap();
                }
                Err(UsbError::WouldBlock) => {
                    // Add error handling
                }
                Err(_err) => {
                    // Add error handling
                }
            }
        })
    }
}
