//! This example demonstrates serial communication with a PC.

//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::{self, peripheral::NVIC};
use cortex_m_rt::entry;
use hal::{
    clocks::{self, Clk48Src, Clocks, CrsSyncSrc},
    gpio::{Pin, PinMode, Port},
    pac,
    usb::{Peripheral, UsbBus, UsbBusType},
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

make_globals!(
    (USB_SERIAL, SerialPort<UsbBusType>),
    (USB_DEVICE, UsbDevice<UsbBusType>),
);

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let mut clock_cfg = Clocks {
        // Enable the HSI48 oscillator, so we don't need an external oscillator, and
        // aren't restricted in our PLL config.
        hsi48_on: true,
        clk48_src: Clk48Src::Hsi48,
        ..Default::default()
    };

    clock_cfg.setup().unwrap();

    // Enable the Clock Recovery System, which improves HSI48 accuracy.
    clock_cfg::enable_crs(CrsSyncSrc::Usb);

    // Enable `pwren`. Note that this is also set up by the `rtc` initialization, so this
    // step isn't required if you have the RTC set up.
    dp.RCC.apb1enr1.modify(|_, w| w.pwren().set_bit());
    // Enable USB power, on applicable MCUs like L4.
    usb::enable_usb_pwr(&mut dp.PWR, &mut dp.RCC);

    // Set up USB pins. Note: This only applies to some MCUs; others don't require this,
    // nor have the appropriate alt functions.
    let _usb_dm = gpioa.new_pin(11, PinMode::Alt(14));
    let _usb_dp = gpioa.new_pin(12, PinMode::Alt(14));

    let usb = Peripheral { usb: dp.USB };
    let usb_bus = UsbBus::new(usb);
    let usb_serial = SerialPort::new(usb_bus);

    let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("A Company")
        .product("Serial port")
        // We use `serial_number` to identify the device to the PC. If it's too long,
        // we get permissions errors on the PC.
        .serial_number("SN")
        .device_class(USB_CLASS_CDC)
        .build();

    with(|cs| {
        USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        USB_SERIAL.borrow(cs).replace(Some(usb_serial));
    });

    unsafe {
        // USB failing to respond can lead to it being disconnected by software; use
        // a high priority.
        // Note that the interrupt name varies based on STM32 family and device.
        cp.NVIC.set_priority(pac::Interrupt::USB_FS, 1);
    }

    loop {
        // It's probably better to do this with an interrupt than polling. Polling here
        // keep the syntax simple. To use in an interrupt, set up the USB-related structs as
        // `Mutex<RefCell<Option<...>>>`, and use the `USB_LP_CAN_RX0` interrupt handler etc.
        // See the `interrupts` example.
        if !usb_device.poll(&mut [usb_serial]) {
            continue;
        }

        let mut buf = [0u8; 8];
        match usb_serial.read(&mut buf) {
            // todo: match all start bits and end bits. Running into an error using the naive approach.
            Ok(count) => {
                usb_serial.write(&[1, 2, 3]).ok();
            }
            Err(_) => {
                //...
            }
        }
    }
}

#[interrupt]
/// Interrupt handler for USB (serial)
/// Note that the name of this handler depends on the variant assigned in the associated PAC.
/// Other examples include `USB_LP` (G4), and `USB_HS` (H7)
fn USB_FS() {
    with(|cs| {
        access_global!(USB_SERIAL, usb_serial, cs);
        access_global!(USB_DEVICE, usb_device, cs);

        if !usb_device.poll(&mut [usb_serial]) {
            return;
        }

        let mut buf = [0u8; 8];

        match usb_serial.read(&mut buf) {
            Ok(_count) => match buf[0..2] {
                [100, 150] => {
                    usb_serial
                        .write(&[]) // Data to write goes in this buffer.
                        .ok();
                }
                _ => {}
            },
            Err(_) => {}
        }
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
