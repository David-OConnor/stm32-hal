//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::{interrupt::free, peripheral::NVIC};

use cortex_m_rt::entry;

use stm32_hal::{
    clocks::Clocks,
    gpio::GpioA,
    pac,
    usb::{Peripheral, UsbBus, UsbBusType},
};

use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    if clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Enable the GPIOA port.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);

    // Set up USB pins.
    let _usb_dm = gpioa.new_pin(PinNum::P11, PinMode::Alt(AltFn::Af14));
    let _usb_dp = gpioa.new_pin(PinNum::P12, PinMode::Alt(AltFn::Af14));

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
                serial.write(&[1, 2, 3]).ok();
            }
            Err(_) => {
                //...
            }
        }
    }
}
