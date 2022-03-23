//! This example demonstrates serial communication with a PC.

//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::{self, interrupt::free, peripheral::NVIC};

use cortex_m_rt::entry;

use stm32_hal2::{
    clocks::{self, Clocks, CrsSyncSrc},
    gpio::{Pin, PinMode, Port},
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

    let mut clock_cfg = Clocks::default();

    // Enable the HSI48 oscillator, so we don't need an external oscillator, and
    // aren't restricted in our PLL config.
    clock_cfg.hsi48_on = true;
    clock_cfg.clk48_src = Clk48Src::Hsi48;

    clock_cfg.setup().unwrap();

    // Enable the Clock Recovery System, which improves HSI48 accuracy.
    clocks::enable_crs(CrsSyncSrc::Usb);

    // Enable `pwren`. Note that this is also set up by the `rtc` initialization, so this
    // step isn't required if you have the RTC set up.
    dp.RCC.apb1enr1.modify(|_, w| w.pwren().set_bit());
    // Enable USB power, on applicable MCUs like L4.
    usb::enable_usb_pwr(&mut dp.PWR, &mut dp.RCC);

    // Set up USB pins.
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

    // todo: Interrupt.
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

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
