//! This example demonstrates serial communication with a PC.

//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::{self, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{CriticalSection, Mutex, with};
use defmt::println;
use defmt_rtt as _;
use hal::{
    clocks::{self, Clk48Src, Clocks, CrsSyncSrc, enable_crs},
    gpio::{Pin, PinMode, Port},
    pac,
    prelude::*,
    usb::{Peripheral, UsbBus, UsbBusType},
};
use panic_probe as _;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

// See note below about if using custom buffer sizes.
// pub type UsbSerial<'a> = SerialPort<'a, UsbBusType, &'a mut [u8], &'a mut [u8]>;

make_globals!(
    (USB_DEV, UsbDevice<'static, UsbBusType>),
    (USB_SERIAL, SerialPort<'static, UsbBusType>),
    // Or, if you use custom buffer sizes:
    // (USB_SERIAL, UsbSerial<'static>),
);

fn init() {
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
    // step isn't required if you have the RTC set up. Only required on some configurations,
    // , e.g. L4, L5, and G0.
    // dp.RCC.apb1enr1.modify(|_, w| w.pwren().set_bit());

    // Enable USB power, on applicable MCUs, e.g. L4, L5, and G0.
    // usb::enable_usb_pwr(&mut dp.PWR, &mut dp.RCC);

    // Set up USB pins. Note: This only applies to some MCUs; others don't require this,
    // nor have the appropriate alt functions.
    // let _usb_dm = gpioa.new_pin(11, PinMode::Alt(14));
    // let _usb_dp = gpioa.new_pin(12, PinMode::Alt(14));

    let usb = Peripheral { usb: dp.USB };
    let usb_bus = UsbBus::new(usb);

    let usb_serial = SerialPort::new(usb_bus);

    // Or, if specifying manual buffers, e.g. larger than the default:
    let usb_serial = SerialPort::new_with_store(
        unsafe { USB_BUS.as_ref().unwrap() },
        unsafe { &mut USB_TX_STORE[..] },
        unsafe { &mut USB_RX_STORE[..] },
    );

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .strings(&[StringDescriptors::default()
        .manufacturer("A Company")
        .product("Serial port")
        // We use `serial_number` to identify the device to the PC. If it's too long,
        // we get permissions errors on the PC.
        .serial_number("SN")])
    .unwrap()
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    init_globals!((USB_DEVICE, usb_device), (USB_SERIAL, usb_serial));

    setup_nvic!([(USB_FS, 1),], cp);
}

/// Handle incoming data from the USB port.
pub fn handle_rx(
    usb_serial: &mut UsbSerial<'static>,
    usb_dev: &mut UsbDevice<'static, UsbBusType>,
    rx_buf: &[u8],
    cs: CriticalSection,
) {
    // We have access to the RX buffer in this function.
    println!("Received a message over USB: {:?}", rx_buf);

    // Example sending a message:
    if rx_buf[0] == 10 {
        // Populate A/R.
        let mut tx_buf = [0; 10];

        let msg_len = 10;

        let mut offset = 0;
        while offset < msg_len {
            match usb_serial.write(&tx_buf[offset..msg_len]) {
                Ok(0) | Err(UsbError::WouldBlock) => {
                    usb_dev.poll(&mut [usb_serial]);
                }
                Ok(written) => offset += written,
                Err(e) => {
                    defmt::warn!("USB write error: {:?}", e);
                    break;
                }
            }
        }

        while usb_serial.flush().err() == Some(UsbError::WouldBlock) {
            usb_dev.poll(&mut [usb_serial]);
        }
    }
}

#[entry]
fn main() -> ! {
    init();

    loop {
        asm::nop();
    }
}

#[interrupt]
/// Interrupt handler for USB (serial)
/// Note that the name of this handler depends on the variant assigned in the associated PAC.
/// Other examples include `USB_LP` (G4), and `USB_HS` and `USB_FS` (H7)
fn USB_LP() {
    with(|cs| {
        access_globals!([(USB_DEV, dev), (USB_SERIAL, serial),], cs);

        if !dev.poll(&mut [serial]) {
            return;
        }

        let mut rx_buf = [0; 10];

        if let Ok(count) = serial.read(&mut rx_buf) {
            println!("Bytes read: {}", count);

            handle_rx(serial, dev, &rx_buf, cs);
        }
    });
}

// Same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
