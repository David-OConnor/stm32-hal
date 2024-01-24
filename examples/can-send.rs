//! Simple CAN-bus send example, it is inspired to the example provided by stm32f4xx-hal
//! Requires a transceiver connected to PB8, 9 (CAN1)

#![no_std]
#![no_main]

use core::panic::PanicInfo;

use bxcan::{filter::Mask32, Frame, StandardId};
use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime
use defmt_rtt as _;
use hal::{
    self,
    can::Can,
    clocks::{self, ApbPrescaler, Clocks, InputSrc, PllSrc, Pllp},
    gpio::{OutputType, Pin, PinMode, Port},
    pac,
};
use nb::block;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC;

    // this line is required if you want to take advantage of ST-Link
    hal::debug_workaround();

    // Configure the clocks accordingly with your hardware [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html).
    // In this configuration we have an external crystal of 8MHz.
    // CAN-bus peripheral is on APB1 bus, it is important to make it work properly and set the right bit timing!
    let clock_cfg = Clocks {
        input_src: InputSrc::Pll(PllSrc::Hse(8_000_000)),
        pllm: 8,
        plln: 180,
        pllp: Pllp::Div2,
        apb1_prescaler: ApbPrescaler::Div2,
        ..Default::default()
    };

    // Write the clock configuration to the MCU.
    clock_cfg.setup().unwrap();

    let mut can = {
        let mut _rx = Pin::new(Port::B, 8, PinMode::Alt(9));
        let mut _tx = Pin::new(Port::B, 9, PinMode::Alt(9));

        _rx.output_type(OutputType::PushPull);
        _tx.output_type(OutputType::PushPull);

        let can = Can::new(dp.CAN1, &mut rcc);

        bxcan::Can::builder(can)
            // APB1 (PCLK1): 45MHz, Bit rate: 1000kBit/s, Sample Point 86.7%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            .set_bit_timing(0x001b0002)
            .enable()
    };

    defmt::info!("CAN enabled!");

    let mut test: [u8; 8] = [0; 8];
    let mut count: u8 = 0;
    let id: u16 = 0x500;

    test[1] = 1;
    test[2] = 2;
    test[3] = 3;
    test[4] = 4;
    test[5] = 5;
    test[6] = 6;
    test[7] = 7;
    let test_frame = Frame::new_data(StandardId::new(id).unwrap(), test);

    loop {
        test[0] = count;
        let test_frame = Frame::new_data(StandardId::new(id).unwrap(), test);
        block!(can.transmit(&test_frame)).unwrap();
        if count < 255 {
            count += 1;
        } else {
            count = 0;
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::udf()
}
