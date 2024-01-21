//! Demonstrates use of the FD-CAN peripheral, using the `can-fd` crate.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{delay::Delay, interrupt, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::{ExtendedId, Id},
    interrupt::{Interrupt, InterruptLine},
    FdCan, NormalOperationMode,
};
use hal::{
    can::{self, Can},
    clocks::Clocks,
    gpio::{self, Edge, OutputSpeed, Pin, PinMode, PinState, Port},
    pac,
};

// Code shortener to isolate typestate syntax.
type Can_ = FdCan<Can, NormalOperationMode>;

/// An example function to set up the pins that don't need to be interacted with directly later.
/// For example, ones used with buses (eg I2C, SPI, UART), USB, ADC, and DAC pins.
/// This may also include input pins that trigger interrupts, and aren't polled.
pub fn setup_pins() {
    let mut can_rx = Pin::new(Port::A, 11, PinMode::Alt(9));
    let mut can_tx = Pin::new(Port::A, 12, PinMode::Alt(9));

    can_tx.output_speed(OutputSpeed::VeryHigh);
    can_rx.output_speed(OutputSpeed::VeryHigh);
}

pub fn setup_can(can_pac: pac::FDCAN1) -> Can_ {
    // todo: CAN clock cfg. Can be on PCLK1 (170Mhz), orPLLQ. (Should be able to
    // todo get a custom speed there)
    let mut can = FdCan::new(Can::new(can_pac)).into_config_mode();

    // Kernel Clock 170MHz, Bit rate: 1MBit/s, Sample Point 87.5%
    // An example of where to get values: http://www.bittiming.can-wiki.info/
    // These depicated are for 1Mbit/s, using a 170Mhz clock.

    // Some values from https://dronecan.github.io/Specification/8._Hardware_design_recommendations/
    let nominal_bit_timing = config::NominalBitTiming {
        prescaler: NonZeroU16::new(10).unwrap(),
        seg1: NonZeroU8::new(14).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    let data_bit_timing = config::DataBitTiming {
        prescaler: NonZeroU8::new(10).unwrap(),
        seg1: NonZeroU8::new(14).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
        transceiver_delay_compensation: true,
    };

    can.set_protocol_exception_handling(false);
    can.set_nominal_bit_timing(nominal_bit_timing);
    can.set_data_bit_timing(data_bit_timing);

    can.set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );

    can.set_extended_filter(
        ExtendedFilterSlot::_0,
        ExtendedFilter::accept_all_into_fifo0(),
    );

    let can_cfg = can
        .get_config()
        .set_frame_transmit(config::FrameTransmissionConfig::AllowFdCanAndBRS);

    can.apply_config(can_cfg);

    can.enable_interrupt(Interrupt::RxFifo0NewMsg);
    can.enable_interrupt_line(InterruptLine::_0, true);

    can.into_normal()
}

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    // Note: Make sure your clocks are set up for CAN. On G4 the default setup should work.
    // On H7, you may need to explicitly enable PLLQ1l, which is the default can clock source.
    // You can change the can clock source using the config as well. For example:
    //
    //  let clock_cfg = Clocks {
    //     pll_src: PllSrc::Hse(16_000_000),
    //     pll1: PllCfg {
    //         divm: 8,
    //         pllq_en: true, // PLLQ for CAN clock. Its default div of 8 is fine.
    //         ..Default::default()
    //     }
    // };

    if clock_cfg.setup().is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Call a function we've made to help organize our pin setup code.
    setup_pins();

    let mut can = setup_can(dp.FDCAN1);

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    println!("Starting send loop; periodically send a message.");

    let tx_buf = [0, 1, 2, 3];
    let mut rx_buf = [0; 4];

    loop {
        let id = Id::Extended(ExtendedId::new(1).unwrap());

        let frame_header = TxFrameHeader {
            len: tx_buf.len(),
            frame_format: FrameFormat::Fdcan,
            id,
            bit_rate_switching: true,
            marker: None,
        };

        if let Err(e) = can.transmit(frame_header, &tx_buf) {
            println!("Error node");
        } else {
            println!("Success node");
        }

        delay.delay_ms(500);

        // An example of receive syntax:
        let rx_result = can.receive0(&mut rx_buf);

        match rx_result {
            Ok(r) => {
                println!("Ok");
                println!("Rx buf: {:?}", rx_buf);
            }
            Err(e) => {
                // println!("error");
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
