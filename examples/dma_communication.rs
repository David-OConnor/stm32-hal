//! This (WIP) example demonstrates how to communicate using Spi and Uart with
//! an external peripheral using Direct Memory Access (DMA)

//! todo: Only a skeleton: Missing critical content.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{self, free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    adc::{Adc, AdcChannel, Align, CkMode, InputType, OperationMode},
    clocks::Clocks,
    dma::{Dma, DmaChannel},
    gpio::{AltFn, Edge, GpioA, GpioAPin, GpioB, GpioBPin, PinMode, PinNum},
    low_power, pac,
    prelude::*,
    spi::{self, Spi, SpiDevice},
    usart::{Usart, UsartConfig, UsartDevice, UsartInterrupt},
};

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

    // Set up ports for GpioA and GpioB.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // Configure DMA, to be used by peripherals.
    let mut dma = Dma::new(&mut dp.DMA1, &mut dp.RCC);

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    // todo: Configure SPI pins

    // Set up an SPI peripheral, running at 4Mhz, in SPI mode 0.
    let spi = Spi::new(
        dp.SPI1,
        SpiDevice::One,
        spi_mode,
        4_000_000,
        &clock_cfg,
        &mut dp.RCC,
    );

    // Configure pins for UART.
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));

    // Set up a UART peripheral.
    // Setup UART for connecting to the host
    let mut uart = Usart::new(
        dp.USART1,
        UsartDevice::One,
        9_600,
        UsartConfig::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    // Unmask interrupt lines associated with the input pins we've configured interrupts
    // for in `setup_pins`.
    unsafe {
        // NVIC::unmask(interrupt::EXTI3);
    }

    loop {
        uart.write_dma(buff, &mut dma)

        // low_power::sleep_now(&mut SCB);
    }
}

// #[interrupt]
// /// Interrupt handler for PB3. This ISR is called when this push button goes low.
// fn EXTI3() {
//     free(|cs| {
//         // Clear the interrupt flag, to prevent continous firing.
//         unsafe { (*EXTI::ptr()).pr1.modify(|_, w| w.pr3().bit(true)) }
//
//         // A helper macro to access the pin we stored in a mutex.
//         access_global!(EXAMPLE_OUTPUT, example_output, cs);
//
//         // Set a pin high;
//         example_output.set_high();
//     });
// }
