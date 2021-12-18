//! This module includes an overview of I2C features available.
//! It demonstrates how to configure and take readings from a TI ADS1115
//! offboard ADC.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaWriteBuf},
    gpio::{Pin, PinMode, Port},
    low_power,
    pac::{self, interrupt},
    prelude::*,
    spi::{self, BaudRate, Spi, SpiConfig, SpiMode},
};

make_globals!((SPI, Spi<SPI1>), (DMA, Dma<DMA1>),);

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    clock_cfg.setup().unwrap();

    // Configure pins for Spi
    let _sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let _miso = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _mosi = Pin::new(Port::A, 7, PinMode::Alt(5));

    let cs = Pin::new(Port::A, 1, PinMode::Output);

    let spi_cfg = SpiConfig {
        mode: SpiMode::mode1(),
        // `SpiConfig::default` is mode 0, full duplex, with software CS.
        ..Default::default()
    };
    // Alternatively, we can configure Mode py polarity and phase:
    // mode: SpiMode {
    //     polarity: SpiPolarity::IdleLow,
    //     phase: SpiPhase::CaptureOnFirstTransition,
    // }

    // Set up an SPI peripheral, running at 4Mhz, in SPI mode 0.
    let spi = Spi::new(
        dp.SPI1,
        spi_cfg,
        BaudRate::Div32, // Eg 80Mhz apb clock / 32 = 2.5Mhz SPI clock.
    );

    // Set up DMA, for nonblocking (generally faster) conversion transfers:
    let mut dma = Dma::new(&mut dp.DMA1, &dp.RCC);

    // We read 3 bytes from the `0x9f` register.
    let mut write_buf = [0x80, 100];
    let mut read_buf = [0x9f, 0, 0, 0];

    // todo: Write example.

    cs.set_low();

    unsafe {
        spi.write_dma(&read_buf, DmaChannel::C3, Default::default(), &mut dma);
        spi.read_dma(&mut read_buf, DmaChannel::C2, Default::default(), &mut dma);
    }

    while !dma.transfer_is_complete(DmaChannel::C2) {}
    spi.stop_dma(DmaChannel::C2, &mut dma);
    spi.stop_dma(DmaChannel::C3, &mut dma);

    cs.set_high();

    defmt::println!("Data: {}", read_buf);

    // Alternatively, use the blocking, non-DMA SPI API` (Also supports `embedded-hal` traits):
    spi.write(&write_buf).ok();
    spi.transfer(&mut read_buf).ok();
    defmt::println!("Data: {}", read_buf);

    // Assign peripheral structs as global, so we can access them in interrupts.
    free(|cs| {
        DMA.borrow(cs).replace(Some(dma));
        SPI.borrow(cs).replace(Some(spi));
    });

    // Unmask the interrupt line. See the `DMA_CH2` and `DMA_CH3` interrupt handlers below.
    unsafe {
        NVIC::unmask(pac::Interrupt::DMA1_CH2);
        NVIC::unmask(pac::Interrupt::DMA1_CH3);
    }

    // Alternatively, we can take readings without DMA. This provides a simpler, memory-safe API,
    // and is compatible with the `embedded_hal::blocking::i2c traits.

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// This interrupt fires when a DMA transmission is complete
fn DMA1_CH3() {
    free(|cs| {
        access_global!(DMA, dma, cs);
        access_global!(SPI, spi, cs);

        dma.clear_interrupt(DmaChannel::C3, DmaInterrupt::TransferComplete);
        spi.stop_dma(DmaChannel::C3, dma);

        unsafe {
            // Set CS high as required.
            // (*pac::GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 15));
        }
    })
}

#[interrupt]
/// This interrupt fires when a DMA read is complete
fn DMA1_CH2() {
    free(|cs| {
        defmt::println!("SPI DMA STOPPED");
        access_global!(DMA, dma, cs);
        access_global!(SPI, spi, cs);

        dma.clear_interrupt(DmaChannel::C2, DmaInterrupt::TransferComplete);
        spi.stop_dma(DmaChannel::C2, dma);

        unsafe {
            // Set CS high as required.
            // (*pac::GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 15));
        }
    })
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
