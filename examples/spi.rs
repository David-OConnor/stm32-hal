//! This module includes an overview of I2C features available.
//! It demonstrates how to configure and take readings from a TI ADS1115
//! offboard ADC.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use hal::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInput, DmaInterrupt, DmaPeriph, DmaWriteBuf},
    gpio::{self, Pin, PinMode, Port},
    low_power,
    pac::{self, interrupt},
    prelude::*,
    spi::{self, BaudRate, Spi, SpiConfig, SpiMode},
};

// Byte 0 is for the address we pass in the `write` transfer; relevant data is in the rest of
// the values.
static mut SPI_READ_BUF: [u8; 4] = [0; 4];
static mut SPI_WRITE_BUF: [u8; 4] = [0x69, 0, 0, 0];

make_globals!((SPI, Spi<SPI1>));

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

    let mut cs = Pin::new(Port::A, 1, PinMode::Output);

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
    let mut spi = Spi::new(
        dp.SPI1,
        spi_cfg,
        BaudRate::Div32, // Eg 80Mhz apb clock / 32 = 2.5Mhz SPI clock.
    );

    // Set up DMA, for nonblocking (generally faster) conversion transfers:
    let mut dma = Dma::new(&mut dp.DMA1, &dp.RCC);

    // Associate a pair of DMA channels with SPI1: One for transmit; one for receive.
    // Note that mux is not used on F3, F4, and most L4s: DMA channels are hard-coded
    // to peripherals on those platforms.
    dma::mux(DmaPeriph::Dma2, DmaChannel::C1, DmaInput::Spi1Tx);
    dma::mux(DmaPeriph::Dma2, DmaChannel::C2, DmaInput::Spi1Rx);

    cs.set_low();

    unsafe {
        // Write to SPI, using DMA.
        // spi.write_dma(&write_buf, DmaChannel::C1, Default::default(), DmaPeriph::Dma2);

        // Read (transfer) from SPI, using DMA.
        spi.transfer_dma(
            // Write buffer, starting with the registers we'd like to access, and 0-padded to
            // read 3 bytes.
            &SPI_WRITE_BUF,
            &mut SPI_READ_BUF,  // Read buf, where the data will go
            DmaChannel::C1,     // Write channel
            DmaChannel::C2,     // Read channel
            Default::default(), // Write channel config
            Default::default(), // Read channel config
            DmaPeriph::Dma2,
        );
    }

    // Alternatively, use the blocking, non-DMA SPI API` (Also supports `embedded-hal` traits):

    // We read 3 bytes from the `0x9f` register.
    let mut write_buf = [0x80, 100];
    let mut read_buf = [0x9f, 0, 0, 0];
    spi.write(&write_buf).ok();
    spi.transfer(&mut read_buf).ok();
    defmt::println!("Data: {}", read_buf);

    // Assign peripheral structs as global, so we can access them in interrupts.
    with(|cs| {
        DMA.borrow(cs).replace(Some(dma));
        SPI.borrow(cs).replace(Some(spi));
    });

    // Unmask the interrupt line for DMA read complete. See the `DMA_CH3` interrupt handlers below,
    // where we set CS high, terminal the DMA read, and display the data read.
    unsafe {
        NVIC::unmask(pac::Interrupt::DMA1_CH2);
    }

    // Alternatively, we can take readings without DMA. This provides a simpler, memory-safe API,
    // and is compatible with the `embedded_hal::blocking::i2c traits.

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// This interrupt fires when a DMA read is complete
fn DMA1_CH2() {
    dma::clear_interrupt(
        DmaPeriph::Dma2,
        DmaChannel::C2,
        DmaInterrupt::TransferComplete,
    );
    with(|cs| {
        defmt::println!("SPI DMA read complete");
        access_global!(SPI, spi, cs);
        spi.stop_dma(DmaChannel::C1, Some(DmaChannel::C2), DmaPeriph::Dma2);

        // See also this convenience function, which clears the interrupt and stops othe Txfer.:
        spi.cleanup_dma(DmaPeriph::Dma2, DmaChannel::C1, Some(DmaChannel::C2));

        unsafe {
            // Ignore byte 0, which is the reg we passed during the write.
            println!("Data read: {:?}", SPI_READ_BUF);
        }

        // Set CS high.
        gpio::set_high(Port::A, 1);
    })
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
