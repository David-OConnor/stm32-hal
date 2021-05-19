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
    delay::Delay,
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaWriteBuf},
    gpio::{Edge, PinMode, PinNum},
    i2c::{I2c, I2cDevice},
    low_power, pac,
};

use embedded_hal::{Read, Write, WriteRead};

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

    let mut delay = Delay::new(cp.SYST, &clock_cfg);

    // Enable the GPIOB port.
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // Configure pins for I2c.
    let mut scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let mut sda = gpiob.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af4));
    sda.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    // Set up an I2C peripheral, running at 100Khz.
    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clock_cfg, &mut dp.RCC);

    // Configure settings for the ADS1115 ADC:
    let addr: u8 = 0x48;
    // This config is the 16-bit register contents to set up the ADC in one-shot mode
    // with certain settings, and initiate a conversion. For I2C communications, we
    // use u8 words. Since this ADC uses 16-bit registers, we split into bytes.
    let cfg = [0b1000_0101, 0b1000_0000];
    let cfg_reg = 0x1;
    let conversion_reg = 0x0;

    // Set up DMA, for nonblocking (generally faster) conversion transfers:
    let mut dma = Dma::new(&mut dp.DMA1, &dp.RCC);

    // todo fill this in)

    // Alternatively, use the blocking, non-DMA I2C API provided by `embedded-hal`:
    let mut read_buf = [0, 0];
    // Write the config register address, then the 2 bytes of the value we're writing.
    i2c.write(addr, &[cfg_reg, cfg[0], cfg[1]]).ok();
    // Now request a reading by passing the conversion reg, and a buffer to write
    // the results to.
    i2c.write_read(addr, &[conversion_reg], &mut read_buf).ok();
    let reading = i16::from_be_bytes([read_buf[0], read_buf[1]]);

    // Unmask the interrupt line. See the `DMA_CH6` and `DMA_CH78` interrupt handlers below.
    unsafe {
        NVIC::unmask(pac::Interrupt::DMA1_CH6);
        NVIC::unmask(pac::Interrupt::DMA1_CH7);
    }

    // Alternatively, we can take readings without DMA. This provides a simpler, memory-safe API,
    // and is compatible with the `embedded_hal::blocking::i2c traits.

    loop {
        low_power::sleep_now(&mut SCB);
    }
}

#[interrupt]
/// This interrupt fires when a DMA transmission is complete
fn DMA1_CH6() {
    free(|cs| {
        // Clear the interrupt flag, to prevent continual running of this ISR.
        unsafe { (*pac::DMA1::ptr()).ifcr.write(|w| w.tcif6().set_bit()) }
        // Or, if you have access to the Dma peripheral struct:
        // dma.clear_interrupt(DmaChannel::C6);
        // dma.stop(DmaChannel::C6);
    });
}

#[interrupt]
/// This interrupt fires when a DMA read is complete
fn DMA1_CH7() {
    free(|cs| unsafe { (*pac::DMA1::ptr()).ifcr.write(|w| w.tcif7().set_bit()) });
}
