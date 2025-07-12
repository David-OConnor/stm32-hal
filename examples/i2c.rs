//! This module includes an overview of I2C features available.
//! It demonstrates how to configure and take readings from a TI ADS1115
//! offboard ADC.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::with;
use defmt::println;
use hal::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaPeriph, DmaWriteBuf},
    gpio::{Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed, NoiseFilter},
    init_globals, low_power,
    pac::{self, I2C1},
    setup_nvic,
};

static WRITE_BUF: [u8; 2] = [0, 0];

static mut READ_BUF: [u8; 8] = [0; 8];

const ADDR: u8 = 0x48;

const DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
const TX_CH: DmaChannel = DmaChannel::C6;
const RX_CH: DmaChannel = DmaChannel::C7;

make_globals!((I2C, I2c<I2C1>),);

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    clock_cfg.setup().unwrap();

    // Configure pins for I2c.
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    // Set up an I2C peripheral, running at 100Khz.
    let i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);

    let i2c_cfg = I2cConfig {
        speed: I2cSpeed::Fast400K, // Set to Fast mode, at 400Khz.
        // Set a digital noise filter instead of the default analog one.
        noise_filter: NoiseFilter::Digitial(5),
        ..Default::default()
    };

    // Or, customize the config, including setting different preset speeds:
    let i2c = I2c::new(dp.I2C1, i2c_cfg, &clock_cfg);

    // todo: Show how to set up SMBUS.

    // Configure settings for the ADS1115 ADC:
    // This config is the 16-bit register contents to set up the ADC in one-shot mode
    // with certain settings, and initiate a conversion. For I2C communications, we
    // use u8 words. Since this ADC uses 16-bit registers, we split into bytes.
    let cfg = [0b1000_0101, 0b1000_0000];
    let cfg_reg = 0x1;
    let conversion_reg = 0x0;

    // Set up DMA, for nonblocking (generally faster) conversion transfers:
    let mut dma = Dma::new(&mut dp.DMA1);

    // Associate DMA channels with I2C1: One for transmit; one for receive.
    // Note that mux is not used on F3, F4, and most L4s: DMA channels are hard-coded
    // to peripherals on those platforms.
    dma::mux(DMA_PERIPH, TX_CH, DmaInput::I2c1Tx);
    dma::mux(DMA_PERIPH, RX_CH, DmaInput::I2c1Rx);

    // Write to DMA, requesting readings,
    // todo: put this in a loop, timer interrupt etc to request readings when required.
    unsafe {
        i2c.write_dma(
            ADDR,
            &WRITE_BUF,
            false,
            TX_CH,
            Default::default(),
            DMA_PERIPH,
        );
    }

    // Alternatively, use the blocking, non-DMA I2C API`; see the interrupt handlers below.
    let mut read_buf = [0, 0];

    // Write the config register address, then the 2 bytes of the value we're writing.
    i2c.write(ADDR, &[cfg_reg, cfg[0], cfg[1]]).ok();
    // Now request a reading by passing the conversion reg, and a buffer to write
    // the results to.
    i2c.write_read(ADDR, &[conversion_reg], &mut read_buf).ok();
    let reading = i16::from_be_bytes([read_buf[0], read_buf[1]]);

    init_globals!((I2C, i2c),);

    // Unmask the interrupt lines. See the `DMA_CH6` and `DMA_CH7` interrupt handlers below.
    setup_nvic!([(DMA1_CH6, 3), (DMA2_CH7, 3),], cp);

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// This interrupt fires when a DMA transmission is complete. Read the results.
fn DMA1_CH6() {
    dma::clear_interrupt(DMA_PERIPH, TX_CH, DmaInterrupt::TransferComplete);

    dma::stop(DMA_PERIPH, TX_CH);

    with(|cs| {
        access_global!(I2C, i2c, cs);
        unsafe {
            i2c.read_dma(ADDR, &mut READ_BUF, RX_CH, Default::default(), DMA_PERIPH);
        }
    });

    println!("I2C write complete; reading.");
}

#[interrupt]
/// This interrupt fires when a DMA read is complete. Handle readings.
fn DMA1_CH7() {
    dma::clear_interrupt(DMA_PERIPH, RX_CH, DmaInterrupt::TransferComplete);

    dma::stop(DMA_PERIPH, RX_CH);

    let buf = unsafe { &READ_BUF };

    println!("I2C data is available: {:?}", buf);

    // The readings (etc) are ready; parse `buf` as required.
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
