//! This example shows a complete project, including file structure, and config
//! needed to flash using an ST-Link. The project structure is based on
//! [Knurling's app-template](https://github.com/knurling-rs/app-template).
//! This file demonstrates an overview of this library's features.

#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::{
    self,
    delay::Delay,
    peripheral::NVIC,
};
use critical_section::{with, Mutex};
use cortex_m_rt::entry;

// These lines are part of our setup for debug printing.
use defmt_rtt as _;
use panic_probe as _;

// Import parts of this library we use. You could use this style, or perhaps import
// less here.
use hal::{
    self,
    adc::{self, Adc, AdcChannel},
    clocks::Clocks,
    dac::{Dac, DacChannel, DacBits},
    dma::{Dma, DmaPeriph, DmaChannel, DmaInput, DmaInterrupt, DmaReadBuf, DmaWriteBuf},
    flash::Flash,
    gpio::{Edge, Pin, Port, PinMode, OutputType, Pull},
    i2c::I2c,
    low_power,
    pac,
    rtc::{Rtc, RtcClockSource, RtcConfig},
    usart::{Usart, UsartInterrupt, UsartConfig},
    spi::{self, BaudRate, Spi, SpiConfig, SpiMode},
    timer::{Timer, TimerInterrupt},
};

#[entry]
fn main() -> ! {
    // Set up ARM Cortex-M peripherals. These are common to many MCUs, including all STM32 ones.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let mut dp = pac::Peripherals::take().unwrap();

    // This line is required to prevent the debugger from disconnecting on entering WFI.
    // This appears to be a limitation of many STM32 families. Not required in production code,
    // and significantly increases power consumption in low-power modes.
    hal::debug_workaround();

    // Create an initial clock configuration that uses the MCU's internal oscillator (HSI),
    // sets the MCU to its maximum system clock speed.
    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    delay.delay_ms(500);

    // Set up the realtime clock. This is useful for keeping track of dates and times, or
    // setting a 'timer', especially for long durations. Can be used to wake up the MCU
    // from most low-power modes.
    let mut rtc = Rtc::new(
        dp.RTC,
        RtcConfig {
            clock_source: RtcClockSource::Lse,
            ..Default::default()
        }
    );

    // Read from and write to the onboard flash memory.
    let mut flash = Flash::new(dp.FLASH);

    // Make sure to select a page farther than the farthest page this program uses!
    let flash_page = 20;
    flash.erase_page(flash_page).ok();
    // Write a byte array to the flash.
    flash.write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    // An example GPIO pin, configured in output mode.
    let mut pa15 = Pin::new(Port::A, 15, PinMode::Output);
    pa15.set_high();

    pa15.enable_interrupt(Edge::Rising);

    // Configure pins for I2c.
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    // Set up an I2C peripheral, running at 100Khz.
    let i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);

    // Configure pins for I2c.
    let _sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let _miso = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _mosi = Pin::new(Port::A, 7, PinMode::Alt(5));

    // Configure DMA, to be used by peripherals.
    let mut dma = Dma::new(&mut dp.DMA1);

    dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Adc2);

    let spi_cfg = SpiConfig {
        mode: SpiMode::mode3(), // SpiConfig::default() uses mode 0.
        ..Default::default()
    };

    // Set up an SPI peripheral, running at 4Mhz, in SPI mode 0.
    let spi = Spi::new(
        dp.SPI1,
        spi_cfg,
        BadRate::Div32,  // Eg 80Mhz apb clock / 32 = 2.5Mhz SPI clock.
    );

    // Configure pins for UART.
    let _uart_tx = Pin::new(Port::A, 9, PinMode::Alt(7));
    let _uart_rx = Pin::new(Port::A, 10, PinMode::Alt(7));

    // Set up a UART peripheral.
    // Setup UART for connecting to the host
    let mut uart = Usart::new(
        dp.USART1,
        9_600,
        UsartConfig::default(),
        &clock_cfg,
    );

    // Or, to set a custom USART config:
    let usart_cfg = UsartConfig {
        parity: Parity::EnabledOdd,
        stop_bits: StopBits::S2,
        ..Default::default()
    };
    let mut uart = Usart::new(
        dp.USART1,
        9_600,
        usart_cfg,
        &clock_cfg,
    );

    // Write a byte array to the UART
    uart.write(&[1, 2, 3, 4]);

    // Read a byte array from the UART.
    let buffer = [0_u8; 10];
    uart.read(&mut uart_buffer, &mut dma);

    // Or, read and write using DMA:
    uart.write_dma(&[1, 2, 3, 4], DmaChannel::C2, Default::default(), DmaPeriph::Dma1);
    uart.read_dma(&mut uart_buffer, DmaChannel::C3, Default::default(), DmaPeriph::Dma1);

    // Set up the Analog-to-digital converter
    let _adc_pin = Pin::new(Port::B, 5, PinMode::Analog);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        Default::default(),
        clock_cfg.systick(),
    );

    // Take a reading from ADC channel 1.
    let reading: u16 = adc.read(1);

    // Set up the Digital-to-analog converter
    let mut _dac_pin = Pin::new(Port::A, 12, PinMode::Analog);
    let mut dac = Dac::new(dp.DAC1, Default::default(), 3.3);
    dac.enable(DacChannel::C1);

    dac.write(DacChannel::C1, 2_048); // Set DAC output voltage to half VCC, eg 1.65V

    // Set up and start a timer; set it to fire interrupts at 5Hz.
    let mut timer = Timer::new_tim1(dp.TIM1, 0.2, Default::default(), &clock_cfg);
    timer.enable_interrupt(TimerInterrupt::Update); // Enable update event interrupts.
    timer.enable();

    // You can read most peripheral status registers with a `read_status()` method: (Returns the
    // 32-bit status register contents).
    let status = timer.read_status();

    // For pins that aren't called directly (Like the ones we set up for I2C, SPI, UART, ADC, and DAC),
    // consider a separate function:
    // fn setup_pins(gpioa: &mut GpioA, gpiob: &mut GpioB, exti: &mut EXTI, syscfg: &mut SYSCFG) {
    //     let mut scl = gpiob.new_pin(6, PinMode::Alt(4));
    //     scl.output_type(OutputType::OpenDrain);
    //     // ...
    // }

    loop {
        defmt::println!("Looping!"); // A print statement using DEFMT.
        // Enter a low power mode. The program will wake once an interrupt fires.
        // For example, the timer and GPIO interrupt above. But we haven't unmasked
        // their lines, so they won't work - see the `interrupts` example for that.
        low_power::sleep_now();
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