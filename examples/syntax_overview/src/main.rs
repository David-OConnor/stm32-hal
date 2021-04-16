//! This example shows a complete project, including file structure, and config
//! needed to flash using an ST-Link. The project structure is based on
//! [Knurling's app-template](https://github.com/knurling-rs/app-template).
//! This file demonstrates an overview of this library's features.

#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;

// These lines are part of our setup for debug printing.
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::spi::{Phase, Polarity, Mode};

// Import parts of this library we use. You could use this style, or perhaps import
// less here.
use stm32_hal2::{
    self,
    adc::{self, Adc, AdcChannel},
    clocks::Clocks,
    dac::{Dac, DacChannel, DacBits},
    delay::Delay,
    flash::Flash,
    gpio::{GpioA, GpioB, Edge, PinNum, PinMode, OutputType, AltFn, Pull},
    i2c::{I2c, I2cDevice},
    low_power,
    pac,
    prelude::*,  // The prelude imports Embedded Hal traits, and some custom ones.
    rtc::{Rtc, RtcClockSource, RtcConfig},
    serial::{self, Serial},
    spi::{self, Spi},
    timer::{Event::TimeOut, Timer},
};

#[entry]
fn main() -> ! {
    // Set up ARM Cortex-M peripherals. These are common to many MCUs, including all STM32 ones.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let mut dp = pac::Peripherals::take().unwrap();

    // This line is required to prevent the debugger from disconnecting on entering WFI.
    // This appears to be a limitation of many STM32 families. Not required in production code.
    stm32_hal2::debug_workaround(&mut dp.DBGMCU, &mut dp.RCC);

    // Create an initial clock configuration that uses the MCU's internal oscillator (HSI),
    // sets the MCU to its maximum system clock speed.
    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    if clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, &clock_cfg);

    // Set up the realtime clock. This is useful for keeping track of dates and times, or
    // setting a 'timer', especially for long durations. Can be used to wake up the MCU
    // from most low-power modes.
    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.RCC,
        &mut dp.PWR,
        RtcConfig::default().clock_source(RtcClockSource::Lse), // .bypass_lse_output(true)
    );

    // Read from and write to the onboard flash memory.
    let mut flash = Flash::new(dp.FLASH);

    // Make sure to select a page farther than the farthest page this program uses!
    let flash_page = 20;
    flash.erase_page(flash_page).ok();
    // Write a byte array to the flash.
    flash.write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    // Enable the GPIOA and GPIOB ports.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // An example GPIO pin, configured in output mode.
    let mut pa15 = gpioa.new_pin(PinNum::P15, PinMode::Output);
    pa15.set_high();

    pa15.enable_interrupt(Edge::Rising, &mut dp.EXTI, &mut dp.SYSCFG);

    // Configure pins for I2c.
    let mut scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let mut sda = gpiob.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af4));
    sda.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    // Set up an I2C peripheral, running at 100Khz.
    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clock_cfg, &mut dp.RCC);

    // Configure pins for I2c.
    let _sck = gpioa.new_pin(PinNum::P5, PinMode::Alt(AltFn::Af5));
    let _miso = gpioa.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af5));
    let _mosi = gpioa.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af5));

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    // Set up an SPI peripheral, running at 4Mhz, in SPI mode 0.
    let spi = Spi::new_spi1(
        dp.SPI1,
        spi_mode,
        4_000_000,
        &clock_cfg,
        &mut dp.RCC,
    );

    // Configure pins for UART.
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));

    // Set up a UART peripheral.
    let mut serial = Serial::new_usart1(
        dp.USART1,
        serial::Config::default().baudrate(9_600),
        &clock_cfg,
        &mut dp.RCC,
    );
    let (tx, rx) = serial.split();

    // Set up the Analog-to-digital converter
    let _adc_pin = gpiob.new_pin(PinNum::P5, PinMode::Analog);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        &mut dp.ADC_COMMON,
        adc::ClockMode::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    let reading: u16 = adc.read(&mut AdcChannel::C1).unwrap();

    // Set up the Digital-to-analog converter
    let mut dac_pin = gpioa.new_pin(PinNum::P12, PinMode::Analog);
    let mut dac = Dac::new(dp.DAC1, DacChannel::One, Bits::TwelveR, 3.3, &mut dp.RCC);
    dac.enable();

    // Set up and start a timer; set it to fire interrupts at 5Hz.
    let mut timer = Timer::new_tim1(dp.TIM1, 0.2, &clock_cfg, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    // For pins that aren't called directly (Like the ones we set up for I2C, SPI, UART, ADC, and DAC),
    // consider a separate function:
    // fn setup_pins(gpioa: &mut GpioA, gpiob: &mut GpioB, exti: &mut EXTI, syscfg: &mut SYSCFG) {
    //     let mut scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    //     scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);
    //     // ...
    // }

    loop {
        defmt::info!("Looping!"); // A print statement using DEFMT.
        // Enter a low power mode. The program will wake once an interrupt fires.
        // For example, the timer and GPIO interrupt above. But we haven't unmasked
        // their lines, so they won't work - see the `interrupts` example for that.
        low_power::sleep_now(&mut cp.SCB);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
