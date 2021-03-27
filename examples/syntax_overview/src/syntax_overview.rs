#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;

use defmt_rtt as _;
use panic_probe as _;

// Import parts of this library we use. You could use this style, or perhaps import
// less here.
use stm32_hal2::{
    adc::{Adc, AdcChannel},
    clocks::Clocks,
    dac::{Dac, Channel as DacChannel, Bits as DacBits},
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

    // Create an initial clock configuration that uses the MCU's internal oscillator (HSI),
    // sets the MCU to its maximum system clock speed.
    let clocks = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    if clocks.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, &clocks);

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
    flash.erase_page(FLASH_PAGE).ok();
    // Write a byte array to the flash.
    flash.write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    // Enable the GPIOA and GPIOB ports.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // An example GPIO pin, configured in output mode.
    let mut pa15 = gpioa.new_pin(PinNum::P15, PinMode::Output);
    pa15.set_high().ok();

    pa15.enable_interrupt(Edge::Rising, &mut dp.EXTI, &mut dp.SYSCFG);

    // Configure pins for I2c. Make sure to do this hbefore running `I2c::new`.
    let mut scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let mut sda = gpiob.new_pin(PinNum::P7, PinMode::Alt(AltFn::AF4));
    sda.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    // Set up an I2C peripheral.
    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clocks, &mut dp.RCC);

    // Configure pins for I2c. Make sure to do this before running `Spi::new_spix`.
    let _sck = gpioa.new_pin(PinNum::P5, PinMode::Alt(AltFn::Af5));
    let _miso = gpioa.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af5));
    let _mosi = gpioa.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af5));

    let spi_mode = Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    // Set up an SPI peripheral.
    let spi = Spi::new_spi1(
        dp.SPI1,
        spi_mode,
        4_000_000,
        &clocks,
        &mut dp.RCC,
    );

    // Set up UART, as above.
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));

    let mut serial = Serial::new_usart1(
        dp.USART1,
        serial::Config::default().baudrate(9_600),
        &clocks,
        &mut dp.RCC,
    );
    let (tx, rx) = serial.split();

    // Set up the Analog-to-digital converter
    let _adc_pin = gpiob.new_pin(PinNum::P5, PinMode::Analog);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        &mut dp.ADC1,
        adc::CkMode::default(),
        &clocks,
        &mut dp.RCC,
    );

    let reading = adc.read(&mut AdcChannel::C1);

    // Set up the Digital-to-analog converter
    let mut dac_pin = gpioa.new_pin(PinNum::P12, PinMode::Analog);
    let mut dac = Dac::new(dp.DAC, dac_pin, DacChannel::One, Bits::TwelveR, 3.3);
    dac.enable(&mut dp.RCC);

    // Set up and start a timer; set it to fire interrupts.
    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, &clocks, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    // For pins that aren't called directly (Like the ones we set up for I2C, SPI, UART, ADC, and DAC),
    // consider a separate function:
    // /// Set up the pins that have structs that don't need to be accessed after.
    // pub fn setup_pins(gpioa: &mut GpioA, gpiob: &mut GpioB, exti: &mut EXTI, syscfg: &mut SYSCFG) {
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
