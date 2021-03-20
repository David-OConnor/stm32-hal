#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal::{
    adc::{Adc, AdcChannel},
    clocks::{ClockCfg, Clocks},
    dac::{Dac, Channel as DacChannel, Bits as DacBits},
    delay::Delay,
    event::Timeout,
    flash::Flash,
    gpio::{GpioA, GpioB, Edge, PinNum, PinMode, OutputType, AltFn, Pull},
    i2c::{I2c, I2cDevice},
    low_power,
    pac,
    prelude::*,  // The prelude imports Embedded Hal traits.
    rtc::{Rtc, RtcClockSource, RtcConfig},
    serial::{self, Serial},
    spi::Spi,
    timer::{Event::TimeOut, Timer},
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clocks = Clocks::default();

    if clocks.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Setup a delay, based on the cortex_m systick.
    let mut delay = Delay::new(cp.SYST, &clocks);

    // Set up the realtime clock.
    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.RCC,
        &mut dp.PWR,
        RtcConfig::default().clock_source(RtcClockSource::Lse), // .bypass_lse_output(true)
    );

    // Read from and write to the flash memory:
    let mut flash = Flash::new(dp.FLASH);

    flash.erase_page(FLASH_PAGE).ok();
    flash.write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    // Enable the GPIOA and GPIOB ports.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // An example GPIO pinn.
    let mut pa15 = gpioa.new_pin(PinNum::P15, PinMode::Output);
    pa15.set_high().ok();

    pa15.enable_interrupt(Edge::Rising, &mut dp.EXTI, &mut dp.SYSCFG);

    // Set up an I2C peripheral
    let scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let sda = gpiob.new_pin(PinNum::P7, PinMode::Alt(AltFn::AF4));
    sda.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let i2c = I2c::new_unchecked(dp.I2C1, I2cDevice::One, 100_000, &clocks, &mut dp.RCC);

    // Set up an SPI peripheral
    let sck = gpioa.new_pin(PinNum::P5, PinMode::Alt(AltFn::Af5));
    let miso = gpioa.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af5));
    let mosi = gpioa.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af5));

    let spi = Spi::spi1_unchecked(
        dp.SPI1,
        spi_mode,
        4_000_000,
        &clocks,
        &mut dp.RCC,
    );

    // Set up UART
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));
    let mut serial = Serial::new_uart1_unchecked(dp.USART1, 9_600, &clocks, &mut dp.RCC);
    let (tx, rx) = serial.split();

    // Set up the Analog-to-digital converter
    let _adc_pin = gpiob.new_pin(PinNum::P5, PinMode::Analog);

    let mut adc = Adc::new_adc1_unchecked(
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
    let mut timer = Timer::tim3_unchecked(dp.TIM3, 0.2, &clocks, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    loop {
        delay.delay_ms(1_000_u16);

        // Enter a low power mode.
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
