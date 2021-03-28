//! This module includes an overview of ADC features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal::{
    adc::{Adc, AdcChannel, Align, CkMode, InputType, OperationMode},
    clocks::Clocks,
    delay::Delay,
    gpio::{Edge, GpioA, PinMode, PinNum},
    low_power, pac,
    prelude::*,
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

    let mut delay = Delay::new(cp.SYST, &clock_cfg);

    let chan_num = 2;

    // Enable the GPIOA port.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);

    let mut adc = Adc::new_adc1_unchecked(
        dp.ADC1,
        &mut dp.ADC_COMMON,
        CkMode::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    // Take a OneShot reading using the embedded HAL trait.
    let reading = adc.read();

    // Or, start reading in continuous mode:
    adc.start_conversion(chan_num, OperationMode::Continuous);
    // Read from the ADC's latest (continuously-running) conversion:
    let reading = adc.read_result();

    // Set up differential mode:
    adc.set_input_type(chan_num, InputType::Differential);

    // Change the sample rate:
    adc.set_sample_time(chan_num, SampleTime::T2);

    // Set left align mode:
    adc.set_align(Align::Left);

    loop {
        low_power::sleep_now(&mut SCB);
    }
}
