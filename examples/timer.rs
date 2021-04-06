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
    adc::{Adc, AdcChannel, Align, ClockMode, InputType, OperationMode},
    clocks::Clocks,
    delay::Delay,
    gpio::{Edge, PinMode, PinNum},
    low_power, pac,
    prelude::*,
    timer::{Channel, CountDir, OutputCompare, Timer},
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

    // Enable the GPIOB port.
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    let mut timer = Timer::new_tim1(dp.TIM1, 0.2, &clock_cfg, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    // Setting auto reload preload allow changing frequency (period) while the timer is running.
    timer.set_auto_reload_preload(true);
    timer.enable_pwm_output(Channel::One, OutputCompare::Pwm1, CountDir::Up, 0.5);

    // todo: Flesh out this example.

    // Unmask the interrupt line.
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM1);
    }

    loop {
        low_power::sleep_now(&mut SCB);
    }
}

#[interrupt]
/// Timer interrupt handler
fn TIM3() {
    free(|cs| {
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        unsafe { (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().clear()) }

        // Alternatively, if you have the timer set up in a global mutex:
        // access_global!(TIMER, timer, cs);
        // timer.clear_interrupt();
    });

    // Do something.
}
