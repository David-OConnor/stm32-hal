//! This module includes an overview of timer
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    clocks::Clocks,
    delay::Delay,
    gpio::{Edge, PinMode, PinNum},
    low_power, pac,
    timer::{Channel, CountDir, OutputCompare, Timer, TimerInterrupt},
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

    // Enable the GPIOB port.
    let mut gpioa = GpioA::new(dp.GPIOB, &mut dp.RCC);

    // Set up a PWM pin
    let _pwm_pin = gpioa.new_pin(PinNum::P0, PinMode::Alt(AltFn::Af1));

    // Set up a PWM timer that will output to PA0, run at 2400Hz in edge-aligned mode,
    // count up, with a 50% duty cycle.
    let mut pwm_timer = Timer::new_tim2(dp.TIM2, 2_400., &clock_cfg, &mut dp.RCC);
    pwm_timer.enable_pwm_output(Channel::One, OutputCompare::Pwm1, CountDir::Up, 0.5);
    // Setting auto reload preload allow changing frequency (period) while the timer is running.
    pwm_timer.set_auto_reload_preload(true);
    pwm_timer.enable();

    let mut countdown_timer = Timer::new_tim3(dp.TIM3, 0.5, &clock_cfg, &mut dp.RCC);
    timer.enable_interrupt(TimerInterrupt::Update); // Enable update event interrupts.
    countdown_timer_timer.enable();

    pwm_timer.set_freq(1_000.); // set to 1000Hz.

    // Or set PSC and ARR manually, eg to set period (freq), while preventing additional calculations.
    pwm_timer.set_auto_reload(100);
    pwm_timer.set_prescaler(100);

    // Unmask the interrupt line.
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM3);
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
        unsafe { (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().set_bit()) }
    });

    // Do something.
}
