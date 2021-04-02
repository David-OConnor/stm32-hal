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
    adc::{Adc, AdcChannel},
    clocks::Clocks,
    gpio::{Edge, GpioA, PinMode, PinNum},
    low_power,
    pac::{self, ADC1, EXTI},
    prelude::*,
    rtc::{Rtc, RtcClockSource, RtcConfig},
    timer::{Event::TimeOut, Timer},
};

// Copy type variables can go in `Cell`s, which are easier to access.
static SENSOR_READING: Mutex<Cell<f32>> = Mutex::new(Cell::new(335.));

// More complex values go in `RefCell`s. Use an option, since we need to set this up
// before we initialize the peripheral it stores.
static ADC: Mutex<RefCell<Option<ADC<ADC1>>>> = Mutex::new(RefCell::new(None));

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

    // Enable the GPIOA port.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);

    // Configure PA0 to trigger a GPIO interrupt.
    let mut button = gpioa.new_pin(PinNum::P0, PinMode::Input);
    button.enable_interrupt(Edge::Falling, &mut dp.EXTI, &mut dp.SYSCFG);

    // Set up and start a timer; set it to fire interrupts every 5 seconds.
    let mut timer_1 = Timer::new_tim3(dp.TIM3, 0.2, &clock_cfg, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    // Set up the realtime clock.
    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.RCC,
        &mut dp.PWR,
        RtcConfig::default().clock_source(RtcClockSource::Lse), // .bypass_lse_output(true)
    );

    // Set the RTC to trigger an interrupt every 30 seconds.
    rtc.set_wakeup(&mut dp.EXTI, 30.);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        &mut dp.ADC_COMMON,
        adc::CkMode::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    // Set up our ADC as a global variable accessible in interrupts, now that it's initialized.
    free(|cs| {
        ADC.borrow(cs).replace(Some(adc));
    });

    // Unmask the interrupt lines.
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI0); // GPIO
        NVIC::unmask(pac::Interrupt::TIM3); // Timer
        NVIC::unmask(pac::Interrupt::RTC_WKUP); // RTC

        // I'm leaving this priority-setting code in for future reference.
        cp.NVIC.set_priority(pac::Interrupt::EXTI0, 0);
        cp.NVIC.set_priority(pac::Interrupt::TIM3, 1);
    }

    loop {
        // Enter a low power mode.
        low_power::stop(
            &mut cp.SCB,
            &mut dp.PWR,
            low_power::StopMode::Two,
            clock_cfg.input_src,
            &mut dp.RCC,
        );
    }
}

#[interrupt]
/// GPIO interrupt
fn EXTI0() {
    free(|cs| {
        unsafe {
            // Clear the interrupt flag, to prevent continous firing.
            (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr0().bit(true));
        }

        // Update our global sensor reading. This section dmeonstrates boilerplate
        // for Mutexes etc.
        let mut s = ADC::borrow(cs).borrow_mut();
        let sensor = s.as_mut().unwrap();
        let reading = sensor.read(AdcChannel::C1).unwrap();
        SENSOR_READING.borrow(cs).replace(reading);
    });
}

#[interrupt]
/// RTC wakeup handler
fn RTC_WKUP() {
    free(|cs| {
        unsafe {
            // Reset pending bit for interrupt line
            (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr20().set_bit());

            // Clear the wakeup timer flag, after disabling write protections.
            (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xCA));
            (*pac::RTC::ptr()).wpr.write(|w| w.bits(0x53));
            (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().clear_bit());

            (*pac::RTC::ptr()).isr.modify(|_, w| w.wutf().clear_bit());

            (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().set_bit());
            (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xFF));
        }

        // A cleaner alternative to the above, if you have the RTC set up in a global Mutex:
        //  unsafe {
        //      (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr20().set_bit());
        //  }
        //  access_global!(RTC, rtc, cs);
        //  rtc.clear_wakeup_flag();

        // Do something.
    });
}

#[interrupt]
/// Timer wakeup handler
fn TIM3() {
    free(|cs| {
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        unsafe { (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().clear()) }
    });

    // Do something.
}
