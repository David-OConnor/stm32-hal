//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use stm32_hal::{
    adc::{Adc, AdcChannel, AdcDevice},
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port},
    low_power,
    pac::{self, interrupt, ADC1, EXTI},
    rtc::{Rtc, RtcClockSource, RtcConfig},
    timer::{Timer, TimerInterrupt},
};

// Copy type variables can go in `Cell`s, which are easier to access.
static SENSOR_READING: Mutex<Cell<f32>> = Mutex::new(Cell::new(335.));
static BOUNCING: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

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

    clock_cfg.setup().unwrap();

    // Configure PA0 to trigger a GPIO interrupt.
    let mut button = Pin::new(Port::A, 0, PinMode::Input);
    button.enable_interrupt(Edge::Falling);

    // Set up and start a timer; set it to fire interrupts every 5 seconds.
    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, Default::default(), &clock_cfg);
    timer.enable_interrupt(TimerInterrupt::Update); // Enable update event interrupts.
    timer.enable();

    let mut debounce_timer = Timer::new_tim15(dp.TIM15, 5., Default::default(), &clock_cfg);
    debounce_timer.enable_interrupt(TimerInterrupt::Update); // Enable update event interrupts.

    // Set up the realtime clock.
    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.PWR,
        RtcConfig {
            clock_source: RtcClockSource::Lse,
            ..Default::default()
        },
    );

    // Set the RTC to trigger an interrupt every 30 seconds.
    rtc.set_wakeup(&mut dp.EXTI, 30.);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        AdcDevice::One,
        Default::default(),
        clock_cfg.systick(),
    );
    adc.enable_interrupt(AdcInterrupt::EndOfConversion);

    // Set up our ADC as a global variable accessible in interrupts, now that it's initialized.
    with(|cs| {
        ADC.borrow(cs).replace(Some(adc));
    });

    // Unmask the interrupt lines.
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI0); // GPIO
        NVIC::unmask(pac::Interrupt::TIM3); // Timer
        NVIC::unmask(pac::Interrupt::TIM15); // Timer
        NVIC::unmask(pac::Interrupt::RTC_WKUP); // RTC

        // Set interrupt priority. See the reference manual's NVIC section for details.
        cp.NVIC.set_priority(pac::Interrupt::EXTI0, 0);
        cp.NVIC.set_priority(pac::Interrupt::TIM3, 1);
    }

    // todo: UART interrupts.

    loop {
        // Enter a low power mode.
        low_power::stop(low_power::StopMode::Two);

        // Turn back on the PLL.
        clocks.reselect_input();
    }
}

#[interrupt]
/// GPIO interrupt
fn EXTI0() {
    // Clear the interrupt flag, to prevent continous firing.
    gpio::clear_exti_interrupt(0);

    with(|cs| {
        let bouncing = BOUNCING.borrow(cs);
        if bouncing.get() {
            return;
        }
        unsafe { (*pac::TIM15::ptr()).cr1.modify(|_, w| w.cen().set_bit()) }
        bouncing.set(true);

        // Update our global sensor reading. This section dmeonstrates boilerplate
        // for Mutexes etc.
        // Alternative syntax using a convenience macro:
        // `access_global!(ADC, sensor, cs);`
        let mut s = ADC.borrow(cs).borrow_mut();
        let sensor = s.as_mut().unwrap();
        let reading = sensor.read(AdcChannel::C1).unwrap();
        SENSOR_READING.borrow(cs).replace(reading);
    });
}

#[interrupt]
/// RTC wakeup handler
fn RTC_WKUP() {
    with(|cs| {
        unsafe {
            // Reset pending bit for interrupt line; RTC uses EXTI line 20.
            gpio::clear_exti_interrupt(20);

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
/// Timer interrupt handler
fn TIM3() {
    with(|cs| {
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        unsafe { (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        // Alternatively, if you have the timer set up in a global mutex:
        // access_global!(TIMER, timer, cs);
        // timer.clear_interrupt(TimerInterrupt::Update);
    });

    // Do something.
}

#[interrupt]
/// We use this timer for button debounce.
fn TIM15() {
    with(|cs| {
        unsafe { (*pac::TIM15::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        BOUNCING.borrow(cs).set(false);

        // Disable the timer until next time you press a button.
        unsafe { (*pac::TIM15::ptr()).cr1.modify(|_, w| w.cen().clear_bit()) }
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
