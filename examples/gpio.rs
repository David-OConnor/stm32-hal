//! This module includes an overview of GPIO features available.
//! Most functionality is included as methods to the
//! [gpio::Pin struct](https://docs.rs/stm32-hal2/latest/stm32_hal2/gpio/struct.Pin.html).
//!
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{delay::Delay, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use embedded_hal::digital::OutputPin;
use hal::{
    adc::{Adc, AdcChannel, Align, CkMode, InputType, OperationMode},
    clocks::Clocks,
    gpio::{self, Edge, OutputSpeed, Pin, PinMode, PinState, Port},
    low_power, pac,
    prelude::*,
};

// Set up an output pin in a globally-accessible mutex. This is useful for accessing
// peripherals in interrupt contexts. We use a macro imported in the
// `prelude` module to simplify this syntax, and accessing it later.
// Arguments are a list of (global name to store as, type) tuples.
// This macro is imported in the prelude.
make_globals!((EXAMPLE_OUTPUT, Pin), (DEBOUNCE_TIMER, Timer<pac::TIM15>),);

/// This function includes type signature examples using `GpioPin`s from this library,
/// and generic ones that implemented `embedded-hal` traits.
fn example_type_sigs<O: OutputPin>(pin1: &mut O, pin2: &mut Pin) {
    let setting = pin2.is_high();

    // If using `embedded-hal` traits, you need to append `.unwrap()`, or `.ok()`, since these
    // traits are fallible, even though our stm32 implementation is not.
    pin1.set_low().ok();
}

/// An example function to set up the pins that don't need to be interacted with directly later.
/// For example, ones used with buses (eg I2C, SPI, UART), USB, ADC, and DAC pins.
/// This may also include input pins that trigger interrupts, and aren't polled.
pub fn setup_pins() {
    // Set up I2C pins
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    // Set up SPI pins
    let _sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let _miso = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _mosi = Pin::new(Port::A, 7, PinMode::Alt(5));

    // Set up UART pins
    let _uart_tx = Pin::new(Port::A, 9, PinMode::Alt(7));
    let _uart_rx = Pin::new(Port::A, 10, PinMode::Alt(7));

    // Set up USB pins
    let _usb_dm = Pin::new(Port::A, 11, PinMode::Alt(14));
    let _usb_dp = Pin::new(Port::A, 12, PinMode::Alt(14));

    // Set the ADC pin to analog mode, to prevent parasitic power use.
    let _adc_pin = Pin::new(Port::B, 0, PinMode::Analog);

    // Set DAC pin to analog mode, to prevent parasitic power use.
    let _dac_pin = Pin::new(Port::A, 4, PinMode::Analog);

    // Set up PWM.  // Timer 2, channel 1.
    let _pwm_pin = Pin::new(Port::A, 0, PinMode::Alt(1));

    // Set up buttons, with pull-up resistors that trigger on the falling edge.
    let mut up_btn = Pin::new(Port::B, 3, PinMode::Input);
    up_btn.pull(Pull::Up);
    up_btn.enable_interrupt(Edge::Falling);

    let mut dn_btn = Pin::new(Port::A, 4, PinMode::Input);
    dn_btn.pull(Pull::Up);
    dn_btn.enable_interrupt(Edge::Falling);
}

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    if clock_cfg.setup().is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Call a function we've made to help organize our pin setup code.
    setup_pins();

    // Example pins PB5 and PB6.
    let mut example_output = Pin::new(Port::B, 5, PinMode::Output);
    let mut example_input = Pin::new(Port::B, 6, PinMode::Input);

    // Set the output speed.
    example_output.output_speed(OutputSpeed::Medium);

    // A simple button debounce: Use a timer with a period between the maximum bouncing
    // time you expect, and the minimum time bewteen actuations. In this time, we've chosen 5Hz,
    // or 200ms. Note that there are other approaches as well.
    let mut debounce_timer = Timer::new_tim15(dp.TIM15, 5., &clock_cfg);
    debounce_timer.enable_interrupt(TimerInterrupt::Update);

    example_type_sigs(&mut example_output, &mut example_input);

    let state = example_input.get_state(); // eg `PinState::High` or `PinState::Low`
    let state2 = example_input.is_high(); // eg `true` or `false`.

    // Set high.
    example_output.set_state(PinState::High);

    // Simpler syntax to set state, and use with a delay, provided by the Cortex-M systick.
    example_output.set_high();
    delay.delay_ms(500);
    example_output.set_low();

    // Unmask interrupt lines associated with the input pins we've configured interrupts
    // for in `setup_pins`.
    unsafe {
        // EXTI line 3 is associated with pins numbered 0 (PA3, PB3 etc)
        NVIC::unmask(pac::Interrupt::EXTI3);
        NVIC::unmask(pac::Interrupt::EXTI4);
        NVIC::unmask(pac::Interrupt::TIM15);
    }

    // Make the debounce timer global, so we can acccess it in interrupt contexts.
    with(|cs| {
        EXAMPLE_OUTPUT.borrow(cs).replace(Some(example_output));
        DEBOUNCE_TIMER.borrow(cs).replace(Some(debounce_timer));
    });

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// Interrupt handler for PB3. This ISR is called when this push button goes low.
fn EXTI3() {
    with(|cs| {
        // with(|cs| {
        // Clear the interrupt flag, to prevent continous firing.
        gpio::clear_exti_interrupt(3);

        // A helper macro to access the pin and timer we stored in mutexes.
        access_global!(DEBOUNCE_TIMER, debounce_timer, cs);
        if debounce_timer.is_enabled() {
            return;
        }

        access_global!(EXAMPLE_OUTPUT, example_output, cs);

        // Set a pin high;
        example_output.set_high();

        debounce_timer.enable();
    });
}

#[interrupt]
/// Interrupt handler for PA4. This ISR is called when this push button goes low.
fn EXTI4() {
    with(|cs| {
        // with(|cs| {
        // Clear the interrupt flag, to prevent continous firing.
        gpio::clear_exti_interrupt(4);

        access_global!(DEBOUNCE_TIMER, debounce_timer, cs);
        if debounce_timer.is_enabled() {
            return;
        }

        // This accomplishes the same as `access_global!`, and demonstrates
        // what that macro does.
        let mut p = EXAMPLE_OUTPUT.borrow(cs).borrow_mut();
        let mut example_output = p.as_mut().unwrap();

        example_output.set_low();

        debounce_timer.enable();
    });
}

#[interrupt]
/// We use tim15 for button debounce.
fn TIM15() {
    // Or, to clear interrupt without a CS: timer::clear_update_interrupt(15);
    with(|cs| {
        access_global!(DEBOUNCE_TIMER, debounce_timer, cs);
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        debounce_timer.clear_interrupt(TimerInterrupt::Update);

        // Disable the timer until next time you press a button.
        debounce_timer.disable();
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
