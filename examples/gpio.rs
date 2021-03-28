//! This module includes an overview of GPIO features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{self, free},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal::{
    adc::{Adc, AdcChannel, Align, CkMode, InputType, OperationMode},
    clocks::Clocks,
    gpio::{AltFn, Edge, GpioA, GpioAPin, GpioB, GpioBPin, PinMode, PinNum, PinState},
    low_power,
    pac,
    prelude::*, // The prelude includes traits we use here like `InputPin` and `OutputPin`.
};

// Set up an output pin in a globally-accessible mutex. This is useful for accessing
// peripherals in interrupt contexts. We use a macro imported in the
// `prelude` module to simplify this syntax, and accessing it later.
setup_globals!((EXAMPLE_OUTPUT, example_output));

/// This function includes type signature examples using `GpioPin`s from this library,
/// and generic ones that implemented `embedded-hal` traits.
fn example_type_sigs<O: OutputPin>(pin1: &mut O, pin2: &mut GpioBPin) {
    let setting = pin2.is_high();

    pin1.set_low();
}

/// An example function to set up the pins that don't need to be interacted with directly later.
/// For example, ones used with buses (eg I2C, SPI, UART), USB, ADC, and DAC pins.
/// This may also include input pins that trigger interrupts, and aren't polled.
pub fn setup_pins(
    gpioa: &mut GpioA,
    gpiob: &mut GpioB,
    exti: &mut pac::EXTI,
    syscfg: &mut pac::SYSCFG,
) {
    // Set up I2C pins
    let mut scl = gpiob.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af4));
    scl.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    let mut sda = gpiob.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af4));
    sda.output_type(OutputType::OpenDrain, &mut gpiob.regs);

    // Set up SPI pins
    let _sck = gpioa.new_pin(PinNum::P5, PinMode::Alt(AltFn::Af5));
    let _miso = gpioa.new_pin(PinNum::P6, PinMode::Alt(AltFn::Af5));
    let _mosi = gpioa.new_pin(PinNum::P7, PinMode::Alt(AltFn::Af5));

    // Set up UART pins
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));

    // Set up USB pins
    let _usb_dm = gpioa.new_pin(PinNum::P11, PinMode::Alt(AltFn::Af14));
    let _usb_dp = gpioa.new_pin(PinNum::P12, PinMode::Alt(AltFn::Af14));

    // Set the ADC pin to analog mode, to prevent parasitic power use.
    let _adc_pin = gpiob.new_pin(PinNum::P0, PinMode::Analog);

    // Set DAC pin to analog mode, to prevent parasitic power use.
    let _dac_pin = gpioa.new_pin(PinNum::P4, PinMode::Analog);

    // Set up PWM.  // Timer 2, channel 1.
    let _pwm_pin = gpioa.new_pin(PinNum::P0, PinMode::Alt(AltFn::Af1));

    // Set up buttons, with pull-up resistors that trigger on the falling edge.
    let mut up_btn = gpiob.new_pin(PinNum::P3, PinMode::Input);
    up_btn.pull(Pull::Up, &mut gpiob.regs);
    up_btn.enable_interrupt(Edge::Falling, exti, syscfg);

    let mut dn_btn = gpioa.new_pin(PinNum::P4, PinMode::Input);
    dn_btn.pull(Pull::Up, &mut gpioa.regs);
    dn_btn.enable_interrupt(Edge::Falling, exti, syscfg);
}

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

    // Set up ports for GpioA and GpioB.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // Call a function we've made to help organize our pin setup code.
    setup_pins(&mut gpia, &mut gpiob, &mut dp.exti, &mut dp.syscfg);

    // Example pins PB5 and PB6.
    let mut example_output = gpiob.new_pin(PinNum::P5, PinMode::Output);
    let mut example_input = gpiob.new_pin(PinNum::P6, PinMode::Input);

    example_type_sigs(&mut example_output, &mut example_input);

    // Set high using the `OutputPin` trait.
    example_output.set_high();

    // Unmask interrupt lines associated with the input pins we've configured interrupts
    // for in `setup_pins`.
    unsafe {
        NVIC::unmask(interrupt::EXTI3);
        NVIC::unmask(interrupt::EXTI4);
    }

    loop {
        low_power::sleep_now(&mut SCB);
    }
}

#[interrupt]
/// Interrupt handler for PB3. This ISR is called when this push button goes low.
fn EXTI3() {
    free(|cs| {
        // Clear the interrupt flag, to prevent continous firing.
        unsafe { (*EXTI::ptr()).pr1.modify(|_, w| w.pr3().bit(true)) }

        // A helper macro to access the pin we stored in a mutex.
        access_global!(EXAMPLE_OUTPUT, example_output, cs);

        // Set a pin high;
        example_output.set_high();
    });
}

#[interrupt]
/// Interrupt handler for PA4. This ISR is called when this push button goes low.
fn EXTI4() {
    free(|cs| {
        // Clear the interrupt flag, to prevent continous firing.
        unsafe { (*EXTI::ptr()).pr1.modify(|_, w| w.pr4().bit(true)) }

        // This accomplishes the same as `access_global!`, and demonstrates
        // what that macro does.
        let mut p = EXAMPLE_OUTPUT.borrow(cs).borrow_mut();
        let mut example_output = p.as_mut().unwrap();

        example_output.set_low();
    });
}
