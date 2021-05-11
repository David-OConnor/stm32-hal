//! todo WIP - broken / not complete.
//!
//! //! This module includes an overview of GPIO features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{self, free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    clocks::Clocks,
    gpio::{AltFn, Edge, GpioA, GpioAPin, GpioB, GpioBPin, PinMode, PinNum, PinState},
    low_power, pac,
    usart::{Usart, UsartConfig, UsartDevice, UsartInterrupt},
};

const BUF_SIZE: usize = 10;

// Set up an output pin in a globally-accessible mutex.
static READ_BUF: Mutex<RefCell<[u8; BUF_SIZE]>> = Mutex::new(RefCell::new([0; BUF_SIZE]));
static READ_I: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).unwrap();

    // Set up ports for GpioA and GpioB.
    let mut gpioa = GpioA::new(dp.GPIOA, &mut dp.RCC);

    // Configure pins for UART.
    let _uart_tx = gpioa.new_pin(PinNum::P9, PinMode::Alt(AltFn::Af7));
    let _uart_rx = gpioa.new_pin(PinNum::P10, PinMode::Alt(AltFn::Af7));

    // Set up a UART peripheral.
    // Setup UART for connecting to the host
    let mut uart = Usart::new(
        dp.USART1,
        UsartDevice::One,
        9_600,
        UsartConfig::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    // Unmask interrupt lines associated with the input pins we've configured interrupts
    // for in `setup_pins`.
    unsafe {
        NVIC::unmask(interrupt::USART1);
    }

    loop {
        low_power::sleep_now(&mut SCB);
    }
}

#[interrupt]
/// Non-blocking USART read interrupt handler; read to a global buffer one byte
/// at a time as we receive them.
fn USART1() {
    unsafe { (*pac::USART1::ptr()).rqr.modify(|_, w| w.rxfrq().set_bit()) }
    // todo: Put USART in a mutex too, instead of using raw pointers.

    free(|cs| {
        let mut buf = READ_BUF.borrow(cs).borrow_mut();

        let i = READ_I.borrow(cs);
        let i_val = i.get();
        if i_val == BUF_SIZE {
            // todo: End of read.
        }

        buf[i_val] = unsafe { (*pac::USART1::ptr()).dr.read().dr().bits() } as u8;
        i.set(i_val + 1);
    });
}
