//! This is the entry point for firmware for the
//! [AnyLeaf conductivity](https://www.anyleaf.org/ec-module) module.
//! Conductivity-measuring code is in `ec.rs`.

#![no_main]
#![no_std]
#![allow(non_snake_case)]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicUsize, Ordering},
};

use cortex_m::{
    self,
    peripheral::NVIC,
};
use critical_section::{with, Mutex};

use cortex_m_rt::entry;

use hal::{
    self,
    clocks::Clocks,
    dac::{Dac, DacBits, DacChannel},
    gpio::{OutputType, Pin, PinMode, Port},
    i2c::I2c,
    low_power,
    pac::{self, interrupt, I2C1, USART1},
    prelude::*,
    timer::{CountDir, OutputCompare, TimChannel, Timer},
    usart::{Usart, UsartConfig, UsartInterrupt},
};

use defmt_rtt as _; // global logger
use panic_probe as _;

mod ec;

// Set up global mutable variables, as `Mutex<RefCell<Option<>>>`. We use a macro to simplify syntax.
make_globals!(
    (I2C, I2c<I2C1>),
    (SENSOR, ec::EcSensor),
    (UART, Usart<USART1>)
);

static EXC_MODE: Mutex<Cell<ExcMode>> = Mutex::new(Cell::new(ExcMode::ReadingOnly));

const MSG_SIZE: usize = 11;
const SUCCESS_MSG: [u8; 3] = [50, 50, 50]; // Send this to indicate success.
const ERROR_MSG: [u8; 3] = [99, 99, 99]; // Send this to indicate an error.

// `OK_BIT` and `ERROR_BIT` are the preceding bit of each reading from the water monitor.
// They indicate a sensor error, not a serial comms error.
const OK_BIT: u8 = 10;
// const ERROR_BIT: u8 = 20;
const MSG_START_BYTES: [u8; 2] = [100, 150];
const MSG_END_BYTES: [u8; 1] = [200];

// See ADS1115 datasheet Section 9.6.3: Config Register. Start a differential conversion on channel
// 0, with 0.512V full scale range, one-shot mode, no alert pin activity.
const EC_CMD: u16 = 0b1100_1001_1000_0000;

// This is the same as the EC command, but uses ADC channel 1. // todo: Higher voltage range?
const T_CMD: u16 = 0b1101_0101_1000_0000;

#[derive(Clone, Copy, PartialEq)]
/// Apply excitation currently only during readings, or always.
pub enum ExcMode {
    ReadingOnly,
    AlwaysOn,
}

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // Set `dac_pin` to analog mode, to prevent parasitic power use.
    let _dac_pin = Pin::new(Port::A, 4, PinMode::Analog);
    let _pwm_pin = Pin::new(Port::A, 0, PinMode::Alt(1));

    // Setup UART for connecting to the host
    let _uart_tx = Pin::new(Port::A, 9, PinMode::Alt(7));
    let _uart_rx = Pin::new(Port::A, 10, PinMode::Alt(7));

    // Set up I2C pins, for communicating with the external ADC.
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);
}

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    // Set up pins with appropriate modes.
    setup_pins();

    // Set up I2C for the TI ADS1115 ADC.
    let i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);

    // todo: Once on new QFN MCU: Gain 0, 1, 2 -> PA6, PA7, PB0
    // Set up pins used to control the gain-resistor-selecting multiplexer.
    let gain0 = Pin::new(Port::B, 0, PinMode::Output);
    let gain1 = Pin::new(Port::B, 1, PinMode::Output);
    let gain2 = Pin::new(Port::B, 2, PinMode::Output);

    // todo: You could probably use the onboard ADC to reduce cost and hardware complexity.

    // todo: Precision voltage ref on VDDA and VSSA, to improve accuracy?
    // set up the DAC, to control voltage into the conductivity circuit.
    let dac = Dac::new(dp.DAC1, DacBits::TwelveR, 3.3);

    // `pwm_timer` is used to change polarity-switching rate of the excitation
    // current across the probe terminals, using an analog switch.
    let mut pwm_timer = Timer::new_tim2(dp.TIM2, 2_400., Default::default(), &clock_cfg);
    pwm_timer.set_auto_reload_preload(true);
    pwm_timer.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.5);
    pwm_timer.enable();

    // Setup UART for connecting to the host
    let mut uart = Usart::new(
        dp.USART1,
        9_600,
        UsartConfig::default(),
        &clock_cfg,
    );

    // Trigger an interrupt if we receive our start character over UART.
    uart.enable_interrupt(UsartInterrupt::CharDetect(MSG_START_BYTES[0]));

    // Initialize to a 1.0 cell constant; will be changed by the user later with a command
    // if required.
    let sensor = ec::EcSensor::new(dac, pwm_timer, (gain0, gain1, gain2), 1.0);

    // Set up the global static variables so we can access them during interrupts.
    with(|cs| {
        I2C.borrow(cs).replace(Some(i2c));
        SENSOR.borrow(cs).replace(Some(sensor));
        UART.borrow(cs).replace(Some(uart));
    });

    unsafe { NVIC::unmask(pac::Interrupt::USART1) }

    loop {
        // Wait until we receive communication over UART; code to handle readings are handled in
        // the `USART1` ISR below.
        low_power::sleep_now();
    }
}

#[interrupt]
/// This Interrupt Service Routine (ISR) is triggered by UART activity. It determines
/// what information is requested using a simple protocol, and adjusts setting,
/// and sends readings over UART as required.
fn USART1() {
    with(|cs| {
        access_global!(UART, uart, cs);
        uart.clear_interrupt(UsartInterrupt::CharDetect(0));

        let mut msg = [0; MSG_SIZE];

        // An alternative approach is to make a global MSG buffer,
        // and populate it with an interrupt at each byte recieved,
        // then process the start and/or end bits specially to trigger
        // this "msg_recieved" code.
        // for i in 0..MSG_SIZE {
        //     // Wait for the next bit
        //     while unsafe {(*pac::USART1::ptr()).isr.read().rxne().bit_is_clear()} {}
        //     msg[i] = rx.read().unwrap_or(0);
        // }
        uart.read(&mut msg);

        // Bits 0:1 are start bits. Bit 2 identifies the command. Bits 3-9
        // can pass additional data to the command. Bit 10 is the end bit.

        if !(msg[0..2] == MSG_START_BYTES && msg[10] == MSG_END_BYTES[0]) {
            uart.write(&ERROR_MSG);
            return;
        }

        match msg[2] {
            10 => {
                // Read conductivity
                access_global!(SENSOR, sensor, cs);
                access_global!(I2C, i2c, cs);

                // Convert the raw reading to a 16-bit integer. It will be the reading in µS/cm / K.
                sensor.dac.enable(DacChannel::C1);

                let reading = sensor.read(0x48, EC_CMD, i2c).unwrap_or(0.);

                // todo: Use integers all the way instead of ADC word -> ec float -> int?

                let reading_to_xmit = (reading * 1_000_000. / sensor.K_cell) as u16;

                if EXC_MODE.borrow(cs).get() == crate::ExcMode::ReadingOnly {
                    sensor.dac.disable(DacChannel::C1);
                }

                // Split the u16 into 2 bytes.
                let r: [u8; 2] = reading_to_xmit.to_be_bytes();
                uart.write(&[OK_BIT, r[0], r[1]]);
            }

            11 => {
                // Read temperature
                access_global!(I2C, i2c, cs);

                let reading = ec::take_reading(0x48, T_CMD, i2c);

                // Split the i16 into 2 bytes. Send these bytes as-is from the ADC;
                // you'll need to decode with the reading software.
                let r: [u8; 2] = reading.to_be_bytes();
                uart.write(&[OK_BIT, r[0], r[1]]);
            }

            12 => {
                // Set excitation current mode
                EXC_MODE.borrow(cs).set(match msg[3] {
                    0 => ExcMode::ReadingOnly,
                    1 => ExcMode::AlwaysOn,
                    _ => {
                        uart.write(&ERROR_MSG);
                        return;
                    }
                });

                uart.write(&SUCCESS_MSG);
            }

            13 => {
                // Set cell constant, to accomodate different probes.
                access_global!(SENSOR, sensor, cs);
                sensor.K_cell = match msg[3] {
                    0 => 0.01,
                    1 => 0.1,
                    2 => 1.,
                    3 => 10.,
                    _ => {
                        uart.write(&ERROR_MSG);
                        return;
                    }
                };

                uart.write(&SUCCESS_MSG);
            }
            _ => {
                uart.write(&ERROR_MSG);
            }
        }
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}