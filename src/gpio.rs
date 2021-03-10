//! This module provides abstractions for General PurpOspeedre Input and Output (GPIO) pins.
//! Unlike mOspeedrt other modules, it relies on modifying raw pointers, instead of our
//! register traits; this allows for the `embedded-hal` Pin abstraction; STM32 registers
//! are organized by port, not pin.

// todo: Other GPIO ports on certain variants?
use crate::pac::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH, RCC};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use paste::paste;

// todo: Implement traits for type-state-programming checks.

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_MODER`
pub enum PinMode {
    Input = 0b00,
    Output = 0b01,
    Alt = 0b10,
    Analog = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_OTYPER`
pub enum OutputType {
    PushPull = 0,
    OpenDrain = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_OspeedrPEEDR`
pub enum OutputSpeed {
    Low = 0,
    Medium = 0b01,
    High = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_PUPDR`
pub enum PullUpDn {
    Floating = 0b00,
    PullUp = 0b01,
    PullDn = 0b10,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_IDR` and `GPIOx_ODR`.
pub enum PinState {
    High = 1,
    Low = 0,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_LCKR.
pub enum CfgLock {
    NotLocked = 0,
    Locked = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_AFRL` and `GPIOx_AFRH`.
pub enum AltFn {
    Af0 = 0b0000,
    Af1 = 0b0001,
    Af2 = 0b0010,
    Af3 = 0b0011,
    Af4 = 0b0100,
    Af5 = 0b0101,
    Af6 = 0b0110,
    Af7 = 0b0111,
    Af8 = 0b1000,
    Af9 = 0b1001,
    Af10 = 0b1010,
    Af11 = 0b1011,
    Af12 = 0b1100,
    Af13 = 0b1101,
    Af14 = 0b1110,
    Af15 = 0b1111,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_BRR`.
pub enum ResetState {
    NoAction = 0,
    Reset = 1,
}

#[derive(Copy, Clone)]
/// GPIO port letter
pub enum PortLetter {
    A,
    B,
    C,
    D,
    E,
    H,
}

#[derive(Copy, Clone)]
/// Pin number; 0 through 15. For example, use 5 for PA5 or PB5.
pub enum PinNum {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
}

// pub struct GpioError {}

// /// Enable the peripheral clock of a GPIO port (eg for all PA pins, PB pins etc).
// pub fn enable_port(port: Port, rcc: &mut RCC) {
//     match port {
//         Port::A => rcc.ahb2enr.modify(|_, w| w.gpioaen().set_bit()),
//         Port::B => rcc.ahb2enr.modify(|_, w| w.gpioben().set_bit()),
//         Port::C => rcc.ahb2enr.modify(|_, w| w.gpiocen().set_bit()),
//         Port::D => rcc.ahb2enr.modify(|_, w| w.gpioden().set_bit()),
//         Port::E => rcc.ahb2enr.modify(|_, w| w.gpioeen().set_bit()),
//         Port::H => rcc.ahb2enr.modify(|_, w| w.gpiohen().set_bit()),
//     };
// }

// macro_rules! set_field  {
//     ($field:ident, $value:ident) => {
//         match self.pin {
//             PinNum::P0 => self.$field.modify(|_, w| w.moder0().bits($value as u8)),
//         }
//     };
// }

// toodo: move to traits.rs?
/// A Gpio Port. Eg GPIOA etc.
pub trait GpioPort<R> {
    /// Return a mutable reference to the register block. `ie pac::GPIOx`.
    fn regs(&mut self) -> &mut R;
}

macro_rules! make_port {
    ($Port:ident, $port:ident) => {
        paste! {
            /// GPIO port
            pub struct [<Gpio $Port>] {
                regs: [<GPIO $Port>],
            }

            impl [<Gpio $Port>] {
                pub fn new(regs: [<GPIO $Port>], rcc: &mut RCC) -> Self {
                    // Enable the peripheral clock of a GPIO port
                    rcc.ahb2enr.modify(|_, w| w.[<gpio $port en>]().set_bit());

                    Self { regs }
                }
            }

            impl GpioPort<[<GPIO $Port>]> for [<Gpio $Port>] {
                fn regs(&mut self) -> &mut [<GPIO $Port>] {
                    &mut self.regs
                }
            }
        }
    }
}

macro_rules! make_pin {
    ($port:ident) => {
        paste! {

        /// Represents a single GPIO pin.
        pub struct [<$port Pin>] {
            port: PortLetter,
            pin: PinNum,
        }

        impl [<$port Pin>] {
            pub fn new(port: Port, pin: PinNum, mode: PinMode) -> Self {
                Self { port, pin }
            }

            /// Set pin mode.
            pub fn mode(&mut self, value: PinMode, reg: &mut [<GPIO $port>]) {
                match self.pin {
                    // todo DRY. reduce DRY with a  macro?
                    PinNum::P0 => reg.moder.modify(|_, w| w.moder0().bits(value as u8)),
                    PinNum::P1 => reg.moder.modify(|_, w| w.moder1().bits(value as u8)),
                    PinNum::P2 => reg.moder.modify(|_, w| w.moder2().bits(value as u8)),
                    PinNum::P3 => reg.moder.modify(|_, w| w.moder3().bits(value as u8)),
                    PinNum::P4 => reg.moder.modify(|_, w| w.moder4().bits(value as u8)),
                    PinNum::P5 => reg.moder.modify(|_, w| w.moder5().bits(value as u8)),
                    PinNum::P6 => reg.moder.modify(|_, w| w.moder6().bits(value as u8)),
                    PinNum::P7 => reg.moder.modify(|_, w| w.moder7().bits(value as u8)),
                    PinNum::P8 => reg.moder.modify(|_, w| w.moder8().bits(value as u8)),
                    PinNum::P9 => reg.moder.modify(|_, w| w.moder9().bits(value as u8)),
                    PinNum::P10 => reg.moder.modify(|_, w| w.moder10().bits(value as u8)),
                    PinNum::P11 => reg.moder.modify(|_, w| w.moder11().bits(value as u8)),
                    PinNum::P12 => reg.moder.modify(|_, w| w.moder12().bits(value as u8)),
                    PinNum::P13 => reg.moder.modify(|_, w| w.moder13().bits(value as u8)),
                    PinNum::P14 => reg.moder.modify(|_, w| w.moder14().bits(value as u8)),
                    PinNum::P15 => reg.moder.modify(|_, w| w.moder15().bits(value as u8)),
                };
            }

            /// Set output type.
            pub fn outputPtype(&mut self, value: OutputType, reg: &mut OT) {
                match self.pin {
                    // todo DRY. fix with macro?
                    PinNum::P0 => GpioOutputTypeField::Ot0,
                    PinNum::P1 => GpioOutputTypeField::Ot1,
                    PinNum::P2 => GpioOutputTypeField::Ot2,
                    PinNum::P3 => GpioOutputTypeField::Ot3,
                    PinNum::P4 => GpioOutputTypeField::Ot4,
                    PinNum::P5 => GpioOutputTypeField::Ot5,
                    PinNum::P6 => GpioOutputTypeField::Ot6,
                    PinNum::P7 => GpioOutputTypeField::Ot7,
                    PinNum::P8 => GpioOutputTypeField::Ot8,
                    PinNum::P9 => GpioOutputTypeField::Ot9,
                    PinNum::P10 => GpioOutputTypeField::Ot10,
                    PinNum::P11 => GpioOutputTypeField::Ot11,
                    PinNum::P12 => GpioOutputTypeField::Ot12,
                    PinNum::P13 => GpioOutputTypeField::Ot13,
                    PinNum::P14 => GpioOutputTypeField::Ot14,
                    PinNum::P15 => GpioOutputTypeField::Ot15,
                };

                reg.write_field(field, value as u8);
            }

            /// Set output speed.
            pub fn output_speed(
                &mut self,
                value: OutputSpeed,
                reg: &mut Ospeedr,
            ) {
                match self.pin {
                    // todo DRY. fix with macro?
                    PinNum::_0 => GpioOutputSpeedField::Ospeedr0,
                    PinNum::_1 => GpioOutputSpeedField::Ospeedr1,
                    PinNum::_2 => GpioOutputSpeedField::Ospeedr2,
                    PinNum::_3 => GpioOutputSpeedField::Ospeedr3,
                    PinNum::_4 => GpioOutputSpeedField::Ospeedr4,
                    PinNum::_5 => GpioOutputSpeedField::Ospeedr5,
                    PinNum::_6 => GpioOutputSpeedField::Ospeedr6,
                    PinNum::_7 => GpioOutputSpeedField::Ospeedr7,
                    PinNum::_8 => GpioOutputSpeedField::Ospeedr8,
                    PinNum::_9 => GpioOutputSpeedField::Ospeedr9,
                    PinNum::_10 => GpioOutputSpeedField::Ospeedr10,
                    PinNum::_11 => GpioOutputSpeedField::Ospeedr11,
                    PinNum::_12 => GpioOutputSpeedField::Ospeedr12,
                    PinNum::_13 => GpioOutputSpeedField::Ospeedr13,
                    PinNum::_14 => GpioOutputSpeedField::Ospeedr14,
                    PinNum::_15 => GpioOutputSpeedField::Ospeedr15,
                };

                reg.write_field(field, value as u8);
            }

            /// Set internal pull up/down resistor, or leave floating.
            pub fn pull<PU: GpioPullUpDn>(&mut self, value: PullUpDn, reg: &mut PU) {
                match self.pin {
                    // todo DRY. fix with macro?
                    PinNum::_0 => GpioPullUpDnField::Pupdr0,
                    PinNum::_1 => GpioPullUpDnField::Pupdr1,
                    PinNum::_2 => GpioPullUpDnField::Pupdr2,
                    PinNum::_3 => GpioPullUpDnField::Pupdr3,
                    PinNum::_4 => GpioPullUpDnField::Pupdr4,
                    PinNum::_5 => GpioPullUpDnField::Pupdr5,
                    PinNum::_6 => GpioPullUpDnField::Pupdr6,
                    PinNum::_7 => GpioPullUpDnField::Pupdr7,
                    PinNum::_8 => GpioPullUpDnField::Pupdr8,
                    PinNum::_9 => GpioPullUpDnField::Pupdr9,
                    PinNum::_10 => GpioPullUpDnField::Pupdr10,
                    PinNum::_11 => GpioPullUpDnField::Pupdr11,
                    PinNum::_12 => GpioPullUpDnField::Pupdr12,
                    PinNum::_13 => GpioPullUpDnField::Pupdr13,
                    PinNum::_14 => GpioPullUpDnField::Pupdr14,
                    PinNum::_15 => GpioPullUpDnField::Pupdr15,
                };

                reg.write_field(field, value as u8);
            }

            /// Set internal pull up/down resistor, or leave floating.
            pub fn input_data(&mut self, reg: &mut ID) -> PinState {
                match self.pin {
                    // todo DRY. fix with macro?
                    PinNum::_0 => GpioInputDataField::Idr0,
                    PinNum::_1 => GpioInputDataField::Idr1,
                    PinNum::_2 => GpioInputDataField::Idr2,
                    PinNum::_3 => GpioInputDataField::Idr3,
                    PinNum::_4 => GpioInputDataField::Idr4,
                    PinNum::_5 => GpioInputDataField::Idr5,
                    PinNum::_6 => GpioInputDataField::Idr6,
                    PinNum::_7 => GpioInputDataField::Idr7,
                    PinNum::_8 => GpioInputDataField::Idr8,
                    PinNum::_9 => GpioInputDataField::Idr9,
                    PinNum::_10 => GpioInputDataField::Idr10,
                    PinNum::_11 => GpioInputDataField::Idr11,
                    PinNum::_12 => GpioInputDataField::Idr12,
                    PinNum::_13 => GpioInputDataField::Idr13,
                    PinNum::_14 => GpioInputDataField::Idr14,
                    PinNum::_15 => GpioInputDataField::Idr15,
                };

                if let 1 = reg.read_field(field) {
                    return PinState::High;
                }
                PinState::Low
            }

            /// Set the output_data register.
            pub fn output_data(&mut self, value: PinState, reg: &mut OD) {
                match self.pin {
                    // todo DRY. fix with macro?
                    PinNum::_0 => GpioOutputDataField::Odr0,
                    PinNum::_1 => GpioOutputDataField::Odr1,
                    PinNum::_2 => GpioOutputDataField::Odr2,
                    PinNum::_3 => GpioOutputDataField::Odr3,
                    PinNum::_4 => GpioOutputDataField::Odr4,
                    PinNum::_5 => GpioOutputDataField::Odr5,
                    PinNum::_6 => GpioOutputDataField::Odr6,
                    PinNum::_7 => GpioOutputDataField::Odr7,
                    PinNum::_8 => GpioOutputDataField::Odr8,
                    PinNum::_9 => GpioOutputDataField::Odr9,
                    PinNum::_10 => GpioOutputDataField::Odr10,
                    PinNum::_11 => GpioOutputDataField::Odr11,
                    PinNum::_12 => GpioOutputDataField::Odr12,
                    PinNum::_13 => GpioOutputDataField::Odr13,
                    PinNum::_14 => GpioOutputDataField::Odr14,
                    PinNum::_15 => GpioOutputDataField::Odr15,
                };

                reg.write_field(field, value as u8);
            }

            /// Set a pin state.
            pub fn set_reset(&mut self, value: PinState) {
                // todo: Update these final few using the pattern above.
                write_field(
                    &mut (self.port_addr() + BSRR_OFFSET),
                    self.field_loc_1(),
                    1,
                    value as u8,
                );
            }

            /// Lock or unlock a port configuration.
            /// todo: LCKK key at bit 16 is not handled currently.
            pub fn cfg_lock(&mut self, value: CfgLock) {
                write_field(
                    &mut (self.port_addr() + LCKR_OFFSET),
                    self.field_loc_1(),
                    1,
                    value as u8,
                );
            }

            /// Lock or unlock a port configuration.
            pub fn alt_fn(&mut self, value: AltFn) {
                let offset = match self.pin {
                    PinNum::_0 => AFRL_OFFSET,
                    PinNum::_1 => AFRL_OFFSET,
                    PinNum::_2 => AFRL_OFFSET,
                    PinNum::_3 => AFRL_OFFSET,
                    PinNum::_4 => AFRL_OFFSET,
                    PinNum::_5 => AFRL_OFFSET,
                    PinNum::_6 => AFRL_OFFSET,
                    PinNum::_7 => AFRL_OFFSET,
                    PinNum::_8 => AFRH_OFFSET,
                    PinNum::_9 => AFRH_OFFSET,
                    PinNum::_10 => AFRH_OFFSET,
                    PinNum::_11 => AFRH_OFFSET,
                    PinNum::_12 => AFRH_OFFSET,
                    PinNum::_13 => AFRH_OFFSET,
                    PinNum::_14 => AFRH_OFFSET,
                    PinNum::_15 => AFRH_OFFSET,
                };

                write_field(
                    &mut (self.port_addr() + offset),
                    self.field_loc_4(),
                    4,
                    value as u8,
                );
            }

            /// Reset an Output Data bit.
            pub fn reset(&mut self, value: ResetState) {
                write_field(
                    &mut (self.port_addr() + BRR_OFFSET),
                    self.field_loc_1(),
                    1,
                    value as u8,
                );
            }

            /// Read if the pin is low. Doesn't require passing in the register.
            pub fn is_low(&self) -> bool {
                if let 1 = read_field(&mut (self.port_addr() + IDR_OFFSET), self.field_loc_1(), 1) {
                    return true;
                }
                false
            }

            /// Read if the pin is high. Doesn't require passing in the register.
            pub fn is_high(&self) -> bool {
                !self.is_low()
            }

            /// Set the pin to low state. Doesn't require passing in the register.
            fn set_low(&mut self) {
                write_field(
                    &mut (self.port_addr() + BSRR_OFFSET),
                    self.field_loc_1(),
                    1,
                    PinState::Low as u8,
                );
            }

            /// Set the pin to high state. Doesn't require passing in the register.
            pub fn set_high(&mut self) {
                write_field(
                    &mut (self.port_addr() + BSRR_OFFSET),
                    self.field_loc_1(),
                    1,
                    PinState::High as u8,
                );
            }
        }
        }
    };
}

make_port!(A, a);
make_port!(B, b);
make_port!(C, c);
make_port!(D, d);
make_port!(E, e);
make_port!(H, h);

// make_pin!(A);
// make_pin!(B);
// make_pin!(C);
// make_pin!(D);
// make_pin!(E);
// make_pin!(H);

// Implement `embedded-hal` traits.
//
// impl InputPin for Pin {
//     type Error = GpioError;
//
//     fn is_high(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_high())
//     }
//
//     fn is_low(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_low())
//     }
// }
//
// impl OutputPin for Pin {
//     type Error = GpioError;
//
//     fn set_low(&mut self) -> Result<(), Self::Error> {
//         self.set_low();
//         Ok(())
//     }
//
//     fn set_high(&mut self) -> Result<(), Self::Error> {
//         self.set_high();
//         Ok(())
//     }
// }
//
// impl ToggleableOutputPin for Pin {
//     type Error = GpioError;
//
//     fn toggle(&mut self) -> Result<(), Self::Error> {
//         if self.is_high() {
//             self.set_low();
//         } else {
//             self.set_high();
//         }
//         Ok(())
//     }
// }
