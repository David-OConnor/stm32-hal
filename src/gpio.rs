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
pub enum Pull {
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

pub struct GpioError {}

// macro_rules! set_field  {
//     ($field:ident, $value:ident) => {
//         match self.pin {
//             PinNum::P0 => self.$field.modify(|_, w| w.moder0().bits($value as u8)),
//         }
//     };
// }

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

                    cfg_if::cfg_if! {
                        if #[cfg(feature = "f3")] {
                            rcc.ahbenr.modify(|_, w| w.[<iop $port en>]().set_bit());
                        } else  {
                            rcc.ahb2enr.modify(|_, w| w.[<gpio $port en>]().set_bit());
                        }
                    }

                    Self { regs }
                }

                pub fn new_pin(&mut self, pin: PinNum, mode: PinMode) -> [<Gpio $Port Pin>] {
                    let mut result = [<Gpio $Port Pin>] { port: PortLetter::[<$Port>], pin };
                    result.mode(mode, &mut self.regs);
                    result
                }
            }
        }
    };
}

macro_rules! make_pin {
    ($Port:ident) => {
        paste! {

        /// Represents a single GPIO pin.
        pub struct [<Gpio $Port Pin>] {
            port: PortLetter,
            pin: PinNum,
        }

        impl [<Gpio $Port Pin>] {
            // pub fn new(port: PortLetter, pin: PinNum, reg: &mut [<GPIO $port>]) -> Self {
            //     Self { port, pin }
            // }

            /// Set pin mode.
            pub fn mode(&mut self, value: PinMode, regs: &mut [<GPIO $Port>]) {
                match self.pin {
                    // todo DRY. reduce DRY with a  macro?
                    PinNum::P0 => regs.moder.modify(|_, w| w.moder0().bits(value as u8)),
                    PinNum::P1 => regs.moder.modify(|_, w| w.moder1().bits(value as u8)),
                    PinNum::P2 => regs.moder.modify(|_, w| w.moder2().bits(value as u8)),
                    PinNum::P3 => regs.moder.modify(|_, w| w.moder3().bits(value as u8)),
                    PinNum::P4 => regs.moder.modify(|_, w| w.moder4().bits(value as u8)),
                    PinNum::P5 => regs.moder.modify(|_, w| w.moder5().bits(value as u8)),
                    PinNum::P6 => regs.moder.modify(|_, w| w.moder6().bits(value as u8)),
                    PinNum::P7 => regs.moder.modify(|_, w| w.moder7().bits(value as u8)),
                    PinNum::P8 => regs.moder.modify(|_, w| w.moder8().bits(value as u8)),
                    PinNum::P9 => regs.moder.modify(|_, w| w.moder9().bits(value as u8)),
                    PinNum::P10 => regs.moder.modify(|_, w| w.moder10().bits(value as u8)),
                    PinNum::P11 => regs.moder.modify(|_, w| w.moder11().bits(value as u8)),
                    PinNum::P12 => regs.moder.modify(|_, w| w.moder12().bits(value as u8)),
                    PinNum::P13 => regs.moder.modify(|_, w| w.moder13().bits(value as u8)),
                    PinNum::P14 => regs.moder.modify(|_, w| w.moder14().bits(value as u8)),
                    PinNum::P15 => regs.moder.modify(|_, w| w.moder15().bits(value as u8)),
                };
            }

            /// Set output type.
            pub fn output_type(&mut self, value: OutputType, regs: &mut [<GPIO $Port>]) {
                match self.pin {
                    PinNum::P0 => regs.otyper.modify(|_, w| w.ot0().bit(value as u8 != 0)),
                    PinNum::P1 => regs.otyper.modify(|_, w| w.ot1().bit(value as u8 != 0)),
                    PinNum::P2 => regs.otyper.modify(|_, w| w.ot2().bit(value as u8 != 0)),
                    PinNum::P3 => regs.otyper.modify(|_, w| w.ot3().bit(value as u8 != 0)),
                    PinNum::P4 => regs.otyper.modify(|_, w| w.ot4().bit(value as u8 != 0)),
                    PinNum::P5 => regs.otyper.modify(|_, w| w.ot5().bit(value as u8 != 0)),
                    PinNum::P6 => regs.otyper.modify(|_, w| w.ot6().bit(value as u8 != 0)),
                    PinNum::P7 => regs.otyper.modify(|_, w| w.ot7().bit(value as u8 != 0)),
                    PinNum::P8 => regs.otyper.modify(|_, w| w.ot8().bit(value as u8 != 0)),
                    PinNum::P9 => regs.otyper.modify(|_, w| w.ot9().bit(value as u8 != 0)),
                    PinNum::P10 => regs.otyper.modify(|_, w| w.ot10().bit(value as u8 != 0)),
                    PinNum::P11 => regs.otyper.modify(|_, w| w.ot11().bit(value as u8 != 0)),
                    PinNum::P12 => regs.otyper.modify(|_, w| w.ot12().bit(value as u8 != 0)),
                    PinNum::P13 => regs.otyper.modify(|_, w| w.ot13().bit(value as u8 != 0)),
                    PinNum::P14 => regs.otyper.modify(|_, w| w.ot14().bit(value as u8 != 0)),
                    PinNum::P15 => regs.otyper.modify(|_, w| w.ot15().bit(value as u8 != 0)),
                };
            }

            /// Set output speed.
            pub fn output_speed(
                &mut self,
                value: OutputSpeed,
                regs: &mut [<GPIO $Port>]
            ) {
                // unsafe here etc applies to f3, but not l4.
                unsafe {
                    match self.pin {
                        PinNum::P0 => regs.ospeedr.modify(|_, w| w.ospeedr0().bits(value as u8)),
                        PinNum::P1 => regs.ospeedr.modify(|_, w| w.ospeedr1().bits(value as u8)),
                        PinNum::P2 => regs.ospeedr.modify(|_, w| w.ospeedr2().bits(value as u8)),
                        PinNum::P3 => regs.ospeedr.modify(|_, w| w.ospeedr3().bits(value as u8)),
                        PinNum::P4 => regs.ospeedr.modify(|_, w| w.ospeedr4().bits(value as u8)),
                        PinNum::P5 => regs.ospeedr.modify(|_, w| w.ospeedr5().bits(value as u8)),
                        PinNum::P6 => regs.ospeedr.modify(|_, w| w.ospeedr6().bits(value as u8)),
                        PinNum::P7 => regs.ospeedr.modify(|_, w| w.ospeedr7().bits(value as u8)),
                        PinNum::P8 => regs.ospeedr.modify(|_, w| w.ospeedr8().bits(value as u8)),
                        PinNum::P9 => regs.ospeedr.modify(|_, w| w.ospeedr9().bits(value as u8)),
                        PinNum::P10 => regs.ospeedr.modify(|_, w| w.ospeedr10().bits(value as u8)),
                        PinNum::P11 => regs.ospeedr.modify(|_, w| w.ospeedr11().bits(value as u8)),
                        PinNum::P12 => regs.ospeedr.modify(|_, w| w.ospeedr12().bits(value as u8)),
                        PinNum::P13 => regs.ospeedr.modify(|_, w| w.ospeedr13().bits(value as u8)),
                        PinNum::P14 => regs.ospeedr.modify(|_, w| w.ospeedr14().bits(value as u8)),
                        PinNum::P15 => regs.ospeedr.modify(|_, w| w.ospeedr15().bits(value as u8)),
                    };
                }
            }

            /// Set internal pull up/down resistor, or leave floating.
            pub fn pull(&mut self, value: Pull, regs: &mut [<GPIO $Port>]) {
                unsafe {
                    match self.pin {
                        PinNum::P0 => regs.pupdr.modify(|_, w| w.pupdr0().bits(value as u8)),
                        PinNum::P1 => regs.pupdr.modify(|_, w| w.pupdr1().bits(value as u8)),
                        PinNum::P2 => regs.pupdr.modify(|_, w| w.pupdr2().bits(value as u8)),
                        PinNum::P3 => regs.pupdr.modify(|_, w| w.pupdr3().bits(value as u8)),
                        PinNum::P4 => regs.pupdr.modify(|_, w| w.pupdr4().bits(value as u8)),
                        PinNum::P5 => regs.pupdr.modify(|_, w| w.pupdr5().bits(value as u8)),
                        PinNum::P6 => regs.pupdr.modify(|_, w| w.pupdr6().bits(value as u8)),
                        PinNum::P7 => regs.pupdr.modify(|_, w| w.pupdr7().bits(value as u8)),
                        PinNum::P8 => regs.pupdr.modify(|_, w| w.pupdr8().bits(value as u8)),
                        PinNum::P9 => regs.pupdr.modify(|_, w| w.pupdr9().bits(value as u8)),
                        PinNum::P10 => regs.pupdr.modify(|_, w| w.pupdr10().bits(value as u8)),
                        PinNum::P11 => regs.pupdr.modify(|_, w| w.pupdr11().bits(value as u8)),
                        PinNum::P12 => regs.pupdr.modify(|_, w| w.pupdr12().bits(value as u8)),
                        PinNum::P13 => regs.pupdr.modify(|_, w| w.pupdr13().bits(value as u8)),
                        PinNum::P14 => regs.pupdr.modify(|_, w| w.pupdr14().bits(value as u8)),
                        PinNum::P15 => regs.pupdr.modify(|_, w| w.pupdr15().bits(value as u8)),
                    };
                }
            }

            /// Set internal pull up/down resistor, or leave floating.
            pub fn input_data(&mut self, regs: &mut [<GPIO $Port>]) -> PinState {
                let val = match self.pin {
                    PinNum::P0 => regs.idr.read().idr0().bit(),
                    PinNum::P1 => regs.idr.read().idr1().bit(),
                    PinNum::P2 => regs.idr.read().idr2().bit(),
                    PinNum::P3 => regs.idr.read().idr3().bit(),
                    PinNum::P4 => regs.idr.read().idr4().bit(),
                    PinNum::P5 => regs.idr.read().idr5().bit(),
                    PinNum::P6 => regs.idr.read().idr6().bit(),
                    PinNum::P7 => regs.idr.read().idr7().bit(),
                    PinNum::P8 => regs.idr.read().idr8().bit(),
                    PinNum::P9 => regs.idr.read().idr9().bit(),
                    PinNum::P10 => regs.idr.read().idr10().bit(),
                    PinNum::P11 => regs.idr.read().idr11().bit(),
                    PinNum::P12 => regs.idr.read().idr12().bit(),
                    PinNum::P13 => regs.idr.read().idr13().bit(),
                    PinNum::P14 => regs.idr.read().idr14().bit(),
                    PinNum::P15 => regs.idr.read().idr15().bit(),
                };

                if val {
                    PinState::High
                } else {
                    PinState::Low
                }
            }

            /// Set the output_data register.
            pub fn output_data(&mut self, value: PinState, regs: &mut [<GPIO $Port>]) {
                match self.pin {
                    PinNum::P0 => regs.odr.modify(|_, w| w.odr0().bit(value as u8 != 0)),
                    PinNum::P1 => regs.odr.modify(|_, w| w.odr1().bit(value as u8 != 0)),
                    PinNum::P2 => regs.odr.modify(|_, w| w.odr2().bit(value as u8 != 0)),
                    PinNum::P3 => regs.odr.modify(|_, w| w.odr3().bit(value as u8 != 0)),
                    PinNum::P4 => regs.odr.modify(|_, w| w.odr4().bit(value as u8 != 0)),
                    PinNum::P5 => regs.odr.modify(|_, w| w.odr5().bit(value as u8 != 0)),
                    PinNum::P6 => regs.odr.modify(|_, w| w.odr6().bit(value as u8 != 0)),
                    PinNum::P7 => regs.odr.modify(|_, w| w.odr7().bit(value as u8 != 0)),
                    PinNum::P8 => regs.odr.modify(|_, w| w.odr8().bit(value as u8 != 0)),
                    PinNum::P9 => regs.odr.modify(|_, w| w.odr9().bit(value as u8 != 0)),
                    PinNum::P10 => regs.odr.modify(|_, w| w.odr10().bit(value as u8 != 0)),
                    PinNum::P11 => regs.odr.modify(|_, w| w.odr11().bit(value as u8 != 0)),
                    PinNum::P12 => regs.odr.modify(|_, w| w.odr12().bit(value as u8 != 0)),
                    PinNum::P13 => regs.odr.modify(|_, w| w.odr13().bit(value as u8 != 0)),
                    PinNum::P14 => regs.odr.modify(|_, w| w.odr14().bit(value as u8 != 0)),
                    PinNum::P15 => regs.odr.modify(|_, w| w.odr15().bit(value as u8 != 0)),
                };
            }

            /// Set a pin state.
            pub fn set_state(&mut self, value: PinState, regs: &mut [<GPIO $Port>]) {
                let offset = match value {
                    PinState::Low => 16,
                    PinState::High => 0,
                };

                unsafe {
                    match self.pin {
                        PinNum::P0 => regs.bsrr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => regs.bsrr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => regs.bsrr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => regs.bsrr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => regs.bsrr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => regs.bsrr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => regs.bsrr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => regs.bsrr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => regs.bsrr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => regs.bsrr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => regs.bsrr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => regs.bsrr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => regs.bsrr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => regs.bsrr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => regs.bsrr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => regs.bsrr.write(|w| w.bits(1 << (offset + 15))),
                    };
                }
            }

            /// Lock or unlock a port configuration.
            pub fn cfg_lock(&mut self, value: CfgLock, regs: &mut [<GPIO $Port>]) {
                match self.pin {
                    PinNum::P0 => regs.lckr.modify(|_, w| w.lck0().bit(value as u8 != 0)),
                    PinNum::P1 => regs.lckr.modify(|_, w| w.lck1().bit(value as u8 != 0)),
                    PinNum::P2 => regs.lckr.modify(|_, w| w.lck2().bit(value as u8 != 0)),
                    PinNum::P3 => regs.lckr.modify(|_, w| w.lck3().bit(value as u8 != 0)),
                    PinNum::P4 => regs.lckr.modify(|_, w| w.lck4().bit(value as u8 != 0)),
                    PinNum::P5 => regs.lckr.modify(|_, w| w.lck5().bit(value as u8 != 0)),
                    PinNum::P6 => regs.lckr.modify(|_, w| w.lck6().bit(value as u8 != 0)),
                    PinNum::P7 => regs.lckr.modify(|_, w| w.lck7().bit(value as u8 != 0)),
                    PinNum::P8 => regs.lckr.modify(|_, w| w.lck8().bit(value as u8 != 0)),
                    PinNum::P9 => regs.lckr.modify(|_, w| w.lck9().bit(value as u8 != 0)),
                    PinNum::P10 => regs.lckr.modify(|_, w| w.lck10().bit(value as u8 != 0)),
                    PinNum::P11 => regs.lckr.modify(|_, w| w.lck11().bit(value as u8 != 0)),
                    PinNum::P12 => regs.lckr.modify(|_, w| w.lck12().bit(value as u8 != 0)),
                    PinNum::P13 => regs.lckr.modify(|_, w| w.lck13().bit(value as u8 != 0)),
                    PinNum::P14 => regs.lckr.modify(|_, w| w.lck14().bit(value as u8 != 0)),
                    PinNum::P15 => regs.lckr.modify(|_, w| w.lck15().bit(value as u8 != 0)),
                };
            }

            /// Lock or unlock a port configuration.
            pub fn alt_fn(&mut self, value: AltFn, regs: &mut [<GPIO $Port>]) {
                match self.pin {
                    PinNum::P0 => regs.afrl.modify(|_, w| w.afrl0().bits(value as u8)),
                    PinNum::P1 => regs.afrl.modify(|_, w| w.afrl1().bits(value as u8)),
                    PinNum::P2 => regs.afrl.modify(|_, w| w.afrl2().bits(value as u8)),
                    PinNum::P3 => regs.afrl.modify(|_, w| w.afrl3().bits(value as u8)),
                    PinNum::P4 => regs.afrl.modify(|_, w| w.afrl4().bits(value as u8)),
                    PinNum::P5 => regs.afrl.modify(|_, w| w.afrl5().bits(value as u8)),
                    PinNum::P6 => regs.afrl.modify(|_, w| w.afrl6().bits(value as u8)),
                    PinNum::P7 => regs.afrl.modify(|_, w| w.afrl7().bits(value as u8)),
                    PinNum::P8 => regs.afrh.modify(|_, w| w.afrh8().bits(value as u8)),
                    PinNum::P9 => regs.afrh.modify(|_, w| w.afrh9().bits(value as u8)),
                    PinNum::P10 => regs.afrh.modify(|_, w| w.afrh10().bits(value as u8)),
                    PinNum::P11 => regs.afrh.modify(|_, w| w.afrh11().bits(value as u8)),
                    PinNum::P12 => regs.afrh.modify(|_, w| w.afrh12().bits(value as u8)),
                    PinNum::P13 => regs.afrh.modify(|_, w| w.afrh13().bits(value as u8)),
                    PinNum::P14 => regs.afrh.modify(|_, w| w.afrh14().bits(value as u8)),
                    PinNum::P15 => regs.afrh.modify(|_, w| w.afrh15().bits(value as u8)),
                };
            }

            // todo: Can't find the field in PAC. Figure it out, adn implement A/R
            // /// Reset an Output Data bit.
            // pub fn reset(&mut self, value: ResetState, regs: &mut [<GPIO $Port>]) {
            //     match self.pin {
            //         PinNum::P0 => regs.brr.modify(|_, w| w.brr0().bits(value as u8)),
            //         PinNum::P1 => regs.brr.modify(|_, w| w.brr1().bits(value as u8)),
            //         PinNum::P2 => regs.brr.modify(|_, w| w.brr2().bits(value as u8)),
            //         PinNum::P3 => regs.brr.modify(|_, w| w.brr3().bits(value as u8)),
            //         PinNum::P4 => regs.brr.modify(|_, w| w.brr4().bits(value as u8)),
            //         PinNum::P5 => regs.brr.modify(|_, w| w.brr5().bits(value as u8)),
            //         PinNum::P6 => regs.brr.modify(|_, w| w.brr6().bits(value as u8)),
            //         PinNum::P7 => regs.br.modify(|_, w| w.brr7().bits(value as u8)),
            //         PinNum::P8 => regs.br.modify(|_, w| w.brr8().bits(value as u8)),
            //         PinNum::P9 => regs.br.modify(|_, w| w.brr9().bits(value as u8)),
            //         PinNum::P10 => regs.br.modify(|_, w| w.brr10().bits(value as u8)),
            //         PinNum::P11 => regs.br.modify(|_, w| w.brr11().bits(value as u8)),
            //         PinNum::P12 => regs.br.modify(|_, w| w.brr12().bits(value as u8)),
            //         PinNum::P13 => regs.br.modify(|_, w| w.brr13().bits(value as u8)),
            //         PinNum::P14 => regs.brr.modify(|_, w| w.brr14().bits(value as u8)),
            //         PinNum::P15 => regs.brr.modify(|_, w| w.brr15().bits(value as u8)),
            //     };
            // }
        }

        // Implement `embedded-hal` traits. We use raw pointers, since these traits can't
        // accept a register block.

        impl InputPin for [<Gpio $Port Pin>] {
            type Error = GpioError;

            fn is_high(&self) -> Result<bool, Self::Error> {
                // todo: DRy with `input_data`.
                unsafe {
                    let val = match self.pin {
                        PinNum::P0 => (*[<GPIO $Port>]::ptr()).idr.read().idr0().bit(),
                        PinNum::P1 => (*[<GPIO $Port>]::ptr()).idr.read().idr1().bit(),
                        PinNum::P2 => (*[<GPIO $Port>]::ptr()).idr.read().idr2().bit(),
                        PinNum::P3 => (*[<GPIO $Port>]::ptr()).idr.read().idr3().bit(),
                        PinNum::P4 => (*[<GPIO $Port>]::ptr()).idr.read().idr4().bit(),
                        PinNum::P5 => (*[<GPIO $Port>]::ptr()).idr.read().idr5().bit(),
                        PinNum::P6 => (*[<GPIO $Port>]::ptr()).idr.read().idr6().bit(),
                        PinNum::P7 => (*[<GPIO $Port>]::ptr()).idr.read().idr7().bit(),
                        PinNum::P8 => (*[<GPIO $Port>]::ptr()).idr.read().idr8().bit(),
                        PinNum::P9 => (*[<GPIO $Port>]::ptr()).idr.read().idr9().bit(),
                        PinNum::P10 => (*[<GPIO $Port>]::ptr()).idr.read().idr10().bit(),
                        PinNum::P11 => (*[<GPIO $Port>]::ptr()).idr.read().idr11().bit(),
                        PinNum::P12 => (*[<GPIO $Port>]::ptr()).idr.read().idr12().bit(),
                        PinNum::P13 => (*[<GPIO $Port>]::ptr()).idr.read().idr13().bit(),
                        PinNum::P14 => (*[<GPIO $Port>]::ptr()).idr.read().idr14().bit(),
                        PinNum::P15 => (*[<GPIO $Port>]::ptr()).idr.read().idr15().bit(),
                    };

                    Ok(val)
                }
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        impl OutputPin for [<Gpio $Port Pin>] {
            type Error = GpioError;

            fn set_low(&mut self) -> Result<(), Self::Error> {
                // tood; DRY with `set_state`
                let offset = 16;

                unsafe {
                    match self.pin {
                        PinNum::P0 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 15))),
                    }
                }
                Ok(())
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                // todo: DRy with `set_low`.
                let offset = 0;

                unsafe {
                    match self.pin {
                        PinNum::P0 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => (*[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 15))),
                    }
                }
                Ok(())

            }
        }

        impl ToggleableOutputPin for [<Gpio $Port Pin>] {
            type Error = GpioError;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_high()? {
                    self.set_low()?;
                } else {
                    self.set_high()?;
                }
                Ok(())
            }
        }
        }
    };
}

make_pin!(A);
make_pin!(B);
make_pin!(C);
make_pin!(D);
make_pin!(E);
make_pin!(H);

make_port!(A, a);
make_port!(B, b);
make_port!(C, c);
make_port!(D, d);
make_port!(E, e);
make_port!(H, h);
