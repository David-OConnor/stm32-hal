//! This module provides abstractions for General PurpOspeedre Input and Output (GPIO) pins.
//! Unlike mOspeedrt other modules, it relies on modifying raw pointers, instead of our
//! register traits; this allows for the `embedded-hal` Pin abstraction; STM32 registers
//! are organized by port, not pin.

use core::convert::Infallible;

// todo: Other GPIO ports on certain variants?
use crate::pac::{EXTI, GPIOA, GPIOB, GPIOC, GPIOD, RCC, SYSCFG};

#[cfg(not(any(feature = "f3x4")))]
use crate::pac::GPIOE;

#[cfg(not(any(feature = "f301", feature = "f373", feature = "f3x4")))]
use crate::pac::GPIOH;

use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use paste::paste;

// todo: Implement traits for type-state-programming checks.

// #[derive(Copy, Clone)]
// #[repr(u8)]
// /// Values for `GPIOx_MODER`
// pub enum PinMode {
//     Input = 0b00,
//     Output = 0b01,
//     Alt(AltFn) = 0b10,
//     Analog = 0b11,
// }

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_MODER`
pub enum PinMode {
    Input,
    Output,
    Alt(AltFn),
    Analog,
}

impl PinMode {
    /// We use this function to find the value bits due to being unable to repr(u8) with
    /// the wrapped `AltFn` value.
    fn val(&self) -> u8 {
        match self {
            Self::Input => 0b00,
            Self::Output => 0b01,
            Self::Alt(_) => 0b10,
            Self::Analog => 0b11,
        }
    }
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
    Up = 0b01,
    Dn = 0b10,
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
    F,
    G,
    H,
}

impl PortLetter {
    /// See F3 ref manual section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            Self::D => 3,
            Self::E => 4,
            Self::F => 5,
            Self::G => 6,
            Self::H => 7,
        }
    }
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

#[derive(Copy, Clone, Debug)]
// A pulse edge, used to trigger interrupts.
pub enum Edge {
    Rising,
    Falling,
}

// pub struct GpioError {}

// todo: Should this trait be in `traits.rs` (or eventually crate) ?
/// Gpio pin traits. Used to check pin config when passing to peripheral constructors.
pub trait GpioPin {
    /// Port letter (eg A)
    fn get_port(&self) -> PortLetter;

    /// Pin num (eg P4)
    fn get_pin(&self) -> PinNum;

    /// Pin mode (input, output, alt, analog), and the alt function if applicable.
    fn get_mode(&self) -> PinMode;

    /// Output type. Ie open drain or push pull.
    fn get_output_type(&self) -> OutputType;
}

macro_rules! make_port {
    ($Port:ident, $port:ident) => {
        paste! {
            /// GPIO port
            pub struct [<Gpio $Port>] {
                pub regs: [<GPIO $Port>],
            }

            impl [<Gpio $Port>] {
                pub fn new(regs: [<GPIO $Port>], rcc: &mut RCC) -> Self {
                    // Enable the peripheral clock of a GPIO port

                    cfg_if::cfg_if! {
                        if #[cfg(feature = "f3")] {
                            rcc.ahbenr.modify(|_, w| w.[<iop $port en>]().set_bit());
                            rcc.ahbrstr.modify(|_, w| w.[<iop $port rst>]().set_bit());
                            rcc.ahbrstr.modify(|_, w| w.[<iop $port rst>]().clear_bit());
                        } else  {
                            rcc.ahb2enr.modify(|_, w| w.[<gpio $port en>]().set_bit());
                            rcc.ahb2rstr.modify(|_, w| w.[<gpio $port rst>]().set_bit());
                            rcc.ahb2rstr.modify(|_, w| w.[<gpio $port rst>]().clear_bit());
                        }
                    }

                    Self { regs }
                }

                pub fn new_pin(&mut self, pin: PinNum, mode: PinMode) -> [<Gpio $Port Pin>] {
                    let mut result = [<Gpio $Port Pin>] {
                        port: PortLetter::[<$Port>],
                        pin,
                        mode,
                        output_type: OutputType::PushPull, // Registers initialize to this.
                    };
                    result.mode(mode, &mut self.regs);

                    result
                }
            }
        }
    };
}

// todo: Can we simplify this further so we don't need to write each match arm?
macro_rules! set_field {
    ($fn_name:ident, $reg:ident, $field:ident, $type:ident, $regs:ident) => {
        paste! {
            pub fn $fn_name(&mut self, value: $type, regs: &mut $regs) {
                unsafe { // Only unsafe on some PACs.
                    match self.pin {
                        // PinNum::[<P $pin>] => regs.$field.modify(|_, w| w.[<$field $pin>]().bits(value as u8)),
                        PinNum::P0 => regs.$reg.modify(|_, w| w.[<$field 0>]().bits(value as u8)),
                        PinNum::P1 => regs.$reg.modify(|_, w| w.[<$field 1>]().bits(value as u8)),
                        PinNum::P2 => regs.$reg.modify(|_, w| w.[<$field 2>]().bits(value as u8)),
                        PinNum::P3 => regs.$reg.modify(|_, w| w.[<$field 3>]().bits(value as u8)),
                        PinNum::P4 => regs.$reg.modify(|_, w| w.[<$field 4>]().bits(value as u8)),
                        PinNum::P5 => regs.$reg.modify(|_, w| w.[<$field 5>]().bits(value as u8)),
                        PinNum::P6 => regs.$reg.modify(|_, w| w.[<$field 6>]().bits(value as u8)),
                        PinNum::P7 => regs.$reg.modify(|_, w| w.[<$field 7>]().bits(value as u8)),
                        PinNum::P8 => regs.$reg.modify(|_, w| w.[<$field 8>]().bits(value as u8)),
                        PinNum::P9 => regs.$reg.modify(|_, w| w.[<$field 9>]().bits(value as u8)),
                        PinNum::P10 => regs.$reg.modify(|_, w| w.[<$field 10>]().bits(value as u8)),
                        PinNum::P11 => regs.$reg.modify(|_, w| w.[<$field 11>]().bits(value as u8)),
                        PinNum::P12 => regs.$reg.modify(|_, w| w.[<$field 12>]().bits(value as u8)),
                        PinNum::P13 => regs.$reg.modify(|_, w| w.[<$field 13>]().bits(value as u8)),
                        PinNum::P14 => regs.$reg.modify(|_, w| w.[<$field 14>]().bits(value as u8)),
                        PinNum::P15 => regs.$reg.modify(|_, w| w.[<$field 15>]().bits(value as u8)),
                    }
                }
            }
        }
    };
}

macro_rules! set_field_bit {
    ($fn_name:ident, $reg:ident, $field:ident, $type:ident, $regs:ident) => {
        paste! {
            pub fn $fn_name(&mut self, value: $type, regs: &mut $regs) {
                match self.pin {
                    // PinNum::[<P $pin>] => regs.$field.modify(|_, w| w.[<$field $pin>]().bits(value as u8)),
                    PinNum::P0 => regs.$reg.modify(|_, w| w.[<$field 0>]().bit(value as u8 != 0)),
                    PinNum::P1 => regs.$reg.modify(|_, w| w.[<$field 1>]().bit(value as u8 != 0)),
                    PinNum::P2 => regs.$reg.modify(|_, w| w.[<$field 2>]().bit(value as u8 != 0)),
                    PinNum::P3 => regs.$reg.modify(|_, w| w.[<$field 3>]().bit(value as u8 != 0)),
                    PinNum::P4 => regs.$reg.modify(|_, w| w.[<$field 4>]().bit(value as u8 != 0)),
                    PinNum::P5 => regs.$reg.modify(|_, w| w.[<$field 5>]().bit(value as u8 != 0)),
                    PinNum::P6 => regs.$reg.modify(|_, w| w.[<$field 6>]().bit(value as u8 != 0)),
                    PinNum::P7 => regs.$reg.modify(|_, w| w.[<$field 7>]().bit(value as u8 != 0)),
                    PinNum::P8 => regs.$reg.modify(|_, w| w.[<$field 8>]().bit(value as u8 != 0)),
                    PinNum::P9 => regs.$reg.modify(|_, w| w.[<$field 9>]().bit(value as u8 != 0)),
                    PinNum::P10 => regs.$reg.modify(|_, w| w.[<$field 10>]().bit(value as u8 != 0)),
                    PinNum::P11 => regs.$reg.modify(|_, w| w.[<$field 11>]().bit(value as u8 != 0)),
                    PinNum::P12 => regs.$reg.modify(|_, w| w.[<$field 12>]().bit(value as u8 != 0)),
                    PinNum::P13 => regs.$reg.modify(|_, w| w.[<$field 13>]().bit(value as u8 != 0)),
                    PinNum::P14 => regs.$reg.modify(|_, w| w.[<$field 14>]().bit(value as u8 != 0)),
                    PinNum::P15 => regs.$reg.modify(|_, w| w.[<$field 15>]().bit(value as u8 != 0)),
                }
            }
        }
    };
}

// macro_rules! moder_row {
//     ($Port:ident) => {
//         paste! {
//         }
//     }
// }

macro_rules! make_pin {
    ($Port:ident) => {
        paste! {

        /// Represents a single GPIO pin.
        pub struct [<Gpio $Port Pin>] {
            pub port: PortLetter,
            pub pin: PinNum,
            pub mode: PinMode,
            pub output_type: OutputType,
        }

        impl [<Gpio $Port Pin>] {
            // We use macros where we can reduce code, and full functions where there's a difference
            // from the macros.

            /// Set pin mode.
            pub fn mode(&mut self, value: PinMode, regs: &mut [<GPIO $Port>]) {
                unsafe {
                    match self.pin {
                        PinNum::P0 => regs.moder.modify(|_, w| w.moder0().bits(value.val())),
                        PinNum::P1 => regs.moder.modify(|_, w| w.moder1().bits(value.val())),
                        PinNum::P2 => regs.moder.modify(|_, w| w.moder2().bits(value.val())),
                        PinNum::P3 => regs.moder.modify(|_, w| w.moder3().bits(value.val())),
                        PinNum::P4 => regs.moder.modify(|_, w| w.moder4().bits(value.val())),
                        PinNum::P5 => regs.moder.modify(|_, w| w.moder5().bits(value.val())),
                        PinNum::P6 => regs.moder.modify(|_, w| w.moder6().bits(value.val())),
                        PinNum::P7 => regs.moder.modify(|_, w| w.moder7().bits(value.val())),
                        PinNum::P8 => regs.moder.modify(|_, w| w.moder8().bits(value.val())),
                        PinNum::P9 => regs.moder.modify(|_, w| w.moder9().bits(value.val())),
                        PinNum::P10 => regs.moder.modify(|_, w| w.moder10().bits(value.val())),
                        PinNum::P11 => regs.moder.modify(|_, w| w.moder11().bits(value.val())),
                        PinNum::P12 => regs.moder.modify(|_, w| w.moder12().bits(value.val())),
                        PinNum::P13 => regs.moder.modify(|_, w| w.moder13().bits(value.val())),
                        PinNum::P14 => regs.moder.modify(|_, w| w.moder14().bits(value.val())),
                        PinNum::P15 => regs.moder.modify(|_, w| w.moder15().bits(value.val())),
                    }
                }

                self.mode = value;

                if let PinMode::Alt(alt) = value {
                    self.alt_fn(alt, regs);
                }
            }

            /// Set output type
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
                }

                self.output_type = value;
            }

            // todo: How do we make these work as doc comments?

            // Set output speed.
            set_field!(output_speed, ospeedr, ospeedr, OutputSpeed, [<GPIO $Port>]);

            // Set internal pull resistor: Pull up, pull down, or floating.
            set_field!(pull, pupdr, pupdr, Pull, [<GPIO $Port>]);

            // Set the output_data register.
            set_field_bit!(output_data, odr, odr, PinState, [<GPIO $Port>]);


            // It appears f373 doesn't have lckr on ports C or E.
            #[cfg(not(feature = "f373"))]
            // Lock or unlock a port configuration.
            set_field_bit!(cfg_lock, lckr, lck, CfgLock, [<GPIO $Port>]);

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

            /// Lock or unlock a port configuration. Private - we set this using `mode`.
            fn alt_fn(&mut self, value: AltFn, regs: &mut [<GPIO $Port>]) {
                #[cfg(feature = "l5")]
                match self.pin {
                    PinNum::P0 => {
                        regs.moder.modify(|_, w| w.moder0().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel0().bits(value as u8));
                    }
                    PinNum::P1 => {
                        regs.moder.modify(|_, w| w.moder1().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel1().bits(value as u8));
                    }
                    PinNum::P2 => {
                        regs.moder.modify(|_, w| w.moder2().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel2().bits(value as u8));
                    }
                    PinNum::P3 => {
                        regs.moder.modify(|_, w| w.moder3().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel3().bits(value as u8));
                    }
                    PinNum::P4 => {
                        regs.moder.modify(|_, w| w.moder4().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel4().bits(value as u8));
                    }
                    PinNum::P5 => {
                        regs.moder.modify(|_, w| w.moder5().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel5().bits(value as u8));
                    }
                    PinNum::P6 => {
                        regs.moder.modify(|_, w| w.moder6().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel6().bits(value as u8));
                    }
                    PinNum::P7 => {
                        regs.moder.modify(|_, w| w.moder7().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afsel7().bits(value as u8));
                    }
                    PinNum::P8 => {
                        regs.moder.modify(|_, w| w.moder8().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel8().bits(value as u8));
                    }
                    PinNum::P9 => {
                        regs.moder.modify(|_, w| w.moder9().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel9().bits(value as u8));
                    }
                    PinNum::P10 => {
                        regs.moder.modify(|_, w| w.moder10().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel10().bits(value as u8));
                    }
                    PinNum::P11 => {
                        regs.moder.modify(|_, w| w.moder11().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel11().bits(value as u8));
                    }
                    PinNum::P12 => {
                        regs.moder.modify(|_, w| w.moder12().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel12().bits(value as u8));
                    }
                    PinNum::P13 => {
                        regs.moder.modify(|_, w| w.moder13().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel13().bits(value as u8));
                    }
                    PinNum::P14 => {
                        regs.moder.modify(|_, w| w.moder14().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel14().bits(value as u8));
                    }
                    PinNum::P15 => {
                        regs.moder.modify(|_, w| w.moder15().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afsel15().bits(value as u8));
                    }
                }

                #[cfg(not(feature = "l5"))]
                match self.pin {
                    PinNum::P0 => {
                        regs.moder.modify(|_, w| w.moder0().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl0().bits(value as u8));
                    }
                    PinNum::P1 => {
                        regs.moder.modify(|_, w| w.moder1().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl1().bits(value as u8));
                    }
                    PinNum::P2 => {
                        regs.moder.modify(|_, w| w.moder2().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl2().bits(value as u8));
                    }
                    PinNum::P3 => {
                        regs.moder.modify(|_, w| w.moder3().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl3().bits(value as u8));
                    }
                    PinNum::P4 => {
                        regs.moder.modify(|_, w| w.moder4().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl4().bits(value as u8));
                    }
                    PinNum::P5 => {
                        regs.moder.modify(|_, w| w.moder5().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl5().bits(value as u8));
                    }
                    PinNum::P6 => {
                        regs.moder.modify(|_, w| w.moder6().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl6().bits(value as u8));
                    }
                    PinNum::P7 => {
                        regs.moder.modify(|_, w| w.moder7().bits(PinMode::Alt(value).val()));
                        regs.afrl.modify(|_, w| w.afrl7().bits(value as u8));
                    }
                    PinNum::P8 => {
                        regs.moder.modify(|_, w| w.moder8().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh8().bits(value as u8));
                    }
                    PinNum::P9 => {
                        regs.moder.modify(|_, w| w.moder9().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh9().bits(value as u8));
                    }
                    PinNum::P10 => {
                        regs.moder.modify(|_, w| w.moder10().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh10().bits(value as u8));
                    }
                    PinNum::P11 => {
                        regs.moder.modify(|_, w| w.moder11().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh11().bits(value as u8));
                    }
                    PinNum::P12 => {
                        regs.moder.modify(|_, w| w.moder12().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh12().bits(value as u8));
                    }
                    PinNum::P13 => {
                        regs.moder.modify(|_, w| w.moder13().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh13().bits(value as u8));
                    }
                    PinNum::P14 => {
                        regs.moder.modify(|_, w| w.moder14().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh14().bits(value as u8));
                    }
                    PinNum::P15 => {
                        regs.moder.modify(|_, w| w.moder15().bits(PinMode::Alt(value).val()));
                        regs.afrh.modify(|_, w| w.afrh15().bits(value as u8));
                    }
                }
            }

            // todo: Can't find the field due to an error in L4 PAC.
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

            // todo: Look up how you do EXTI on L5.
            #[cfg(not(any(feature = "f373", feature = "l5")))]  // Does f373 not have GPIO interrupts?
            /// Configure this pin as an interrupt source.
            pub fn enable_interrupt(&mut self, edge: Edge, exti: &mut EXTI, syscfg: &mut SYSCFG) {
                let rise_trigger = match edge {
                    Edge::Rising => {
                        // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
                        true
                    }
                    Edge::Falling => {
                        // configure EXTI line to trigger on falling edge, disable trigger on rising edge.
                        false
                    }
                };

                match self.pin {
                    // todo: This DRY is worse than the ones above due to 4 lines each.
                    PinNum::P0 => {
                        exti.imr1.modify(|_, w| w.mr0().unmasked());  // Unmask the line.
                        // Configure the trigger edge
                        exti.rtsr1.modify(|_, w| w.tr0().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr0().bit(!rise_trigger));
                        // Select this GPIO pin as source input for EXTI line external interrupt
                        syscfg
                            .exticr1
                            .modify(|_, w| unsafe { w.exti0().bits(self.port.cr_val()) });
                    }
                    PinNum::P1 => {
                        exti.imr1.modify(|_, w| w.mr1().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr1().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr1().bit(!rise_trigger));
                        syscfg
                            .exticr1
                            .modify(|_, w| unsafe { w.exti1().bits(self.port.cr_val()) });
                    }
                    PinNum::P2 => {
                        exti.imr1.modify(|_, w| w.mr2().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr2().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr2().bit(!rise_trigger));
                        syscfg
                            .exticr1
                            .modify(|_, w| unsafe { w.exti2().bits(self.port.cr_val()) });
                    }
                    PinNum::P3 => {
                        exti.imr1.modify(|_, w| w.mr3().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr3().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr3().bit(!rise_trigger));
                        syscfg
                            .exticr1
                            .modify(|_, w| unsafe { w.exti3().bits(self.port.cr_val()) });
                    }
                    PinNum::P4 => {
                        exti.imr1.modify(|_, w| w.mr4().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr4().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr4().bit(!rise_trigger));
                        syscfg
                            .exticr2
                            .modify(|_, w| unsafe { w.exti4().bits(self.port.cr_val()) });
                    }
                    PinNum::P5 => {
                        exti.imr1.modify(|_, w| w.mr5().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr5().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr5().bit(!rise_trigger));
                        syscfg
                            .exticr2
                            .modify(|_, w| unsafe { w.exti5().bits(self.port.cr_val()) });
                    }
                    PinNum::P6 => {
                        exti.imr1.modify(|_, w| w.mr6().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr6().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr6().bit(!rise_trigger));
                        syscfg
                            .exticr2
                            .modify(|_, w| unsafe { w.exti6().bits(self.port.cr_val()) });
                    }
                    PinNum::P7 => {
                        exti.imr1.modify(|_, w| w.mr7().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr7().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr7().bit(!rise_trigger));
                        syscfg
                            .exticr2
                            .modify(|_, w| unsafe { w.exti7().bits(self.port.cr_val()) });
                    }
                    PinNum::P8 => {
                        exti.imr1.modify(|_, w| w.mr8().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr8().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr8().bit(!rise_trigger));
                        syscfg
                            .exticr3
                            .modify(|_, w| unsafe { w.exti8().bits(self.port.cr_val()) });
                    }
                    PinNum::P9 => {
                        exti.imr1.modify(|_, w| w.mr9().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr9().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr9().bit(!rise_trigger));
                        syscfg
                            .exticr3
                            .modify(|_, w| unsafe { w.exti9().bits(self.port.cr_val()) });
                    }
                    PinNum::P10 => {
                        exti.imr1.modify(|_, w| w.mr10().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr10().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr10().bit(!rise_trigger));
                        syscfg
                            .exticr3
                            .modify(|_, w| unsafe { w.exti10().bits(self.port.cr_val()) });
                    }
                    PinNum::P11 => {
                        exti.imr1.modify(|_, w| w.mr11().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr11().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr11().bit(!rise_trigger));
                        syscfg
                            .exticr3
                            .modify(|_, w| unsafe { w.exti11().bits(self.port.cr_val()) });
                    }
                    PinNum::P12 => {
                        exti.imr1.modify(|_, w| w.mr12().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr12().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr12().bit(!rise_trigger));
                        syscfg
                            .exticr4
                            .modify(|_, w| unsafe { w.exti12().bits(self.port.cr_val()) });
                    }
                    PinNum::P13 => {
                        exti.imr1.modify(|_, w| w.mr13().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr13().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr14().bit(!rise_trigger));
                        syscfg
                            .exticr4
                            .modify(|_, w| unsafe { w.exti13().bits(self.port.cr_val()) });
                    }
                    PinNum::P14 => {
                        exti.imr1.modify(|_, w| w.mr14().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr14().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr14().bit(!rise_trigger));
                        syscfg
                            .exticr4
                            .modify(|_, w| unsafe { w.exti14().bits(self.port.cr_val()) });
                    }
                    PinNum::P15 => {
                        exti.imr1.modify(|_, w| w.mr15().unmasked());
                        exti.rtsr1.modify(|_, w| w.tr15().bit(rise_trigger));
                        exti.ftsr1.modify(|_, w| w.tr15().bit(!rise_trigger));
                        syscfg
                            .exticr4
                            .modify(|_, w| unsafe { w.exti15().bits(self.port.cr_val()) });
                    }
                };
            }

            /// Disable interrupts on this pin.
            pub fn disable_interrupt() {
                // todo
            }
        }

        // Implement `embedded-hal` traits. We use raw pointers, since these traits can't
        // accept a register block.

        impl InputPin for [<Gpio $Port Pin>] {
            type Error = Infallible;

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
            type Error = Infallible;

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
            type Error = Infallible;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_high()? {
                    self.set_low()?;
                } else {
                    self.set_high()?;
                }
                Ok(())
            }
        }

        impl GpioPin for [<Gpio $Port Pin>] {
            fn get_port(&self) -> PortLetter {
                self.port
            }

            fn get_pin(&self) -> PinNum {
                self.pin
            }

            fn get_mode(&self) -> PinMode {
                self.mode
            }

            fn get_output_type(&self) -> OutputType {
                self.output_type
            }
        }

        }
    };
}

make_pin!(A);
make_pin!(B);
make_pin!(C);
make_pin!(D);

make_port!(A, a);
make_port!(B, b);
make_port!(C, c);
make_port!(D, d);

// todo: Missing EFGH impls on some variants that have them.

#[cfg(not(any(feature = "f301", feature = "f3x4")))]
make_pin!(E);

#[cfg(not(any(feature = "f301", feature = "f3x4")))]
make_port!(E, e);

#[cfg(not(any(feature = "f373", feature = "f301", feature = "f3x4", feature = "l4")))]
make_pin!(H);

#[cfg(not(any(feature = "f373", feature = "f301", feature = "f3x4", feature = "l4")))]
make_port!(H, h);
