//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIOx register functions. It also configures GPIO interrupts using SYSCFG and EXTI
//! registers as appropriate.

use core::convert::Infallible;

use cortex_m::interrupt::free;

use crate::{
    pac::{self, EXTI, RCC},
    rcc_en_reset,
};

#[cfg(not(any(feature = "l5", feature = "g0")))]
use crate::pac::SYSCFG;

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use cfg_if::cfg_if;
use paste::paste;

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_MODER`
pub enum PinMode {
    Input,
    Output,
    Alt(u8),
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
/// Values for `GPIOx_BRR`.
pub enum ResetState {
    NoAction = 0,
    Reset = 1,
}

// todo: If you get rid of Port struct, rename this enum Port
#[derive(Copy, Clone)]
/// GPIO port letter
pub enum Port {
    A,
    B,
    C,
    #[cfg(not(any(feature = "f410")))]
    D,
    #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
    E,
    #[cfg(not(any(
    feature = "f401",
    feature = "f410",
    feature = "f411",
    feature = "l4x1",
    feature = "l4x2",
    feature = "l412",
    feature = "l4x3",
    feature = "wb",
    feature = "wl"
    )))]
    F,
    // G,
    #[cfg(not(any(
        feature = "f373",
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4",
        feature = "g0",
        feature = "g4",
        feature = "wb",
        feature = "wl"
    )))]
    H,
}

impl Port {
    /// See F3 ref manual section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            #[cfg(not(any(feature = "f410")))]
            Self::D => 3,
            #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
            Self::E => 4,
            #[cfg(not(any(
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "wb",
                feature = "wl"
                )))]
            Self::F => 5,
            // Self::G => 6,
            #[cfg(not(any(
                feature = "f373",
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "l4",
                feature = "g0",
                feature = "g4",
                feature = "wb",
                feature = "wl"
            )))]
            Self::H => 7,
        }
    }
}

#[derive(Copy, Clone, Debug)]
// A pulse edge, used to trigger interrupts.
pub enum Edge {
    Rising,
    Falling,
}
//
// macro_rules! make_port {
//     ($Port:ident, $port:ident) => {
//         paste! {
//             /// Represents a single GPIO port, and owns its register block. Provides
//             /// methods to enable the port. To change pin properties, pass its `regs`
//             /// field as a mutable reference to `GpioXPin` methods.
//             pub struct [<Gpio $Port>] {
//                 pub regs: pac::[<GPIO $Port>],
//             }
//
//             impl [<Gpio $Port>] {
//                 pub fn new(regs: pac::[<GPIO $Port>]) -> Self {
//                     // Enable the peripheral clock of a GPIO port
//                     free(|_| {
//                         let rcc = unsafe { &(*RCC::ptr()) };
//
//                         cfg_if! {
//                             if #[cfg(feature = "f3")] {
//                                 rcc_en_reset!(ahb1, [<iop $port>], rcc);
//                             } else if #[cfg(feature = "h7")] {
//                                 rcc.ahb4enr.modify(|_, w| w.[<gpio $port en>]().set_bit());
//                                 rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().set_bit());
//                                 rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().clear_bit());
//                             } else if #[cfg(feature = "f4")] {
//                                 rcc_en_reset!(ahb1, [<gpio $port>], rcc);
//                             } else if #[cfg(feature = "g0")] {
//                                 rcc.iopenr.modify(|_, w| w.[<iop $port en>]().set_bit());
//                                 rcc.ioprstr.modify(|_, w| w.[<iop $port rst>]().set_bit());
//                                 rcc.ioprstr.modify(|_, w| w.[<iop $port rst>]().clear_bit());
//                             } else { // L4, L5, G4
//                                 rcc_en_reset!(ahb2, [<gpio $port>], rcc);
//                             }
//                         }
//                     });
//
//                     Self { regs }
//                 }
//
//                 // pub fn new_pin(&mut self, pin: u8, mode: PinMode) -> [<Gpio $Port Pin>] {
//                 pub fn new_pin(&mut self, pin: u8, mode: PinMode) -> Pin {
//                     assert!(pin <= 15, "Pin must be 0 - 15.");
//
//                     // let mut result = [<Gpio $Port Pin>] {
//                     //     port: Port::[<$Port>],
//                     //     pin,
//                     // };
//                     // result.mode(mode, &mut self.regs);
//
//                     let mut result = Pin {
//                         port: Port::[<$Port>],
//                         pin,
//                     };
//
//                     result.mode(mode);
//
//                     result
//                 }
//             }
//         }
//     };
// }

// Reduce DRY for setting fields.
// macro_rules! set_field {
//     ($pin:expr, $regs:expr, $reg:ident, $field:ident, $bit:ident, $val:expr, [$($num:expr),+]) => {
//         paste! {
//             // Unsafe may or may not be required, depending on the PAC.
//             unsafe {
//                 match $pin {
//                     $(
//                         $num => $regs.$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
//                     )+
//                     _ => panic!("GPIO pins must be 0 - 15."),
//                 }
//             }
//         }
//     }
// }

// todo: Remove the old set_field in favor of this if you ditch the GpioAPin concept etc.

/// Reduce DRY for setting fields.
macro_rules! set_field2 { // for `Pin` struct.
    ($pin:expr, $port_letter:expr, $reg:ident, $field:ident, $bit:ident, $val:expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $port_letter {
                    Port::A => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOA::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::B => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOB::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::C => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOC::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f410")))]
                    Port::D => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOD::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
                    Port::E => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOE::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f401",
                        feature = "f410",
                        feature = "f411",
                        feature = "l4x1",
                        feature = "l4x2",
                        feature = "l412",
                        feature = "l4x3",
                        feature = "wb",
                        feature = "wl"
                        )))]
                    Port::F => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOF::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f373",
                        feature = "f301",
                        feature = "f3x4",
                        feature = "f410",
                        feature = "l4",
                        feature = "g0",
                        feature = "g4",
                        feature = "wb",
                        feature = "wl"
                    )))]
                    Port::H => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOH::ptr()).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                }
            }
        }
    }
}

/// Reduce DRY for setting fields.
macro_rules! set_alt2 { // for `Pin` struct.
    ($pin:expr, $port_letter:expr, $field_af:ident, $val:expr, [$(($num:expr, $lh:ident)),+]) => {
        paste! {
            unsafe {
                match $port_letter {
                    Port::A => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOA::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOA::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOA::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::B => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOB::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOB::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOB::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::C => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOC::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOC::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOC::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f410")))]
                    Port::D => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOD::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOD::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOD::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
                    Port::E => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOE::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOE::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOE::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f401",
                        feature = "f410",
                        feature = "f411",
                        feature = "l4x1",
                        feature = "l4x2",
                        feature = "l412",
                        feature = "l4x3",
                        feature = "wb",
                        feature = "wl"
                        )))]
                    Port::F => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOF::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOF::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOF::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f373",
                        feature = "f301",
                        feature = "f3x4",
                        feature = "f410",
                        feature = "l4",
                        feature = "g0",
                        feature = "g4",
                        feature = "wb",
                        feature = "wl"
                    )))]
                    Port::H => {
                        match $pin {
                            $(
                                $num => {
                                    (*pac::GPIOH::ptr()).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                                    #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
                                    (*pac::GPIOH::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                                    #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
                                    (*pac::GPIOH::ptr()).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                                }
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                }
            }
        }
    }
}

// todo: DRY on port feature gates for these macros

/// Reduce DRY getting input data
macro_rules! get_input_data { // for `Pin` struct.
    ($pin:expr, $port_letter:expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $port_letter {
                    Port::A => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOA::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::B => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOB::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::C => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOC::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f410")))]
                    Port::D => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOD::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
                    Port::E => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOE::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f401",
                        feature = "f410",
                        feature = "f411",
                        feature = "l4x1",
                        feature = "l4x2",
                        feature = "l412",
                        feature = "l4x3",
                        feature = "wb",
                        feature = "wl"
                        )))]
                    Port::F => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOF::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f373",
                        feature = "f301",
                        feature = "f3x4",
                        feature = "f410",
                        feature = "l4",
                        feature = "g0",
                        feature = "g4",
                        feature = "wb",
                        feature = "wl"
                    )))]
                    Port::H => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOH::ptr()).idr.read().[<idr $num>]().bit_is_set(),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                }
            }
        }
    }
}

/// Reduce DRY setting pin state
macro_rules! set_state { // for `Pin` struct.
    ($pin:expr, $port_letter:expr, $offset: expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $port_letter {
                    Port::A => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOA::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::B => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOB::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    Port::C => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOC::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f410")))]
                    Port::D => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOD::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
                    Port::E => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOE::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f401",
                        feature = "f410",
                        feature = "f411",
                        feature = "l4x1",
                        feature = "l4x2",
                        feature = "l412",
                        feature = "l4x3",
                        feature = "wb",
                        feature = "wl"
                        )))]
                    Port::F => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOF::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                    #[cfg(not(any(
                        feature = "f373",
                        feature = "f301",
                        feature = "f3x4",
                        feature = "f410",
                        feature = "l4",
                        feature = "g0",
                        feature = "g4",
                        feature = "wb",
                        feature = "wl"
                    )))]
                    Port::H => {
                        match $pin {
                            $(
                                $num => (*pac::GPIOH::ptr()).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                            )+
                            _ => panic!("GPIO pins must be 0 - 15."),
                        }
                    }
                }
            }
        }
    }
}

// todo: Consolidate these exti macros

/// Reduce DRY for setting up interrupts.
macro_rules! set_exti {
    ($pin:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        let syscfg  = unsafe { &(*pac::SYSCFG::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                    // todo: Core 2 interrupts for wb. (?)
                        cfg_if! {
                            if #[cfg(all(feature = "h7", not(any(feature = "h747cm4", feature = "h747cm7"))))] {
                                exti.cpuimr1.modify(|_, w| w.[<mr $num>]().set_bit());
                            } else if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
                                exti.c1imr1.modify(|_, w| w.[<mr $num>]().set_bit());
                            }else if #[cfg(any(feature = "g4", feature = "wb", feature = "wl"))] {
                                exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());
                            } else {
                                exti.imr1.modify(|_, w| w.[<mr $num>]().set_bit());
                            }
                        }

                        cfg_if! {
                            if #[cfg(any(feature = "g4", feature = "wb", feature = "wl"))] {
                                exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($trigger));
                                exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit(!$trigger));
                            // } else if #[cfg(any(feature = "wb", feature = "wl"))] {
                            //     // todo: Missing in PAC, so we read+write. https://github.com/stm32-rs/stm32-rs/issues/570
                            //     let val_r =  $exti.rtsr1.read().bits();
                            //     $exti.rtsr1.write(|w| unsafe { w.bits(val_r | (1 << $num)) });
                            //     let val_f =  $exti.ftsr1.read().bits();
                            //     $exti.ftsr1.write(|w| unsafe { w.bits(val_f | (1 << $num)) });
                            //     // todo: Core 2 interrupts.
                            } else {
                                exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($trigger));
                                exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit(!$trigger));
                            }
                        }
                        syscfg
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}

#[cfg(feature = "f4")]
/// Similar to `set_exti`, but with reg names sans `1`.
macro_rules! set_exti_f4 {
    ($pin:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        let syscfg  = unsafe { &(*pac::SYSCFG::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr.modify(|_, w| w.[<mr $num>]().unmasked());
                        exti.rtsr.modify(|_, w| w.[<tr $num>]().bit($trigger));
                        exti.ftsr.modify(|_, w| w.[<tr $num>]().bit(!$trigger));
                        syscfg
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}

#[cfg(feature = "l5")]
/// For L5 See `set_exti!`. Different method naming pattern for exticr.
macro_rules! set_exti_l5 {
    ($pin:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($trigger));  // Rising trigger
                        exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit(!$trigger));   // Falling trigger
                        exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num2>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}

#[cfg(feature = "g0")]
/// ForG0. See `set_exti!`. Todo? Reduce DRY.
macro_rules! set_exti_g0 {
    ($pin:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($trigger));  // Rising trigger
                        exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit(!$trigger));   // Falling trigger
                        exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num2>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}

// /// Reduce DRY for setting up alternate functions. Note that there are at least 3
// /// different names for the `afrl` field to modify based on variants.
// macro_rules! set_alt {
//     ($pin:expr, $regs:expr, $field_af:ident, $val:expr, [$(($num:expr, $lh:ident)),+]) => {
//         paste! {
//             unsafe {
//                 match $pin {
//                     $(
//                         $num => {
//                             $regs.moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
//                             #[cfg(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl"))]
//                             $regs.[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
//                             #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7", feature = "wb", feature = "wl")))]
//                             $regs.[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
//                         }
//                     )+
//                     _ => panic!("GPIO pins must be 0 - 15."),
//                 }
//             }
//         }
//     }
// }


/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
    pub port: Port,
    pub pin: u8,
}

// todo: Critical sections on unsafe calls to avoid race conditions?

impl Pin {
    /// Create a new pin, with a specific mode. Enables the RCC peripheral clock to the port,
    /// if not already enabled.
    pub fn new(port: Port, pin: u8, mode: PinMode) -> Self {
        assert!(pin <= 15, "Pin must be 0 - 15.");

        free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };

            match port {
                Port::A => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopaen().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpioaen().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpioaen().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpioarst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpioarst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpioaen().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpioa, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopaen().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopaen().set_bit());
                                rcc.ioprstr.modify(|_, w| w.ioparst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.ioparst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpioaen().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpioa, rcc);
                            }
                        }
                    }
                }
                Port::B => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopben().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpioben().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpioben().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiobrst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiobrst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpioben().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpiob, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopben().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopben().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopbrst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopbrst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpioben().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpiob, rcc);
                            }
                        }
                    }
                }
                Port::C => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopcen().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpiocen().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpiocen().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiocrst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiocrst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpiocen().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpioc, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopcen().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopcen().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopcrst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopcrst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpiocen().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpioc, rcc);
                            }
                        }
                    }
                }
                #[cfg(not(any(feature = "f410")))]
                Port::D => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopden().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpioden().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpioden().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiodrst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiodrst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpioden().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpiod, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopden().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopden().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopdrst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopdrst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpioden().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpiod, rcc);
                            }
                        }
                    }
                }
                #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))]
                Port::E => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopeen().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpioeen().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpioeen().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpioerst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpioerst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpioeen().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpioe, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopeen().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopeen().set_bit());
                                rcc.ioprstr.modify(|_, w| w.ioperst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.ioperst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpioeen().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpioe, rcc);
                            }
                        }
                    }
                }
                #[cfg(not(any(
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "wb",
                feature = "wl"
                )))]
                Port::F => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iopfen().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpiofen().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpiofen().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiofrst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiofrst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpiofen().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpiof, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iopfen().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iopfen().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopfrst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iopfrst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpiofen().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpiof, rcc);
                            }
                        }
                    }
                }
                #[cfg(not(any(
                    feature = "f373",
                    feature = "f301",
                    feature = "f3x4",
                    feature = "f410",
                    feature = "l4",
                    feature = "g0",
                    feature = "g4",
                    feature = "wb",
                    feature = "wl"
                )))]
                Port::H => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcc.ahbenr.read().iophen().bit_is_clear() {
                                rcc_en_reset!(ahb1, iopa, rcc);
                            }
                        } else if #[cfg(feature = "h7")] {
                            if rcc.ahb4enr.read().gpiohen().bit_is_clear() {
                                rcc.ahb4enr.modify(|_, w| w.gpiohen().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiohrst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.gpiohrst().clear_bit());
                            }
                        } else if #[cfg(feature = "f4")] {
                            if rcc.ahb1enr.read().gpiohen().bit_is_clear() {
                                rcc_en_reset!(ahb1, gpioh, rcc);
                            }
                        } else if #[cfg(feature = "g0")] {
                            if rcc.iopenr.read().iophen().bit_is_clear() {
                                rcc.iopenr.modify(|_, w| w.iophen().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iophrst().set_bit());
                                rcc.ioprstr.modify(|_, w| w.iophrst().clear_bit());
                            }
                        } else { // L4, L5, G4
                            if rcc.ahb2enr.read().gpiohen().bit_is_clear() {
                                rcc_en_reset!(ahb2, gpioa, rcc);
                            }
                        }
                    }
                }
            }
        });

        let mut result = Self { port, pin };
        result.mode(mode);

        result
    }

    /// Set pin mode. Eg, Output, Input, Analog, or Alt. Sets the `MODER` register.
    pub fn mode(&mut self, value: PinMode) {
        set_field2!(
            self.pin,
            self.port,
            moder,
            moder,
            bits,
            value.val(),
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );

        if let PinMode::Alt(alt) = value {
            self.alt_fn(alt);
        }
    }

    /// Set output type. Sets the `OTYPER` register.
    pub fn output_type(&mut self, value: OutputType) {
        set_field2!(
            self.pin,
            self.port,
            otyper,
            ot,
            bit,
            value as u8 != 0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set output speed to Low, Medium, or High. Sets the `OSPEEDR` register.
    pub fn output_speed(&mut self, value: OutputSpeed) {
        set_field2!(
            self.pin,
            self.port,
            ospeedr,
            ospeedr,
            bits,
            value as u8,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set internal pull resistor: Pull up, pull down, or floating. Sets the `PUPDR` register.
    pub fn pull(&mut self, value: Pull) {
        set_field2!(
            self.pin,
            self.port,
            pupdr,
            pupdr,
            bits,
            value as u8,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set the output_data register, eg set the pin to low or high. See also `set_high()` and
    /// `set_low()`. Sets the `ODR` register. Compared to `set_state`, `set_high()`, and `set_low()`,
    /// this is non-atomic.
    pub fn output_data(&mut self, value: PinState) {
        set_field2!(
            self.pin,
            self.port,
            odr,
            odr,
            bit,
            value as u8 != 0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    // It appears f373 doesn't have lckr on ports C or E. (PAC error?)
    #[cfg(not(feature = "f373"))]
    /// Lock or unlock a port configuration. Sets the `LCKR` register.
    pub fn cfg_lock(&mut self, value: CfgLock) {
        set_field2!(
            self.pin,
            self.port,
            lckr,
            lck,
            bit,
            value as u8 != 0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Read the input data register. Eg determine if the pin is high or low. See also `is_high()`
    /// and `is_low()`. Reads from the `IDR` register.
    pub fn input_data(&mut self) -> PinState {
        let val = get_input_data!(
            self.pin,
            self.port,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
        if val {
            PinState::High
        } else {
            PinState::Low
        }
    }

    /// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
    /// `set_low()` functions. Sets the `BSRR` register. Compared to `output_data`, this is
    /// atomic.
    pub fn set_state(&mut self, value: PinState) {
        let offset = match value {
            PinState::Low => 16,
            PinState::High => 0,
        };

        set_state!(
            self.pin,
            self.port,
            offset,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set up a pin's alternate function. We set this up initially using `mode()`.
    fn alt_fn(&mut self, value: u8) {
        assert!(value <= 15, "Alt function must be 0 to 15.");

        cfg_if! {
            if #[cfg(any(feature = "l5", feature = "g0", feature = "wb", feature = "wl"))] {
                set_alt2!(self.pin, self.port, afsel, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            } else if #[cfg(feature = "h7")] {
                set_alt2!(self.pin, self.port, afr, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            } else {  // f3, f4, l4, g4
                set_alt2!(self.pin, self.port, afr, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            }
        }
    }
    // todo: Implement.
    //     // todo Error on these PACS, or are they missing BRR?
    //     #[cfg(not(any(feature = "l4", feature = "h7", feature = "f4")))]
    //     /// Reset an Output Data bit. Sets the `BRR` register.
    //     pub fn reset(&mut self, value: ResetState) {
    //         let regs = unsafe { get_reg_ptr!(Port) };
    //
    //         let offset = match value {
    //             ResetState::NoAction => 16,
    //             ResetState::Reset => 0,
    //         };
    //         unsafe {
    //             regs.brr.write(|w| match self.pin {
    //                 0 => w.bits(1 << (offset + 0)),
    //                 1 => w.bits(1 << (offset + 1)),
    //                 2 => w.bits(1 << (offset + 2)),
    //                 3 => w.bits(1 << (offset + 3)),
    //                 4 => w.bits(1 << (offset + 4)),
    //                 5 => w.bits(1 << (offset + 5)),
    //                 6 => w.bits(1 << (offset + 6)),
    //                 7 => w.bits(1 << (offset + 7)),
    //                 8 => w.bits(1 << (offset + 8)),
    //                 9 => w.bits(1 << (offset + 9)),
    //                 10 => w.bits(1 << (offset + 10)),
    //                 11 => w.bits(1 << (offset + 11)),
    //                 12 => w.bits(1 << (offset + 12)),
    //                 13 => w.bits(1 << (offset + 13)),
    //                 14 => w.bits(1 << (offset + 14)),
    //                 15 => w.bits(1 << (offset + 15)),
    //                 _ => panic!("GPIO pins must be 0 - 15."),
    //             });
    //         }
    //     }
    //
    #[cfg(not(feature = "f373"))]
    /// Configure this pin as an interrupt source. Set the edge as Rising or Falling.
    pub fn enable_interrupt(&mut self, edge: Edge) {
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

        cfg_if! {
            if #[cfg(feature = "g0")] {
                set_exti_g0!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                    (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                    (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                    (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]
                );
            } else if #[cfg(feature = "l5")] {
                set_exti_l5!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                    (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                    (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                    (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]
                );
            } else if #[cfg(feature = "f4")] {
                set_exti_f4!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                        (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                        (13, 4), (14, 4), (15, 4)]
                );
            } else {
                set_exti!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                    (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                    (13, 4), (14, 4), (15, 4)]
                );
            }
        }
    }

    /// Check if the pin's input voltage is high. Reads from the `IDR` register.
    pub fn is_high(&self) -> bool {
        get_input_data!(
            self.pin,
            self.port,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        )
    }

    /// Check if the pin's input voltage is low. Reads from the `IDR` register.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Set the pin's output voltage to high. Sets the `BSRR` register.
    pub fn set_high(&mut self) {
        set_state!(
            self.pin,
            self.port,
            0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set the pin's output voltage to low. Sets the `BSRR` register.
    pub fn set_low(&mut self) {
        set_state!(
            self.pin,
            self.port,
            16,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }
}
//
#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl InputPin for Pin {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl OutputPin for Pin {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl ToggleableOutputPin for Pin {
    type Error = Infallible;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        if self.is_high() {
            Pin::set_low(self);
        } else {
            Pin::set_high(self);
        }
        Ok(())
    }
}

// macro_rules! make_pin {
//     ($Port:ident) => {
//         paste! {
//
//         /// Represents a single GPIO pin. Provides methods that, when passed a mutable reference
//         /// to its port's register block, can change and read various properties of the pin.
//         pub struct [<Gpio $Port Pin>] {
//             pub port: Port,
//             pub pin: u8,
//         }
//
//         impl [<Gpio $Port Pin>] {
//             // We use macros where we can reduce code, and full functions where there's a difference
//             // from the macros.
//
//             /// Set pin mode.
//             pub fn mode(&mut self, value: PinMode, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, moder, moder, bits, value.val(), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//
//                 if let PinMode::Alt(alt) = value {
//                     self.alt_fn(alt, regs);
//                 }
//             }
//
//             /// Set output type.
//             pub fn output_type(&mut self, value: OutputType, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, otyper, ot, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//             }
//
//             /// Set output speed.
//             pub fn output_speed(&mut self, value: OutputSpeed, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, ospeedr, ospeedr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//             }
//
//             /// Set internal pull resistor: Pull up, pull down, or floating.
//             pub fn pull(&mut self, value: Pull, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, pupdr, pupdr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//             }
//
//             /// Set the output_data register.
//             pub fn output_data(&mut self, value: PinState, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, odr, odr, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//             }
//
//             // It appears f373 doesn't have lckr on ports C or E.
//             #[cfg(not(feature = "f373"))]
//             /// Lock or unlock a port configuration.
//             pub fn cfg_lock(&mut self, value: CfgLock, regs: &mut pac::[<GPIO $Port>]) {
//                 set_field!(self.pin, regs, lckr, lck, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
//             }
//
//             /// Read the input data register.
//             pub fn input_data(&mut self, regs: &mut pac::[<GPIO $Port>]) -> PinState {
//                 let reg_val = regs.idr.read();
//                 let val = match self.pin {
//                     0 => reg_val.idr0().bit_is_set(),
//                     1 => reg_val.idr1().bit_is_set(),
//                     2 => reg_val.idr2().bit_is_set(),
//                     3 => reg_val.idr3().bit_is_set(),
//                     4 => reg_val.idr4().bit_is_set(),
//                     5 => reg_val.idr5().bit_is_set(),
//                     6 => reg_val.idr6().bit_is_set(),
//                     7 => reg_val.idr7().bit_is_set(),
//                     8 => reg_val.idr8().bit_is_set(),
//                     9 => reg_val.idr9().bit_is_set(),
//                     10 => reg_val.idr10().bit_is_set(),
//                     11 => reg_val.idr11().bit_is_set(),
//                     12 => reg_val.idr12().bit_is_set(),
//                     13 => reg_val.idr13().bit_is_set(),
//                     14 => reg_val.idr14().bit_is_set(),
//                     15 => reg_val.idr15().bit_is_set(),
//                     _ => panic!("GPIO pins must be 0 - 15."),
//                 };
//
//                 if val {
//                     PinState::High
//                 } else {
//                     PinState::Low
//                 }
//             }
//
//             /// Set a pin state (ie set high or low output voltage level).
//             pub fn set_state(&mut self, value: PinState, regs: &mut pac::[<GPIO $Port>]) {
//                 let offset = match value {
//                     PinState::Low => 16,
//                     PinState::High => 0,
//                 };
//
//                 unsafe {
//                     regs.bsrr.write(|w| match self.pin {
//                         0 => w.bits(1 << (offset + 0)),
//                         1 => w.bits(1 << (offset + 1)),
//                         2 => w.bits(1 << (offset + 2)),
//                         3 => w.bits(1 << (offset + 3)),
//                         4 => w.bits(1 << (offset + 4)),
//                         5 => w.bits(1 << (offset + 5)),
//                         6 => w.bits(1 << (offset + 6)),
//                         7 => w.bits(1 << (offset + 7)),
//                         8 => w.bits(1 << (offset + 8)),
//                         9 => w.bits(1 << (offset + 9)),
//                         10 => w.bits(1 << (offset + 10)),
//                         11 => w.bits(1 << (offset + 11)),
//                         12 => w.bits(1 << (offset + 12)),
//                         13 => w.bits(1 << (offset + 13)),
//                         14 => w.bits(1 << (offset + 14)),
//                         15 => w.bits(1 << (offset + 15)),
//                         _ => panic!("GPIO pins must be 0 - 15."),
//                     });
//                 }
//             }
//
//             /// Set up a pin's alternate function. We set this up initially using `mode()`.
//             fn alt_fn(&mut self, value: u8, regs: &mut pac::[<GPIO $Port>]) {
//                 assert!(value <= 15, "Alt function must be 0 to 15.");
//                 cfg_if! {
//                     if #[cfg(any(feature = "l5", feature = "g0", feature = "wb", feature = "wl"))] {
//                         set_alt!(self.pin, regs, afsel, value, [(0, l), (1, l), (2, l),
//                             (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
//                             (13, h), (14, h), (15, h)])
//                     } else if #[cfg(feature = "h7")] {
//                         set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
//                             (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
//                             (13, h), (14, h), (15, h)])
//                     } else {  // f3, f4, l4, g4
//                         set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
//                             (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
//                             (13, h), (14, h), (15, h)])
//                     }
//                 }
//             }
//
//             // todo Error on these PACS, or are they missing BRR?
//             #[cfg(not(any(feature = "l4", feature = "h7", feature = "f4")))]
//             /// Reset an Output Data bit.
//             pub fn reset(&mut self, value: ResetState, regs: &mut pac::[<GPIO $Port>]) {
//                 let offset = match value {
//                     ResetState::NoAction => 16,
//                     ResetState::Reset => 0,
//                 };
//                 unsafe {
//                     regs.brr.write(|w| match self.pin {
//                         0 => w.bits(1 << (offset + 0)),
//                         1 => w.bits(1 << (offset + 1)),
//                         2 => w.bits(1 << (offset + 2)),
//                         3 => w.bits(1 << (offset + 3)),
//                         4 => w.bits(1 << (offset + 4)),
//                         5 => w.bits(1 << (offset + 5)),
//                         6 => w.bits(1 << (offset + 6)),
//                         7 => w.bits(1 << (offset + 7)),
//                         8 => w.bits(1 << (offset + 8)),
//                         9 => w.bits(1 << (offset + 9)),
//                         10 => w.bits(1 << (offset + 10)),
//                         11 => w.bits(1 << (offset + 11)),
//                         12 => w.bits(1 << (offset + 12)),
//                         13 => w.bits(1 << (offset + 13)),
//                         14 => w.bits(1 << (offset + 14)),
//                         15 => w.bits(1 << (offset + 15)),
//                         _ => panic!("GPIO pins must be 0 - 15."),
//                     });
//                 }
//             }
//
//             // We split into 2 separate functions, so newer MCUs don't need to pass the SYSCFG register.
//             cfg_if! {
//                 if #[cfg(any(feature = "g0", feature = "l5"))] {
//                     /// Configure this pin as an interrupt source.
//                     pub fn enable_interrupt(&mut self, edge: Edge) {
//                         // todo: On newer ones, don't accept SYSCFG for this function.
//                         let rise_trigger = match edge {
//                             Edge::Rising => {
//                                 // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
//                                 true
//                             }
//                             Edge::Falling => {
//                                 // configure EXTI line to trigger on falling edge, disable trigger on rising edge.
//                                 false
//                             }
//                         };
//
//                         #[cfg(feature = "g0")]
//                         set_exti_g0!(self.pin, exti, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
//                             (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
//                             (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
//                             (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]);
//
//                         #[cfg(feature = "l5")]
//                         set_exti_l5!(self.pin, exti, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
//                             (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
//                             (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
//                             (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]);
//
//                     }
//                 } else if #[cfg(not(feature = "f373"))] {
//                     /// Configure this pin as an interrupt source.
//                     pub fn enable_interrupt(&mut self, edge: Edge) {
//                         // todo: On newer ones, don't accept SYSCFG for this function.
//                         let rise_trigger = match edge {
//                             Edge::Rising => {
//                                 // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
//                                 true
//                             }
//                             Edge::Falling => {
//                                 // configure EXTI line to trigger on falling edge, disable trigger on rising edge.
//                                 false
//                             }
//                         };
//
//                         cfg_if! {
//                             if #[cfg(feature = "f4")] {
//                                 set_exti_f4!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
//                                     (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
//                                     (13, 4), (14, 4), (15, 4)])
//                             } else {
//                                 set_exti!(self.pin, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
//                                     (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
//                                     (13, 4), (14, 4), (15, 4)])
//                             }
//                         }
//                     }
//                 }
//             }
//
//             /// Check if the pin's input voltage is high (VCC).
//             pub fn is_high(&self) -> bool {
//                 // todo: DRy with `input_data`.
//                 let reg_val = unsafe { (*pac::[<GPIO $Port>]::ptr()).idr.read() };
//                 match self.pin {
//                     0 => reg_val.idr0().bit_is_set(),
//                     1 => reg_val.idr1().bit_is_set(),
//                     2 => reg_val.idr2().bit_is_set(),
//                     3 => reg_val.idr3().bit_is_set(),
//                     4 => reg_val.idr4().bit_is_set(),
//                     5 => reg_val.idr5().bit_is_set(),
//                     6 => reg_val.idr6().bit_is_set(),
//                     7 => reg_val.idr7().bit_is_set(),
//                     8 => reg_val.idr8().bit_is_set(),
//                     9 => reg_val.idr9().bit_is_set(),
//                     10 => reg_val.idr10().bit_is_set(),
//                     11 => reg_val.idr11().bit_is_set(),
//                     12 => reg_val.idr12().bit_is_set(),
//                     13 => reg_val.idr13().bit_is_set(),
//                     14 => reg_val.idr14().bit_is_set(),
//                     15 => reg_val.idr15().bit_is_set(),
//                     _ => panic!("GPIO pins must be 0 - 15."),
//                 }
//             }
//
//             /// Check if the pin's input voltage is low (ground).
//             pub fn is_low(&self) -> bool {
//                 !self.is_high()
//             }
//
//             /// Set the pin's output voltage to high (VCC).
//             pub fn set_high(&mut self) {
//                 // todo: DRY with self.set_low().
//                 let offset = 0;
//
//                 unsafe {
//                     (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| match self.pin {
//                         0 => w.bits(1 << (offset + 0)),
//                         1 => w.bits(1 << (offset + 1)),
//                         2 => w.bits(1 << (offset + 2)),
//                         3 => w.bits(1 << (offset + 3)),
//                         4 => w.bits(1 << (offset + 4)),
//                         5 => w.bits(1 << (offset + 5)),
//                         6 => w.bits(1 << (offset + 6)),
//                         7 => w.bits(1 << (offset + 7)),
//                         8 => w.bits(1 << (offset + 8)),
//                         9 => w.bits(1 << (offset + 9)),
//                         10 => w.bits(1 << (offset + 10)),
//                         11 => w.bits(1 << (offset + 11)),
//                         12 => w.bits(1 << (offset + 12)),
//                         13 => w.bits(1 << (offset + 13)),
//                         14 => w.bits(1 << (offset + 14)),
//                         15 => w.bits(1 << (offset + 15)),
//                         _ => panic!("GPIO pins must be 0 - 15."),
//                     });
//                 }
//             }
//
//             /// Set the pin's output voltage to ground (low).
//             pub fn set_low(&mut self) {
//                // todo; DRY with `set_state`
//                 let offset = 16;
//
//                 unsafe {
//                     (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| match self.pin {
//                         0 => w.bits(1 << (offset + 0)),
//                         1 => w.bits(1 << (offset + 1)),
//                         2 => w.bits(1 << (offset + 2)),
//                         3 => w.bits(1 << (offset + 3)),
//                         4 => w.bits(1 << (offset + 4)),
//                         5 => w.bits(1 << (offset + 5)),
//                         6 => w.bits(1 << (offset + 6)),
//                         7 => w.bits(1 << (offset + 7)),
//                         8 => w.bits(1 << (offset + 8)),
//                         9 => w.bits(1 << (offset + 9)),
//                         10 => w.bits(1 << (offset + 10)),
//                         11 => w.bits(1 << (offset + 11)),
//                         12 => w.bits(1 << (offset + 12)),
//                         13 => w.bits(1 << (offset + 13)),
//                         14 => w.bits(1 << (offset + 14)),
//                         15 => w.bits(1 << (offset + 15)),
//                         _ => panic!("GPIO pins must be 0 - 15."),
//                     });
//                 }
//             }
//         }
//
//         #[cfg(feature = "embedded-hal")]
//         #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
//         // Implement `embedded-hal` traits. We use raw pointers, since these traits can't
//         // accept a register block.
//         impl InputPin for [<Gpio $Port Pin>] {
//             type Error = Infallible;
//
//             fn is_high(&self) -> Result<bool, Self::Error> {
//                 Ok([<Gpio $Port Pin>]::is_high(self))
//             }
//
//             fn is_low(&self) -> Result<bool, Self::Error> {
//                 Ok([<Gpio $Port Pin>]::is_low(self))
//             }
//         }
//
//         #[cfg(feature = "embedded-hal")]
//         #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
//         impl OutputPin for [<Gpio $Port Pin>] {
//             type Error = Infallible;
//
//             fn set_low(&mut self) -> Result<(), Self::Error> {
//                 [<Gpio $Port Pin>]::set_low(self);
//                 Ok(())
//             }
//
//             fn set_high(&mut self) -> Result<(), Self::Error> {
//                 [<Gpio $Port Pin>]::set_high(self);
//                 Ok(())
//             }
//         }
//
//         #[cfg(feature = "embedded-hal")]
//         #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
//         impl ToggleableOutputPin for [<Gpio $Port Pin>] {
//             type Error = Infallible;
//
//             fn toggle(&mut self) -> Result<(), Self::Error> {
//                 if self.is_high() {
//                     [<Gpio $Port Pin>]::set_low(self);
//                 } else {
//                     [<Gpio $Port Pin>]::set_high(self);
//                 }
//                 Ok(())
//             }
//         }
//         }
//     };
// }

// todo: Missing EFGH impls on some variants that have them.
//
// make_port!(A, a);
// make_port!(B, b);
// make_port!(C, c);
//
// // make_pin!(A);
// // make_pin!(B);
// // make_pin!(C);
//
// cfg_if! {
//     if #[cfg(not(any(feature = "f410")))] {
//         make_port! (D, d);
//         // make_pin!(D);
//     }
// }
//
// cfg_if! {
//     // note: WB has port E, but is missing some field values etc. Not sure if in PAC or actual.
//     if #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0", feature = "wb", feature = "wl")))] {
//         make_port!(E, e);
//         // make_pin!(E);
//     }
// }
//
// cfg_if! {
//     if #[cfg(not(any(
//     feature = "f401",
//     feature = "f410",
//     feature = "f411",
//     feature = "l4x1",
//     feature = "l4x2",
//     feature = "l412",
//     feature = "l4x3",
//     feature = "wb",
//     feature = "wl"
//     )))] {
//         make_port!(F, f);
//         // make_pin!(F);
//     }
// }
//
// // todo: WB (and others?) have GPIOH 0-1 and 3. How can we fit that into this module layout?
// cfg_if! {
//     if #[cfg(not(any(
//         feature = "f373",
//         feature = "f301",
//         feature = "f3x4",
//         feature = "f410",
//         feature = "l4",
//         feature = "g0",
//         feature = "g4",
//         feature = "wb",
//         feature = "wl"
//     )))] {
//         make_port!(H, h);
//         // make_pin!(H);
//     }
// }
