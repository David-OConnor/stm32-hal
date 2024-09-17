//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIOx register functions. It also configures GPIO interrupts using SYSCFG and EXTI
//! registers as appropriate. It allows pin mode configuration, interrupts, and DMA.
//!
//! The primary API uses a `Pin` struct, with its methods. There are also standalone functions
//! available to set and read pin state, and clear interrupts, without access to a `Pin`.

// todo: WL is missing port C here due to some pins being missing, and this being tough
// todo to change with our current model. Note sure if PAC, or MCU limitation
// todo: WL is also missing interrupt support.

#[cfg(feature = "embedded_hal")]
use core::convert::Infallible;

#[cfg(feature = "embedded_hal")]
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use crate::pac::{self, EXTI, RCC};
#[cfg(not(feature = "h7"))]
use crate::util::rcc_en_reset;

// #[cfg(not(any(
//     // feature = "g0",
//     feature = "f4",
//     // feature = "l5",
//     feature = "f3",
//     feature = "l4"
// )))]
// use core::ops::Deref;

cfg_if! {
    if #[cfg(all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))))] {
        use crate::pac::DMA as DMA1;
    } else if #[cfg(any(feature = "f4", feature = "h5"))] {} else {
        use crate::pac::DMA1;
    }
}

use cfg_if::cfg_if;
use paste::paste;

#[cfg(not(any(feature = "f4", feature = "l552", feature = "h5")))]
use crate::dma::{self, ChannelCfg, DmaChannel};

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_MODER`. Sets pin to input, output, and other functionality.
pub enum PinMode {
    /// An input pin; read by firmware; set by something connected to the pin.
    Input,
    /// An output pin; set by firmware; read by something connected to the pin.
    Output,
    /// An alternate function, as defined in the MCU's user manual. Used for various
    /// onboard peripherals like buses, timers etc.
    Alt(u8),
    /// For use with the onboard ADC and DAC. Prevent parasitic power loss on the pin
    // if using it for one of these functionalities.
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
/// Values for `GPIOx_OTYPER`.
pub enum OutputType {
    PushPull = 0,
    OpenDrain = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_OSPEEDR`. This configures I/O output speed. See the user manual
/// for your MCU for what speeds these are. Note that Fast speed (0b10) is not
/// available on all STM32 families.
pub enum OutputSpeed {
    Low = 0b00,
    Medium = 0b01,
    #[cfg(not(feature = "f3"))]
    High = 0b10, // Called "Fast" on some families.
    VeryHigh = 0b11, // Called "High" on some families.
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_PUPDR`. Sets if the pin uses the internal pull-up or pull-down
// resistor.
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
/// Values for `GPIOx_LCKR`.
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
#[derive(Copy, Clone, PartialEq)]
/// GPIO port letter
pub enum Port {
    A,
    B,
    #[cfg(not(feature = "wl"))]
    C,
    #[cfg(not(any(feature = "f410", feature = "wl")))]
    D,
    #[cfg(not(any(
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "g0",
        feature = "wb",
        feature = "wl"
    )))]
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
    #[cfg(not(any(
        feature = "f373",
        feature = "f301",
        feature = "f3x4",
        feature = "f401",
        feature = "f410",
        feature = "f411",
        feature = "l4x1",
        feature = "l4x2",
        feature = "l412",
        feature = "l4x3",
        feature = "g0",
        feature = "wb",
        feature = "wl"
    )))]
    G,
    #[cfg(not(any(
        feature = "f373",
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4x1",
        feature = "l4x2",
        feature = "l412",
        feature = "l4x3",
        feature = "g0",
        feature = "g4",
        feature = "wb",
        feature = "wl"
    )))]
    H,
    #[cfg(any(feature = "h747cm4", feature = "h747cm7", feature = "l4x6",))]
    I,
}

impl Port {
    /// See F303 RM section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            #[cfg(not(feature = "wl"))]
            Self::C => 2,
            #[cfg(not(any(feature = "f410", feature = "wl")))]
            Self::D => 3,
            #[cfg(not(any(
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "g0",
                feature = "wb",
                feature = "wl"
            )))]
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
            #[cfg(not(any(
                feature = "f373",
                feature = "f301",
                feature = "f3x4",
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "g0",
                // feature = "g4",
                feature = "wb",
                feature = "wl"
            )))]
            Self::G => 6,
            #[cfg(not(any(
                feature = "f373",
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "g0",
                feature = "g4",
                feature = "wb",
                feature = "wl"
            )))]
            Self::H => 7,
            #[cfg(any(feature = "h747cm4", feature = "h747cm7", feature = "l4x6",))]
            Self::I => 8,
        }
    }
}

#[derive(Copy, Clone, Debug)]
/// The pulse edge used to trigger interrupts. Either rising, falling, or either.
pub enum Edge {
    /// Interrupts trigger on rising pin edge.
    Rising,
    /// Interrupts trigger on falling pin edge.
    Falling,
    /// Interrupts trigger on either rising or falling pin edges.
    Either,
}

// These macros are used to interate over pin number, for use with PAC fields.
macro_rules! set_field {
    ($regs:expr, $pin:expr, $reg:ident,$field:ident, $bit:ident, $val:expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => (*$regs).$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! set_alt {
    ($regs: expr, $pin:expr, $field_af:ident, $val:expr, [$(($num:expr, $lh:ident)),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => {
                            #[cfg(feature = "h5")]
                            (*$regs).moder.modify(|_, w| w.[<mode $num>]().bits(PinMode::Alt(0).val()));
                            #[cfg(not(feature = "h5"))]
                            (*$regs).moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                            #[cfg(any(feature = "l5", feature = "g0", feature = "h5", feature = "h7", feature = "wb"))]
                            (*$regs).[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val));
                            #[cfg(not(any(feature = "l5", feature = "g0", feature = "h5", feature = "h7", feature = "wb")))]
                            (*$regs).[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val));
                        }
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! get_input_data {
    ($regs: expr, $pin:expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        #[cfg(feature = "h5")]
                        $num => (*$regs).idr.read().[<id $num>]().bit_is_set(),
                        #[cfg(not(feature = "h5"))]
                        $num => (*$regs).idr.read().[<idr $num>]().bit_is_set(),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! set_state {
    ($regs: expr, $pin:expr, $offset: expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => (*$regs).bsrr.write(|w| w.bits(1 << ($offset + $num))),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

// todo: Consolidate these exti macros

// Reduce DRY for setting up interrupts.
macro_rules! set_exti {
    ($pin:expr, $rising:expr, $falling:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
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
                            } else if #[cfg(any(feature = "g4", feature = "wb", feature = "wl"))] {
                                exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());
                            } else {
                                exti.imr1.modify(|_, w| w.[<mr $num>]().set_bit());
                            }
                        }

                        cfg_if! {
                            if #[cfg(any(feature = "g4", feature = "wb", feature = "wl"))] {
                                exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($rising));
                                exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit($falling));
                            // } else if #[cfg(any(feature = "wb", feature = "wl"))] {
                            //     // todo: Missing in PAC, so we read+write. https://github.com/stm32-rs/stm32-rs/issues/570
                            //     let val_r =  $exti.rtsr1.read().bits();
                            //     $exti.rtsr1.write(|w| unsafe { w.bits(val_r | (1 << $num)) });
                            //     let val_f =  $exti.ftsr1.read().bits();
                            //     $exti.ftsr1.write(|w| unsafe { w.bits(val_f | (1 << $num)) });
                            //     // todo: Core 2 interrupts.
                            } else {
                                exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($rising));
                                exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit($falling));
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
// Similar to `set_exti`, but with reg names sans `1`.
macro_rules! set_exti_f4 {
    ($pin:expr, $rising:expr, $falling:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        let syscfg  = unsafe { &(*pac::SYSCFG::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr.modify(|_, w| w.[<mr $num>]().unmasked());
                        exti.rtsr.modify(|_, w| w.[<tr $num>]().bit($rising));
                        exti.ftsr.modify(|_, w| w.[<tr $num>]().bit($falling));
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

#[cfg(any(feature = "l5", feature = "h5"))]
// For L5 See `set_exti!`. Different method naming pattern for exticr.
macro_rules! set_exti_l5 {
    ($pin:expr, $rising:expr, $falling:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($rising));  // Rising trigger
                        exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit($falling));   // Falling trigger

                        #[cfg(feature = "l5")]
                        exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num2>]().bits($val) });

                        #[cfg(feature = "h5")]
                        exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}

#[cfg(feature = "g0")]
// For G0. See `set_exti!`. Todo? Reduce DRY.
macro_rules! set_exti_g0 {
    ($pin:expr, $rising:expr, $falling:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };

        paste! {
            match $pin {
                $(
                    $num => {
                        exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($rising));  // Rising trigger
                        // This field name is probably a PAC error.
                        exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit($falling));   // Falling trigger
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

#[derive(Clone)]
/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
    /// The GPIO Port letter. Eg A, B, C.
    pub port: Port,
    /// The pin number: 1 - 15.
    pub pin: u8,
}

impl Pin {
    /// Internal function to get the appropriate GPIO block pointer.
    const fn regs(&self) -> *const pac::gpioa::RegisterBlock {
        // Note that we use this `const` fn and pointer casting since not all ports actually
        // deref to GPIOA in PAC.
        regs(self.port)
    }

    /// Create a new pin, with a specific mode. Enables the RCC peripheral clock to the port,
    /// if not already enabled. Example: `let pa1 = Pin::new(Port::A, 1, PinMode::Output);` Leaves settings
    /// other than mode and alternate function (if applicable) at their hardware defaults.
    pub fn new(port: Port, pin: u8, mode: PinMode) -> Self {
        assert!(pin <= 15, "Pin must be 0 - 15.");

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
                            rcc_en_reset!(ahb1, iopb, rcc);
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
            #[cfg(not(feature = "wl"))]
            Port::C => {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        if rcc.ahbenr.read().iopcen().bit_is_clear() {
                            rcc_en_reset!(ahb1, iopc, rcc);
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
            #[cfg(not(any(feature = "f410", feature = "wl")))]
            Port::D => {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        if rcc.ahbenr.read().iopden().bit_is_clear() {
                            rcc_en_reset!(ahb1, iopd, rcc);
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
            #[cfg(not(any(
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "g0",
                feature = "wb",
                feature = "wl"
            )))]
            Port::E => {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        if rcc.ahbenr.read().iopeen().bit_is_clear() {
                            rcc_en_reset!(ahb1, iope, rcc);
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
                            rcc_en_reset!(ahb1, iopf, rcc);
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
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "g0",
                feature = "wb",
                feature = "wl"
            )))]
            Port::G => {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        if rcc.ahbenr.read().iopgen().bit_is_clear() {
                            rcc_en_reset!(ahb1, iopg, rcc);
                        }
                    } else if #[cfg(feature = "h7")] {
                        if rcc.ahb4enr.read().gpiogen().bit_is_clear() {
                            rcc.ahb4enr.modify(|_, w| w.gpiogen().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.gpiogrst().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.gpiogrst().clear_bit());
                        }
                    } else if #[cfg(feature = "f4")] {
                        if rcc.ahb1enr.read().gpiogen().bit_is_clear() {
                            rcc_en_reset!(ahb1, gpiog, rcc);
                        }
                    } else if #[cfg(feature = "g0")] {
                        if rcc.iopenr.read().iopgen().bit_is_clear() {
                            rcc.iopenr.modify(|_, w| w.iopgen().set_bit());
                            rcc.ioprstr.modify(|_, w| w.iopgrst().set_bit());
                            rcc.ioprstr.modify(|_, w| w.iopgrst().clear_bit());
                        }
                    } else { // L4, L5, G4
                        if rcc.ahb2enr.read().gpiogen().bit_is_clear() {
                            rcc_en_reset!(ahb2, gpiog, rcc);

                            #[cfg(feature = "l4x6")]
                            {
                                let pwr = unsafe { &(*pac::PWR::ptr()) };
                                // RM0351: Setting this bit (IOSV) is mandatory to use PG[15:2].
                                rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());
                                pwr.cr2.modify(|_, w| w.iosv().set_bit());
                            }
                        }
                    }
                }
                #[cfg(feature = "l5")]
                // also for RM0351 L4 variants, which we don't currently support
                // L5 RM: "[The IOSV bit] is used to validate the VDDIO2 supply for electrical and logical isolation purpose.
                // Setting this bit is mandatory to use PG[15:2]."
                {
                    unsafe {
                        (*crate::pac::PWR::ptr())
                            .cr2
                            .modify(|_, w| w.iosv().set_bit());
                    }
                }
            }
            #[cfg(not(any(
                feature = "f373",
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "g0",
                feature = "g4",
                feature = "wb",
                feature = "wl"
            )))]
            Port::H => {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        if rcc.ahbenr.read().iophen().bit_is_clear() {
                            rcc_en_reset!(ahb1, ioph, rcc);
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
                            rcc_en_reset!(ahb2, gpioh, rcc);
                        }
                    }
                }
            }
            #[cfg(any(feature = "l4x6", feature = "h747cm4", feature = "h747cm7"))]
            Port::I => {
                cfg_if! {
                    if #[cfg(feature = "h7")] {
                        if rcc.ahb4enr.read().gpioien().bit_is_clear() {
                            rcc.ahb4enr.modify(|_, w| w.gpioien().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.gpioirst().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.gpioirst().clear_bit());
                        }
                    } else if #[cfg(feature = "l4")] {
                        if rcc.ahb2enr.read().gpioien().bit_is_clear() {
                            rcc.ahb2enr.modify(|_,w| w.gpioien().set_bit());
                            rcc.ahb2rstr.modify(|_, w| w.gpioirst().set_bit());
                            rcc.ahb2rstr.modify(|_, w| w.gpioirst().clear_bit());
                        }
                    }
                }
            }
        }

        let mut result = Self { port, pin };
        result.mode(mode);

        result
    }

    /// Set pin mode. Eg, Output, Input, Analog, or Alt. Sets the `MODER` register.
    pub fn mode(&mut self, value: PinMode) {
        #[cfg(feature = "h5")] // todo: Probably needs a PAC fix for H5.
        set_field!(
            self.regs(),
            self.pin,
            moder,
            mode,
            bits,
            value.val(),
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );

        #[cfg(not(feature = "h5"))] // todo: Probably needs a PAC fix for H5.
        set_field!(
            self.regs(),
            self.pin,
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
        set_field!(
            self.regs(),
            self.pin,
            otyper,
            ot,
            bit,
            value as u8 != 0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set output speed to Low, Medium, or High. Sets the `OSPEEDR` register.
    pub fn output_speed(&mut self, value: OutputSpeed) {
        #[cfg(not(feature = "h5"))] // todo: Probably needs a PAC fix for H5.
        set_field!(
            self.regs(),
            self.pin,
            ospeedr,
            ospeedr,
            bits,
            value as u8,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set internal pull resistor: Pull up, pull down, or floating. Sets the `PUPDR` register.
    pub fn pull(&mut self, value: Pull) {
        #[cfg(feature = "h5")] // todo: Probably needs a PAC fix for H5.
        set_field!(
            self.regs(),
            self.pin,
            pupdr,
            pupd,
            bits,
            value as u8,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );

        #[cfg(not(feature = "h5"))] // todo: Probably needs a PAC fix for H5.
        set_field!(
            self.regs(),
            self.pin,
            pupdr,
            pupdr,
            bits,
            value as u8,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    // TODO: F373 doesn't have LOCKR on ports C, E, F. You can impl for others
    #[cfg(not(feature = "f373"))]
    /// Lock or unlock a port configuration. Sets the `LCKR` register.
    pub fn cfg_lock(&mut self, value: CfgLock) {
        set_field!(
            self.regs(),
            self.pin,
            lckr,
            lck,
            bit,
            value as u8 != 0,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Read the input data register. Eg determine if the pin is high or low. See also `is_high()`
    /// and `is_low()`. Reads from the `IDR` register.
    pub fn get_state(&mut self) -> PinState {
        let val = get_input_data!(
            self.regs(),
            self.pin,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
        if val {
            PinState::High
        } else {
            PinState::Low
        }
    }

    /// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
    /// `set_low()`. Sets the `BSRR` register. Atomic.
    pub fn set_state(&mut self, value: PinState) {
        let offset = match value {
            PinState::Low => 16,
            PinState::High => 0,
        };

        set_state!(
            self.regs(),
            self.pin,
            offset,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set up a pin's alternate function. We set this up initially using `mode()`.
    fn alt_fn(&mut self, value: u8) {
        assert!(value <= 15, "Alt function must be 0 to 15.");

        cfg_if! {
            if #[cfg(any(feature = "l5", feature = "g0", feature = "wb", feature = "h5"))] {
                set_alt!(self.regs(), self.pin, afsel, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            } else if #[cfg(feature = "h7")] {
                set_alt!(self.regs(), self.pin, afr, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            } else {  // f3, f4, l4, g4, wl(?)
                set_alt!(self.regs(), self.pin, afr, value, [(0, l), (1, l), (2, l),
                    (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                    (13, h), (14, h), (15, h)])
            }
        }
    }

    #[cfg(not(any(feature = "f373", feature = "wl")))]
    /// Configure this pin as an interrupt source. Set the edge as Rising or Falling.
    pub fn enable_interrupt(&mut self, edge: Edge) {
        let rising = match edge {
            Edge::Falling => false,
            _ => true, // rising or either.
        };

        let falling = match edge {
            Edge::Rising => false,
            _ => true, // falling or either.
        };

        cfg_if! {
            if #[cfg(feature = "g0")] {
                set_exti_g0!(self.pin, rising, falling, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                    (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                    (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                    (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]
                );
            } else if #[cfg(any(feature = "l5", feature = "h5"))] {
                set_exti_l5!(self.pin, rising, falling, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                    (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                    (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                    (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]
                );
            } else if #[cfg(feature = "f4")] {
                set_exti_f4!(self.pin, rising, falling, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                        (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                        (13, 4), (14, 4), (15, 4)]
                );
            } else {
                set_exti!(self.pin, rising, falling, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                    (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                    (13, 4), (14, 4), (15, 4)]
                );
            }
        }
    }

    #[cfg(feature = "l4x6")]
    /// For the ADC, DAC, OPAMP and COMP, configure the desired I/O in analog mode
    /// in the GPIOx_MODER register and configure the required function in the ADC,
    /// DAC, OPAMP, and COMP registers. For the ADC, it is necessary to configure the
    /// GPIOx_ASCR register (only for STM32L47x/L48x). Note that our `l4x6` feature
    /// gate is good enough here, since the most popular variants affected are L476 and L486.
    /// todo: Disconnect method?
    pub fn connect_to_adc(&mut self) {
        set_field!(
            self.regs(),
            self.pin,
            ascr,
            asc,
            bit,
            true,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Check if the pin's input voltage is high. Reads from the `IDR` register.
    pub fn is_high(&self) -> bool {
        get_input_data!(
            self.regs(),
            self.pin,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        )
    }

    /// Check if the pin's input voltage is low. Reads from the `IDR` register.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Set the pin's output voltage to high. Sets the `BSRR` register. Atomic.
    pub fn set_high(&mut self) {
        self.set_state(PinState::High);
    }

    /// Set the pin's output voltage to low. Sets the `BSRR` register. Atomic.
    pub fn set_low(&mut self) {
        self.set_state(PinState::Low);
    }

    /// Toggle output voltage between low and high. Sets the `BSRR` register. Atomic.
    pub fn toggle(&mut self) {
        // if self.is_high() {
        if Pin::is_high(self) {
            Pin::set_low(self);
            // self.set_low();
        } else {
            // self.set_high();
            Pin::set_high(self);
        }
    }
}

#[cfg(feature = "embedded_hal")]
impl ErrorType for Pin {
    type Error = Infallible;
}

#[cfg(feature = "embedded_hal")]
impl InputPin for Pin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }
}

#[cfg(feature = "embedded_hal")]
impl OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

#[cfg(feature = "embedded_hal")]
impl StatefulOutputPin for Pin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        Pin::toggle(self);
        Ok(())
    }
}

/// Check if a pin's input voltage is high. Reads from the `IDR` register.
/// Does not require a `Pin` struct.
pub fn is_high(port: Port, pin: u8) -> bool {
    get_input_data!(
        regs(port),
        pin,
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    )
}

/// Check if a pin's input voltage is low. Reads from the `IDR` register.
/// Does not require a `Pin` struct.
pub fn is_low(port: Port, pin: u8) -> bool {
    !is_high(port, pin)
}

/// Set a pin's output voltage to high. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
pub fn set_high(port: Port, pin: u8) {
    set_state(port, pin, PinState::High);
}

/// Set a pin's output voltage to low. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
pub fn set_low(port: Port, pin: u8) {
    set_state(port, pin, PinState::Low);
}

/// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
/// `set_low()`. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
pub fn set_state(port: Port, pin: u8, value: PinState) {
    let offset = match value {
        PinState::Low => 16,
        PinState::High => 0,
    };

    set_state!(
        regs(port),
        pin,
        offset,
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    );
}

/// Clear an EXTI interrupt, lines 0 - 15. Note that this function currently doesn't support
/// higher extis, but will work for all GPIO interrupts.
pub fn clear_exti_interrupt(line: u8) {
    // todo: Macro to avoid DRY?
    unsafe {
        cfg_if! {
            if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
                (*EXTI::ptr()).c1pr1.modify(|_, w| {
                    match line {
                        0 => w.pr0().set_bit(),
                        1 => w.pr1().set_bit(),
                        2 => w.pr2().set_bit(),
                        3 => w.pr3().set_bit(),
                        4 => w.pr4().set_bit(),
                        5 => w.pr5().set_bit(),
                        6 => w.pr6().set_bit(),
                        7 => w.pr7().set_bit(),
                        8 => w.pr8().set_bit(),
                        9 => w.pr9().set_bit(),
                        10 => w.pr10().set_bit(),
                        11 => w.pr11().set_bit(),
                        12 => w.pr12().set_bit(),
                        13 => w.pr13().set_bit(),
                        14 => w.pr14().set_bit(),
                        15 => w.pr15().set_bit(),
                        _ => panic!(),
                    }
                });
            } else if #[cfg(feature = "h7")] {
                (*EXTI::ptr()).cpupr1.modify(|_, w| {
                    match line {
                        0 => w.pr0().set_bit(),
                        1 => w.pr1().set_bit(),
                        2 => w.pr2().set_bit(),
                        3 => w.pr3().set_bit(),
                        4 => w.pr4().set_bit(),
                        5 => w.pr5().set_bit(),
                        6 => w.pr6().set_bit(),
                        7 => w.pr7().set_bit(),
                        8 => w.pr8().set_bit(),
                        9 => w.pr9().set_bit(),
                        10 => w.pr10().set_bit(),
                        11 => w.pr11().set_bit(),
                        12 => w.pr12().set_bit(),
                        13 => w.pr13().set_bit(),
                        14 => w.pr14().set_bit(),
                        15 => w.pr15().set_bit(),
                        _ => panic!(),
                    }
                });
            } else if #[cfg(any(feature = "l5", feature = "g0"))] {
                (*EXTI::ptr()).rpr1.modify(|_, w| {
                    match line {
                        0 => w.rpif0().set_bit(),
                        1 => w.rpif1().set_bit(),
                        2 => w.rpif2().set_bit(),
                        3 => w.rpif3().set_bit(),
                        4 => w.rpif4().set_bit(),
                        5 => w.rpif5().set_bit(),
                        6 => w.rpif6().set_bit(),
                        7 => w.rpif7().set_bit(),
                        8 => w.rpif8().set_bit(),
                        9 => w.rpif9().set_bit(),
                        10 => w.rpif10().set_bit(),
                        11 => w.rpif11().set_bit(),
                        12 => w.rpif12().set_bit(),
                        13 => w.rpif13().set_bit(),
                        14 => w.rpif14().set_bit(),
                        15 => w.rpif15().set_bit(),
                        _ => panic!(),
                    }
                });
            } else if #[cfg(any(feature = "f373", feature = "f4"))] {
                (*EXTI::ptr()).pr.modify(|_, w| {
                    match line {
                        0 => w.pr0().set_bit(),
                        1 => w.pr1().set_bit(),
                        2 => w.pr2().set_bit(),
                        3 => w.pr3().set_bit(),
                        4 => w.pr4().set_bit(),
                        5 => w.pr5().set_bit(),
                        6 => w.pr6().set_bit(),
                        7 => w.pr7().set_bit(),
                        8 => w.pr8().set_bit(),
                        9 => w.pr9().set_bit(),
                        10 => w.pr10().set_bit(),
                        11 => w.pr11().set_bit(),
                        12 => w.pr12().set_bit(),
                        13 => w.pr13().set_bit(),
                        14 => w.pr14().set_bit(),
                        15 => w.pr15().set_bit(),
                        _ => panic!(),
                    }
                });
            } else if #[cfg(any(feature = "f3", feature = "l4"))] {
                (*EXTI::ptr()).pr1.modify(|_, w| {
                    match line {
                        0 => w.pr0().set_bit(),
                        1 => w.pr1().set_bit(),
                        2 => w.pr2().set_bit(),
                        3 => w.pr3().set_bit(),
                        4 => w.pr4().set_bit(),
                        5 => w.pr5().set_bit(),
                        6 => w.pr6().set_bit(),
                        7 => w.pr7().set_bit(),
                        8 => w.pr8().set_bit(),
                        9 => w.pr9().set_bit(),
                        10 => w.pr10().set_bit(),
                        11 => w.pr11().set_bit(),
                        12 => w.pr12().set_bit(),
                        13 => w.pr13().set_bit(),
                        14 => w.pr14().set_bit(),
                        15 => w.pr15().set_bit(),
                        _ => panic!(),
                    }
                });
              } else if #[cfg(feature = "h5")] {
                (*EXTI::ptr()).rpr1.modify(|_, w| {
                    match line {
                        0 => w.rpif0().set_bit(),
                        1 => w.rpif1().set_bit(),
                        2 => w.rpif2().set_bit(),
                        3 => w.rpif3().set_bit(),
                        4 => w.rpif4().set_bit(),
                        5 => w.rpif5().set_bit(),
                        6 => w.rpif6().set_bit(),
                        7 => w.rpif7().set_bit(),
                        8 => w.rpif8().set_bit(),
                        9 => w.rpif9().set_bit(),
                        10 => w.rpif10().set_bit(),
                        11 => w.rpif11().set_bit(),
                        12 => w.rpif12().set_bit(),
                        13 => w.rpif13().set_bit(),
                        14 => w.rpif14().set_bit(),
                        15 => w.rpif15().set_bit(),
                        _ => panic!(),
                    }
                });
            } else { // eg G4
                (*EXTI::ptr()).pr1.modify(|_, w| {
                    match line {
                        0 => w.pif0().set_bit(),
                        1 => w.pif1().set_bit(),
                        2 => w.pif2().set_bit(),
                        3 => w.pif3().set_bit(),
                        4 => w.pif4().set_bit(),
                        5 => w.pif5().set_bit(),
                        6 => w.pif6().set_bit(),
                        7 => w.pif7().set_bit(),
                        8 => w.pif8().set_bit(),
                        9 => w.pif9().set_bit(),
                        10 => w.pif10().set_bit(),
                        11 => w.pif11().set_bit(),
                        12 => w.pif12().set_bit(),
                        13 => w.pif13().set_bit(),
                        14 => w.pif14().set_bit(),
                        15 => w.pif15().set_bit(),
                        _ => panic!(),
                    }
                });
            }
        }
    }
}

const fn regs(port: Port) -> *const pac::gpioa::RegisterBlock {
    // Note that we use this `const` fn and pointer casting since not all ports actually
    // deref to GPIOA in PAC.
    match port {
        Port::A => crate::pac::GPIOA::ptr(),
        Port::B => crate::pac::GPIOB::ptr() as _,
        #[cfg(not(feature = "wl"))]
        Port::C => crate::pac::GPIOC::ptr() as _,
        #[cfg(not(any(feature = "f410", feature = "wl")))]
        Port::D => crate::pac::GPIOD::ptr() as _,
        #[cfg(not(any(
            feature = "f301",
            feature = "f3x4",
            feature = "f410",
            feature = "g0",
            feature = "wb",
            feature = "wl"
        )))]
        Port::E => crate::pac::GPIOE::ptr() as _,
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
        Port::F => crate::pac::GPIOF::ptr() as _,
        #[cfg(not(any(
            feature = "f373",
            feature = "f301",
            feature = "f3x4",
            feature = "f401",
            feature = "f410",
            feature = "f411",
            feature = "l4x1",
            feature = "l4x2",
            feature = "l412",
            feature = "l4x3",
            feature = "g0",
            feature = "wb",
            feature = "wl"
        )))]
        Port::G => crate::pac::GPIOG::ptr() as _,
        #[cfg(not(any(
            feature = "f373",
            feature = "f301",
            feature = "f3x4",
            feature = "f410",
            feature = "l4x1",
            feature = "l4x2",
            feature = "l412",
            feature = "l4x3",
            feature = "g0",
            feature = "g4",
            feature = "wb",
            feature = "wl"
        )))]
        Port::H => crate::pac::GPIOH::ptr() as _,
        #[cfg(any(feature = "h747cm4", feature = "h747cm7", feature = "l4x6"))]
        Port::I => crate::pac::GPIOI::ptr() as _,
    }
}

#[cfg(not(any(
    feature = "f4",
    feature = "l5",
    feature = "f3",
    feature = "l4",
    feature = "h5"
)))]
/// Write a series of words to the BSRR (atomic output) register. Note that these are direct writes
/// to the full, 2-sided register - not a series of low/high values.
pub unsafe fn write_dma(
    buf: &[u32],
    port: Port,
    dma_channel: DmaChannel,
    channel_cfg: ChannelCfg,
    dma_periph: dma::DmaPeriph,
) {
    let (ptr, len) = (buf.as_ptr(), buf.len());

    let periph_addr = &(*(regs(port))).bsrr as *const _ as u32;

    #[cfg(feature = "h7")]
    let num_data = len as u32;
    #[cfg(not(feature = "h7"))]
    let num_data = len as u16;

    match dma_periph {
        dma::DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            dma::cfg_channel(
                &mut regs,
                dma_channel,
                periph_addr,
                ptr as u32,
                num_data,
                dma::Direction::ReadFromMem,
                dma::DataSize::S32,
                dma::DataSize::S32,
                channel_cfg,
            );
        }
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        dma::DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            dma::cfg_channel(
                &mut regs,
                dma_channel,
                periph_addr,
                ptr as u32,
                num_data,
                dma::Direction::ReadFromMem,
                dma::DataSize::S32,
                dma::DataSize::S32,
                channel_cfg,
            );
        }
    }
}

#[cfg(not(any(
    feature = "f4",
    feature = "l5",
    feature = "f3",
    feature = "l4",
    feature = "h5"
)))]
/// Read a series of words from the IDR register.
pub unsafe fn read_dma(
    buf: &[u32],
    port: Port,
    dma_channel: DmaChannel,
    channel_cfg: ChannelCfg,
    dma_periph: dma::DmaPeriph,
) {
    let (ptr, len) = (buf.as_ptr(), buf.len());

    let periph_addr = &(*(regs(port))).idr as *const _ as u32;

    #[cfg(feature = "h7")]
    let num_data = len as u32;
    #[cfg(not(feature = "h7"))]
    let num_data = len as u16;

    match dma_periph {
        dma::DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            dma::cfg_channel(
                &mut regs,
                dma_channel,
                periph_addr,
                ptr as u32,
                num_data,
                dma::Direction::ReadFromPeriph,
                dma::DataSize::S32,
                dma::DataSize::S32,
                channel_cfg,
            );
        }
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        dma::DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            dma::cfg_channel(
                &mut regs,
                dma_channel,
                periph_addr,
                ptr as u32,
                num_data,
                dma::Direction::ReadFromPeriph,
                dma::DataSize::S32,
                dma::DataSize::S32,
                channel_cfg,
            );
        }
    }
}
