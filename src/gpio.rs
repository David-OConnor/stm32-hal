//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIOx register functions, and interrupts.
//! It includes implementations of `embedded-hal` pin abstraction.

use core::convert::Infallible;

use crate::{
    pac::{self, EXTI, RCC},
    rcc_en_reset,
};

#[cfg(not(any(feature = "l5", feature = "g0")))]
use crate::pac::SYSCFG;

use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use cfg_if::cfg_if;
use paste::paste;

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
    Af0 = 0,
    Af1 = 1,
    Af2 = 2,
    Af3 = 3,
    Af4 = 4,
    Af5 = 5,
    Af6 = 6,
    Af7 = 7,
    Af8 = 8,
    Af9 = 9,
    Af10 = 10,
    Af11 = 11,
    Af12 = 12,
    Af13 = 13,
    Af14 = 14,
    Af15 = 15,
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

// // todo: Should this trait be in `traits.rs` (or eventually crate) ?
// /// Gpio pin traits. Used to check pin config when passing to peripheral constructors.
// pub trait GpioPin {
//     /// Port letter (eg A)
//     fn get_port(&self) -> PortLetter;
//
//     /// Pin num (eg P4)
//     fn get_pin(&self) -> PinNum;
//
//     /// Pin mode (input, output, alt, analog), and the alt function if applicable.
//     fn get_mode(&self) -> PinMode;
//
//     /// Output type. Ie open drain or push pull.
//     fn get_output_type(&self) -> OutputType;
// }

macro_rules! make_port {
    ($Port:ident, $port:ident) => {
        paste! {
            /// Represents a single GPIO port, and owns its register block. Provides
            /// methods to enable the port. To change pin properties, pass its `regs`
            /// field as a mutable reference to `GpioXPin` methods.
            pub struct [<Gpio $Port>] {
                pub regs: pac::[<GPIO $Port>],
            }

            impl [<Gpio $Port>] {
                pub fn new(regs: pac::[<GPIO $Port>], rcc: &mut RCC) -> Self {
                    // Enable the peripheral clock of a GPIO port

                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            rcc_en_reset!(ahb1, [<iop $port>], rcc);
                        } else if #[cfg(feature = "h7")] {
                            rcc.ahb4enr.modify(|_, w| w.[<gpio $port en>]().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().clear_bit());
                        } else if #[cfg(feature = "f4")] {
                            rcc_en_reset!(ahb1, [<gpio $port>], rcc);
                        } else if #[cfg(feature = "g0")] {
                            rcc.iopenr.modify(|_, w| w.[<iop $port en>]().set_bit());
                            rcc.ioprstr.modify(|_, w| w.[<iop $port rst>]().set_bit());
                            rcc.ioprstr.modify(|_, w| w.[<iop $port rst>]().clear_bit());
                        } else { // L4, L5, G4
                            rcc_en_reset!(ahb2, [<gpio $port>], rcc);
                        }
                    }

                    Self { regs }
                }

                pub fn new_pin(&mut self, pin: PinNum, mode: PinMode) -> [<Gpio $Port Pin>] {
                    let mut result = [<Gpio $Port Pin>] {
                        port: PortLetter::[<$Port>],
                        pin,
                    };
                    result.mode(mode, &mut self.regs);

                    result
                }
            }
        }
    };
}

/// Reduce DRY for setting fields.
macro_rules! set_field {
    ($pin:expr, $regs:expr, $reg:ident, $field:ident, $bit:ident, $val:expr, [$($num:expr),+]) => {
        paste! {
            // Unsafe may or may not be required, depending on the PAC.
            unsafe {
                match $pin {
                    $(
                        PinNum::[<P $num>] => $regs.$reg.modify(|_, w| w.[<$field $num>]().$bit($val)),
                    )+
                }
            }
        }
    }
}

// todo: Consolidate these exti macros

/// Reduce DRY for setting up interrupts.
macro_rules! set_exti {
    ($pin:expr, $exti:expr, $syscfg:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        paste! {
            match $pin {
                $(
                    PinNum::[<P $num>] => {
                        cfg_if! {
                            if #[cfg(all(feature = "h7", not(any(feature = "h747cm4", feature = "h747cm7"))))] {
                                $exti.cpuimr1.modify(|_, w| w.[<mr $num>]().unmasked());
                            } else if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
                                $exti.c1imr1.modify(|_, w| w.[<mr $num>]().unmasked());
                            }else if #[cfg(feature = "g4")] {
                                $exti.imr1.modify(|_, w| w.[<im $num>]().unmasked());
                            } else {
                                $exti.imr1.modify(|_, w| w.[<mr $num>]().unmasked());
                            }
                        }

                        cfg_if! {
                            if #[cfg(feature = "g4")] {
                                $exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($trigger));
                                $exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit(!$trigger));
                            } else {
                                $exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($trigger));
                                $exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit(!$trigger));
                            }
                        }
                        $syscfg
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
                    }
                )+
            }
        }
    }
}

#[cfg(feature = "f4")]
/// Similar to `set_exti`, but with reg names sans `1`.
macro_rules! set_exti_f4 {
    ($pin:expr, $exti:expr, $syscfg:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        paste! {
            match $pin {
                $(
                    PinNum::[<P $num>] => {
                        $exti.imr.modify(|_, w| w.[<mr $num>]().unmasked());
                        $exti.rtsr.modify(|_, w| w.[<tr $num>]().bit($trigger));
                        $exti.ftsr.modify(|_, w| w.[<tr $num>]().bit(!$trigger));
                        $syscfg
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
                    }
                )+
            }
        }
    }
}

#[cfg(feature = "l5")]
/// For L5 See `set_exti!`. Different method naming pattern for exticr.
macro_rules! set_exti_l5 {
    ($pin:expr, $exti:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        paste! {
            match $pin {
                $(
                    PinNum::[<P $num>] => {
                        $exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        $exti.rtsr1.modify(|_, w| w.[<rt $num>]().bit($trigger));  // Rising trigger
                        $exti.ftsr1.modify(|_, w| w.[<ft $num>]().bit(!$trigger));   // Falling trigger
                        $exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num2>]().bits($val) });
                    }
                )+
            }
        }
    }
}

#[cfg(feature = "g0")]
/// ForG0. See `set_exti!`. Todo? Reduce DRY.
macro_rules! set_exti_g0 {
    ($pin:expr, $exti:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr, $num2:expr)),+]) => {
        paste! {
            match $pin {
                $(
                    PinNum::[<P $num>] => {
                        $exti.imr1.modify(|_, w| w.[<im $num>]().set_bit());  // unmask
                        $exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($trigger));  // Rising trigger
                        $exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit(!$trigger));   // Falling trigger
                        $exti
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num2>]().bits($val) });
                    }
                )+
            }
        }
    }
}

/// Reduce DRY for setting up alternate functions. Note that there are at least 3
/// different names for the `afrl` field to modify based on variants.
macro_rules! set_alt {
    ($pin:expr, $regs:expr, $field_af:ident, $val:expr, [$(($num:expr, $lh:ident)),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        PinNum::[<P $num>] => {
                            $regs.moder.modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(AltFn::Af0).val()));
                            #[cfg(any(feature = "l5", feature = "g0", feature = "h7"))]
                            $regs.[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val as u8));
                            #[cfg(not(any(feature = "l5", feature = "g0", feature = "h7")))]
                            $regs.[<afr $lh>].modify(|_, w| w.[<$field_af $lh $num>]().bits($val as u8));
                        }
                    )+
                }
            }
        }
    }
}

macro_rules! make_pin {
    ($Port:ident) => {
        paste! {

        /// Represents a single GPIO pin. Provides methods that, when passed a mutable reference
        /// to its port's register block, can change and read various properties of the pin.
        pub struct [<Gpio $Port Pin>] {
            pub port: PortLetter,
            pub pin: PinNum,
        }

        impl [<Gpio $Port Pin>] {
            // We use macros where we can reduce code, and full functions where there's a difference
            // from the macros.

            /// Set pin mode.
            pub fn mode(&mut self, value: PinMode, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, moder, moder, bits, value.val(), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);

                if let PinMode::Alt(alt) = value {
                    self.alt_fn(alt, regs);
                }
            }

            /// Set output type.
            pub fn output_type(&mut self, value: OutputType, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, otyper, ot, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Set output speed.
            pub fn output_speed(&mut self, value: OutputSpeed, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, ospeedr, ospeedr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Set internal pull resistor: Pull up, pull down, or floating.
            pub fn pull(&mut self, value: Pull, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, pupdr, pupdr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Set the output_data register.
            pub fn output_data(&mut self, value: PinState, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, odr, odr, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            // It appears f373 doesn't have lckr on ports C or E.
            #[cfg(not(feature = "f373"))]
            /// Lock or unlock a port configuration.
            pub fn cfg_lock(&mut self, value: CfgLock, regs: &mut pac::[<GPIO $Port>]) {
                set_field!(self.pin, regs, lckr, lck, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Read the input data register.
            pub fn input_data(&mut self, regs: &mut pac::[<GPIO $Port>]) -> PinState {
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

            /// Set a pin state (ie set high or low output voltage level).
            pub fn set_state(&mut self, value: PinState, regs: &mut pac::[<GPIO $Port>]) {
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

            /// Set up a pin's alternate function. We set this up initially using `mode()`.
            fn alt_fn(&mut self, value: AltFn, regs: &mut pac::[<GPIO $Port>]) {
                cfg_if! {
                    if #[cfg(any(feature = "l5", feature = "g0"))] {
                        set_alt!(self.pin, regs, afsel, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
                    } else if #[cfg(feature = "h7")] {
                        set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
                    } else {  // f3, f4, l4, g4
                        set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
                    }
                }
            }

            // todo Error on these PACS, or are they missing BRR?
            #[cfg(not(any(feature = "l4", feature = "h7", feature = "f4")))]
            /// Reset an Output Data bit.
            pub fn reset(&mut self, value: ResetState, regs: &mut pac::[<GPIO $Port>]) {
                let offset = match value {
                    ResetState::NoAction => 16,
                    ResetState::Reset => 0,
                };
                unsafe {
                    match self.pin {
                        PinNum::P0 => regs.brr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => regs.brr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => regs.brr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => regs.brr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => regs.brr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => regs.brr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => regs.brr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => regs.brr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => regs.brr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => regs.brr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => regs.brr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => regs.brr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => regs.brr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => regs.brr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => regs.brr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => regs.brr.write(|w| w.bits(1 << (offset + 15))),
                    };
                }
            }

            // We split into 2 separate functions, so newer MCUs don't need to pass the SYSCFG register.
            cfg_if! {
                if #[cfg(any(feature = "g0", feature = "l5"))] {
                    /// Configure this pin as an interrupt source.
                    pub fn enable_interrupt(&mut self, edge: Edge, exti: &mut EXTI) {
                        // todo: On newer ones, don't accept SYSCFG for this function.
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

                        #[cfg(feature = "g0")]
                        set_exti_g0!(self.pin, exti, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                            (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                            (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                            (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]);

                        #[cfg(feature = "l5")]
                        set_exti_l5!(self.pin, exti, rise_trigger, self.port.cr_val(), [(0, 1, 0_7), (1, 1, 0_7), (2, 1, 0_7),
                            (3, 1, 0_7), (4, 2, 0_7), (5, 2, 0_7), (6, 2, 0_7), (7, 2, 0_7), (8, 3, 8_15),
                            (9, 3, 8_15), (10, 3, 8_15), (11, 3, 8_15), (12, 4, 8_15),
                            (13, 4, 8_15), (14, 4, 8_15), (15, 4, 8_15)]);

                    }
                } else if #[cfg(not(feature = "f373"))] {
                    /// Configure this pin as an interrupt source.
                    pub fn enable_interrupt(&mut self, edge: Edge, exti: &mut EXTI, syscfg: &mut SYSCFG) {
                        // todo: On newer ones, don't accept SYSCFG for this function.
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
                            if #[cfg(feature = "f4")] {
                                set_exti_f4!(self.pin, exti, syscfg, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                                    (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                                    (13, 4), (14, 4), (15, 4)])
                            } else {
                                set_exti!(self.pin, exti, syscfg, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                                    (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                                    (13, 4), (14, 4), (15, 4)])
                            }
                        }
                    }
                }
            }

            /// Check if the pin's input voltage is high (VCC).
            pub fn is_high(&self) -> bool {
                // todo: DRy with `input_data`.
                unsafe {
                    match self.pin {
                        PinNum::P0 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr0().bit(),
                        PinNum::P1 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr1().bit(),
                        PinNum::P2 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr2().bit(),
                        PinNum::P3 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr3().bit(),
                        PinNum::P4 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr4().bit(),
                        PinNum::P5 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr5().bit(),
                        PinNum::P6 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr6().bit(),
                        PinNum::P7 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr7().bit(),
                        PinNum::P8 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr8().bit(),
                        PinNum::P9 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr9().bit(),
                        PinNum::P10 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr10().bit(),
                        PinNum::P11 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr11().bit(),
                        PinNum::P12 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr12().bit(),
                        PinNum::P13 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr13().bit(),
                        PinNum::P14 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr14().bit(),
                        PinNum::P15 => (*pac::[<GPIO $Port>]::ptr()).idr.read().idr15().bit(),
                    }
                }
            }

            /// Check if the pin's input voltage is low (ground).
            pub fn is_low(&self) -> bool {
                !self.is_high()
            }

            /// Set the pin's output voltage to high (VCC).
            pub fn set_high(&mut self) {
                // todo: DRY with self.set_low().
                let offset = 0;

                unsafe {
                    match self.pin {
                        PinNum::P0 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 15))),
                    }
                }
            }

            /// Set the pin's output voltage to ground (low).
            pub fn set_low(&mut self) {
               // todo; DRY with `set_state`
                let offset = 16;

                unsafe {
                    match self.pin {
                        PinNum::P0 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 0))),
                        PinNum::P1 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 1))),
                        PinNum::P2 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 2))),
                        PinNum::P3 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 3))),
                        PinNum::P4 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 4))),
                        PinNum::P5 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 5))),
                        PinNum::P6 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 6))),
                        PinNum::P7 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 7))),
                        PinNum::P8 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 8))),
                        PinNum::P9 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 9))),
                        PinNum::P10 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 10))),
                        PinNum::P11 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 11))),
                        PinNum::P12 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 12))),
                        PinNum::P13 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 13))),
                        PinNum::P14 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 14))),
                        PinNum::P15 => (*pac::[<GPIO $Port>]::ptr()).bsrr.write(|w| w.bits(1 << (offset + 15))),
                    }
                }
            }
        }

        // Implement `embedded-hal` traits. We use raw pointers, since these traits can't
        // accept a register block.
        impl InputPin for [<Gpio $Port Pin>] {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok([<Gpio $Port Pin>]::is_high(self))
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok([<Gpio $Port Pin>]::is_low(self))
            }
        }

        impl OutputPin for [<Gpio $Port Pin>] {
            type Error = Infallible;

            fn set_low(&mut self) -> Result<(), Self::Error> {
                [<Gpio $Port Pin>]::set_low(self);
                Ok(())
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                [<Gpio $Port Pin>]::set_high(self);
                Ok(())
            }
        }

        impl ToggleableOutputPin for [<Gpio $Port Pin>] {
            type Error = Infallible;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_high() {
                    [<Gpio $Port Pin>]::set_low(self);
                } else {
                    [<Gpio $Port Pin>]::set_high(self);
                }
                Ok(())
            }
        }
        }
    };
}

// todo: Missing EFGH impls on some variants that have them.

make_port!(A, a);
make_port!(B, b);
make_port!(C, c);

make_pin!(A);
make_pin!(B);
make_pin!(C);

cfg_if! {
    if #[cfg(not(any(feature = "f410")))] {
        make_port! (D, d);
        make_pin!(D);
    }
}

cfg_if! {
    if #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f410", feature = "g0")))] {
        make_port!(E, e);
        make_pin!(E);
    }
}

// todo: 412 workaround
cfg_if! {
    if #[cfg(not(any(feature = "f401", feature = "f410", feature = "f411", feature = "l4x1", feature = "l4x2", feature = "l412", feature = "l4x3")))] {
        make_port!(F, f);
        make_pin!(F);
    }
}

cfg_if! {
    if #[cfg(not(any(
        feature = "f373",
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4",
        feature = "g0",
        feature = "g4"
    )))] {
        make_port!(H, h);
        make_pin!(H);
    }
}
