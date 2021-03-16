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
                        } else if #[cfg(feature = "h7")] {
                            rcc.ahb4enr.modify(|_, w| w.[<gpio $port en>]().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().set_bit());
                            rcc.ahb4rstr.modify(|_, w| w.[<gpio $port rst>]().clear_bit());
                        } else {
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

/// Reduce DRY for sett ing up interrupts.
macro_rules! set_exti {
    ($pin:expr, $exti:expr, $syscfg:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        paste! {
            match $pin {
                $(
                    PinNum::[<P $num>] => {
                        // Unmask the line.
                        $exti.imr1.modify(|_, w| w.[<mr $num>]().unmasked());
                        // Configure the trigger edge
                        $exti.rtsr1.modify(|_, w| w.[<tr $num>]().bit($trigger));  // Rising trigger
                        $exti.ftsr1.modify(|_, w| w.[<tr $num>]().bit(!$trigger));   // Falling trigger
                        // Select this GPIO pin as source input for EXTI line external interrupt
                        $syscfg
                            .[<exticr $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num>]().bits($val) });
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
                            $regs.moder.modify(|_, w| w.moder0().bits(PinMode::Alt($val).val()));
                            #[cfg(any(feature = "l5", feature = "h7"))]
                            $regs.[<afr $lh>].modify(|_, w| w.[<$field_af $num>]().bits($val as u8));
                            #[cfg(not(any(feature = "l5", feature = "h7")))]
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
                set_field!(self.pin, regs, moder, moder, bits, value.val(), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);

                self.mode = value;

                if let PinMode::Alt(alt) = value {
                    self.alt_fn(alt, regs);
                }
            }

            /// Set output type
            pub fn output_type(&mut self, value: OutputType, regs: &mut [<GPIO $Port>]) {
                set_field!(self.pin, regs, otyper, ot, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);

                self.output_type = value;
            }

            /// Set output speed.
            pub fn output_speed(&mut self, value: OutputSpeed, regs: &mut [<GPIO $Port>]) {
                set_field!(self.pin, regs, ospeedr, ospeedr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Set internal pull resistor: Pull up, pull down, or floating.
            pub fn pull(&mut self, value: Pull, regs: &mut [<GPIO $Port>]) {
                set_field!(self.pin, regs, pupdr, pupdr, bits, value as u8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            /// Set the output_data register.
            pub fn output_data(&mut self, value: PinState, regs: &mut [<GPIO $Port>]) {
                set_field!(self.pin, regs, odr, odr, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
            }

            // It appears f373 doesn't have lckr on ports C or E.
            #[cfg(not(feature = "f373"))]
            /// Lock or unlock a port configuration.
            pub fn cfg_lock(&mut self, value: CfgLock, regs: &mut [<GPIO $Port>]) {
                set_field!(self.pin, regs, lckr, lck, bit, value as u8 != 0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
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

            /// Set up a pin's alternate function. We set this up initially using `mode()`.
            fn alt_fn(&mut self, value: AltFn, regs: &mut [<GPIO $Port>]) {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "l5")] {
                        set_alt!(self.pin, regs, afsel, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
                    } else if #[cfg(feature = "h7")] {
                        set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
                    } else {
                        set_alt!(self.pin, regs, afr, value, [(0, l), (1, l), (2, l),
                            (3, l), (4, l), (5, l), (6, l), (7, l), (8, h), (9, h), (10, h), (11, h), (12, h),
                            (13, h), (14, h), (15, h)])
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

            // todo: Look up how you do EXTI on L5 and H7.
            #[cfg(not(any(feature = "f373", feature = "l5", feature = "h7")))]
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

                set_exti!(self.pin, exti, syscfg, rise_trigger, self.port.cr_val(), [(0, 1), (1, 1), (2, 1),
                (3, 1), (4, 2), (5, 2), (6, 2), (7, 2), (8, 3), (9, 3), (10, 3), (11, 3), (12, 4),
                (13, 4), (14, 4), (15, 4)])
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
