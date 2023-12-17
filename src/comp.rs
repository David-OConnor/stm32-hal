//! Comparator
//!
//! TODO:
//! - Window Mode Configuration (COMP1 and COMP2 have different configs)
//! - Blanking Source Configuration (COMP1 and COMP2 have different configs)
//! - More Inputs For Inverting Input (STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx)
//! - Moving Peripheral into Struct (pac needs to change)
//! - Add Configuration Defaults
//! - Interrupts?

use core::marker::PhantomData;

use cfg_if::cfg_if;
use paste::paste;

use crate::pac;
#[cfg(any(feature = "g473"))]
use crate::pac::comp::{C1CSR, C2CSR, C3CSR, C4CSR, C5CSR, C6CSR, C7CSR};
#[cfg(any(feature = "l4x6"))]
use crate::pac::comp::{COMP1_CSR, COMP2_CSR};
#[cfg(any(feature = "h747cm4", feature = "h747cm7"))]
use crate::pac::comp1::{CFGR1, CFGR2};

// Config enums
/// Comparator power mode
#[cfg(any(feature = "l4"))]
pub enum PowerMode {
    /// High speed/full power (Lowest propagation delay).
    HighSpeed = 0x00000000,
    /// Medium speed/medium power (Medium propagation delay).
    MediumSpeed = 0x00000004,
    /// Low speed/ultra-low power (Highest propagation delay).
    LowSpeed = 0x0000000c,
}

/// Comparator input plus (Non-inverting Input)
#[cfg(any(feature = "g473", feature = "h7"))]
#[derive(Clone, Copy)]
#[repr(u8)]
// STM32G4 reference manual section 24.3.2 table 196
pub enum NonInvertingInput {
    Io1 = 0b0,
    Io2 = 0b1,
}

#[cfg(any(feature = "l4"))]
pub enum NonInvertingInput {
    /// From the first GPIO pin connected to the comparator.
    ///
    /// The GPIO pin used depends on the MCU and comparator used.
    Io1 = 0x00000000,
    /// From the second GPIO pin connected to the comparator.
    ///
    /// The GPIO pin used depends on the MCU and comparator used.
    Io2 = 0x00000080,
    // PA1/PA3 for STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx
    // TODO: Include stm32l471
    #[cfg(any(feature = "stm32l4x1", feature = "stm32l4x2", feature = "stm32l4x3",))]
    /// From the third GPIO pin connected to the comparator.
    ///
    /// The GPIO pin used depends on the MCU and comparator used.
    Io3 = 0x00000100,
}

/// Comparator input minus (Inverted Input)
#[cfg(any(feature = "g473", feature = "h7", feature = "l4"))]
#[derive(Clone, Copy)]
#[repr(u8)]
// STM32G4 reference manual section 24.3.2 table 197
pub enum InvertingInput {
    OneQuarterVref = 0b000,
    OneHalfVref = 0b001,
    ThreeQuarterVref = 0b010,
    Vref = 0b011,
    Dac1 = 0b100,
    Dac2 = 0b101,
    Io1 = 0b110,
    Io2 = 0b111,
}

/// Comparator hysterisis
#[cfg(any(feature = "g473"))]
#[derive(Clone, Copy)]
pub enum Hysterisis {
    None = 0b000,
    TenMilliVolt = 0b001,
    TwentyMilliVolt = 0b010,
    ThirtyMilliVolt = 0b011,
    FourtyMilliVolt = 0b100,
    FiftyMilliVolt = 0b101,
    SixtyMilliVolt = 0b110,
    SeventyMilliVolt = 0b111,
}

#[cfg(any(feature = "l4", feature = "h7"))]
pub enum Hysterisis {
    /// No Hysterisis.
    NoHysterisis = 0b00,
    /// Low Hysterisis.
    LowHysteresis = 0b01,
    /// Medium Hysterisis.
    MediumHysteresis = 0b10,
    /// High Hysterisis.
    HighHysteresis = 0b11,
}

/// Comparator output polarity
///
/// When [OutputPolarity::NotInverted] is used.
/// The comparator output will be high (1) when [NonInvertingInput] has higher
/// voltage than [InvertingInput]. The comparator output will be low (0) when
/// [NonInvertingInput] has lower voltage than [InvertingInput].
///
/// When [OutputPolarity::Inverted] is used.
/// The comparator output will be high (1) when [NonInvertingInput] has lower
/// voltage than [InvertingInput]. The comparator output will be low (0) when
/// [NonInvertingInput] has higher voltage than [InvertingInput].

#[cfg(any(feature = "g473", feature = "h7"))]
#[derive(Clone, Copy)]
pub enum OutputPolarity {
    NotInverted = 0b0,
    Inverted = 0b1,
}

#[cfg(any(feature = "l4"))]
pub enum OutputPolarity {
    /// Comparator output will not be inverted.
    NotInverted = 0x00000000,
    /// Comparator output will be inverted.
    Inverted = 0x00008000,
}

/// Comparator blanking source
pub enum BlankingSource {
    /// No Blanking.
    None = 0x00000000,
    /// TIM1 OC5 as the blanking source.
    Timloc5 = 0x400000,
}

/// Comparator devices avaiable.
#[cfg(any(feature = "g473"))]
pub enum CompDevice {
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
}

// Structs
/// Initial configuration data for the comparator peripheral.

#[cfg(any(feature = "g473"))]
#[derive(Clone, Copy)]
pub struct CompConfig {
    pub inpsel: NonInvertingInput,
    pub inmsel: InvertingInput,
    pub hyst: Hysterisis,
    pub polarity: OutputPolarity,
}

#[cfg(any(feature = "l4"))]
pub struct CompConfig {
    /// Comparator power mode.
    pub pwrmode: PowerMode,
    /// Comparator non-inverting input.
    pub inpsel: NonInvertingInput,
    /// Comparator inverting input.
    pub inmsel: InvertingInput,
    /// Comparator hysterisis.
    pub hyst: Hysterisis,
    /// Comparator output polarity.
    pub polarity: OutputPolarity,
    // Comparator blanking source.
    // pub blanking: BlankingSource,
}

/// Represents an Analog Comparator peripheral.
pub struct Comp<T> {
    phantom: PhantomData<T>,
    /// The lock status of the comparator.
    is_locked: bool,
}

// Macro to implement a comparator using generics
// This will create `new_compX` methods to instantiate a new comparator
// and provide a csr() method to access the register scoped to this comparator
macro_rules! make_comp {
    ($csr_type:ident, $csr_reg:ident, $comp:ident) => {
        impl Comp<$csr_type> {
            paste! {
                pub fn [<new_ $comp>]() -> Self {
                    unsafe {
                        Self {
                            phantom: PhantomData,
                            is_locked: false
                        }
                    }
                }
            }

            // Get a reference to the CSR from the COMP RegisterBlock for this comparator
            pub fn csr(&self) -> &$csr_type {
                #[cfg(any(feature = "g473", feature = "l4"))]
                unsafe {
                    &(*pac::COMP::ptr()).$csr_reg
                }
                #[cfg(any(feature = "h747cm4", feature = "h747cm7"))]
                unsafe {
                    &(*pac::COMP1::ptr()).$csr_reg
                }
            }

            pub fn enable(&self) {
                self.csr().modify(|_, w| w.en().set_bit());
            }

            pub fn disable(&self) {
                self.csr().modify(|_, w| w.en().clear_bit());
            }

            // Sets the inverting input in the CSR
            pub fn set_inverting_input(&self, input: InvertingInput) {
                cfg_if! {
                    if #[cfg(feature = "l4")] {
                        unsafe { self.csr().modify(|_,w| w.inmsel().bits(input as u8)) };
                    } else {
                        self.csr().modify(|_,w| w.inmsel().variant(input as u8));
                    }
                }
            }

            pub fn set_non_inverting_input(&self, input: NonInvertingInput) {
                cfg_if! {
                    if #[cfg(feature = "l4")] {
                        self.csr().modify(|_,w| w.inpsel().bit(input as u8 != 0));
                    } else {
                        self.csr().modify(|_,w| w.inpsel().variant(input as u8 != 0));
                    }
                }
            }

            pub fn set_polarity(&self, polarity: OutputPolarity) {
                cfg_if! {
                    if #[cfg(feature = "l4")] {
                        self.csr().modify(|_,w| w.polarity().bit((polarity as u8) != 0));
                    } else {
                        #[cfg(feature = "g4")]
                        self.csr().modify(|_,w| w.pol().variant((polarity as u8) != 0));

                        #[cfg(feature = "h7")]
                        self.csr().modify(|_,w| w.polarity().variant((polarity as u8) != 0));
                    }
                }
            }

            pub fn set_hysterisis(&self, hyst: Hysterisis) {
                cfg_if! {
                    if #[cfg(feature = "l4")] {
                        self.csr().modify(|_,w| w.hyst().bits((hyst as u8)));
                    } else {
                        self.csr().modify(|_,w| w.hyst().variant((hyst as u8)));
                    }
                }
            }

            pub fn set_blanking_source(&self, source: u8) {
                #[cfg(feature = "g473")]
                self.csr().modify(|_, w| w.blanksel().variant(source));

                #[cfg(feature = "h7")]
                self.csr().modify(|_, w| w.blanking().variant(source));
            }

            /// Locks the comparator.
            ///
            /// This locks the comparator registers making it only read-only.
            ///
            /// **Note:** The lock also applies to the lock bit itself. Therefore,
            /// the comparator register/configuration **cannot** be changed until
            /// a hardware reset.
            pub fn lock(&mut self) {
                self.is_locked = true;
                self.csr().modify(|_, w| w.lock().set_bit());
            }

            /// Gets the output level of the comparator
            ///
            /// The output level depends on the configuration of the comparator.
            /// If the [polarity](CompConfig::polarity) is [NotInverted](OutputPolarity::NotInverted)
            /// - It will output high (1) if the non-inverting input is higher than
            /// the output of inverting input.
            /// - It will output low (0) if the non-inverting input is lower than
            /// the output of the inverting input.
            ///
            /// The oposite will be out inverted if [polarity](CompConfig::polarity) is
            /// [Inverted](OutputPolarity::NotInverted).
            pub fn get_output_level(&self) -> bool {
                cfg_if! {
                    if #[cfg(any(feature = "g4", feature = "l4"))] {
                        self.csr().read().value().bit()
                    }
                    else if #[cfg(feature = "h7")] {
                        // On the H7, the comparator output is in a separate status register
                        let status = unsafe { &(*pac::COMP1::ptr()).sr.read() };

                        // Check which channel this impl is for
                        match "$csr_reg" {
                            "c1csr" => {
                                status.c1val().bit_is_set()
                            }

                            "c2csr" => {
                                status.c2val().bit_is_set()
                            }

                            _ => todo!(),
                        }
                    }
                }
            }
        }
    };
}

cfg_if! {
    if #[cfg(any(feature = "g4"))] {
        make_comp!(C1CSR, c1csr, comp1);
        make_comp!(C2CSR, c2csr, comp2);
        make_comp!(C3CSR, c3csr, comp3);
        make_comp!(C4CSR, c4csr, comp4);
        make_comp!(C5CSR, c5csr, comp5);
        make_comp!(C6CSR, c6csr, comp6);
        make_comp!(C7CSR, c7csr, comp7);
    } else if #[cfg(any(feature = "l4x6"))] {
        make_comp!(COMP1_CSR, comp1_csr, comp1);
        make_comp!(COMP2_CSR, comp2_csr, comp2);
    } else if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
        make_comp!(CFGR1, cfgr1, comp1);
        make_comp!(CFGR2, cfgr2, comp2);
    }
}
