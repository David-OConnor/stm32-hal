//! Digital to Analog converter.

// Note that we don't use macros hereto the same extent as with other modules,
// since all families appear to only have a single DAC register block.

use core::fmt;

use crate::{
    gpio::{GpioPin, PinMode},
    pac::{self, RCC},
    traits::SingleChannelDac,
};

use cfg_if::cfg_if;

#[derive(Clone, Copy)]
/// Select the channel
pub enum DacNum {
    One,
    Two,
}

#[derive(Clone, Copy)]
/// Select the channel
pub enum Channel {
    /// Channel 1
    One,
    /// Channel 2
    Two,
} // todo: More channels?

#[derive(Clone, Copy)]
/// Three options are available to set DAC precision.
pub enum Bits {
    /// Eight bit precision, right-aligned.
    EightR,
    /// 12-bit precision, left-aligned.
    TwelveL,
    /// 12-bit precision, right-aligned.
    TwelveR,
}

#[derive(Clone, Copy)]
/// Select a trigger, used by some features.
pub enum Trigger {
    /// Timer 6
    Tim6,
    /// Timers 3 or 8
    Tim3_8,
    /// Timer 7
    Tim7,
    /// Timer 15
    Tim15,
    /// Timer 2
    Tim2,
    /// Timer 4
    Tim4,
    /// Eg, for interrupts
    Exti9,
    /// A software trigger
    Swtrig,
}

impl Trigger {
    pub fn bits(&self) -> u8 {
        match self {
            Self::Tim6 => 0b000,
            Self::Tim3_8 => 0b001,
            Self::Tim7 => 0b010,
            Self::Tim15 => 0b011,
            Self::Tim2 => 0b100,
            Self::Tim4 => 0b101,
            Self::Exti9 => 0b110,
            Self::Swtrig => 0b111,
        }
    }
}

/// Digital to Analog converter peripheral
pub struct Dac<DAC> {
    regs: DAC,
    channel: Channel,
    bits: Bits,
    vref: f32,
}

// note that L5 uses a different register names, hence the verbose macro. ( eg`dac_cr`)
macro_rules! make_impl {
    ($DAC:ident, $cr:ident, $d81:ident, $d12l1:ident, $d12r1:ident, $d82:ident, $d12l2:ident, $d12r2:ident) => {
        // todo: Checked constructor that makes sure the pin is a valid DAC pin configured in analog mode.
        impl Dac<pac::$DAC> {
            /// Create a new DAC instance.
            // pub fn new<P: GpioPin>(regs: DAC, pin: P, channel: Channel, bits: Bits, vref: f32) -> Self {
            //     // todo: Check for a valid pin too.
            //     match pin.get_mode() {
            //         PinMode::Analog => (),
            //         _ => panic!("DAC pin must be configured as analog")
            //     }
            //
            //     Self::new_unchecked(regs, channel, bits, vref)
            // }

            /// Create a new DAC instance, without checking the output pin
            pub fn new_unchecked(
                regs: pac::$DAC,
                num: DacNum,
                channel: Channel,
                bits: Bits,
                vref: f32,
            ) -> Self {
                Self {
                    regs,
                    channel,
                    bits,
                    vref,
                }
            }

            // todo: Mod enable to reflect teh num.
            /// Enable the DAC.
            pub fn enable(&mut self, rcc: &mut RCC) {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        rcc.apb1enr.modify(|_, w| w.dac1en().set_bit());
                    } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                        rcc.apb1enr1.modify(|_, w| w.dac1en().set_bit());
                    } else if #[cfg(feature = "h7")] {
                        rcc.apb1lenr.modify(|_, w| w.dac12en().set_bit());
                    }
                }

                match self.channel {
                    Channel::One => self.regs.$cr.modify(|_, w| w.en1().set_bit()),
                    Channel::Two => self.regs.$cr.modify(|_, w| w.en2().set_bit()),
                }
            }

            /// Disable the DAC
            pub fn disable(&mut self, rcc: &mut RCC) {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        rcc.apb1enr.modify(|_, w| w.dac1en().clear_bit());
                    } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                        rcc.apb1enr1.modify(|_, w| w.dac1en().clear_bit());
                    } else if #[cfg(feature = "h7")] {
                        rcc.apb1lenr.modify(|_, w| w.dac12en().clear_bit());
                    }
                }

                match self.channel {
                    Channel::One => self.regs.$cr.modify(|_, w| w.en1().clear_bit()),
                    Channel::Two => self.regs.$cr.modify(|_, w| w.en2().clear_bit()),
                }
            }

            /// Set the DAC value as an integer.
            pub fn set_value(&mut self, val: u32) {
                match self.channel {
                    Channel::One => match self.bits {
                        Bits::EightR => self.regs.$d81.modify(|_, w| unsafe { w.bits(val) }),
                        Bits::TwelveL => self.regs.$d12l1.modify(|_, w| unsafe { w.bits(val) }),
                        Bits::TwelveR => self.regs.$d12r1.modify(|_, w| unsafe { w.bits(val) }),
                    },
                    Channel::Two => match self.bits {
                        Bits::EightR => self.regs.$d82.modify(|_, w| unsafe { w.bits(val) }),
                        Bits::TwelveL => self.regs.$d12l2.modify(|_, w| unsafe { w.bits(val) }),
                        Bits::TwelveR => self.regs.$d12r2.modify(|_, w| unsafe { w.bits(val) }),
                    },
                }
            }

            /// Set the DAC voltage. `v` is in Volts.
            pub fn set_voltage(&mut self, volts: f32) {
                let val = match self.bits {
                    Bits::EightR => ((volts / self.vref) * 255.) as u32,
                    Bits::TwelveL => ((volts / self.vref) * 4_095.) as u32,
                    Bits::TwelveR => ((volts / self.vref) * 4_095.) as u32,
                };

                self.set_value(val);
            }

            // todo: Trouble finding right `tsel` fields for l5. RM shows same as others. PAC bug?
            // todo Or is the PAC breaking the bits field into multiple bits?
            #[cfg(not(feature = "l5"))]
            /// Select and activate a trigger. See f303 Reference manual, section 16.5.4.
            pub fn set_trigger(&mut self, trigger: Trigger) {
                match self.channel {
                    Channel::One => {
                        self.regs.$cr.modify(|_, w| w.ten1().set_bit());

                        self.regs
                            .$cr
                            .modify(|_, w| unsafe { w.tsel1().bits(trigger.bits()) });
                    }
                    Channel::Two => {
                        self.regs.$cr.modify(|_, w| w.ten2().set_bit());

                        self.regs
                            .$cr
                            .modify(|_, w| unsafe { w.tsel2().bits(trigger.bits()) });
                    }
                }
            }

            #[cfg(not(feature = "l5"))] // See note on `set_trigger`.
            /// Independent trigger with single LFSR generation
            /// See f303 Reference Manual section 16.5.2
            pub fn trigger_lfsr(&mut self, trigger: Trigger, data: u32) {
                // todo: This may not be correct.
                match self.channel {
                    Channel::One => {
                        self.regs.$cr.modify(|_, w| unsafe { w.wave1().bits(0b01) });
                        self.regs.$cr.modify(|_, w| unsafe { w.mamp1().bits(0b01) });
                    }
                    Channel::Two => {
                        self.regs.$cr.modify(|_, w| unsafe { w.wave2().bits(0b01) });
                        self.regs.$cr.modify(|_, w| unsafe { w.mamp2().bits(0b01) });
                    }
                }
                self.set_trigger(trigger);
                self.set_value(data);
            }

            #[cfg(not(feature = "l5"))] // See note on `set_trigger`.
            /// Independent trigger with single triangle generation
            /// See f303 Reference Manual section 16.5.2
            pub fn trigger_triangle(&mut self, trigger: Trigger, data: u32) {
                // todo: This may not be correct.
                match self.channel {
                    Channel::One => {
                        self.regs.$cr.modify(|_, w| unsafe { w.wave1().bits(0b10) });
                        self.regs.$cr.modify(|_, w| unsafe { w.mamp1().bits(0b10) });
                    }
                    Channel::Two => {
                        self.regs.$cr.modify(|_, w| unsafe { w.wave2().bits(0b10) });
                        self.regs.$cr.modify(|_, w| unsafe { w.mamp2().bits(0b10) });
                    }
                }
                self.set_trigger(trigger);
                self.set_value(data);
            }
        }

        pub struct DacError {}

        impl SingleChannelDac<u32> for Dac<pac::$DAC> {
            type Error = DacError;

            /// Set the DAC value as an integer.
            fn try_set_value(&mut self, val: u32) -> Result<(), DacError> {
                self.set_value(val);
                Ok(())
            }
        }
    };
}

#[cfg(feature = "l5")]
make_impl!(
    DAC,
    dac_cr,
    dac_dhr8r2,
    dac_dhr12l2,
    dac_dhr12r2,
    dac_dhr8r2,
    dac_dhr12l2,
    dac_dhr12r2
);

#[cfg(feature = "l4")]
make_impl!(DAC1, cr, dhr8r1, dhr12l1, dhr12r1, dhr8r2, dhr12l2, dhr12r2);

#[cfg(feature = "f3")]
make_impl!(DAC1, cr, dhr8r1, dhr12l1, dhr12r1, dhr8r2, dhr12l2, dhr12r2);

#[cfg(feature = "h7")]
make_impl!(DAC, cr, dhr8r1, dhr12l1, dhr12r1, dhr8r2, dhr12l2, dhr12r2);
