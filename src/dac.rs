//! Configure the internal DAC on the stm32f3xx.
//! Incomplete, but includes basic operation.

use core::fmt;

use crate::pac::{DAC1, RCC};

/// Trait representing a single-channel digital-to-analog converter (DAC).
pub trait SingleChannelDac<Word> {
    /// Error type returned by DAC methods
    type Error;

    /// Output a constant signal, given a bit word.
    fn try_set_value(&mut self, value: Word) -> Result<(), Self::Error>;
}

// /// This is an abstraction to ensure that the DAC output pin is configured
// /// as an analog output.
// pub trait Pins {}
// impl Pins for PA4<Analog> {}
// impl Pins for PA5<Analog> {}

#[derive(Clone, Copy, Debug)]
/// Select the channel
pub enum Channel {
    /// Channel 1
    One,
    /// Channel 2
    Two,
}

#[derive(Clone, Copy, Debug)]
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

pub struct Dac {
    regs: DAC1,
    channel: Channel,
    bits: Bits,
    vref: f32,
}

impl Dac {
    /// Create a new DAC instance
    pub fn new<P>(regs: DAC1, _pins: P, channel: Channel, bits: Bits, vref: f32) -> Self {
        Self {
            regs,
            channel,
            bits,
            vref,
        }
    }

    /// Enable the DAC.
    pub fn enable(&mut self, rcc: &mut RCC) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "f3")] {
                rcc.apb1enr.modify(|_, w| w.dac1en().set_bit());
            } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                rcc.apb1enr1.modify(|_, w| w.dac1en().set_bit());
            }
        }

        match self.channel {
            Channel::One => self.regs.cr.modify(|_, w| w.en1().set_bit()),
            Channel::Two => self.regs.cr.modify(|_, w| w.en2().set_bit()),
        }
    }

    /// Disable the DAC
    pub fn disable(&mut self, rcc: &mut RCC) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "f3")] {
                rcc.apb1enr.modify(|_, w| w.dac1en().clear_bit());
            } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                rcc.apb1enr1.modify(|_, w| w.dac1en().clear_bit());
            }
        }

        match self.channel {
            Channel::One => self.regs.cr.modify(|_, w| w.en1().clear_bit()),
            Channel::Two => self.regs.cr.modify(|_, w| w.en2().clear_bit()),
        }
    }

    /// Set the DAC value as an integer.
    pub fn set_value(&mut self, val: u32) {
        match self.channel {
            Channel::One => match self.bits {
                Bits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                Bits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                Bits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            Channel::Two => match self.bits {
                Bits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                Bits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                Bits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
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

    // Select and activate a trigger. See f303 Reference manual, section 16.5.4.
    pub fn set_trigger(&mut self, trigger: Trigger) {
        match self.channel {
            Channel::One => {
                self.regs.cr.modify(|_, w| w.ten1().set_bit());
                self.regs
                    .cr
                    .modify(|_, w| unsafe { w.tsel1().bits(trigger.bits()) });
            }
            Channel::Two => {
                self.regs.cr.modify(|_, w| w.ten2().set_bit());
                self.regs
                    .cr
                    .modify(|_, w| unsafe { w.tsel2().bits(trigger.bits()) });
            }
        }
    }

    /// Independent trigger with single LFSR generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_lfsr(&mut self, trigger: Trigger, data: u32) {
        // todo: This may not be correct.
        match self.channel {
            Channel::One => {
                self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b01) });
                self.regs.cr.modify(|_, w| unsafe { w.mamp1().bits(0b01) });
            }
            Channel::Two => {
                self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b01) });
                self.regs.cr.modify(|_, w| unsafe { w.mamp2().bits(0b01) });
            }
        }
        self.set_trigger(trigger);
        self.set_value(data);
    }

    /// Independent trigger with single triangle generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_triangle(&mut self, trigger: Trigger, data: u32) {
        // todo: This may not be correct.
        match self.channel {
            Channel::One => {
                self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b10) });
                self.regs.cr.modify(|_, w| unsafe { w.mamp1().bits(0b10) });
            }
            Channel::Two => {
                self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b10) });
                self.regs.cr.modify(|_, w| unsafe { w.mamp2().bits(0b10) });
            }
        }
        self.set_trigger(trigger);
        self.set_value(data);
    }
}

impl fmt::Debug for Dac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Dac")
            .field("channel", &self.channel)
            .field("bits", &self.bits)
            .field("vret", &self.vref)
            .finish()
    }
}

pub struct DacError {}

impl SingleChannelDac<u32> for Dac {
    type Error = DacError;

    /// Set the DAC value as an integer.
    fn try_set_value(&mut self, val: u32) -> Result<(), DacError> {
        self.set_value(val);
        Ok(())
    }
}
