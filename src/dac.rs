//! Support for the digital to Analog converter peripheral.

// Note that we don't use macros hereto the same extent as with other modules,
// since all families appear to only have a single DAC register block. For example,
// the `Dac` struct doesn't accept a trait of its reg block. We may have to
// change this later as we find exceptions.

use core::ops::Deref;

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
};

cfg_if! {
    if #[cfg(any(all(feature = "f3", not(feature = "f302")), all(feature = "l4", not(feature = "l4x6")), feature = "g4"))] {
        use crate::pac::dac1 as dac_p;
    } else {
        use crate::pac::dac as dac_p;
    }
}

// Todo: The 4 MCR register modes.

use cfg_if::cfg_if;

#[derive(Clone, Copy)]
/// Select the DAC to use.
pub enum DacDevice {
    // Note: F3 has up to 2 DACs. G4 has up to 4. L4, L5, G0, and H7(?) have only 1.
    // WB doesn't have a DAC(?), so it doesn't import this module.
    One,
    #[cfg(any(
        feature = "f303",
        feature = "f373",
        feature = "f3x4",
        feature = "f4",
        feature = "g4"
    ))]
    Two,
    #[cfg(feature = "g4")]
    Three,
    #[cfg(feature = "g4")]
    Four,
}

#[derive(Clone, Copy)]
/// Select the channel to output to. Most MCUs only use 2 channels.
pub enum DacChannel {
    C1,
    #[cfg(not(feature = "wl"))] // WL only has one channel.
    C2,
}

#[derive(Clone, Copy)]
/// Three options are available to set DAC precision. Sets the DHR8R1 etc register contents.
pub enum DacBits {
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

/// Represents a Digital to Analog Converter (DAC) peripheral.
pub struct Dac<R> {
    regs: R,
    device: DacDevice,
    bits: DacBits,
    vref: f32,
}

// todo: Calculate the VDDA vref, as you do with onboard ADCs!

impl<R> Dac<R>
where
    R: Deref<Target = dac_p::RegisterBlock>,
{
    /// Create a new DAC instance.
    pub fn new(regs: R, device: DacDevice, bits: DacBits, vref: f32, rcc: &mut RCC) -> Self {
        cfg_if! {
            if #[cfg(all(feature = "h7", not(feature = "h7b3")))] {
                rcc_en_reset!(apb1, dac12, rcc);
            } else if #[cfg(feature = "f3")] {
                match device {
                    DacDevice::One => { rcc_en_reset!(apb1, dac1, rcc); }
                    #[cfg(any(feature = "f303", feature = "f373", feature = "f3x4", feature = "f4", feature = "g4"))]
                    DacDevice::Two => { rcc_en_reset!(apb1, dac2, rcc); }
                };
            } else if #[cfg(feature = "g4")] {
                match device {
                    DacDevice::One => { rcc_en_reset!(ahb2, dac1, rcc); }
                    DacDevice::Two => { rcc_en_reset!(ahb2, dac2, rcc); }
                    DacDevice::Three => { rcc_en_reset!(ahb2, dac3, rcc); }
                    DacDevice::Four => { rcc_en_reset!(ahb2, dac4, rcc); }
                };
            } else if #[cfg(feature = "f4")] {
                // F4 only uses 1 enable, despite having 2 devices. (each with 1 channel)
                rcc_en_reset!(apb1, dac, rcc);
            } else {
                rcc_en_reset!(apb1, dac1, rcc);
            }
        }

        Self {
            regs,
            device,
            bits,
            vref,
        }
    }

    /// Enable the DAC.
    pub fn enable(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.en1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.en2().set_bit(),
        });
    }

    /// Disable the DAC
    pub fn disable(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.en1().clear_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.en2().clear_bit(),
        });
    }

    /// Set the DAC value as an integer.
    pub fn set_value(&mut self, channel: DacChannel, val: u32) {
        // RM: DAC conversion
        // The DAC_DORx cannot be written directly and any data transfer to the DAC channelx must
        // be performed by loading the DAC_DHRx register (write operation to DAC_DHR8Rx,
        // DAC_DHR12Lx, DAC_DHR12Rx, DAC_DHR8RD, DAC_DHR12RD or DAC_DHR12LD).
        // Data stored in the DAC_DHRx register are automatically transferred to the DAC_DORx
        // register after one APB1 clock cycle, if no hardware trigger is selected (TENx bit in DAC_CR
        // register is reset). However, when a hardware trigger is selected (TENx bit in DAC_CR
        // register is set) and a trigger occurs, the transfer is performed three APB1 clock cycles after
        // the trigger signal.
        // When DAC_DORx is loaded with the DAC_DHRx contents, the analog output voltage
        // becomes available after a time tSETTLING that depends on the power supply voltage and the
        // analog output load.

        #[cfg(any(feature = "l5", feature = "g4"))]
        match channel {
            DacChannel::C1 => match self.bits {
                DacBits::EightR => self.regs.dac_dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dac_dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dac_dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match self.bits {
                DacBits::EightR => self.regs.dac_dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dac_dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dac_dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }

        #[cfg(not(any(feature = "l5", feature = "g4")))]
        match channel {
            DacChannel::C1 => match self.bits {
                DacBits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match self.bits {
                DacBits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }
    }

    /// Set the DAC voltage. `v` is in Volts.
    pub fn set_voltage(&mut self, channel: DacChannel, volts: f32) {
        let max_word = match self.bits {
            DacBits::EightR => 255.,
            DacBits::TwelveL => 4_095.,
            DacBits::TwelveR => 4_095.,
        };

        let val = ((volts / self.vref) * max_word) as u32;
        self.set_value(channel, val);
    }

    // todo: Trouble finding right `tsel` fields for l5 and WL. RM shows same as others. PAC bug?
    // todo Or is the PAC breaking the bits field into multiple bits?
    #[cfg(not(any(feature = "l5", feature = "wl")))]
    /// Select and activate a trigger. See f303 Reference manual, section 16.5.4.
    pub fn set_trigger(&mut self, channel: DacChannel, trigger: Trigger) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.ten1().set_bit();
                    w.tsel1().bits(trigger.bits())
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.ten2().set_bit();
                    w.tsel2().bits(trigger.bits())
                });
            }
        }
    }

    #[cfg(not(any(feature = "l5", feature = "wl")))] // See note on `set_trigger`.
    /// Independent trigger with single LFSR generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_lfsr(&mut self, channel: DacChannel, trigger: Trigger, data: u32) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        // todo: This may not be correct.
        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.mamp1().bits(0b01);
                    w.wave1().bits(0b01)
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.wave2().bits(0b01);
                    w.mamp2().bits(0b01)
                });
            }
        }
        self.set_trigger(channel, trigger);
        self.set_value(channel, data);
    }

    #[cfg(not(any(feature = "l5", feature = "wl")))] // See note on `set_trigger`.
    /// Independent trigger with single triangle generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_triangle(&mut self, channel: DacChannel, trigger: Trigger, data: u32) {
        // todo: This may not be correct.
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.wave1().bits(0b10);
                    w.mamp1().bits(0b10)
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.wave2().bits(0b10);
                    w.mamp2().bits(0b10)
                });
            }
        }
        self.set_trigger(channel, trigger);
        self.set_value(channel, data);
    }

    /// Enable the DMA Underrun interrupt - the only interrupt available.
    pub fn enable_interrupt(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.dmaudrie1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.dmaudrie2().set_bit(),
        });
    }

    #[cfg(not(any(feature = "l5", feature = "g4")))] // todo: PAC ommission? SR missing on L5/G4? In RM.
    /// Clear the DMA Underrun interrupt - the only interrupt available.
    pub fn clear_interrupt(&mut self, channel: DacChannel) {
        self.regs.sr.modify(|_, w| match channel {
            DacChannel::C1 => w.dmaudr1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.dmaudr2().set_bit(),
        });
    }
}
