//! Serial audio interface support. Used for I2S, PCM/DSP, TDM, AC'97 etc.
//! See L443 Reference Manual, section 41.

// todo: WIP

use core::ops::Deref;

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

use cfg_if::cfg_if;

#[derive(Clone, Copy)]
/// Specify the SAI device to use. Used internally for setting the appropriate APB.
pub enum SaiDevice {
    One,
    Two,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select Master or Slave mode.
pub enum SaiMode {
    MasterTransmitter = 0b00,
    MasterReceiver = 0b01,
    SlaveTransmitter = 0b10,
    SlaveReceiver = 0b11
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select Stereo or Mono mode
pub enum Mono {
    Stereo = 0,
    Mono = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Specify wheather sub-clocks A and B are synchronized.
pub enum Sync {
    /// Audio sub-block in asynchronous mode
    Async = 0b00,
    /// Audio sub-block is synchronous with the other internal audio sub-block. In this case, the audio
    /// sub-block must be configured in slave mode
    Sync = 0b01
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select the audio protocol to use
pub enum Protocol {
    ///  Free protocol. Free protocol allows to use the powerful configuration of the audio block to
    /// address a specific audio protocol (such as I2S, LSB/MSB justified, TDM, PCM/DSP...) by setting
    /// most of the configuration register bits as well as frame configuration register.
    Free = 0b00,
    Spdif = 0b01,
    Ac97 = 0b10,
}

#[derive(Clone, Copy)]
/// The type of SAI interrupt to configure. Reference Section 41.5 of the L4 RM.
pub enum SaiInterrupt {
    Freq,
    Ovrudr,
    AfsDet,
    LfsDet,
    CnRdy,
    MuteDet,
    WckCfg,
}

#[derive(Clone, Copy)]
pub enum Channel {
    // todo: Is this the right name?
    A,
    B,
}

pub struct SaiConfig {
    mode: SaiMode,
    protocol: Protocol,
    mono: Mono,
    sync: Sync,
}

impl Default for SaiConfig {
    fn default() -> Self {
        Self {
            mode: SaiMode::MasterTransmitter,
            protocol: Protocol::Free,
            mono: Mono::Stereo,
            sync: Sync::Async,
        }
    }
}

/// Represents the USART peripheral, for serial communications.
pub struct Sai<S> {
    regs: S,
    pub config_a: SaiConfig,
    pub config_b: SaiConfig,
}

impl<S> Sai<S>
    where
        S: Deref<Target = pac::sai1::RegisterBlock>,
{
    pub fn new<C: ClockCfg>(
        regs: S,
        device: SaiDevice,
        config_a: SaiConfig,
        config_b: SaiConfig,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        // todo: Hard set to usart 1 to get started
        match device {
            SaiDevice::One => {
                rcc_en_reset!(apb2, sai1, rcc);
            }
            SaiDevice::Two => {
                rcc_en_reset!(apb2, sai2, rcc);
            }
        }

        regs.acr1().modify(|_, w| w.mode().bits(config_a.mode as u8));
        regs.bcr1().modify(|_, w| w.mode().bits(config_b.mode as u8));

        regs.acr1().modify(|_, w| w.prtcfg().bits(config_a.protocol as u8));
        regs.bcr1().modify(|_, w| w.prtcfg().bits(config_b.protocol as u8));

        regs.acr1().modify(|_, w| w.mono().bit(config_a.mono as u8 != 0));
        regs.bcr1().modify(|_, w| w.mono().bit(config_b.mono as u8 != 0));

        regs.acr1().modify(|_, w| w.syncen().bits(config_a.sync as u8));
        regs.bcr1().modify(|_, w| w.syncen().bits(config_b.sync as u8));

        // todo: Do we always want to configure and enable both A and B?
        regs.acr1().modify(|_, w| w.saien().set_bit());
        regs.bcr1().modify(|_, w| w.saien().set_bit());


        Self { regs, config_a, config_b}
    }

    /// Enable a specific type of interrupt. See L4 RM, Table 220: "SAI interrupt sources".
    pub fn enable_interrupt(&mut self, interrupt_type: SaiInterrupt, channel: Channel) {
        // Disable the UART to allow writing the `add` and `addm7` bits
        // L4 RM: Follow the sequence below to enable an interrupt:
        // 1. Disable SAI interrupt.
        // 2. Configure SAI.
        // 3. Configure SAI interrupt source.
        // 4. Enable SAI.

        //todo: Does that mean we need to disable and re-enable SAI here?

        match channel {
            Channel::A => {
                match interrupt_type {
                    SaiInterrupt::Freq => self.regs.aim().freqie().set_bit(),
                    SaiInterrupt::Ovrudr => self.regs.aim().ovrudrie().set_bit(),
                    SaiInterrupt::AfsDet => self.regs.aim().afsdetie().set_bit(),
                    SaiInterrupt::LfsDet => self.regs.aim().lfsdetie().set_bit(),
                    SaiInterrupt::CnRdy => self.regs.aim().cnrdyie().set_bit(),
                    SaiInterrupt::MuteDet => self.regs.aim().mutedetie().set_bit(),
                    SaiInterrupt::WckCfg => self.regs.aim().wckcfgie().set_bit(),
                }
            }
            Channel::B => {
                // todo DRY
                match interrupt_type {
                    SaiInterrupt::Freq => self.regs.bim().freqie().set_bit(),
                    SaiInterrupt::Ovrudr => self.regs.bim().ovrudrie().set_bit(),
                    SaiInterrupt::AfsDet => self.regs.bim().afsdetie().set_bit(),
                    SaiInterrupt::LfsDet => self.regs.bim().lfsdetie().set_bit(),
                    SaiInterrupt::CnRdy => self.regs.bim().cnrdyie().set_bit(),
                    SaiInterrupt::MuteDet => self.regs.bim().mutedetie().set_bit(),
                    SaiInterrupt::WckCfg => self.regs.bim().wckcfgie().set_bit(),
                }
            }
        }
    }

    /// Clears the interrupt pending flag for a specific type of interrupt.
    pub fn clear_interrupt(&mut self, interrupt_type: UsartInterrupt, channel: Channel) {
        match interrupt_type {
            SaiInterrupt::Freq => {},
            SaiInterrupt::Ovrudr => {},
            SaiInterrupt::AfsDet => {},
            SaiInterrupt::LfsDet => {},
            SaiInterrupt::CnRdy => {},
            SaiInterrupt::MuteDet => {},
            SaiInterrupt::WckCfg => {},
        }
    }


}