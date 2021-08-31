//! Serial audio interface support. Used for I2S, PCM/DSP, TDM, AC'97 etc.
//! See L443 Reference Manual, section 41. H743 FM, section 51.
//!
//! For now, only supports a limited set of I2S features.

// todo: WIP

use core::ops::Deref;

use crate::{clocks::Clocks, pac::RCC, rcc_en_reset};

#[cfg(not(feature = "h7"))]
use crate::pac::sai1 as sai;
#[cfg(feature = "h7")]
use crate::pac::sai4 as sai;

use cfg_if::cfg_if;

#[derive(Clone, Copy)]
/// Specify the SAI device to use. Used internally for setting the appropriate APB.
pub enum SaiDevice {
    One,
    #[cfg(feature = "h7")]
    Two,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select Master or Slave mode.
pub enum SaiMode {
    MasterTransmitter = 0b00,
    MasterReceiver = 0b01,
    SlaveTransmitter = 0b10,
    SlaveReceiver = 0b11,
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
    Sync = 0b01,
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
pub struct Sai<R> {
    regs: R,
    pub config_a: SaiConfig,
    pub config_b: SaiConfig,
}

impl<R> Sai<R>
where
    R: Deref<Target = sai::RegisterBlock>,
{
    pub fn new(
        regs: R,
        device: SaiDevice,
        config_a: SaiConfig,
        config_b: SaiConfig,
        clocks: &Clocks,
        rcc: &mut RCC,
    ) -> Self {
        // todo: Hard set to usart 1 to get started
        match device {
            SaiDevice::One => {
                rcc_en_reset!(apb2, sai1, rcc);
            }
            // todo: What other MCUs support what SAI#s?
            #[cfg(feature = "h7")]
            SaiDevice::Two => {
                rcc_en_reset!(apb2, sai2, rcc);
            } // todo: More devices for H7
        }

        // todo: Do we always want to configure and enable both A and B? Probably not!

        regs.cha.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_a.mode as u8);
            w.prtcfg().bits(config_a.protocol as u8);
            w.mono().bit(config_a.mono as u8 != 0);
            w.syncen().bits(config_a.sync as u8);
            w.saien().set_bit()
        });

        regs.chb.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_b.mode as u8);
            w.prtcfg().bits(config_b.protocol as u8);
            w.mono().bit(config_b.mono as u8 != 0);
            w.syncen().bits(config_b.sync as u8);
            w.saien().set_bit()
        });

        Self {
            regs,
            config_a,
            config_b,
        }
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
                self.regs.cha.im.modify(|_, w| match interrupt_type {
                    SaiInterrupt::Freq => w.freqie().set_bit(),
                    SaiInterrupt::Ovrudr => w.ovrudrie().set_bit(),
                    SaiInterrupt::AfsDet => w.afsdetie().set_bit(),
                    SaiInterrupt::LfsDet => w.lfsdetie().set_bit(),
                    SaiInterrupt::CnRdy => w.cnrdyie().set_bit(),
                    SaiInterrupt::MuteDet => w.mutedetie().set_bit(),
                    SaiInterrupt::WckCfg => w.wckcfgie().set_bit(),
                });
            }
            Channel::B => {
                // todo DRY
                self.regs.chb.im.modify(|_, w| match interrupt_type {
                    SaiInterrupt::Freq => w.freqie().set_bit(),
                    SaiInterrupt::Ovrudr => w.ovrudrie().set_bit(),
                    SaiInterrupt::AfsDet => w.afsdetie().set_bit(),
                    SaiInterrupt::LfsDet => w.lfsdetie().set_bit(),
                    SaiInterrupt::CnRdy => w.cnrdyie().set_bit(),
                    SaiInterrupt::MuteDet => w.mutedetie().set_bit(),
                    SaiInterrupt::WckCfg => w.wckcfgie().set_bit(),
                });
            }
        }
    }

    /// Clears the interrupt pending flag for a specific type of interrupt.
    pub fn clear_interrupt(&mut self, interrupt_type: SaiInterrupt, channel: Channel) {
        // todo
        match interrupt_type {
            SaiInterrupt::Freq => {}
            SaiInterrupt::Ovrudr => {}
            SaiInterrupt::AfsDet => {}
            SaiInterrupt::LfsDet => {}
            SaiInterrupt::CnRdy => {}
            SaiInterrupt::MuteDet => {}
            SaiInterrupt::WckCfg => {}
        }
    }
}
