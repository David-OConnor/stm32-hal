//! Serial audio interface support. Used for I2S, PCM/DSP, TDM, AC'97 etc.
//! See L443 Reference Manual, section 41. H743 FM, section 51.
//!
//! For now, only supports a limited set of I2S features.

// todo: WIP

use core::ops::Deref;

use cortex_m::interrupt::free;

use crate::{clocks::Clocks, pac::RCC, rcc_en_reset};

#[cfg(not(feature = "h7"))]
use crate::pac::sai1 as sai;
#[cfg(feature = "h7")]
use crate::pac::sai4 as sai;

use cfg_if::cfg_if;

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(
    feature = "f3",
    feature = "l4",
    feature = "g4",
    feature = "h7",
    feature = "wb"
))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "f4", feature = "l5")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

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
pub enum SyncMode {
    /// Audio sub-block in asynchronous mode
    Async = 0b00,
    /// Audio sub-block is synchronous with the other internal audio sub-block. In this case, the audio
    /// sub-block must be configured in slave mode
    Sync = 0b01,
    /// Audio subblock is synchronous with an external SAI embedded peripheral. In this case the audio
    /// subblock should be configured in Slave mode.
    SyncExternal = 0b10, // todo: May only be valid for some MCUs, eg ones with multiple SAI devices.
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select the audio protocol to use. xCR1 register, PRTCFG field.
pub enum Protocol {
    /// Free protocol. Free protocol allows to use the powerful configuration of the audio block to
    /// address a specific audio protocol (such as I2S, LSB/MSB justified, TDM, PCM/DSP...) by setting
    /// most of the configuration register bits as well as frame configuration register.
    Free = 0b00,
    /// SPDIF protocol
    Spdif = 0b01,
    /// AC'97 protocol
    Ac97 = 0b10,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select the data size to use. xCR1 register, DS field.
pub enum DataSize {
    /// 8 bits
    S8 = 0b010,
    /// 10 bits
    S10 = 0b011,
    /// 16 bits
    S16 = 0b100,
    /// 20 bits
    S20 = 0b101,
    /// 24 bits
    S24 = 0b110,
    /// 32 bits
    S32 = 0b111,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select wheather the master clock is generated. xDR1 register, NOMCK field.
pub enum MasterClock {
    Used = 0,
    NotUsed = 1,
}

#[derive(Clone, Copy)]
/// The type of SAI interrupt to configure. Reference Section 41.5 of the L4 RM.
/// Enabled in xIM register, yIE fields. See H743 RM, section 51.5: SAI interrupts.
pub enum SaiInterrupt {
    Freq,
    ///When the audio block is configured as receiver, an overrun condition may appear if data are
    /// received in an audio frame when the FIFO is full and not able to store the received data. In
    /// this case, the received data are lost, the flag OVRUDR in the SAI_xSR register is set and an
    /// interrupt is generated if OVRUDRIE bit is set in the SAI_xIM register.
    ///
    /// An underrun may occur when the audio block in the SAI is a transmitter and the FIFO is
    /// empty when data need to be transmitted. If an underrun is detected, the slot number for
    /// which the event occurs is stored and MUTE value (00) is sent until the FIFO is ready to
    /// transmit the data corresponding to the slot for which the underrun was detected (refer to
    /// Figure 664). This avoids desynchronization between the memory pointer and the slot in the
    /// audio frame.
    Ovrudr,
    /// The AFSDET flag is used only in slave mode. It is never asserted in master mode. It
    /// indicates that a frame synchronization (FS) has been detected earlier than expected since
    /// the frame length, the frame polarity, the frame offset are defined and known.
    AfsDet,
    /// The LFSDET flag in the SAI_xSR register can be set only when the SAI audio block
    /// operates as a slave. The frame length, the frame polarity and the frame offset configuration
    /// are known in register SAI_xFRCR.
    LfsDet,
    /// The CNRDY flag in the SAI_xSR register is relevant only if the SAI audio block is configured
    /// to operate in AC’97 mode (PRTCFG[1:0] = 10 in the SAI_xCR1 register). If CNRDYIE bit is
    /// set in the SAI_xIM register, an interrupt is generated when the CNRDY flag is set.
    /// CNRDY is asserted when the Codec is not ready to communicate during the reception of
    /// the TAG 0 (slot0) of the AC’97 audio frame.
    CnRdy,
    /// Mute detection
    MuteDet,
    /// When the audio block operates as a master (MODE[1] = 0) and NOMCK bit is equal to 0,
    /// the WCKCFG flag is set as soon as the SAI is enabled if the following conditions are met:
    /// • (FRL+1) is not a power of 2, and
    /// • (FRL+1) is not between 8 and 256.
    /// MODE, NOMCK, and SAIEN bits belong to SAI_xCR1 register and FRL to SAI_xFRCR
    /// register.
    WckCfg,
}

#[derive(Clone, Copy)]
pub enum SaiChannel {
    A,
    B,
}

pub struct SaiConfig {
    pub mode: SaiMode,
    pub protocol: Protocol,
    pub mono: Mono,
    /// An audio subblock can be configured to operate synchronously with the second audio
    /// subblock in the same SAI. In this case, the bit clock and the frame synchronization signals
    /// are shared to reduce the number of external pins used for the communication.
    pub sync: SyncMode,
    pub datasize: DataSize,
    pub frame_length: u8,
    pub master_clock: MasterClock,
}

impl Default for SaiConfig {
    fn default() -> Self {
        Self {
            mode: SaiMode::MasterTransmitter,
            protocol: Protocol::Free,
            mono: Mono::Stereo,
            sync: SyncMode::Async,
            datasize: DataSize::S32,
            frame_length: 10,
            master_clock: MasterClock::Used, // todo?
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
    ) -> Self {
        free(|cs| {
            let rcc = unsafe { &(*RCC::ptr()) };

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
        });

        // todo: Do we always want to configure and enable both A and B?

        // todo: Sort out mken/nomck. PAC issues?

        // For info on modes, reference H743 RM, section 51.4.3: "Configuring and
        // Enabling SAI modes".
        regs.cha.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_a.mode as u8);
            w.prtcfg().bits(config_a.protocol as u8);
            w.mono().bit(config_a.mono as u8 != 0);
            w.syncen().bits(config_a.sync as u8);
            // The NOMCK bit of the SAI_xCR1 register is used to define whether the master clock is
            // generated or not.
            // #[cfg(not(feature = "h7"))]
            // w.nomck().bits(config_a.master_clock as u8 != 0);
            #[cfg(feature = "h7")]
            w.mcken().bit(config_a.master_clock as u8 == 0);
            // The audio frame can target different data sizes by configuring bit DS[2:0] in the SAI_xCR1
            // register. The data sizes may be 8, 10, 16, 20, 24 or 32 bits. During the transfer, either the
            // MSB or the LSB of the data are sent first, depending on the configuration of bit LSBFIRST in
            // the SAI_xCR1 register.
            w.ds().bits(config_a.datasize as u8)
        });
        // todo: MCKEN vice NOMCK?? Make sure your enum reflects how you handle it.

        regs.chb.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_b.mode as u8);
            w.prtcfg().bits(config_b.protocol as u8);
            w.mono().bit(config_b.mono as u8 != 0);
            w.syncen().bits(config_b.sync as u8);
            // #[cfg(not(feature = "h7"))]
            // w.nomck().bits(config_b.master_clock as u8 != 0);
            #[cfg(feature = "h7")]
            w.mcken().bit(config_b.master_clock as u8 == 0);
            w.ds().bits(config_b.datasize as u8)
        });

        // The audio frame length can be configured to up to 256 bit clock cycles, by setting
        // FRL[7:0] field in the SAI_xFRCR register.
        regs.cha
            .frcr
            .modify(|_, w| unsafe { w.frl().bits(config_a.frame_length) });

        regs.chb
            .frcr
            .modify(|_, w| unsafe { w.frl().bits(config_b.frame_length) });

        // todo: Slot configuration (NBSLOT) ? xSLOTR?

        Self {
            regs,
            config_a,
            config_b,
        }
    }

    /// Enable an audio subblock (channel).
    pub fn enable(&mut self, channel: SaiChannel) {
        match channel {
            SaiChannel::A => self.regs.cha.cr1.modify(|_, w| w.saien().set_bit()),
            SaiChannel::B => self.regs.chb.cr1.modify(|_, w| w.saien().set_bit()),
        }
    }

    /// Disable an audio subblock (channel). See H743 RM, section 51.4.15.
    /// The SAI audio block can be disabled at any moment by clearing SAIEN bit in the SAI_xCR1
    /// register. All the already started frames are automatically completed before the SAI is stops
    /// working. SAIEN bit remains High until the SAI is completely switched-off at the end of the
    /// current audio frame transfer.
    /// If an audio block in the SAI operates synchronously with the other one, the one which is the
    /// master must be disabled first.
    pub fn disable(&mut self, channel: SaiChannel) {
        match channel {
            SaiChannel::A => self.regs.cha.cr1.modify(|_, w| w.saien().clear_bit()),
            SaiChannel::B => self.regs.chb.cr1.modify(|_, w| w.saien().clear_bit()),
        }
    }

    /// Read 2 words of data from a channel.
    /// A read from the SR register empties the FIFO if the FIFO is not empty
    pub fn read(&self, channel: SaiChannel) -> (u32, u32) {
        // A read from this register empties the FIFO if the FIFO is not empty
        match channel {
            SaiChannel::A => (
                self.regs.cha.dr.read().bits(),
                self.regs.cha.dr.read().bits(),
            ),
            SaiChannel::B => (
                self.regs.chb.dr.read().bits(),
                self.regs.chb.dr.read().bits(),
            ),
        }

        // todo: Check FIFO level?
        //
        // match audio_ch.sr.read().flvl().variant() {
        //     Val(sr::FLVL_A::EMPTY) => Err(nb::Error::WouldBlock),
        //     _ => Ok((audio_ch.dr.read().bits(), audio_ch.dr.read().bits())),
        // }
    }

    /// Send 2 words of data to a single channel.
    /// A write to the SR register loads the FIFO provided the FIFO is not full.
    pub fn write(&mut self, channel: SaiChannel, left_word: u32, right_word: u32) {
        match channel {
            SaiChannel::A => self
                .regs
                .cha
                .dr
                .write(|w| unsafe { w.bits(left_word).bits(right_word) }),
            SaiChannel::B => self
                .regs
                .chb
                .dr
                .write(|w| unsafe { w.bits(left_word).bits(right_word) }),
        }

        // todo: Why 2 words?
        // todo: Check FIFO level?

        // The FIFO is 8 words long. A write consists of 2 words, in stereo mode.
        // Therefore you need to wait for 3/4s to ensure 2 words are available for writing.
        // match audio_ch.sr.read().flvl().variant() {
        //     Val(sr::FLVL_A::FULL) => Err(nb::Error::WouldBlock),
        //     Val(sr::FLVL_A::QUARTER4) => Err(nb::Error::WouldBlock),
        //     _ => {
        //         unsafe {
        //             audio_ch.dr.write(|w| w.bits(left_word).bits(right_word));
        //         }
        //         Ok(())
        //     }
        // }
    }

    /// Send data over SAI with DMA. H743 RM, section 51.4.16: SAI DMA Interface.
    /// To free the CPU and to optimize bus bandwidth, each SAI audio block has an independent
    /// DMA interface to read/write from/to the SAI_xDR register (to access the internal FIFO).
    /// There is one DMA channel per audio subblock supporting basic DMA request/acknowledge
    /// protocol.
    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    pub unsafe fn write_dma<D>(
        &mut self,
        buf: &[u32], // todo size?
        sai_channel: SaiChannel,
        dma_channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_ptr(), buf.len());

        // todo: Impl these non-DMAMUx features.
        // // L44 RM, Table 41. "DMA1 requests for each channel
        // // todo: DMA2 support.
        // #[cfg(any(feature = "f3", feature = "l4"))]
        //     let channel = match self.device {
        //     AdcDevice::One => DmaInput::Adc1.dma1_channel(),
        //     AdcDevice::Two => DmaInput::Adc2.dma1_channel(),
        //     _ => panic!("DMA on ADC beyond 2 is not supported. If it is for your MCU, please submit an issue \
        //         or PR on Github.")
        // };
        //
        // #[cfg(feature = "l4")]
        // match self.device {
        //     AdcDevice::One => dma.channel_select(DmaInput::Adc1),
        //     AdcDevice::Two => dma.channel_select(DmaInput::Adc2),
        //     _ => unimplemented!(),
        // }

        // To configure the audio subblock for DMA transfer, set DMAEN bit in the SAI_xCR1 register.
        // The DMA request is managed directly by the FIFO controller depending on the FIFO
        // threshold level (for more details refer to Section 51.4.9: Internal FIFOs). DMA transfer
        // direction is linked to the SAI audio subblock configuration:
        // • If the audio block operates as a transmitter, the audio block FIFO controller outputs a
        // DMA request to load the FIFO with data written in the SAI_xDR register.
        // • If the audio block is operates as a receiver, the DMA request is related to read
        // operations from the SAI_xDR register.
        match sai_channel {
            SaiChannel::A => self.regs.cha.cr1.modify(|_, w| w.dmaen().set_bit()),
            SaiChannel::B => self.regs.chb.cr1.modify(|_, w| w.dmaen().set_bit()),
        }

        // Follow the sequence below to configure the SAI interface in DMA mode:
        // 1. Configure SAI and FIFO threshold levels to specify when the DMA request will be
        // launched.
        // todo!
        // 2. Configure SAI DMA channel. (handled by `dma.cfg_channel`)
        // 3. Enable the DMA. (handled by `dma.cfg_channel`)

        let periph_addr = match sai_channel {
            SaiChannel::A => &self.regs.cha.dr as *const _ as u32,
            SaiChannel::B => &self.regs.chb.dr as *const _ as u32,
        };

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        dma.cfg_channel(
            dma_channel,
            periph_addr,
            ptr as u32,
            len,
            dma::Direction::ReadFromMem,
            dma::DataSize::S32, // todo?
            dma::DataSize::S32, // todo?
            channel_cfg,
        );

        // 4. Enable the SAI interface. (handled by `Sai::enable() in user code`.)
    }

    /// Read data from SAI with DMA. H743 RM, section 51.4.16: SAI DMA Interface.
    /// To free the CPU and to optimize bus bandwidth, each SAI audio block has an independent
    /// DMA interface to read/write from/to the SAI_xDR register (to access the internal FIFO).
    /// There is one DMA channel per audio subblock supporting basic DMA request/acknowledge
    /// protocol.
    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    pub unsafe fn read_dma<D>(
        &mut self,
        buf: &mut [u32], // todo size?
        sai_channel: SaiChannel,
        dma_channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        // See commends on `write_dma`.

        // todo: Impl these non-DMAMUx features.
        // // L44 RM, Table 41. "DMA1 requests for each channel
        // // todo: DMA2 support.
        // #[cfg(any(feature = "f3", feature = "l4"))]
        //     let channel = match self.device {
        //     AdcDevice::One => DmaInput::Adc1.dma1_channel(),
        //     AdcDevice::Two => DmaInput::Adc2.dma1_channel(),
        //     _ => panic!("DMA on ADC beyond 2 is not supported. If it is for your MCU, please submit an issue \
        //         or PR on Github.")
        // };
        //
        // #[cfg(feature = "l4")]
        // match self.device {
        //     AdcDevice::One => dma.channel_select(DmaInput::Adc1),
        //     AdcDevice::Two => dma.channel_select(DmaInput::Adc2),
        //     _ => unimplemented!(),
        // }

        match sai_channel {
            SaiChannel::A => self.regs.cha.cr1.modify(|_, w| w.dmaen().set_bit()),
            SaiChannel::B => self.regs.chb.cr1.modify(|_, w| w.dmaen().set_bit()),
        }

        let periph_addr = match sai_channel {
            SaiChannel::A => &self.regs.cha.dr as *const _ as u32,
            SaiChannel::B => &self.regs.chb.dr as *const _ as u32,
        };

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        dma.cfg_channel(
            dma_channel,
            periph_addr,
            ptr as u32,
            len,
            dma::Direction::ReadFromPeriph,
            dma::DataSize::S32, // todo?
            dma::DataSize::S32, // todo?
            channel_cfg,
        );

        // 4. Enable the SAI interface. (handled by `Sai::enable() in user code`.)
    }

    /// Enable a specific type of interrupt. See L4 RM, Table 220: "SAI interrupt sources".
    pub fn enable_interrupt(&mut self, interrupt_type: SaiInterrupt, channel: SaiChannel) {
        // Disable the UART to allow writing the `add` and `addm7` bits
        // L4 RM: Follow the sequence below to enable an interrupt:
        // 1. Disable SAI interrupt.
        // 2. Configure SAI.
        // 3. Configure SAI interrupt source.
        // 4. Enable SAI.

        //todo: Does that mean we need to disable and re-enable SAI here?

        match channel {
            SaiChannel::A => {
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
            SaiChannel::B => {
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
    pub fn clear_interrupt(&mut self, interrupt_type: SaiInterrupt, channel: SaiChannel) {
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
