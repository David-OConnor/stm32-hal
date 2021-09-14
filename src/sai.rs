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

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

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
/// Set which bit is transmitted first: Least significant, or Most significant. You may have to
/// choose the one used by your SAI device. Sets xCR1 register, FSOFF field.
/// This bit is set and cleared by software. It is meaningless and is not used in AC’97 or SPDIF audio
/// block configuration. This bit must be configured when the audio block is disabled.
pub enum FsOffset {
    /// FS is asserted on the first bit of the slot 0.
    FirstBit = 0,
    /// FS is asserted one bit before the first bit of the slot 0
    BeforeFirstBit = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// This bit is set and cleared by software. It is used to configure the level of the start of frame on the FS
/// signal. It is meaningless and is not used in AC’97 or SPDIF audio block configuration.
/// This bit must be configured when the audio block is disabled.
pub enum FsPolarity {
    /// FS is active low (falling edge)
    ActiveLow = 0,
    /// FS is active high (rising edge)
    ActiveHigh = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]

pub enum FsSignal {
    /// Start of frame, like for instance the PCM/DSP, TDM, AC’97, audio protocols,
    Frame = 0,
    /// Start of frame and channel side identification within the audio frame like for the I2S,
    /// the MSB or LSB-justified protocols.
    FrameAndChannel = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Set which bit is transmitted first: Least significant, or Most significant. You may have to
/// choose the one used by your SAI device. Sets xCR1 register, LSBFIRST field.
pub enum FirstBit {
    /// Data are transferred with MSB first
    LsbFirst = 0,
    /// Data are transferred with LSB first
    MsbFirst = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// FIFO threshold. Affects xCR2 reg, FTH field.
pub enum FifoThresh {
    /// FIFO empty
    Empty = 0b000,
    /// 1/4 FIFO
    T1_4 = 0b001,
    /// 1/2 FIFO
    T1_2 = 0b010,
    /// 3/4 FIFO
    T3_4 = 0b011,
    /// FIFO full
    Full = 0b100,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Oversampling ratio for master clock. You may have to
/// choose the one used by your SAI device. Sets xCR1 register, OSR field.
pub enum OversamplingRatio {
    /// Master clock frequency = F_FS x 256
    FMul256 = 0,
    /// Master clock frequency = F_FS x 512
    FMul512 = 1,
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
/// Select wheather the master clock is generated. xDR1 register, NOMCK field on H7.
/// on other variants such as WB, affects the MCKEN and NODIV fields (?).
pub enum MasterClock {
    // These bit values are for NOMCK, ie on H7. We use inverse logic when setting the bits
    // on other variants.
    /// (H7): Master clock generator is enabled
    Used = 0,
    /// (H7):  Master clock generator is disabled. The clock divider controlled by MCKDIV can still be used to
    /// generate the bit clock.
    NotUsed = 1,
}

#[derive(Clone, Copy)]
/// The type of SAI interrupt to configure. Reference Section 41.5 of the L4 RM.
/// Enabled in xIM register, yIE fields. See H743 RM, section 51.5: SAI interrupts.
pub enum SaiInterrupt {
    /// FIFO request interrupt enable. When this bit is set, an interrupt is generated if the FREQ bit in the SAI_xSR register is set.
    /// Since the audio block defaults to operate as a transmitter after reset, the MODE bit must be
    /// configured before setting FREQIE to avoid a parasitic interrupt in receiver mode
    Freq,
    /// When the audio block is configured as receiver, an overrun condition may appear if data are
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

/// Configuration for the SAI peripheral. Mainly affects the ACR and BCR registers.
/// Used for either channel. For details, see documentation of individual structs and fields.
/// You may be forced into certain settings based on the device used.
#[derive(Clone, Copy)]
pub struct SaiConfig {
    pub mode: SaiMode,
    /// Select protocols between Free, Ac'97, and SPDIF. Defaults to Free.
    pub protocol: Protocol,
    /// Select mono or stereo modes. Default to mono.
    pub mono: Mono,
    /// An audio subblock can be configured to operate synchronously with the second audio
    /// subblock in the same SAI. In this case, the bit clock and the frame synchronization signals
    /// are shared to reduce the number of external pins used for the communication. Default to async.
    pub sync: SyncMode,
    pub datasize: DataSize,
    /// Select wheather the master clock out is enabled, eg for syncing external devices. Defaults
    /// to disabled.
    pub master_clock: MasterClock,
    pub first_bit: FirstBit,
    pub oversampling_ratio: OversamplingRatio,
    /// Eefine the audio frame length expressed in number
    /// of SCK clock cycles: the number of bits in the frame is equal to FRL[7:0] + 1.
    /// The minimum number of bits to transfer in an audio frame must be equal to 8, otherwise the audio
    /// block will behaves in an unexpected way. This is the case when the data size is 8 bits and only one
    /// slot 0 is defined in NBSLOT[4:0] of SAI_xSLOTR register (NBSLOT[3:0] = 0000).
    /// In master mode, if the master clock (available on MCLK_x pin) is used, the frame length should be
    /// aligned with a number equal to a power of 2, ranging from 8 to 256. When the master clock is not
    /// used (NOMCK = 1), it is recommended to program the frame length to an value ranging from 8 to
    /// 256.
    pub frame_length: u16, // u16 to allow the value of 256.
    // /// Specify the length in number of bit clock
    // /// (SCK) + 1 (FSALL[6:0] + 1) of the active level of the FS signal in the audio frame
    // /// These bits are meaningless and are not used in AC’97 or SPDIF audio block configuration.
    // /// They must be configured when the audio block is disabled
    // pub fs_level_len: u8,
    pub fs_offset: FsOffset,
    /// Active high, or active low polarity. Defaults to active high.
    pub fs_polarity: FsPolarity,
    /// Default to frame and channel.
    pub fs_signal: FsSignal,
    /// Number of slots. Defaults to 2.
    pub num_slots: u8,
    /// The FIFO threshold configures when the FREQ interrupt is generated based on how full
    /// the FIFO is.
    pub fifo_thresh: FifoThresh,
}

impl Default for SaiConfig {
    fn default() -> Self {
        Self {
            mode: SaiMode::MasterTransmitter,
            protocol: Protocol::Free,
            mono: Mono::Stereo,
            sync: SyncMode::Async,
            datasize: DataSize::S24,
            master_clock: MasterClock::NotUsed,
            first_bit: FirstBit::MsbFirst,
            oversampling_ratio: OversamplingRatio::FMul256,
            frame_length: 64,
            // fs_level_len: 32, // todo: Is this always frame_length / 2???
            fs_offset: FsOffset::FirstBit,
            fs_polarity: FsPolarity::ActiveHigh,
            fs_signal: FsSignal::FrameAndChannel, // Use FrameAndChannel for I2S.
            num_slots: 2,
            fifo_thresh: FifoThresh::T1_4,
        }
    }
}

// todo: Populate these presets
impl SaiConfig {
    /// Default configuration for I2S.
    pub fn i2s_preset() -> Self {
        Self {
            // Use our default of 2 slots, and a frame length of 64 bits, to allow for up
            // to 32 bits per slot.
            // We also use our default fifo thresh of 1/4 of the total size of 8 words,
            // ie 1 word per channel
            ..Default::default()
        }
    }

    /// Default configuration for PDM
    pub fn pdm_preset() -> Self {
        Self {
            ..Default::default()
        }
    }

    /// Default configuration for AC'97
    pub fn ac97_preset() -> Self {
        Self {
            protocol: Protocol::Ac97,
            fs_signal: FsSignal::Frame,
            // Note that AC97 uses 13 slots, but with the AC97 protocol set, the slots setting is
            // ignored.
            ..Default::default()
        }
    }

    /// Default configuration for SPDIF
    pub fn spdif_preset() -> Self {
        Self {
            protocol: Protocol::Spdif,
            ..Default::default()
        }
    }
}

/// Represents the Serial Audio Interface (SAI) peripheral, used for digital audio
/// input and output.
pub struct Sai<R> {
    pub regs: R,
    config_a: SaiConfig,
    config_b: SaiConfig,
}

impl<R> Sai<R>
where
    R: Deref<Target = sai::RegisterBlock>,
{
    /// Initialize a SAI peripheral, including  enabling and resetting
    /// its RCC peripheral clock. For now, set up with default clocks selected.
    /// ie, pll1q.
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
        // Set the master clock divider.

        // todo: Set these based on inputting a sampling freq and SAI clock speed.
        // todo: Hard coded now for 98Mhz SAI clock, 48K sampling, and no master out.
        // See H7 RM, Table 421.
        // let mckdiv_a = 32;
        // let mckdiv_b = 32;

        // 0 and 1 both divide the clock by 1.
        let mckdiv_a = 1;
        let mckdiv_b = 1;

        // mckdiv = SAI clock / (sampling freq * 256) ?? (512 for oversampling?)

        // with NOMCK = 1: (No master clock
        // F_SCK = F_sai_ker_ck / MCKDIV
        // F_FS = F_sai_ker_ck / ((FRL + 1) * MCKDIV)

        // For info on modes, reference H743 RM, section 51.4.3: "Configuring and
        // Enabling SAI modes".
        regs.cha.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_a.mode as u8);
            w.prtcfg().bits(config_a.protocol as u8);
            w.mono().bit(config_a.mono as u8 != 0);
            w.syncen().bits(config_a.sync as u8);
            // The NOMCK bit of the SAI_xCR1 register is used to define whether the master clock is
            // generated or not.
            // Inversed polarity on non-H7 based on how we have `MasterClock` enabled.
            #[cfg(not(any(feature = "h7", feature = "l4", feature = "l5")))]
            w.mcken().bit(config_a.master_clock as u8 == 0);
            #[cfg(feature = "h7")]
            // Due to an H7 PAC error, xCR bit 19 is called NODIV (Which is how it is on other platforms).
            // This is actually the NOMCK bit.
            w.nodiv().bit(config_a.master_clock as u8 != 0);
            // The audio frame can target different data sizes by configuring bit DS[2:0] in the SAI_xCR1
            // register. The data sizes may be 8, 10, 16, 20, 24 or 32 bits. During the transfer, either the
            // MSB or the LSB of the data are sent first, depending on the configuration of bit LSBFIRST in
            // the SAI_xCR1 register.
            w.ds().bits(config_a.datasize as u8);
            #[cfg(not(feature = "l4"))]
            w.osr().bit(config_a.oversampling_ratio as u8 != 0);
            // This bit is set and cleared by software. It must be configured when the audio block is disabled. This
            // bit has no meaning in AC’97 audio protocol since AC’97 data are always transferred with the MSB
            // first. This bit has no meaning in SPDIF audio protocol since in SPDIF data are always transferred
            // with LSB first
            w.lsbfirst().bit(config_a.first_bit as u8 != 0);
            w.mckdiv().bits(mckdiv_a)
        });
        // todo: MCKEN vice NOMCK?? Make sure your enum reflects how you handle it.

        regs.chb.cr1.modify(|_, w| unsafe {
            w.mode().bits(config_b.mode as u8);
            w.prtcfg().bits(config_b.protocol as u8);
            w.mono().bit(config_b.mono as u8 != 0);
            w.syncen().bits(config_b.sync as u8);
            #[cfg(not(any(feature = "h7", feature = "l4", feature = "l5")))]
            w.mcken().bit(config_b.master_clock as u8 == 0);
            #[cfg(feature = "h7")]
            w.nodiv().bit(config_b.master_clock as u8 != 0);
            w.ds().bits(config_b.datasize as u8);
            #[cfg(not(feature = "l4"))]
            w.osr().bit(config_b.oversampling_ratio as u8 != 0);
            w.lsbfirst().bit(config_b.first_bit as u8 != 0);
            w.mckdiv().bits(mckdiv_b)
        });

        // todo: Add this to config and don't hard-set.
        regs.cha.cr2.modify(|_, w| unsafe {
            w.comp().bits(0);
            w.cpl().clear_bit();
            #[cfg(feature = "wb")]
            w.mutecn().bits(0); // rec only
            #[cfg(not(feature = "wb"))]
            w.muteval().clear_bit(); // xmitter only
            w.mute().clear_bit(); // xmitter only
            w.tris().clear_bit(); // xmitter only
                                  // The FIFO pointers can be reinitialized when the SAI is disabled by setting bit FFLUSH in the
                                  // SAI_xCR2 register. If FFLUSH is set when the SAI is enabled the data present in the FIFO
                                  // will be lost automatically.
            w.fflush().set_bit();
            // FIFO threshold
            w.fth().bits(config_a.fifo_thresh as u8)
        });

        regs.chb.cr2.modify(|_, w| unsafe {
            w.comp().bits(0);
            w.cpl().clear_bit();
            #[cfg(feature = "wb")]
            w.mutecn().bits(0); // rec only
            #[cfg(not(feature = "wb"))]
            w.muteval().clear_bit(); // xmitter only
            w.mute().clear_bit(); // xmitter only
            w.tris().clear_bit(); // xmitter only
            w.fflush().set_bit();
            w.fth().bits(config_b.fifo_thresh as u8)
        });

        // The FS signal can have a different meaning depending on the FS function. FSDEF bit in the
        // SAI_xFRCR register selects which meaning it will have:
        // • 0: start of frame, like for instance the PCM/DSP, TDM, AC’97, audio protocols,
        // • 1: start of frame and channel side identification within the audio frame like for the I2S,
        // the MSB or LSB-justified protocols.
        // When the FS signal is considered as a start of frame and channel side identification within
        // the frame, the number of declared slots must be considered to be half the number for the left
        // channel and half the number for the right channel. If the number of bit clock cycles on half
        // audio frame is greater than the number of slots dedicated to a channel side, and TRIS = 0, 0
        // is sent for transmission for the remaining bit clock cycles in the SAI_xCR2 register.
        // Otherwise if TRIS = 1, the SD line is released to HI-Z. In reception mode, the remaining bit
        // clock cycles are not considered until the channel side changes.

        if config_a.frame_length < 8
            || config_b.frame_length < 8
            || config_a.frame_length > 256
            || config_b.frame_length > 256
        {
            panic!("Frame length must be bewteen 8 and 256")
        }

        // The audio frame length can be configured to up to 256 bit clock cycles, by setting
        // FRL[7:0] field in the SAI_xFRCR register.
        regs.cha.frcr.modify(|_, w| unsafe {
            w.fsoff().bit(config_a.fs_offset as u8 != 0);
            w.fspol().bit(config_a.fs_polarity as u8 != 0);
            w.fsdef().bit(config_a.fs_signal as u8 != 0);
            // Hard-set a 50% duty cycle. Don't think this is a safe assumption? Send in an issue
            // or PR.
            w.fsall().bits((config_a.frame_length / 2) as u8 - 1);
            w.frl().bits((config_a.frame_length - 1) as u8)
        });

        regs.chb.frcr.modify(|_, w| unsafe {
            w.fsoff().bit(config_a.fs_offset as u8 != 0);
            w.fspol().bit(config_b.fs_polarity as u8 != 0);
            w.fsdef().bit(config_b.fs_signal as u8 != 0);
            w.fsall().bits((config_b.frame_length / 2) as u8 - 1);
            w.frl().bits((config_b.frame_length - 1) as u8)
        });

        // slot en bits???
        let slot_en_bits: u16 = (2_u32.pow(config_a.num_slots as u32) - 1) as u16;
        // todo: Slot configuration (NBSLOT) ? xSLOTR?
        regs.cha.slotr.modify(|_, w| unsafe {
            w.sloten().bits(slot_en_bits);
            w.nbslot().bits(config_a.num_slots);
            w.slotsz().bits(0b10); // 32-bit for 24 bytes?
            w.fboff().bits(0) // todo: User-customizable?
        });

        regs.chb.slotr.modify(|_, w| unsafe {
            w.sloten().bits(slot_en_bits);
            w.nbslot().bits(config_b.num_slots);
            w.slotsz().bits(0b10); // 32-bit for 24 bytes?
            w.fboff().bits(0)
        });

        Self {
            regs,
            config_a,
            config_b,
        }
    }

    /// Enable an audio subblock (channel).
    pub fn enable(&mut self, channel: SaiChannel) {
        // Each of the audio blocks in the SAI are enabled by SAIEN bit in the SAI_xCR1 register. As
        // soon as this bit is active, the transmitter or the receiver is sensitive to the activity on the
        // clock line, data line and synchronization line in slave mode.
        // In master TX mode, enabling the audio block immediately generates the bit clock for the
        // external slaves even if there is no data in the FIFO, However FS signal generation is
        // conditioned by the presence of data in the FIFO. After the FIFO receives the first data to
        // transmit, this data is output to external slaves. If there is no data to transmit in the FIFO, 0
        // values are then sent in the audio frame with an underrun flag generation.
        // In slave mode, the audio frame starts when the audio block is enabled and when a start of
        // frame is detected.
        // In Slave TX mode, no underrun event is possible on the first frame after the audio block is
        // enabled, because the mandatory operating sequence in this case is:
        // 1. Write into the SAI_xDR (by software or by DMA).
        // 2. Wait until the FIFO threshold (FLH) flag is different from 0b000 (FIFO empty).
        // 3. Enable the audio block in slave transmitter mode.

        match channel {
            SaiChannel::A => {
                // todo: Do we want to flush?
                self.regs.cha.cr2.modify(|_, w| w.fflush().set_bit());
                self.regs.cha.cr1.modify(|_, w| w.saien().set_bit());

                // Note: This read check only fires the WCKCFG bit if Master out is enabled.

                if self.regs.cha.sr.read().wckcfg().bit_is_set() {
                    panic!("Wrong clock configuration. Clock configuration does not respect the rule concerning
the frame length specification defined in Section 51.4.6: Frame synchronization (configuration of
FRL[7:0] bit in the SAI_xFRCR register)
This bit is used only when the audio block operates in master mode (MODE[1] = 0) and NOMCK = 0.
It can generate an interrupt if WCKCFGIE bit is set in SAI_xIM register");
                }
            }
            SaiChannel::B => {
                self.regs.chb.cr2.modify(|_, w| w.fflush().set_bit());
                self.regs.chb.cr1.modify(|_, w| w.saien().set_bit());

                if self.regs.chb.sr.read().wckcfg().bit_is_set() {
                    panic!("Wrong clock configuration. Clock configuration does not respect the rule concerning the frame length specification defined in
Section 51.4.6: Frame synchronization (configuration of FRL[7:0] bit in the SAI_xFRCR register)
This bit is used only when the audio block operates in master mode (MODE[1] = 0) and NOMCK = 0.
It can generate an interrupt if WCKCFGIE bit is set in SAI_xIM register");
                }
            }
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

    /// Read 2 words of data from a channel: Left and Right channel, in that order.
    /// A read from the SR register empties the FIFO if the FIFO is not empty
    pub fn read(&self, channel: SaiChannel) -> (u32, u32) {
        // while self.regs.cha.sr.read().flvl().bits() == FifoThresh::Empty as u8 {} // todo?

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

    /// Send 2 words of data to a single channel: Left and right channel, in that order.
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
    /// Before configuring the SAI block, the SAI DMA channel must be disabled.
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

        // todo: DMA2 support.

        // L44 RM, Table 41. "DMA1 requests for each channel"
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match sai_channel {
            SaiChannel::A => DmaInput::Sai1A.dma1_channel(),
            SaiChannel::B => DmaInput::Sai1B.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match sai_channel {
            SaiChannel::A => dma.channel_select(DmaInput::Sai1A),
            SaiChannel::B => dma.channel_select(DmaInput::Sai1B),
        };

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
        // (Set in `new`).
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

        let sai_cfg = match sai_channel {
            SaiChannel::A => self.config_a,
            SaiChannel::B => self.config_b,
        };

        let datasize = match sai_cfg.datasize {
            DataSize::S8 => dma::DataSize::S8,
            DataSize::S10 => dma::DataSize::S16,
            DataSize::S16 => dma::DataSize::S16,
            _ => dma::DataSize::S32,
        };

        dma.cfg_channel(
            dma_channel,
            periph_addr,
            ptr as u32,
            len,
            dma::Direction::ReadFromPeriph,
            datasize,
            datasize,
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
        match channel {
            SaiChannel::A => {
                self.regs.cha.clrfr.write(|w| match interrupt_type {
                    // This Interrupt (FREQ bit in SAI_xSR register) is
                    // cleared by hardware when the FIFO becomes empty (FLVL[2:0] bits in SAI_xSR is equal
                    // to 0b000) i.e no data are stored in FIFO.
                    SaiInterrupt::Freq => w.cmutedet().set_bit(), // There is no Freq flag.
                    SaiInterrupt::Ovrudr => w.covrudr().set_bit(),
                    SaiInterrupt::AfsDet => w.cafsdet().set_bit(),
                    SaiInterrupt::LfsDet => w.clfsdet().set_bit(),
                    SaiInterrupt::CnRdy => w.ccnrdy().set_bit(),
                    SaiInterrupt::MuteDet => w.cmutedet().set_bit(),
                    SaiInterrupt::WckCfg => w.cwckcfg().set_bit(),
                });
            }
            SaiChannel::B => {
                self.regs.chb.clrfr.write(|w| match interrupt_type {
                    SaiInterrupt::Freq => w.cmutedet().set_bit(),
                    SaiInterrupt::Ovrudr => w.covrudr().set_bit(),
                    SaiInterrupt::AfsDet => w.cafsdet().set_bit(),
                    SaiInterrupt::LfsDet => w.clfsdet().set_bit(),
                    SaiInterrupt::CnRdy => w.ccnrdy().set_bit(),
                    SaiInterrupt::MuteDet => w.cmutedet().set_bit(),
                    SaiInterrupt::WckCfg => w.cwckcfg().set_bit(),
                });
            }
        }
    }
}
