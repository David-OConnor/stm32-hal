//! Support for the Direct Memory Access (DMA) peripheral. This module handles initialization, and transfer
//! configuration for DMA. The `Dma::cfg_channel` method is called by modules that use DMA.

// todo: This module could be greatly simplified if [this issue](https://github.com/stm32-rs/stm32-rs/issues/610)
// todo is addressed: Ie H7 PAC approach adopted by other modules.

// todo: Use this clip or something similar to end terminate while loops, as in other modules.
// let mut i = 0;
// while asdf {
//     i += 1;
//     if i >= MAX_ITERS {
//         return Err(Error::Hardware);
//     }
// }

use core::{
    ops::Deref,
    sync::atomic::{self, Ordering},
};

use crate::{
    pac::{self, RCC},
    util::rcc_en_reset,
    MAX_ITERS,
};

cfg_if! {
    if #[cfg(all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))))] {
        use crate::pac::{dma as dma1, DMA as DMA1};
    } else if #[cfg(feature = "f3x4")] {
        use crate::pac::{dma1, DMA1};
    }
    else {
        use crate::pac::{dma1, dma2, DMA1, DMA2};
    }
}

// use embedded_dma::{ReadBuffer, WriteBuffer};
use cfg_if::cfg_if;
#[cfg(any(feature = "g0", feature = "g4", feature = "wl"))]
use pac::DMAMUX;
// todo: DMAMUX2 support (Not sure if WB has it, but H7 has both).
#[cfg(any(feature = "l5", feature = "wb", feature = "h7"))]
use pac::DMAMUX1 as DMAMUX;
#[cfg(feature = "h7")]
use pac::DMAMUX2;
use paste::paste;

// todo: Several sections of this are only correct for DMA1.

#[derive(Clone, Copy)]
pub enum DmaPeriph {
    Dma1,
    #[cfg(not(any(
        feature = "f3x4",
        all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))),
        feature = "wb",
    )))] // todo: f3x4 too
    Dma2,
}

#[derive(Copy, Clone)]
#[repr(usize)]
#[cfg(not(any(feature = "h7", feature = "wl")))]
/// A list of DMA input sources. The integer values represent their DMAMUX register value, on
/// MCUs that use this. G4 RM, Table 91: DMAMUX: Assignment of multiplexer inputs to resources.
pub enum DmaInput {
    // This (on G4) goes up to 115. For now, just implement things we're likely
    // to use in this HAL. Make sure this is compatible beyond G4.
    Adc1 = 5,
    Dac1Ch1 = 6,
    Dac1Ch2 = 7,
    Tim6Up = 8,
    Tim7Up = 9,
    Spi1Rx = 10,
    Spi1Tx = 11,
    Spi2Rx = 12,
    Spi2Tx = 13,
    Spi3Rx = 14,
    Spi3Tx = 15,
    I2c1Rx = 16,
    I2c1Tx = 17,
    I2c2Rx = 18,
    I2c2Tx = 19,
    I2c3Rx = 20,
    I2c3Tx = 21,
    I2c4Rx = 22,
    I2c4Tx = 23,
    Usart1Rx = 24,
    Usart1Tx = 25,
    Usart2Rx = 26,
    Usart2Tx = 27,
    Usart3Rx = 28,
    Usart3Tx = 29,
    Uart4Rx = 30,
    Uart4Tx = 31,
    Uart5Rx = 32,
    Uart5Tx = 33,
    Lpuart1Rx = 34,
    Lpuart1Tx = 35,
    Adc2 = 36,
    Adc3 = 37,
    Adc4 = 38,
    Adc5 = 39,
    Quadspi = 40,
    Dac2Ch1 = 41,
    Tim1Ch1 = 42,
    Tim1Ch2 = 43,
    Tim1Ch3 = 44,
    Tim1Ch4 = 45,
    TimUp = 46,
    Tim1Trig = 47,
    Tim1Com = 48,
    Tim8Ch1 = 49,
    Tim8Ch2 = 50,
    Tim8Ch3 = 51,
    Tim8Ch4 = 52,
    Tim8Up = 53,
    Tim8Trig = 54,
    Tim8Com = 55,
    Tim2Ch1 = 56,
    Tim2Ch2 = 57,
    Tim2Ch3 = 58,
    Tim2Ch4 = 59,
    Tim2Up = 60,
    Tim3Ch1 = 61,
    Tim3Ch2 = 62,
    Tim3Ch3 = 63,
    Tim3Ch4 = 64,
    Tim3Up = 65,
    Tim3Trig = 66,
    Tim4Ch1 = 67,
    Tim4Ch2 = 68,
    Tim4Ch3 = 69,
    Tim4Ch4 = 70,
    Tim4Up = 71,
    Sai1A = 108,
    Sai1B = 109,
    // todo: These SAI2 values are bogus; can't find on G4 DMA mux.
    Sai2A = 203,
    Sai2B = 204,
    // todo: Can't find DFSDM periph on G4 rm: These assigned values are bogus.
    Dfsdm1F0 = 200,
    Dfsdm1F1 = 201,
    Dfsdm1F2 = 205,
    Dfsdm1F3 = 206,
}

#[derive(Copy, Clone)]
#[repr(usize)]
#[cfg(feature = "wl")]
/// WL RM, 14.3.2: DMAMUX1 mapping
pub enum DmaInput {
    Adc = 5,
    DacOut1 = 6,
    Dac1Ch2 = 7,
    Spi1Rx = 7,
    Spi1Tx = 8,
    Spi2Rx = 9,
    Spi2Tx = 10,
    I2c1Rx = 11,
    I2c1Tx = 12,
    I2c2Rx = 13,
    I2c2Tx = 14,
    I2c3Rx = 15,
    I2c3Tx = 16,
    Usart1Rx = 17,
    Usart1Tx = 18,
    Usart2Rx = 19,
    Usart2Tx = 20,
    Lpuart1Rx = 21,
    Lpuart1Tx = 22,
    Tim1Ch1 = 23,
    Tim1Ch2 = 24,
    Tim1Ch3 = 25,
    Tim1Ch4 = 26,
    TimUp = 27,
    Tim1Trig = 28,
    Tim1Com = 29,
    Tim2Ch1 = 30,
    Tim2Ch2 = 31,
    Tim2Ch3 = 32,
    Tim2Ch4 = 33,
    Tim2Up = 34,
    Tim16Ch1 = 35,
    Tim16Up = 36,
    Tim17Ch1 = 37,
    Tim17Up = 38,
    AesIn = 39,
    AesOut = 40,
    SubghzSpiRx = 41,
    SubghzSpiTx = 42,
}

// todo: Trigger, synchronization etc mappings. Perhaps DmaTrigger, DmaSync enums etc.

#[derive(Copy, Clone)]
#[repr(usize)]
#[cfg(feature = "h7")]
/// A list of DMA input sources. The integer values represent their DMAMUX register value, on
/// MCUs that use this. H743 RM, Table 121: DMAMUX1: Assignment of multiplexer inputs to resources.
/// (Table 118 in RM0468)
/// Note that this is only for DMAMUX1
pub enum DmaInput {
    Adc1 = 9,
    Adc2 = 10,
    Tim1Ch1 = 11,
    Tim1Ch2 = 12,
    Tim1Ch3 = 13,
    Tim1Ch4 = 14,
    Tim1Up = 15,
    Tim1Trig = 16,
    Tim1Com = 17,
    Tim2Ch1 = 18,
    Tim2Ch2 = 19,
    Tim2Ch3 = 20,
    Tim2Ch4 = 21,
    Tim2Up = 22,
    Tim3Ch1 = 23,
    Tim3Ch2 = 24,
    Tim3Ch3 = 25,
    Tim3Ch4 = 26,
    Tim3Up = 27,
    Tim3Trig = 28,
    Tim4Ch1 = 29,
    Tim4Ch2 = 30,
    Tim4Ch3 = 31,
    Tim4Up = 32,
    I2c1Rx = 33,
    I2c1Tx = 34,
    I2c2Rx = 35,
    I2c2Tx = 36,
    Spi1Rx = 37,
    Spi1Tx = 38,
    Spi2Rx = 39,
    Spi2Tx = 40,
    Usart1Rx = 41,
    Usart1Tx = 42,
    Usart2Rx = 43,
    Usart2Tx = 44,
    Usart3Rx = 45,
    Usart3Tx = 46,
    Tim5Ch1 = 55,
    Tim5Ch2 = 56,
    Tim5Ch3 = 57,
    Tim5Ch4 = 58,
    Tim5Up = 59,
    Tim5Trig = 60,
    Spi3Rx = 61,
    Spi3Tx = 62,
    Uart4Rx = 63,
    Uart4Tx = 64,
    Uart5Rx = 65,
    Uart5Tx = 66,
    DacCh1 = 67,
    DacCh2 = 68,
    Tim6Up = 69,
    Tim7Up = 70,
    Uart6Rx = 71,
    Uart6Tx = 72,
    I2c3Rx = 73,
    I2c3Tx = 74,
    Dcmi = 75,
    CrypIn = 76,
    CrypOut = 77,
    HashIn = 78,
    Uart7Rx = 79,
    Uart7Tx = 80,
    Uart8Rx = 81,
    Uart8Tx = 82,
    Sai1A = 87,
    Sai1B = 88,
    Sai2A = 89,
    Sai2B = 90,
    Dfsdm1F0 = 101,
    Dfsdm1F1 = 102,
    Dfsdm1F2 = 103,
    Dfsdm1F3 = 104,
    Sai3A = 113,
    Sai3B = 114,
    Adc3 = 115,
    Uart9Rx = 116,
    Uart9Tx = 117,
    Uart10Rx = 118,
    Uart10Tx = 119,
}

#[derive(Copy, Clone)]
#[repr(usize)]
#[cfg(feature = "h7")]
/// A list of DMA input sources for DMAMUX2. Used for BDMA. See H742 RM, Table 124.
pub enum DmaInput2 {
    Lpuart1Rx = 9,
    Lpuart1Tx = 10,
    Spi6Rx = 11,
    Spi6Tx = 12,
    I2c4Rx = 13,
    I3crTx = 14,
    Sai4A = 15,
    Sai4B = 16,
}

impl DmaInput {
    #[cfg(any(feature = "f3", feature = "l4"))]
    /// Select the hard set channel associated with a given input source. See L44 RM, Table 41.
    pub fn dma1_channel(&self) -> DmaChannel {
        match self {
            Self::Adc1 => DmaChannel::C1,
            Self::Dac1Ch1 => DmaChannel::C3,
            Self::Dac1Ch2 => DmaChannel::C4,
            // Self::Tim6Up => 8,
            // Self::Tim7Up => 9,
            Self::Spi1Rx => DmaChannel::C2,
            Self::Spi1Tx => DmaChannel::C3,
            Self::Spi2Rx => DmaChannel::C4,
            Self::Spi2Tx => DmaChannel::C5,
            // Self::Spi3Rx => 14,
            // Self::Spi3Tx => 15,
            Self::I2c1Rx => DmaChannel::C7,
            Self::I2c1Tx => DmaChannel::C6,
            Self::I2c2Rx => DmaChannel::C5,
            Self::I2c2Tx => DmaChannel::C4,
            Self::I2c3Rx => DmaChannel::C3,
            // Self::I2c3Tx => 21,
            // Self::I2c4Rx => 22,
            // Self::I2c4Tx => 23,
            Self::Usart1Rx => DmaChannel::C5,
            Self::Usart1Tx => DmaChannel::C4,
            Self::Usart2Rx => DmaChannel::C6,
            Self::Usart2Tx => DmaChannel::C7,
            Self::Usart3Rx => DmaChannel::C3,
            Self::Usart3Tx => DmaChannel::C2,
            // Self::Uart4Rx => 30,
            // Self::Uart4Tx => 31,
            // Self::Uart5Rx => 32,
            // Self::Uart5Tx => 33,
            // Self::Lpuart1Rx => 34,
            // Self::Lpuart1Tx => 35,
            Self::Adc2 => DmaChannel::C2,
            // Self::Adc3 => 37,
            // Self::Adc4 => 38,
            // Self::Adc5 => 39,
            // Note: Sai1 appears to be DMA2 only.
            Self::Sai2A => DmaChannel::C6,
            Self::Sai2B => DmaChannel::C7,
            Self::Dfsdm1F0 => DmaChannel::C4,
            Self::Dfsdm1F1 => DmaChannel::C5,
            Self::Dfsdm1F2 => DmaChannel::C6,
            Self::Dfsdm1F3 => DmaChannel::C7,
            _ => unimplemented!(),
        }
    }

    #[cfg(feature = "l4")]
    /// Find the value to set in the DMA_CSELR register, for L4. Ie, channel select value for a given DMA input.
    /// See L44 RM, Table 41.
    pub fn dma1_channel_select(&self) -> u8 {
        match self {
            Self::Adc1 => 0b000,
            Self::Dac1Ch1 => 0b0110,
            Self::Dac1Ch2 => 0b0101,
            // Self::Tim6Up => 8,
            // Self::Tim7Up => 9,
            Self::Spi1Rx => 0b001,
            Self::Spi1Tx => 0b001,
            Self::Spi2Rx => 0b001,
            Self::Spi2Tx => 0b001,
            // Self::Spi3Rx => 14,
            // Self::Spi3Tx => 15,
            Self::I2c1Rx => 0b011,
            Self::I2c1Tx => 0b011,
            Self::I2c2Rx => 0b011,
            Self::I2c2Tx => 0b011,
            Self::I2c3Rx => 0b011,
            // Self::I2c3Tx => 21,
            // Self::I2c4Rx => 22,
            // Self::I2c4Tx => 23,
            Self::Usart1Rx => 0b010,
            Self::Usart1Tx => 0b010,
            Self::Usart2Rx => 0b010,
            Self::Usart2Tx => 0b010,
            Self::Usart3Rx => 0b010,
            Self::Usart3Tx => 0b010,
            // Self::Uart4Rx => 30,
            // Self::Uart4Tx => 31,
            // Self::Uart5Rx => 32,
            // Self::Uart5Tx => 33,
            // Self::Lpuart1Rx => 34,
            // Self::Lpuart1Tx => 35,
            Self::Adc2 => 0b000,
            // Self::Adc3 => 37,
            // Self::Adc4 => 38,
            // Self::Adc5 => 39,
            Self::Dfsdm1F0 => 0b0000,
            Self::Dfsdm1F1 => 0b0000,
            Self::Dfsdm1F2 => 0b0000, // todo: QC?
            Self::Dfsdm1F3 => 0b0000,
            _ => unimplemented!(),
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// L4 RM, 11.4.3, "DMA arbitration":
/// The priorities are managed in two stages:
/// • software: priority of each channel is configured in the DMA_CCRx register, to one of
/// the four different levels:
/// – very high
/// – high
/// – medium
/// – low
/// • hardware: if two requests have the same software priority level, the channel with the
/// lowest index gets priority. For example, channel 2 gets priority over channel 4.
/// Only write to this when the channel is disabled.
pub enum Priority {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    VeryHigh = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Represents a DMA channel to select, eg when configuring for use with a peripheral.
/// u8 representation is used to index registers on H7 PAC (And hopefully on future PACs if they
/// adopt H7's approach)
pub enum DmaChannel {
    // H7 calls these Streams. We use the `Channel` name for consistency.
    #[cfg(feature = "h7")]
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    // todo: Some G0 variants have channels 6 and 7 and DMA1. (And up to 5 channels on DMA2)
    #[cfg(not(feature = "g0"))]
    C6 = 6,
    #[cfg(not(feature = "g0"))]
    C7 = 7,
    // todo: Which else have 8? Also, note that some have diff amounts on dma1 vs 2.
    #[cfg(any(feature = "l5", feature = "g4"))]
    C8 = 8,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Set in CCR.
/// Can only be set when channel is disabled.
pub enum Direction {
    /// DIR = 0 defines typically a peripheral-to-memory transfer
    ReadFromPeriph = 0,
    /// DIR = 1 defines typically a memory-to-peripheral transfer.
    ReadFromMem = 1,
    MemToMem = 2,
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
/// Set in CCR.
/// Can only be set when channel is disabled.
pub enum Circular {
    Disabled = 0,
    Enabled = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Peripheral and memory increment mode. (CCR PINC and MINC bits)
/// Can only be set when channel is disabled.
pub enum IncrMode {
    // Can only be set when channel is disabled.
    Disabled = 0,
    Enabled = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Peripheral and memory increment mode. (CCR PSIZE and MSIZE bits)
/// Can only be set when channel is disabled.
pub enum DataSize {
    S8 = 0b00, // ie 8 bits
    S16 = 0b01,
    S32 = 0b10,
}

#[derive(Copy, Clone)]
/// Interrupt type. Set in CCR using TEIE, HTIE, and TCIE bits.
/// Can only be set when channel is disabled.
pub enum DmaInterrupt {
    TransferError,
    HalfTransfer,
    TransferComplete,
    #[cfg(feature = "h7")]
    DirectModeError,
    #[cfg(feature = "h7")]
    FifoError,
}

/// Reduce DRY over channels when configuring a channel's CCR.
/// We must use a macro here, since match arms balk at the incompatible
/// types of `CCR1`, `CCR2` etc.
#[cfg(not(feature = "h7"))]
macro_rules! set_ccr {
    ($ccr:expr, $priority:expr, $direction:expr, $circular:expr, $periph_incr:expr, $mem_incr:expr, $periph_size:expr, $mem_size:expr) => {
        // "The register fields/bits MEM2MEM, PL[1:0], MSIZE[1:0], PSIZE[1:0], MINC, PINC, and DIR
        // are read-only when EN = 1"
        let originally_enabled = $ccr.read().en().bit_is_set();
        if originally_enabled {
            $ccr.modify(|_, w| w.en().clear_bit());
            while $ccr.read().en().bit_is_set() {}
        }

        if let Circular::Enabled = $circular {
            $ccr.modify(|_, w| w.mem2mem().clear_bit());
        }

        $ccr.modify(|_, w| unsafe {
            // – the channel priority
            w.pl().bits($priority as u8);
            // – the data transfer direction
            // This bit [DIR] must be set only in memory-to-peripheral and peripheral-to-memory modes.
            // 0: read from peripheral
            w.dir().bit($direction as u8 != 0);
            // – the circular mode
            w.circ().bit($circular as u8 != 0);
            // – the peripheral and memory incremented mode
            w.pinc().bit($periph_incr as u8 != 0);
            w.minc().bit($mem_incr as u8 != 0);
            // – the peripheral and memory data size
            w.psize().bits($periph_size as u8);
            w.msize().bits($mem_size as u8);
            // – the interrupt enable at half and/or full transfer and/or transfer error
            w.tcie().set_bit();
            // (See `Step 5` above.)
            w.en().set_bit()
        });

        if originally_enabled {
            $ccr.modify(|_, w| w.en().set_bit());
            while $ccr.read().en().bit_is_clear() {}
        }
    }
}

/// Reduce DRY over channels when configuring a channel's interrupts.
#[cfg(not(feature = "h7"))]
macro_rules! enable_interrupt {
    ($ccr:expr, $interrupt_type:expr) => {
        // "It must not be written when the channel is enabled (EN = 1)."
        let originally_enabled = $ccr.read().en().bit_is_set();
        if originally_enabled {
            $ccr.modify(|_, w| w.en().clear_bit());
            while $ccr.read().en().bit_is_set() {}
        }

        $ccr.modify(|_, w| match $interrupt_type {
            DmaInterrupt::TransferError => w.teie().set_bit(),
            DmaInterrupt::HalfTransfer => w.htie().set_bit(),
            DmaInterrupt::TransferComplete => w.tcie().set_bit(),
        });

        if originally_enabled {
            $ccr.modify(|_, w| w.en().set_bit());
            while $ccr.read().en().bit_is_clear() {}
        }
    };
}

/// Reduce DRY over channels when configuring a channel's interrupts.
#[cfg(not(feature = "h7"))]
macro_rules! disable_interrupt {
    ($ccr:expr, $interrupt_type:expr) => {
        // "It must not be written when the channel is disabled (EN = 1)."
        let originally_disabled = $ccr.read().en().bit_is_set();
        if originally_disabled {
            $ccr.modify(|_, w| w.en().clear_bit());
            while $ccr.read().en().bit_is_set() {}
        }

        $ccr.modify(|_, w| match $interrupt_type {
            DmaInterrupt::TransferError => w.teie().clear_bit(),
            DmaInterrupt::HalfTransfer => w.htie().clear_bit(),
            DmaInterrupt::TransferComplete => w.tcie().clear_bit(),
        });

        if originally_disabled {
            $ccr.modify(|_, w| w.en().set_bit());
            while $ccr.read().en().bit_is_clear() {}
        }
    };
}

/// This struct is used to pass common (non-peripheral and non-use-specific) data when configuring
/// a channel.
#[derive(Clone)]
pub struct ChannelCfg {
    /// Channel priority compared to other channels; can be low, medium, high, or very high. Defaults
    /// to medium.
    pub priority: Priority,
    /// Enable or disable circular DMA. If enabled, the transfer continues after reaching the end of
    /// the buffer, looping to the beginning. A TC interrupt first each time the end is reached, if
    /// set. Defaults to disabled.
    pub circular: Circular,
    /// Whether we increment the peripheral address on data word transfer; generally (and by default)
    /// disabled.
    pub periph_incr: IncrMode,
    /// Whether we increment the buffer address on data word transfer; generally (and by default)
    /// enabled.
    pub mem_incr: IncrMode,
}

impl Default for ChannelCfg {
    fn default() -> Self {
        Self {
            priority: Priority::Medium,
            circular: Circular::Disabled,
            // Increment the buffer address, not the peripheral address.
            periph_incr: IncrMode::Disabled,
            mem_incr: IncrMode::Enabled,
        }
    }
}

/// Represents a Direct Memory Access (DMA) peripheral.
pub struct Dma<D> {
    pub regs: D,
}

impl<D> Dma<D>
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    /// Initialize a DMA peripheral, including enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: D) -> Self {
        // todo: Enable RCC for DMA 2 etc!
        let rcc = unsafe { &(*RCC::ptr()) };
        cfg_if! {
            if #[cfg(feature = "f3")] {
                rcc.ahbenr.modify(|_, w| w.dma1en().set_bit()); // no dmarst on F3.
            } else if #[cfg(feature = "g0")] {
                rcc_en_reset!(ahb1, dma, rcc);
            } else {
                rcc_en_reset!(ahb1, dma1, rcc);
            }
        }

        Self { regs }
    }

    #[cfg(not(feature = "h7"))] // due to num_data size diff
    /// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
    /// interrupt. Note that this fn has been (perhaps) depreciated by the standalone fn.
    pub fn cfg_channel(
        &mut self,
        channel: DmaChannel,
        periph_addr: u32,
        mem_addr: u32,
        num_data: u16,
        direction: Direction,
        periph_size: DataSize,
        mem_size: DataSize,
        cfg: ChannelCfg,
    ) {
        cfg_channel(
            &mut self.regs,
            channel,
            periph_addr,
            mem_addr,
            num_data,
            direction,
            periph_size,
            mem_size,
            cfg,
        )
    }

    #[cfg(feature = "h7")]
    /// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
    /// interrupt. Note that this fn has been (perhaps) depreciated by the standalone fn.
    pub fn cfg_channel(
        &mut self,
        channel: DmaChannel,
        periph_addr: u32,
        mem_addr: u32,
        num_data: u32,
        direction: Direction,
        periph_size: DataSize,
        mem_size: DataSize,
        cfg: ChannelCfg,
    ) {
        cfg_channel(
            &mut self.regs,
            channel,
            periph_addr,
            mem_addr,
            num_data,
            direction,
            periph_size,
            mem_size,
            cfg,
        )
    }

    #[cfg(feature = "l4")]
    pub(crate) fn channel_select(&mut self, input: DmaInput) {
        channel_select(&mut self.regs, input);
    }

    /// Stop a DMA transfer, if in progress.
    pub fn stop(&mut self, channel: DmaChannel) {
        stop_internal(&mut self.regs, channel);
    }

    /// Clear an interrupt flag.
    pub fn clear_interrupt(&mut self, channel: DmaChannel, interrupt: DmaInterrupt) {
        clear_interrupt_internal(&mut self.regs, channel, interrupt);
    }

    // todo: G0 removed from this fn due to a bug introduced in PAC 0.13
    #[cfg(not(any(feature = "h7", feature = "g0")))]
    pub fn transfer_is_complete(&mut self, channel: DmaChannel) -> bool {
        let isr_val = self.regs.isr.read();
        match channel {
            DmaChannel::C1 => isr_val.tcif1().bit_is_set(),
            DmaChannel::C2 => isr_val.tcif2().bit_is_set(),
            DmaChannel::C3 => isr_val.tcif3().bit_is_set(),
            DmaChannel::C4 => isr_val.tcif4().bit_is_set(),
            DmaChannel::C5 => isr_val.tcif5().bit_is_set(),
            #[cfg(not(feature = "g0"))]
            DmaChannel::C6 => isr_val.tcif6().bit_is_set(),
            #[cfg(not(feature = "g0"))]
            DmaChannel::C7 => isr_val.tcif7().bit_is_set(),
            #[cfg(any(feature = "l5", feature = "g4"))]
            DmaChannel::C8 => isr_val.tcif8().bit_is_set(),
        }
    }

    #[cfg(feature = "h7")]
    pub fn transfer_is_complete(&mut self, channel: DmaChannel) -> bool {
        match channel {
            DmaChannel::C0 => self.regs.lisr.read().tcif0().bit_is_set(),
            DmaChannel::C1 => self.regs.lisr.read().tcif1().bit_is_set(),
            DmaChannel::C2 => self.regs.lisr.read().tcif2().bit_is_set(),
            DmaChannel::C3 => self.regs.lisr.read().tcif3().bit_is_set(),
            DmaChannel::C4 => self.regs.hisr.read().tcif4().bit_is_set(),
            DmaChannel::C5 => self.regs.hisr.read().tcif5().bit_is_set(),
            DmaChannel::C6 => self.regs.hisr.read().tcif6().bit_is_set(),
            DmaChannel::C7 => self.regs.hisr.read().tcif7().bit_is_set(),
        }
    }

    /// Enable a specific type of interrupt.
    pub fn enable_interrupt(&mut self, channel: DmaChannel, interrupt: DmaInterrupt) {
        enable_interrupt_internal(&mut self.regs, channel, interrupt);
    }

    /// Disable a specific type of interrupt.
    /// todo: Non-H7 version too!
    #[cfg(feature = "h7")]
    pub fn disable_interrupt(&mut self, channel: DmaChannel, interrupt: DmaInterrupt) {
        // Can only be set when the channel is disabled.
        // todo: Is this true for disabling interrupts true, re the channel must be disabled?
        let cr = &self.regs.st[channel as usize].cr;

        let originally_enabled = cr.read().en().bit_is_set();

        if originally_enabled {
            cr.modify(|_, w| w.en().clear_bit());
            while cr.read().en().bit_is_set() {}
        }

        match interrupt {
            DmaInterrupt::TransferError => cr.modify(|_, w| w.teie().clear_bit()),
            DmaInterrupt::HalfTransfer => cr.modify(|_, w| w.htie().clear_bit()),
            DmaInterrupt::TransferComplete => cr.modify(|_, w| w.tcie().clear_bit()),
            DmaInterrupt::DirectModeError => cr.modify(|_, w| w.dmeie().clear_bit()),
            DmaInterrupt::FifoError => self.regs.st[channel as usize]
                .fcr
                .modify(|_, w| w.feie().clear_bit()),
        }

        if originally_enabled {
            cr.modify(|_, w| w.en().set_bit());
            while cr.read().en().bit_is_clear() {}
        }
    }
}

/// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
/// interrupt. This is the function called by various module `read_dma` and `write_dma` functions.
#[cfg(not(feature = "h7"))]
pub fn cfg_channel<D>(
    regs: &mut D,
    channel: DmaChannel,
    periph_addr: u32,
    mem_addr: u32,
    num_data: u16,
    direction: Direction,
    periph_size: DataSize,
    mem_size: DataSize,
    cfg: ChannelCfg,
) where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // See the comments in the H7 variant for a description of what's going on.

    unsafe {
        match channel {
            DmaChannel::C1 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch1.par;
                    } else {
                        let cpar = &regs.cpar1;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            DmaChannel::C2 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch2.par;
                    } else {
                        let cpar = &regs.cpar2;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            DmaChannel::C3 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch3.par;
                    } else {
                        let cpar = &regs.cpar3;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            DmaChannel::C4 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch4.par;
                    } else {
                        let cpar = &regs.cpar4;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            DmaChannel::C5 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch5.par;
                    } else {
                        let cpar = &regs.cpar5;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C6 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch6.par;
                    } else {
                        let cpar = &regs.cpar6;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C7 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cpar = &regs.ch7.par;
                    } else {
                        let cpar = &regs.cpar7;
                    }
                }
                cpar.write(|w| w.bits(periph_addr));
            }
            #[cfg(any(feature = "l5", feature = "g4"))]
            DmaChannel::C8 => {
                let cpar = &regs.cpar8;
                cpar.write(|w| w.bits(periph_addr));
            }
        }
    }

    atomic::compiler_fence(Ordering::SeqCst);
    unsafe {
        match channel {
            DmaChannel::C1 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch1.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar1;
                    } else {
                        let cmar = &regs.cmar1;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            DmaChannel::C2 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch2.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar2;
                    } else {
                        let cmar = &regs.cmar2;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            DmaChannel::C3 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch3.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar3;
                    } else {
                        let cmar = &regs.cmar3;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            DmaChannel::C4 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch4.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar4;
                    } else {
                        let cmar = &regs.cmar4;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            DmaChannel::C5 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch5.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar5;
                    } else {
                        let cmar = &regs.cmar5;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C6 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch6.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar6;
                    } else {
                        let cmar = &regs.cmar6;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C7 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cmar = &regs.ch7.mar;
                    } else if #[cfg(feature = "l5")] {
                        let cmar = &regs.cm0ar7;
                    } else {
                        let cmar = &regs.cmar7;
                    }
                }
                cmar.write(|w| w.bits(mem_addr));
            }
            #[cfg(any(feature = "l5", feature = "g4"))]
            DmaChannel::C8 => {
                #[cfg(feature = "l5")]
                let cmar = &regs.cm0ar8;
                #[cfg(feature = "g4")]
                let cmar = &regs.cmar8;
                cmar.write(|w| w.bits(mem_addr));
            }
        }
    }

    #[cfg(any(feature = "l5", feature = "wl"))]
    let num_data = num_data as u32;

    #[cfg(not(feature = "l5"))] // todo: PAC ommission? ndt fields missing for diff ndt regs.
    unsafe {
        match channel {
            DmaChannel::C1 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch1.ndtr;
                    } else {
                        let cndtr = &regs.cndtr1;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            DmaChannel::C2 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch2.ndtr;
                    } else {
                        let cndtr = &regs.cndtr2;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            DmaChannel::C3 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch3.ndtr;
                    } else {
                        let cndtr = &regs.cndtr3;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            DmaChannel::C4 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch4.ndtr;
                    } else {
                        let cndtr = &regs.cndtr4;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            DmaChannel::C5 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch5.ndtr;
                    } else {
                        let cndtr = &regs.cndtr5;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C6 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch6.ndtr;
                    } else {
                        let cndtr = &regs.cndtr6;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            #[cfg(not(feature = "g0"))]
            DmaChannel::C7 => {
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "g0"))] {
                        let cndtr = &regs.ch7.ndtr;
                    } else {
                        let cndtr = &regs.cndtr7;
                    }
                }
                cndtr.write(|w| w.ndt().bits(num_data));
            }
            #[cfg(any(feature = "l5", feature = "g4"))]
            DmaChannel::C8 => {
                let cndtr = &regs.cndtr8;
                cndtr.write(|w| w.ndt().bits(num_data));
            }
        }
    }

    match channel {
        DmaChannel::C1 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch1.cr;
                } else {
                    let ccr = &regs.ccr1;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        DmaChannel::C2 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch2.cr;
                } else {
                    let ccr = &regs.ccr2;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        DmaChannel::C3 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch3.cr;
                } else {
                    let ccr = &regs.ccr3;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        DmaChannel::C4 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch4.cr;
                } else {
                    let ccr = &regs.ccr4;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        DmaChannel::C5 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch5.cr;
                } else {
                    let ccr = &regs.ccr5;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C6 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch6.cr;
                } else {
                    let ccr = &regs.ccr6;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C7 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch7.cr;
                } else {
                    let ccr = &regs.ccr7;
                }
            }
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
        #[cfg(any(feature = "l5", feature = "g4"))]
        DmaChannel::C8 => {
            let ccr = &regs.ccr8;
            set_ccr!(
                ccr,
                cfg.priority,
                direction,
                cfg.circular,
                cfg.periph_incr,
                cfg.mem_incr,
                periph_size,
                mem_size
            );
        }
    }
}

/// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
/// interrupt. This is the function called by various module `read_dma` and `write_dma` functions.
#[cfg(feature = "h7")]
pub fn cfg_channel<D>(
    regs: &mut D,
    channel: DmaChannel,
    periph_addr: u32,
    mem_addr: u32,
    num_data: u32,
    direction: Direction,
    periph_size: DataSize,
    mem_size: DataSize,
    cfg: ChannelCfg,
) where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // todo: The H7 sections are different, but we consolidated the comments. Figure out
    // todo what's different and fix it by following the steps

    regs.st[channel as usize]
        .cr
        .modify(|_, w| w.en().clear_bit());
    while regs.st[channel as usize].cr.read().en().bit_is_set() {}

    // H743 RM Section 15.3.19 The following sequence is needed to configure a DMA stream x:
    // 1. Set the peripheral register address in the DMA_CPARx register.
    // The data is moved from/to this address to/from the memory after the peripheral event,
    // or after the channel is enabled in memory-to-memory mode.
    regs.st[channel as usize]
        .par
        .write(|w| unsafe { w.bits(periph_addr) });

    atomic::compiler_fence(Ordering::SeqCst);

    // 2. Set the memory address in the DMA_CMARx register.
    // The data is written to/read from the memory after the peripheral event or after the
    // channel is enabled in memory-to-memory mode.
    regs.st[channel as usize]
        .m0ar
        .write(|w| unsafe { w.bits(mem_addr) });

    // todo: m1ar too, if in double-buffer mode.

    // 3. Configure the total number of data to transfer in the DMA_CNDTRx register.
    // After each data transfer, this value is decremented.
    regs.st[channel as usize]
        .ndtr
        .write(|w| unsafe { w.bits(num_data) });

    // 4. Configure the parameters listed below in the DMA_CCRx register:
    // (These are listed below by their corresponding reg write code)

    // todo: See note about sep reg writes to disable channel, and when you need to do this.

    // 5. Activate the channel by setting the EN bit in the DMA_CCRx register.
    // A channel, as soon as enabled, may serve any DMA request from the peripheral connected
    // to this channel, or may start a memory-to-memory block transfer.
    // Note: The two last steps of the channel configuration procedure may be merged into a single
    // access to the DMA_CCRx register, to configure and enable the channel.
    // When a channel is enabled and still active (not completed), the software must perform two
    // separate write accesses to the DMA_CCRx register, to disable the channel, then to
    // reprogram the channel for another next block transfer.
    // Some fields of the DMA_CCRx register are read-only when the EN bit is set to 1

    // (later): The circular mode must not be used in memory-to-memory mode. Before enabling a
    // channel in circular mode (CIRC = 1), the software must clear the MEM2MEM bit of the
    // DMA_CCRx register. When the circular mode is activated, the amount of data to transfer is
    // automatically reloaded with the initial value programmed during the channel configuration
    // phase, and the DMA requests continue to be served

    // (See remainder of steps in `set_ccr()!` macro.

    // todo: Let user set mem2mem mode?

    // See the [Embedonomicon section on DMA](https://docs.rust-embedded.org/embedonomicon/dma.html)
    // for info on why we use `compiler_fence` here:
    // "We use Ordering::Release to prevent all preceding memory operations from being moved
    // after [starting DMA], which performs a volatile write."

    let cr = &regs.st[channel as usize].cr;

    let originally_enabled = cr.read().en().bit_is_set();
    if originally_enabled {
        cr.modify(|_, w| w.en().clear_bit());
        while cr.read().en().bit_is_set() {}
    }

    cr.modify(|_, w| unsafe {
        // – the channel priority
        w.pl().bits(cfg.priority as u8);
        // – the data transfer direction
        // This bit [DIR] must be set only in memory-to-peripheral and peripheral-to-memory modes.
        // 0: read from peripheral
        w.dir().bits(direction as u8);
        // – the circular mode
        w.circ().bit(cfg.circular as u8 != 0);
        // – the peripheral and memory incremented mode
        w.pinc().bit(cfg.periph_incr as u8 != 0);
        w.minc().bit(cfg.mem_incr as u8 != 0);
        // – the peripheral and memory data size
        w.psize().bits(periph_size as u8);
        w.msize().bits(mem_size as u8);
        // – the interrupt enable at half and/or full transfer and/or transfer error
        w.tcie().set_bit();
        // (See `Step 5` above.)
        w.en().set_bit()
    });

    if originally_enabled {
        cr.modify(|_, w| w.en().set_bit());
        while cr.read().en().bit_is_clear() {}
    }
}

/// Stop a DMA transfer, if in progress.
#[cfg(not(feature = "h7"))]
fn stop_internal<D>(regs: &mut D, channel: DmaChannel)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // L4 RM:
    // Once the software activates a channel, it waits for the completion of the programmed
    // transfer. The DMA controller is not able to resume an aborted active channel with a possible
    // suspended bus transfer.
    // To correctly stop and disable a channel, the software clears the EN bit of the DMA_CCRx
    // register.

    match channel {
        DmaChannel::C1 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch1.cr;
                } else {
                    let ccr = &regs.ccr1;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        DmaChannel::C2 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch2.cr;
                } else {
                    let ccr = &regs.ccr2;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        DmaChannel::C3 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch3.cr;
                } else {
                    let ccr = &regs.ccr3;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        DmaChannel::C4 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch4.cr;
                } else {
                    let ccr = &regs.ccr4;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        DmaChannel::C5 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch5.cr;
                } else {
                    let ccr = &regs.ccr5;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C6 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch6.cr;
                } else {
                    let ccr = &regs.ccr6;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C7 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch7.cr;
                } else {
                    let ccr = &regs.ccr7;
                }
            }
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
        #[cfg(any(feature = "l5", feature = "g4"))]
        DmaChannel::C8 => {
            let ccr = &regs.ccr8;
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
    };

    // The software secures that no pending request from the peripheral is served by the
    // DMA controller before the transfer completion.
    // todo?

    // The software waits for the transfer complete or transfer error interrupt.
    // (Handed by calling code)

    // (todo: set ifcr.cficx bit to clear all interrupts?)

    // When a channel transfer error occurs, the EN bit of the DMA_CCRx register is cleared by
    // hardware. This EN bit can not be set again by software to re-activate the channel x, until the
    // TEIFx bit of the DMA_ISR register is set
}

/// Stop a DMA transfer, if in progress.
#[cfg(feature = "h7")]
fn stop_internal<D>(regs: &mut D, channel: DmaChannel)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // L4 RM:
    // Once the software activates a channel, it waits for the completion of the programmed
    // transfer. The DMA controller is not able to resume an aborted active channel with a possible
    // suspended bus transfer.
    // To correctly stop and disable a channel, the software clears the EN bit of the DMA_CCRx
    // register.

    // The software secures that no pending request from the peripheral is served by the
    // DMA controller before the transfer completion.
    // todo?

    let cr = &regs.st[channel as usize].cr;
    cr.modify(|_, w| w.en().clear_bit());
    while cr.read().en().bit_is_set() {}

    // The software waits for the transfer complete or transfer error interrupt.
    // (Handed by calling code)

    // (todo: set ifcr.cficx bit to clear all interrupts?)

    // When a channel transfer error occurs, the EN bit of the DMA_CCRx register is cleared by
    // hardware. This EN bit can not be set again by software to re-activate the channel x, until the
    // TEIFx bit of the DMA_ISR register is set
}

/// Stop a DMA transfer, if in progress.
pub fn stop(periph: DmaPeriph, channel: DmaChannel) {
    match periph {
        DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            stop_internal(&mut regs, channel);
        }
        #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
        DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            stop_internal(&mut regs, channel);
        }
    }
}

fn clear_interrupt_internal<D>(regs: &mut D, channel: DmaChannel, interrupt: DmaInterrupt)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    cfg_if! {
        if #[cfg(any(feature = "g4", feature = "wl"))] {
            regs.ifcr.write(|w| match channel {
                DmaChannel::C1 => match interrupt {
                    DmaInterrupt::TransferError => w.teif1().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif1().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif1().set_bit(),
                }
                DmaChannel::C2 => match interrupt {
                    DmaInterrupt::TransferError => w.teif2().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif2().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif2().set_bit(),
                }
                DmaChannel::C3 => match interrupt {
                    DmaInterrupt::TransferError => w.teif3().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif3().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif3().set_bit(),
                }
                DmaChannel::C4 => match interrupt {
                    DmaInterrupt::TransferError => w.teif4().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif4().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif4().set_bit(),
                }
                DmaChannel::C5 => match interrupt {
                    DmaInterrupt::TransferError => w.teif5().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif5().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif5().set_bit(),
                }
                DmaChannel::C6 => match interrupt {
                    DmaInterrupt::TransferError => w.teif6().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif6().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif6().set_bit(),
                }
                DmaChannel::C7 => match interrupt {
                    DmaInterrupt::TransferError => w.teif7().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif7().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif7().set_bit(),
                }
                #[cfg(not(feature = "wl"))]
                DmaChannel::C8 => match interrupt {
                    DmaInterrupt::TransferError => w.teif8().set_bit(),
                    DmaInterrupt::HalfTransfer => w.htif8().set_bit(),
                    DmaInterrupt::TransferComplete => w.tcif8().set_bit(),
                }
            });
        } else if #[cfg(feature = "h7")] {
            match channel {
                DmaChannel::C0 => match interrupt {
                    DmaInterrupt::TransferError => regs.lifcr.write(|w| w.cteif0().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.lifcr.write(|w| w.chtif0().set_bit()),
                    DmaInterrupt::TransferComplete => regs.lifcr.write(|w| w.ctcif0().set_bit()),
                    DmaInterrupt::DirectModeError => regs.lifcr.write(|w| w.cdmeif0().set_bit()),
                    DmaInterrupt::FifoError => regs.lifcr.write(|w| w.cfeif0().set_bit()),
                }
                DmaChannel::C1 => match interrupt {
                    DmaInterrupt::TransferError => regs.lifcr.write(|w| w.cteif1().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.lifcr.write(|w| w.chtif1().set_bit()),
                    DmaInterrupt::TransferComplete => regs.lifcr.write(|w| w.ctcif1().set_bit()),
                    DmaInterrupt::DirectModeError => regs.lifcr.write(|w| w.cdmeif1().set_bit()),
                    DmaInterrupt::FifoError => regs.lifcr.write(|w| w.cfeif1().set_bit()),
                }
                DmaChannel::C2 => match interrupt {
                    DmaInterrupt::TransferError => regs.lifcr.write(|w| w.cteif2().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.lifcr.write(|w| w.chtif2().set_bit()),
                    DmaInterrupt::TransferComplete => regs.lifcr.write(|w| w.ctcif2().set_bit()),
                    DmaInterrupt::DirectModeError => regs.lifcr.write(|w| w.cdmeif2().set_bit()),
                    DmaInterrupt::FifoError => regs.lifcr.write(|w| w.cfeif2().set_bit()),
                }
                DmaChannel::C3 => match interrupt {
                    DmaInterrupt::TransferError => regs.lifcr.write(|w| w.cteif3().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.lifcr.write(|w| w.chtif3().set_bit()),
                    DmaInterrupt::TransferComplete => regs.lifcr.write(|w| w.ctcif3().set_bit()),
                    DmaInterrupt::DirectModeError => regs.lifcr.write(|w| w.cdmeif3().set_bit()),
                    DmaInterrupt::FifoError => regs.lifcr.write(|w| w.cfeif3().set_bit()),
                }
                DmaChannel::C4 => match interrupt {
                    DmaInterrupt::TransferError => regs.hifcr.write(|w| w.cteif4().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.hifcr.write(|w| w.chtif4().set_bit()),
                    DmaInterrupt::TransferComplete => regs.hifcr.write(|w| w.ctcif4().set_bit()),
                    DmaInterrupt::DirectModeError => regs.hifcr.write(|w| w.cdmeif4().set_bit()),
                    DmaInterrupt::FifoError => regs.hifcr.write(|w| w.cfeif4().set_bit()),
                }
                DmaChannel::C5 => match interrupt {
                    DmaInterrupt::TransferError => regs.hifcr.write(|w| w.cteif5().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.hifcr.write(|w| w.chtif5().set_bit()),
                    DmaInterrupt::TransferComplete => regs.hifcr.write(|w| w.ctcif5().set_bit()),
                    DmaInterrupt::DirectModeError => regs.hifcr.write(|w| w.cdmeif5().set_bit()),
                    DmaInterrupt::FifoError => regs.hifcr.write(|w| w.cfeif5().set_bit()),
                }
                DmaChannel::C6 => match interrupt {
                    DmaInterrupt::TransferError => regs.hifcr.write(|w| w.cteif6().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.hifcr.write(|w| w.chtif6().set_bit()),
                    DmaInterrupt::TransferComplete => regs.hifcr.write(|w| w.ctcif6().set_bit()),
                    DmaInterrupt::DirectModeError => regs.hifcr.write(|w| w.cdmeif6().set_bit()),
                    DmaInterrupt::FifoError => regs.hifcr.write(|w| w.cfeif6().set_bit()),
                }
                DmaChannel::C7 => match interrupt {
                    DmaInterrupt::TransferError => regs.hifcr.write(|w| w.cteif7().set_bit()),
                    DmaInterrupt::HalfTransfer => regs.hifcr.write(|w| w.chtif7().set_bit()),
                    DmaInterrupt::TransferComplete => regs.hifcr.write(|w| w.ctcif7().set_bit()),
                    DmaInterrupt::DirectModeError => regs.hifcr.write(|w| w.cdmeif7().set_bit()),
                    DmaInterrupt::FifoError => regs.hifcr.write(|w| w.cfeif7().set_bit()),
                }
            }
            // todo: G0 PAC 0.14 had a reversion where these flags used to work, but now don't.
        } else if #[cfg(not(feature = "g0"))] {
            regs.ifcr.write(|w| match channel {
                DmaChannel::C1 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif1().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif1().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif1().set_bit(),
                }
                DmaChannel::C2 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif2().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif2().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif2().set_bit(),
                }
                DmaChannel::C3 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif3().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif3().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif3().set_bit(),
                }
                DmaChannel::C4 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif4().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif4().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif4().set_bit(),
                }
                DmaChannel::C5 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif5().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif5().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif5().set_bit(),
                }
                #[cfg(not(feature = "g0"))]
                DmaChannel::C6 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif6().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif6().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif6().set_bit(),
                }
                #[cfg(not(feature = "g0"))]
                DmaChannel::C7 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif7().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif7().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif7().set_bit(),
                }
                #[cfg(any(feature = "l5", feature = "g4"))]
                DmaChannel::C8 => match interrupt {
                    DmaInterrupt::TransferError => w.cteif8().set_bit(),
                    DmaInterrupt::HalfTransfer => w.chtif8().set_bit(),
                    DmaInterrupt::TransferComplete => w.ctcif8().set_bit(),
                }
            });
        }
    }
}

/// Enable an interrupt.
#[cfg(not(feature = "h7"))]
fn enable_interrupt_internal<D>(regs: &mut D, channel: DmaChannel, interrupt: DmaInterrupt)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // Can only be set when the channel is disabled.
    match channel {
        DmaChannel::C1 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch1.cr;
                } else {
                    let ccr = &regs.ccr1;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C2 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch2.cr;
                } else {
                    let ccr = &regs.ccr2;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C3 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch3.cr;
                } else {
                    let ccr = &regs.ccr3;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C4 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch4.cr;
                } else {
                    let ccr = &regs.ccr4;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C5 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch5.cr;
                } else {
                    let ccr = &regs.ccr5;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C6 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch6.cr;
                } else {
                    let ccr = &regs.ccr6;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C7 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch7.cr;
                } else {
                    let ccr = &regs.ccr7;
                }
            }
            enable_interrupt!(ccr, interrupt);
        }
        #[cfg(any(feature = "l5", feature = "g4"))]
        DmaChannel::C8 => {
            let ccr = &regs.ccr8;
            enable_interrupt!(ccr, interrupt);
        }
    };
}

/// Disable an interrupt.
#[cfg(not(feature = "h7"))]
fn disable_interrupt_internal<D>(regs: &mut D, channel: DmaChannel, interrupt: DmaInterrupt)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // Can only be set when the channel is disabled.
    match channel {
        DmaChannel::C1 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch1.cr;
                } else {
                    let ccr = &regs.ccr1;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C2 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch2.cr;
                } else {
                    let ccr = &regs.ccr2;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C3 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch3.cr;
                } else {
                    let ccr = &regs.ccr3;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C4 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch4.cr;
                } else {
                    let ccr = &regs.ccr4;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        DmaChannel::C5 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch5.cr;
                } else {
                    let ccr = &regs.ccr5;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C6 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch6.cr;
                } else {
                    let ccr = &regs.ccr6;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        #[cfg(not(feature = "g0"))]
        DmaChannel::C7 => {
            cfg_if! {
                if #[cfg(any(feature = "f3", feature = "g0"))] {
                    let ccr = &regs.ch7.cr;
                } else {
                    let ccr = &regs.ccr7;
                }
            }
            disable_interrupt!(ccr, interrupt);
        }
        #[cfg(any(feature = "l5", feature = "g4"))]
        DmaChannel::C8 => {
            let ccr = &regs.ccr8;
            disable_interrupt!(ccr, interrupt);
        }
    };
}

#[cfg(feature = "h7")]
fn enable_interrupt_internal<D>(regs: &mut D, channel: DmaChannel, interrupt: DmaInterrupt)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // Can only be set when the channel is disabled.
    let cr = &regs.st[channel as usize].cr;

    match interrupt {
        DmaInterrupt::TransferError => cr.modify(|_, w| w.teie().set_bit()),
        DmaInterrupt::HalfTransfer => cr.modify(|_, w| w.htie().set_bit()),
        DmaInterrupt::TransferComplete => cr.modify(|_, w| w.tcie().set_bit()),
        DmaInterrupt::DirectModeError => cr.modify(|_, w| w.dmeie().set_bit()),
        DmaInterrupt::FifoError => regs.st[channel as usize]
            .fcr
            .modify(|_, w| w.feie().set_bit()),
    }
}

#[cfg(feature = "h7")]
fn disable_interrupt_internal<D>(regs: &mut D, channel: DmaChannel, interrupt: DmaInterrupt)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // Can only be set when the channel is disabled.
    let cr = &regs.st[channel as usize].cr;

    // todo DRY

    match interrupt {
        DmaInterrupt::TransferError => cr.modify(|_, w| w.teie().clear_bit()),
        DmaInterrupt::HalfTransfer => cr.modify(|_, w| w.htie().clear_bit()),
        DmaInterrupt::TransferComplete => cr.modify(|_, w| w.tcie().clear_bit()),
        DmaInterrupt::DirectModeError => cr.modify(|_, w| w.dmeie().clear_bit()),
        DmaInterrupt::FifoError => regs.st[channel as usize]
            .fcr
            .modify(|_, w| w.feie().clear_bit()),
    }
}

/// Enable a specific type of interrupt.
pub fn enable_interrupt(periph: DmaPeriph, channel: DmaChannel, interrupt: DmaInterrupt) {
    match periph {
        DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            enable_interrupt_internal(&mut regs, channel, interrupt);
        }
        #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
        DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            enable_interrupt_internal(&mut regs, channel, interrupt);
        }
    }
}

/// Disable a specific type of interrupt.
pub fn disable_interrupt(periph: DmaPeriph, channel: DmaChannel, interrupt: DmaInterrupt) {
    match periph {
        DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            disable_interrupt_internal(&mut regs, channel, interrupt);
        }
        #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
        DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            disable_interrupt_internal(&mut regs, channel, interrupt);
        }
    }
}

/// Clear an interrupt flag.
pub fn clear_interrupt(periph: DmaPeriph, channel: DmaChannel, interrupt: DmaInterrupt) {
    match periph {
        DmaPeriph::Dma1 => {
            let mut regs = unsafe { &(*DMA1::ptr()) };
            clear_interrupt_internal(&mut regs, channel, interrupt);
        }
        #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
        DmaPeriph::Dma2 => {
            let mut regs = unsafe { &(*pac::DMA2::ptr()) };
            clear_interrupt_internal(&mut regs, channel, interrupt);
        }
    }
}

#[cfg(any(
    feature = "l5",
    feature = "g0",
    feature = "g4",
    feature = "h7",
    feature = "wb",
    feature = "wl",
))]
/// Configure a specific DMA channel to work with a specific peripheral.
pub fn mux(periph: DmaPeriph, channel: DmaChannel, input: DmaInput) {
    // Note: This is similar in API and purpose to `channel_select` above,
    // for different families. We're keeping it as a separate function instead
    // of feature-gating within the same function so the name can be recognizable
    // from the RM etc.

    // G4 example:
    // "The mapping of resources to DMAMUX is hardwired.
    // DMAMUX is used with DMA1 and DMA2:
    // For category 3 and category 4 devices:
    // •
    // DMAMUX channels 0 to 7 are connected to DMA1 channels 1 to 8
    // •
    // DMAMUX channels 8 to 15 are connected to DMA2 channels 1 to 8
    // For category 2 devices:
    // •
    // DMAMUX channels 0 to 5 are connected to DMA1 channels 1 to 6
    // •
    // DMAMUX channels 6 to 11 are connected to DMA2 channels 1 to 6"
    //
    // H723/25/33/35"
    // DMAMUX1 is used with DMA1 and DMA2 in D2 domain
    // •
    // DMAMUX1 channels 0 to 7 are connected to DMA1 channels 0 to 7
    // •
    // DMAMUX1 channels 8 to 15 are connected to DMA2 channels 0 to 7
    // (Note: The H7 and G4 cat 3/4 mappings are the same, except for H7's use of 0-7, and G4's use of 1-8.)

    // todo: With this in mind, some of the mappings below are not correct on some G4 variants.

    unsafe {
        let mux = unsafe { &(*DMAMUX::ptr()) };

        #[cfg(feature = "g4")]
        enable_mux1();

        match periph {
            DmaPeriph::Dma1 => {
                #[cfg(not(feature = "h7"))]
                match channel {
                    // Note the offset by 1, due to mismatch in DMA channels starting at 1, and DMAMUX
                    // channels starting at 0. Ops tested this is correct on G4.
                    DmaChannel::C1 => mux.c0cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C2 => mux.c1cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C3 => mux.c2cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C4 => mux.c3cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C5 => mux.c4cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(not(feature = "g0"))]
                    DmaChannel::C6 => mux.c5cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(not(feature = "g0"))]
                    DmaChannel::C7 => mux.c6cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(any(feature = "l5", feature = "g4"))]
                    DmaChannel::C8 => mux.c7cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                }

                #[cfg(feature = "h7")]
                mux.ccr[channel as usize].modify(|_, w| w.dmareq_id().bits(input as u8));
            }
            #[cfg(not(any(
                all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))),
                feature = "wb"
            )))]
            DmaPeriph::Dma2 => {
                #[cfg(not(feature = "h7"))]
                match channel {
                    DmaChannel::C1 => mux.c8cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C2 => mux.c9cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C3 => mux.c10cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C4 => mux.c11cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    DmaChannel::C5 => mux.c12cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(not(feature = "g0"))]
                    DmaChannel::C6 => mux.c13cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(not(any(feature = "g0", feature = "wb", feature = "wl")))]
                    DmaChannel::C7 => mux.c14cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                    #[cfg(any(feature = "wb", feature = "wl"))]
                    DmaChannel::C7 => (), // Maybe no channel 7 on DMA2 on these platforms.
                    #[cfg(any(feature = "l5", feature = "g4"))]
                    DmaChannel::C8 => mux.c15cr.modify(|_, w| w.dmareq_id().bits(input as u8)),
                }

                #[cfg(feature = "h7")]
                mux.ccr[channel as usize + 8].modify(|_, w| w.dmareq_id().bits(input as u8));
            }
        }
    }
}

#[cfg(feature = "h7")]
/// Configure a specific DMA channel to work with a specific peripheral, on DMAMUX2.
pub fn mux2(periph: DmaPeriph, channel: DmaChannel, input: DmaInput2, mux: &mut DMAMUX2) {
    mux.ccr[channel as usize].modify(|_, w| unsafe { w.dmareq_id().bits(input as u8) });
}

// todo: Enable this for other MCUs as requried
/// Enable the DMA mux RCC clock. Applicable to some variants, but no others. (H7 and G0 don't use it,
/// for example)
#[cfg(any(feature = "g4", feature = "wb"))]
pub fn enable_mux1() {
    let rcc = unsafe { &(*RCC::ptr()) };

    cfg_if! {
        if #[cfg(feature = "g4")] {
            // Note inconsistency between `dmamux` and `dmamux`; can't use macro here.
            rcc.ahb1enr.modify(|_, w| w.dmamuxen().set_bit());
            rcc.ahb1rstr.modify(|_, w| w.dmamux1rst().set_bit());
            rcc.ahb1rstr.modify(|_, w| w.dmamux1rst().clear_bit());
        } else {
            rcc_en_reset!(ahb1, dmamux, rcc);
        }
    }
}

#[cfg(feature = "l4")] // Only required on L4
/// Select which peripheral on a given channel we're using.
/// See L44 RM, Table 41.
pub(crate) fn channel_select<D>(regs: &mut D, input: DmaInput)
where
    D: Deref<Target = dma1::RegisterBlock>,
{
    // todo: Allow selecting channels in pairs to save a write.
    let val = input.dma1_channel_select();
    regs.cselr.modify(|_, w| match input.dma1_channel() {
        DmaChannel::C1 => w.c1s().bits(val),
        DmaChannel::C2 => w.c2s().bits(val),
        DmaChannel::C3 => w.c3s().bits(val),
        DmaChannel::C4 => w.c4s().bits(val),
        DmaChannel::C5 => w.c5s().bits(val),
        DmaChannel::C6 => w.c6s().bits(val),
        DmaChannel::C7 => w.c7s().bits(val),
    });
}

// todo: Code below is for experimental struct-per-channel API
macro_rules! make_chan_struct {
    // ($Periph:ident, $PERIPH:ident, $periph:ident, $ch:expr) => {
    ($periph: expr, $ch:expr) => {
        paste! {
            /// Experimental/WIP channel-based DMA struct.
            pub struct [<Dma $periph Ch $ch>] {
                // #[cfg(feature = "h7")]
                // regs: dma1::st // todo?
            }

            impl [<Dma $periph Ch $ch>] {
                /// Initialize a DMA peripheral, including enabling and resetting
                /// its RCC peripheral clock.
                /// Note that the clock may have already been enabled by a different channel's
                /// constructor.
                pub fn new() -> Self {
                    // todo: Enable RCC for DMA 2 etc!
                    let rcc = unsafe { &(*RCC::ptr()) };
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            rcc.ahbenr.modify(|_, w| w.dma1en().set_bit()); // no dmarst on F3.
                        } else if #[cfg(feature = "g0")] {
                            rcc_en_reset!(ahb1, dma, rcc);
                        } else {
                            rcc_en_reset!(ahb1, [<dma $periph>], rcc);
                        }
                    }

                    Self { }
                }

                fn regs(&self) -> &[<dma $periph>]::RegisterBlock {
                    unsafe { &(*[<DMA $periph>]::ptr())}
                }

                #[cfg(feature = "h7")]
                fn ccr(&self) -> &[<dma $periph>]::st::CR {
                // fn ccr(&self) -> &u8 {
                    &self.regs().st[$ch].cr
                }

                #[cfg(not(any(feature = "h7", feature = "f3", feature = "g0")))]
                fn ccr(&self) -> &[<dma $periph>]::[<CCR $ch>] {
                    &self.regs().[<ccr $ch>]
                }

                #[cfg(any(feature = "f3", feature = "g0"))]
                // fn ccr(&self) -> &[<dma $periph>]::ch::cr {
                 fn ccr(&self) -> i8 {
                    &self.regs().[<ch $ch>].cr
                }

                #[cfg(not(feature = "h7"))] // due to num_data size diff
                /// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
                /// interrupt. Note that this fn has been (perhaps) depreciated by the standalone fn.
                pub fn cfg_channel(
                    &mut self,
                    periph_addr: u32,
                    mem_addr: u32,
                    num_data: u16,
                    direction: Direction,
                    periph_size: DataSize,
                    mem_size: DataSize,
                    cfg: ChannelCfg,
                ) {
                    cfg_channel(
                        &mut self.regs(),
                        DmaChannel::[<C $ch>],
                        periph_addr,
                        mem_addr,
                        num_data,
                        direction,
                        periph_size,
                        mem_size,
                        cfg,
                    )
                }

                #[cfg(feature = "h7")]
                /// Configure a DMA channel. See L4 RM 0394, section 11.4.4. Sets the Transfer Complete
                /// interrupt. Note that this fn has been (perhaps) depreciated by the standalone fn.
                pub fn cfg_channel(
                    &mut self,
                    periph_addr: u32,
                    mem_addr: u32,
                    num_data: u32,
                    direction: Direction,
                    periph_size: DataSize,
                    mem_size: DataSize,
                    cfg: ChannelCfg,
                ) {
                    cfg_channel(
                        &mut self.regs(),
                        DmaChannel::[<C $ch>],
                        periph_addr,
                        mem_addr,
                        num_data,
                        direction,
                        periph_size,
                        mem_size,
                        cfg,
                    )
                }

                /// Stop a DMA transfer, if in progress.
                pub fn stop(&mut self) {
                    let ccr = self.ccr();

                    ccr.modify(|_, w| w.en().clear_bit());
                    while ccr.read().en().bit_is_set() {}
                }

                /// Enable a specific type of interrupt.
                pub fn enable_interrupt(&mut self, interrupt: DmaInterrupt) {
                    enable_interrupt_internal(&mut self.regs(), DmaChannel::[<C $ch>], interrupt);
                }

                /// Clear an interrupt flag.
                pub fn clear_interrupt(&mut self, interrupt: DmaInterrupt) {
                    clear_interrupt_internal(&mut self.regs(), DmaChannel::[<C $ch>], interrupt);
                }
                // todo: Other methods.
            }
        }
    };
}

// todo: As above, you may need more feature-gating, esp on
// todo DMA2.
// Note: G0 is limited, eg for some variants only up to DMA1, ch5.
cfg_if! {
    if #[cfg(not(any(feature = "f3", feature = "g0")))] {
        #[cfg(feature = "h7")]
        make_chan_struct!(1, 0);
        make_chan_struct!(1, 1);
        make_chan_struct!(1, 2);
        make_chan_struct!(1, 3);
        make_chan_struct!(1, 4);
        make_chan_struct!(1, 5);
        #[cfg(not(feature = "g0"))]
        make_chan_struct!(1, 6);
        #[cfg(not(feature = "g0"))]
        make_chan_struct!(1, 7);
        #[cfg(any(feature = "l5", feature = "g4"))]
        make_chan_struct!(1, 8);

        #[cfg(feature = "h7")]
        make_chan_struct!(2, 0);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 1);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 2);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 3);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 4);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 5);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 6);
        #[cfg(not(any(feature = "g0", feature = "wb")))]
        make_chan_struct!(2, 7);
        #[cfg(any(feature = "l5", feature = "g4"))]
        make_chan_struct!(2, 8);
    }
}
