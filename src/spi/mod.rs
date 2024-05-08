//! Support for the Serial Peripheral Interface (SPI) bus peripheral.
//! Provides APIs to configure, read, and write from
//! SPI, with blocking, nonblocking, and DMA functionality.

use core::{ops::Deref, ptr};

cfg_if::cfg_if! {
    if #[cfg(any(feature = "h5", feature = "h7"))] {
        mod h;
        pub use h::*;
    } else {
        mod baseline;
        pub use baseline::*;
    }
}

use cfg_if::cfg_if;

use crate::{pac, util::RccPeriph};

cfg_if! {
    if #[cfg(all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))))] {
        use crate::pac::dma as dma_p;
        use crate::pac::DMA as DMA1;
    } else {
        use crate::pac::dma1 as dma_p;
        use crate::pac::DMA1;
    }
}

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;
#[cfg(not(any(feature = "f4", feature = "l552")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel}; // todo temp

#[macro_export]
macro_rules! check_errors {
    ($sr:expr) => {
        #[cfg(feature = "h7")]
        let crc_error = $sr.crce().bit_is_set();
        #[cfg(not(feature = "h7"))]
        let crc_error = $sr.crcerr().bit_is_set();

        if $sr.ovr().bit_is_set() {
            return Err(SpiError::Overrun);
        } else if $sr.modf().bit_is_set() {
            return Err(SpiError::ModeFault);
        } else if crc_error {
            return Err(SpiError::Crc);
        }
    };
}

/// SPI error
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub enum SpiError {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    Hardware,
    DuplexFailed, // todo temp?
}

/// Set the factor to divide the APB clock by to set baud rate. Sets `SPI_CR1` register, `BR` field.
/// On H7, sets CFG1 register, `MBR` field.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum BaudRate {
    Div2 = 0b000,
    Div4 = 0b001,
    Div8 = 0b010,
    Div16 = 0b011,
    Div32 = 0b100,
    Div64 = 0b101,
    Div128 = 0b110,
    Div256 = 0b111,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// FIFO reception threshold Sets `SPI_CR2` register, `FRXTH` field.
pub enum ReceptionThresh {
    /// RXNE event is generated if the FIFO level is greater than or equal to 1/2 (16-bit)
    D16 = 0,
    /// RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
    D8 = 1,
}

#[derive(Clone, Copy, PartialEq)]
/// Select the duplex communication mode between the 2 devices. Sets `CR1` register, `BIDIMODE`,
/// and `RXONLY` fields.
pub enum SpiCommMode {
    FullDuplex,
    HalfDuplex,
    /// Simplex Transmit only. (Cfg same as Full Duplex, but ignores input)
    TransmitOnly,
    /// Simplex Receive only.
    ReceiveOnly,
}

#[derive(Clone, Copy, PartialEq)]
/// Used for managing NSS / CS pin. Sets CR1 register, SSM field.
/// On H7, sets CFG2 register, `SSOE` field.
pub enum SlaveSelect {
    ///  In this configuration, slave select information
    /// is driven internally by the SSI bit value in register SPIx_CR1. The external NSS pin is
    /// free for other application uses.
    Software,
    /// This configuration is only used when the
    /// MCU is set as master. The NSS pin is managed by the hardware. The NSS signal
    /// is driven low as soon as the SPI is enabled in master mode (SPE=1), and is kept
    /// low until the SPI is disabled (SPE =0). A pulse can be generated between
    /// continuous communications if NSS pulse mode is activated (NSSP=1). The SPI
    /// cannot work in multimaster configuration with this NSS setting.
    HardwareOutEnable,
    /// If the microcontroller is acting as the
    /// master on the bus, this configuration allows multimaster capability. If the NSS pin
    /// is pulled low in this mode, the SPI enters master mode fault state and the device is
    /// automatically reconfigured in slave mode. In slave mode, the NSS pin works as a
    /// standard “chip select” input and the slave is selected while NSS line is at low level.
    HardwareOutDisable,
}

cfg_if! {
    if #[cfg(feature = "embedded_hal")] {
        type SpiModeType = embedded_hal::spi::Mode;
    } else {
        #[derive(Clone, Copy)]
        #[repr(u8)]
        /// Clock polarity. Sets CFGR2 register, CPOL field. Stored in the config as a field of `SpiMode`.
        pub enum SpiPolarity {
            /// Clock signal low when idle
            IdleLow = 0,
            /// Clock signal high when idle
            IdleHigh = 1,
        }

        #[derive(Clone, Copy)]
        #[repr(u8)]
        /// Clock phase. Sets CFGR2 register, CPHA field. Stored in the config as a field of `SpiMode`.
        pub enum SpiPhase {
            /// Data in "captured" on the first clock transition
            CaptureOnFirstTransition = 0,
            /// Data in "captured" on the second clock transition
            CaptureOnSecondTransition = 1,
        }

        #[derive(Clone, Copy)]
        /// SPI mode. Sets CFGR2 reigster, CPOL and CPHA fields.
        pub struct SpiMode {
            /// Clock polarity
            pub polarity: SpiPolarity,
            /// Clock phase
            pub phase: SpiPhase,
        }

        impl SpiMode {
            /// Set Spi Mode 0: Idle low, capture on first transition.
            /// Data sampled on rising edge and shifted out on the falling edge
            pub fn mode0() -> Self {
                Self {
                    polarity: SpiPolarity::IdleLow,
                    phase: SpiPhase::CaptureOnFirstTransition,
                }
            }

            /// Set Spi Mode 1: Idle low, capture on second transition.
            /// Data sampled on the falling edge and shifted out on the rising edge
            pub fn mode1() -> Self {
                Self {
                    polarity: SpiPolarity::IdleLow,
                    phase: SpiPhase::CaptureOnSecondTransition,
                }
            }

            /// Set Spi Mode 2: Idle high, capture on first transition.
            /// Data sampled on the rising edge and shifted out on the falling edge
            pub fn mode2() -> Self {
                Self {
                    polarity: SpiPolarity::IdleHigh,
                    phase: SpiPhase::CaptureOnFirstTransition,
                }
            }

            /// Set Spi Mode 3: Idle high, capture on second transition.
            /// Data sampled on the falling edge and shifted out on the rising edge
            pub fn mode3() -> Self {
                Self {
                    polarity: SpiPolarity::IdleHigh,
                    phase: SpiPhase::CaptureOnSecondTransition,
                }
            }
        }

        type SpiModeType = SpiMode;
    }
}

#[derive(Clone)]
/// Configuration data for SPI.
pub struct SpiConfig {
    /// SPI mode associated with Polarity and Phase. Defaults to Mode0: Idle low, capture on first transition.
    pub mode: SpiModeType,
    /// Sets the (duplex) communication mode between the devices. Defaults to full duplex.
    pub comm_mode: SpiCommMode,
    /// Controls use of hardware vs software CS/NSS pin. Defaults to software.
    pub slave_select: SlaveSelect,
    /// Data size. Defaults to 8 bits.
    pub data_size: DataSize,
    /// FIFO reception threshhold. Defaults to 8 bits.
    pub fifo_reception_thresh: ReceptionThresh,
    // pub cs_delay: f32,
    // pub swap_miso_mosi: bool,
    // pub suspend_when_inactive: bool,
}

impl Default for SpiConfig {
    fn default() -> Self {
        cfg_if! {
            if #[cfg(feature = "embedded_hal")] {
                let mode0 = embedded_hal::spi::MODE_0;
            } else {
                let mode0 = SpiModeType::mode0();
            }
        }

        Self {
            mode: mode0,
            comm_mode: SpiCommMode::FullDuplex,
            slave_select: SlaveSelect::Software,
            data_size: DataSize::D8,
            fifo_reception_thresh: ReceptionThresh::D8,
        }
    }
}

/// Represents a Serial Peripheral Interface (SPI) peripheral.
pub struct Spi<R> {
    pub regs: R,
    pub cfg: SpiConfig,
}

impl<R> Spi<R>
where
    R: Deref<Target = pac::spi1::RegisterBlock> + RccPeriph,
{
    /// Stop a DMA transfer. Stops the channel, and disables the `txdmaen` and `rxdmaen` bits.
    /// Run this after each transfer completes - you may wish to do this in an interrupt
    /// (eg DMA transfer complete) instead of blocking. `channel2` is an optional second channel
    /// to stop; eg if you have both a tx and rx channel.
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub fn stop_dma(
        &mut self,
        channel: DmaChannel,
        channel2: Option<DmaChannel>,
        dma_periph: dma::DmaPeriph,
    ) {
        // (RM:) To close communication it is mandatory to follow these steps in order:
        // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.

        dma::stop(dma_periph, channel);
        if let Some(ch2) = channel2 {
            dma::stop(dma_periph, ch2);
        };

        // 2. Disable the SPI by following the SPI disable procedure:
        // self.disable();

        // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
        // SPI_CR2 register, if DMA Tx and/or DMA Rx are used.

        #[cfg(not(feature = "h7"))]
        self.regs.cr2.modify(|_, w| {
            w.txdmaen().clear_bit();
            w.rxdmaen().clear_bit()
        });

        #[cfg(feature = "h7")]
        self.regs.cfg1.modify(|_, w| {
            w.txdmaen().clear_bit();
            w.rxdmaen().clear_bit()
        });
    }

    /// Convenience function that clears the interrupt, and stops the transfer. For use with the TC
    /// interrupt only.
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub fn cleanup_dma(
        &mut self,
        dma_periph: dma::DmaPeriph,
        channel_tx: DmaChannel,
        channel_rx: Option<DmaChannel>,
    ) {
        // The hardware seems to automatically enable Tx too; and we use it when transmitting.
        dma::clear_interrupt(dma_periph, channel_tx, dma::DmaInterrupt::TransferComplete);

        if let Some(ch_rx) = channel_rx {
            dma::clear_interrupt(dma_periph, ch_rx, dma::DmaInterrupt::TransferComplete);
        }

        self.stop_dma(channel_tx, channel_rx, dma_periph);
    }

    /// Print the (raw) contents of the status register.
    pub fn read_status(&self) -> u32 {
        unsafe { self.regs.sr.read().bits() }
    }
}
