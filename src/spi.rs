//! Support for the Serial Peripheral Interface (SPI) bus peripheral.
//! Provides APIs to configure, read, and write from
//! SPI, with blocking, nonblocking, and DMA functionality.

use core::{ops::Deref, ptr};

#[cfg(feature = "embedded_hal")]
use embedded_hal::spi::FullDuplex;

use crate::{
    pac::{self, RCC},
    util::RccPeriph,
    MAX_ITERS,
};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(all(feature = "g0", not(any(feature = "g0b1", feature = "g0c1"))))] {
        use crate::pac::dma as dma_p;
        use crate::pac::DMA as DMA1;
    } else {
        use crate::pac::dma1 as dma_p;
        use crate::pac::DMA1;
    }
}

#[cfg(not(any(feature = "f4", feature = "l552")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

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
}

/// Possible interrupt types. Enable these in SPIx_CR2. Check and clear with SR. There is no explicit
/// way to clear these.
#[cfg(not(feature = "h7"))]
#[derive(Copy, Clone)]
pub enum SpiInterrupt {
    /// Tx buffer empty (TXEIE)
    TxBufEmpty,
    /// Rx buffer not empty (RXNEIE)
    RxBufNotEmpty,
    /// Error (ERRIE)
    Error,
}

/// Possible interrupt types. Enable these in SPIx_IER. Check with SR. Clear with IFCR
#[cfg(feature = "h7")]
#[derive(Copy, Clone)]
pub enum SpiInterrupt {
    /// Additional number of transactions reload interrupt enable (TSERFIE)
    NumberOfTransactionsReload,
    /// Mode fault (MODFIE)
    ModeFault,
    /// TIFRE (TIFREIE)
    Tifre,
    /// CRC error (CRCEIE)
    CrcError,
    /// Overrun (OVRIE)
    Overrun,
    /// Underrun (UNDRIE)
    Underrun,
    /// TXTFIE
    Txtfie,
    /// EOT, SUSP, and TXC (EOTIE)
    EotSuspTxc,
    /// DXP (TXPIE)
    Dxp,
    /// TXP (TXPIE)
    Txp,
    /// RXP (RXPIE)
    Rxp,
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

/// These bits configure the data length for SPI transfers. Sets `SPI_CR2` register, `DS` field.
#[cfg(not(feature = "h7"))]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum DataSize {
    D4 = 0b0011,
    D5 = 0b0100,
    D6 = 0b0101,
    D7 = 0b0110,
    D8 = 0b0111,
    D9 = 0b1000,
    D10 = 0b1001,
    D11 = 0b1010,
    D12 = 0b1011,
    D13 = 0b1100,
    D14 = 0b1101,
    D15 = 0b1110,
    D16 = 0b1111,
}

/// Number of bits in at single SPI data frame. Sets `CFGR1` register, `DSIZE` field.
#[cfg(feature = "h7")]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum DataSize {
    D4 = 3,
    D5 = 4,
    D6 = 5,
    D7 = 6,
    D8 = 7,
    D9 = 8,
    D10 = 9,
    D11 = 10,
    D12 = 11,
    D13 = 12,
    D14 = 13,
    D15 = 14,
    D16 = 15,
    D17 = 16,
    D18 = 17,
    D19 = 18,
    D20 = 19,
    D21 = 20,
    D22 = 21,
    D23 = 22,
    D24 = 23,
    D25 = 24,
    D26 = 25,
    D27 = 26,
    D28 = 27,
    D29 = 28,
    D30 = 29,
    D31 = 30,
    D32 = 31,
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
    /// Initialize an SPI peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, cfg: SpiConfig, baud_rate: BaudRate) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        R::en_reset(rcc);

        cfg_if! {
            if #[cfg(feature = "h7")] {
                // H743 RM, section 50.4.8: Configuration of SPI.
                // 1. Write the proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
                // ( Handled in application code)

                // 2. Write to the SPI_CFG1 and SPI_CFG2 registers to set up proper values of all not
                // reserved bits and bit fields included there with next exceptions:
                // a) SSOM, SSOE, MBR[2:0], MIDI[3:0] and MSSI[3:0] are required at master mode
                // only, the MSSI bits take effect when SSOE is set, MBR setting is required for slave
                // at TI mode, too
                // b) UDRDET[1:0] and UDRCFG[1:0] are required at slave mode only,
                // c) CRCSIZE[4:0] is required if CRCEN is set,
                // d) CPOL, CPHA, LSBFRST, SSOM, SSOE, SSIOP and SSM are not required at TI
                // mode.
                // e) Once the AFCNTR bit is set at SPI_CFG2 register, all the SPI outputs start to be
                // propagated onto the associated GPIO pins regardless the peripheral enable so
                // any later configurations changes of the SPI_CFG1 and SPI_CFG2 registers can
                // affect level of signals at these pins.
                // f) The I2SMOD bit at SPI_I2SCFGR register has to be kept cleared to prevent any
                // unexpected influence of occasional I2S configuration.

                // [St forum thread on how to set up SPI in master mode avoiding mode faults:
                // https://community.st.com/s/question/0D50X0000AFrHS6SQN/stm32h7-what-is-the-proper-
                // way-to-make-spi-work-in-master-mode
                regs.cr1.modify(|_, w| {
                    w.ssi().bit(cfg.slave_select == SlaveSelect::Software)
                });

                regs.cfg1.modify(|_, w| {
                    w.mbr().bits(baud_rate as u8);
                    w.dsize().bits(cfg.data_size as u8);
                    w.crcen().clear_bit()
                });

                regs.cfg2.modify(|_, w| {
                    w.cpol().bit(cfg.mode.polarity as u8 != 0);
                    w.cpha().bit(cfg.mode.phase as u8 != 0);
                    w.master().set_bit();
                    w.ssm().bit(cfg.slave_select == SlaveSelect::Software);
                    w.ssoe().bit(cfg.slave_select != SlaveSelect::Software);
                    w.comm().bits(0b00) // Full-duplex mode
                    // w.comm().lsbfrst().clear_bit() // MSB first
                    // w.ssoe().bit(cfg.slave_select != SlaveSelect::Software)
                });

                // todo: You may not need this master line separate. TSing SS config issues.
                // regs.cfg2.modify(|_, w| w.master().set_bit());

                // todo: CR2.tsize should be > 0.


                // 3. Write to the SPI_CR2 register to select length of the transfer, if it is not known TSIZE
                // has to be programmed to zero.
                regs.cr2.modify(|_, w| {
                    w.tsize().bits(0) // todo?
                });

                // 4. Write to SPI_CRCPOLY and into TCRCINI, RCRCINI and CRC33_17 bits at
                // SPI2S_CR1 register to configure the CRC polynomial and CRC calculation if needed.

                // 5. Configure DMA streams dedicated for the SPI Tx and Rx in DMA registers if the DMA
                // streams are used (see chapter Communication using DMA).

                // 6. Program the IOLOCK bit in the SPI_CFG1 register if the configuration protection is
                // required (for safety).

                regs.cr1.modify(|_, w| {
                    w.spe().set_bit()
                });
            } else {
                // L44 RM, section 40.4.7: Configuration of SPI
                // The configuration procedure is almost the same for master and slave. For specific mode
                // setups, follow the dedicated sections. When a standard communication is to be initialized,
                // perform these steps:

                // 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
                // (Handled in GPIO modules and user code)

                // 2. Write to the SPI_CR1 register:
                regs.cr1.modify(|_, w| unsafe {
                    // a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 4)
                    w.br().bits(baud_rate as u8);
                    // b) Configure the CPOL and CPHA bits combination to define one of the four
                    // relationships between the data transfer and the serial clock (CPHA must be
                    // cleared in NSSP mode). (Note: 2 - except the case when CRC is enabled at TI
                    // mode).
                    w.cpol().bit(cfg.mode.polarity as u8 != 0);
                    w.cpha().bit(cfg.mode.phase as u8 != 0);
                    // c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
                    // BIDIOE (RXONLY and BIDIMODE can't be set at the same time).
                    w.bidimode().bit(cfg.comm_mode == SpiCommMode::HalfDuplex);
                    w.rxonly().bit(cfg.comm_mode == SpiCommMode::ReceiveOnly);
                    // d) Configure the LSBFIRST bit to define the frame format (Note: 2).
                    w.lsbfirst().clear_bit();
                    // e) Configure the CRCL and CRCEN bits if CRC is needed (while SCK clock signal is
                    // at idle state).
                    w.crcen().clear_bit();
                    // f) Configure SSM and SSI (Notes: 2 & 3).
                    w.ssm().bit(cfg.slave_select == SlaveSelect::Software);
                    w.ssi().bit(cfg.slave_select == SlaveSelect::Software);
                    // g) Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on
                    // NSS if master is configured to prevent MODF error).
                    w.mstr().set_bit();
                    w.spe().set_bit() // Enable SPI
                });

                // 3. Write to SPI_CR2 register:
                #[cfg(feature = "f4")]
                regs.cr2.modify(|_, w| w.ssoe().bit(cfg.slave_select == SlaveSelect::HardwareOutEnable));

                #[cfg(not(feature = "f4"))]
                regs.cr2
                    .modify(|_, w| unsafe {
                        // a) Configure the DS[3:0] bits to select the data length for the transfer.
                        w.ds().bits(cfg.data_size as u8);
                        // b) Configure SSOE (Notes: 1 & 2 & 3).
                        w.ssoe().bit(cfg.slave_select == SlaveSelect::HardwareOutEnable);
                        // e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the read
                        // access size for the SPIx_DR register.
                        w.frxth().bit(cfg.fifo_reception_thresh as u8 != 0)
                    });

                // c) Set the FRF bit if the TI protocol is required (keep NSSP bit cleared in TI mode).
                // d) Set the NSSP bit if the NSS pulse mode between two data units is required (keep
                // CHPA and TI bits cleared in NSSP mode).

                // f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
                // 4. Write to SPI_CRCPR register: Configure the CRC polynomial if needed.
                // 5. Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in
                // DMA registers if the DMA streams are used.
            }

            // todo: It sounds like you should enable and disable spi during writes, not on init!
            // todo: This lets you use hardware CS management, and seems to be teh way the RM
            // todo steers you towards regardless.
        }
        Self { regs, cfg }
    }

    /// Change the SPI baud rate.
    pub fn reclock(&mut self, baud_rate: BaudRate) {
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        #[cfg(not(feature = "h7"))]
        self.regs.cr1.modify(|_, w| unsafe {
            w.br().bits(baud_rate as u8);
            w.spe().set_bit()
        });

        #[cfg(any(feature = "h5", feature = "h7"))]
        self.regs
            .cfg1
            .modify(|_, w| unsafe { w.mbr().bits(baud_rate as u8) });

        #[cfg(any(feature = "h5", feature = "h7"))]
        self.regs.cr1.modify(|_, w| w.spe().set_bit());
    }

    /// L44 RM, section 40.4.9: "Procedure for disabling the SPI"
    /// When SPI is disabled, it is mandatory to follow the disable procedures described in this
    /// paragraph. It is important to do this before the system enters a low-power mode when the
    /// peripheral clock is stopped. Ongoing transactions can be corrupted in this case. In some
    /// modes the disable procedure is the only way to stop continuous communication running.
    pub fn disable(&mut self) {
        // The correct disable procedure is (except when receive only mode is used):
        cfg_if! {
            if #[cfg(feature = "h7")] {
                // 1. Wait until TXC=1 and/or EOT=1 (no more data to transmit and last data frame sent).
                // When CRC is used, it is sent automatically after the last data in the block is processed.
                // TXC/EOT is set when CRC frame is completed in this case. When a transmission is
                // suspended the software has to wait till CSTART bit is cleared.
                while self.regs.sr.read().txc().bit_is_clear() {}
                while self.regs.sr.read().eot().bit_is_clear() {}
                // 2. Read all RxFIFO data (until RXWNE=0 and RXPLVL=00)
                while self.regs.sr.read().rxwne().bit_is_set() || self.regs.sr.read().rxplvl().bits() != 0  {
                    unsafe { ptr::read_volatile(&self.regs.rxdr as *const _ as *const u8) };
                }
                // 3. Disable the SPI (SPE=0).
                self.regs.cr1.modify(|_, w| w.spe().clear_bit());
            } else {
                 // 1. Wait until FTLVL[1:0] = 00 (no more data to transmit).
                #[cfg(not(feature = "f4"))]
                while self.regs.sr.read().ftlvl().bits() != 0 {}
                // 2. Wait until BSY=0 (the last data frame is processed).
                while self.regs.sr.read().bsy().bit_is_set() {}
                // 3. Disable the SPI (SPE=0).
                // todo: Instructions say to stop SPI (including to close DMA comms), but this breaks non-DMA writes, which assume
                // todo SPI is enabled, the way we structure things.
                self.regs.cr1.modify(|_, w| w.spe().clear_bit());
                // 4. Read data until FRLVL[1:0] = 00 (read all the received data).
                #[cfg(not(feature = "f4"))]
                while self.regs.sr.read().frlvl().bits() != 0 {
                    unsafe { ptr::read_volatile(&self.regs.dr as *const _ as *const u8) };
                }
            }
        }
    }

    /// Read a single byte if available, or block until it's available.
    #[cfg(not(any(feature = "h5", feature = "h7")))]
    pub fn read(&mut self) -> Result<u8, SpiError> {
        let sr = self.regs.sr.read();
        let crce = sr.crcerr().bit_is_set();

        if sr.ovr().bit_is_set() {
            return Err(SpiError::Overrun);
        } else if sr.modf().bit_is_set() {
            return Err(SpiError::ModeFault);
        } else if crce {
            return Err(SpiError::Crc);
        }

        let mut i = 0;
        while !self.regs.sr.read().rxne().bit_is_set() {
            i += 1;
            if i >= MAX_ITERS {
                return Err(SpiError::Hardware);
            }
        }

        Ok(unsafe { ptr::read_volatile(&self.regs.dr as *const _ as *const u8) })
    }

    /// Write a single byte if available, or block until it's available.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    #[cfg(not(any(feature = "h5", feature = "h7")))]
    pub fn write_one(&mut self, byte: u8) -> Result<(), SpiError> {
        let sr = self.regs.sr.read();
        let crce = sr.crcerr().bit_is_set();

        if sr.ovr().bit_is_set() {
            return Err(SpiError::Overrun);
        } else if sr.modf().bit_is_set() {
            return Err(SpiError::ModeFault);
        } else if crce {
            return Err(SpiError::Crc);
        }

        let mut i = 0;
        while !self.regs.sr.read().txe().bit_is_set() {
            i += 1;
            if i >= MAX_ITERS {
                return Err(SpiError::Hardware);
            }
        }

        unsafe { ptr::write_volatile(&self.regs.dr as *const _ as *mut u8, byte) };

        Ok(())
    }

    /// Write multiple bytes on the SPI line, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    #[cfg(not(any(feature = "h5", feature = "h7")))]
    pub fn write(&mut self, words: &[u8]) -> Result<(), SpiError> {
        for word in words {
            self.write_one(*word)?;
            self.read()?;
        }

        Ok(())
    }

    /// Read multiple bytes to a buffer, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    #[cfg(not(any(feature = "h5", feature = "h7")))]
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<(), SpiError> {
        for word in words.iter_mut() {
            self.write_one(*word)?;
            *word = self.read()?;
        }

        Ok(())
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    fn read_one(&mut self) -> Result<u8, SpiError> {
        // NOTE(read_volatile) read only 1 word
        unsafe { Ok(ptr::read_volatile(&self.regs.rxdr as *const _ as *const u8)) }

        Ok(())
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    fn send(&mut self, word: u8) -> Result<(), SpiError> {
        // NOTE(write_volatile) see note above
        unsafe {
            ptr::write_volatile(&self.regs.txdr as *const _ as *mut u8, word);
        }
        // write CSTART to start a transaction in
        // master mode
        self.regs.cr1.modify(|_, w| w.cstart().set_bit());

        Ok(())
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    fn exchange_duplex(&mut self, word: u8) -> Result<u8, SpiError> {
        // todo DRY
        let sr = self.regs.sr.read();

        let crce = sr.crce().bit_is_set();

        if sr.ovr().bit_is_set() {
            return Err(SpiError::Overrun);
        } else if sr.modf().bit_is_set() {
            return Err(SpiError::ModeFault);
        } else if crce {
            return Err(SpiError::Crc);
        }

        unsafe {
            ptr::write_volatile(&self.regs.txdr as *const _ as *mut _, word);
            Ok(ptr::read_volatile(&self.regs.rxdr as *const _ as *const u8))
        }
        //
        // { // else if sr.txc().is_completed() {
        //     txc, is_completed,
        //     {
        //         let sr = self.regs.sr.read(); // Read SR again on a subsequent PCLK cycle
        //
        //         if sr.txc().is_completed() && !sr.rxp().is_not_empty() {
        //             // The Tx FIFO completed, but no words were
        //             // available in the Rx FIFO. This is a duplex failure
        //             nb::Error::Other(Error::DuplexFailed)
        //         } else {
        //             nb::Error::WouldBlock
        //         }
        //     }
        // }

        // Ok(())
    }

    /// Internal implementation for reading a word
    ///
    /// * Assumes the transaction has started (CSTART handled externally)
    /// * Assumes at least one word has already been written to the Tx FIFO
    #[cfg(any(feature = "h5", feature = "h7"))]
    fn read_duplex(&mut self) -> Result<u8, SpiError> {
        // NOTE(read_volatile) read only 1 word
        // todo DRY
        let sr = self.regs.sr.read();

        let crce = sr.crce().bit_is_set();

        if sr.ovr().bit_is_set() {
            return Err(SpiError::Overrun);
        } else if sr.modf().bit_is_set() {
            return Err(SpiError::ModeFault);
        } else if crce {
            return Err(SpiError::Crc);
        }

        unsafe { Ok(ptr::read_volatile(&self.regs.rxdr as *const _ as *const u8)) }
        // , { // else if sr.txc().is_completed()
        //         txc, is_completed,
        //         {
        //             let sr = self.regs.sr.read(); // Read SR again on a subsequent PCLK cycle
        //
        //             if sr.txc().is_completed() && !sr.rxp().is_not_empty() {
        //                 // The Tx FIFO completed, but no words were
        //                 // available in the Rx FIFO. This is a duplex failure
        //                 nb::Error::Other(Error::DuplexFailed)
        //             } else {
        //                 nb::Error::WouldBlock
        //             }
        //         }
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    pub fn write<'w>(&mut self, write_words: &'w [u8]) -> Result<(), SpiError> {
        // Depth of FIFO to use. All current SPI implementations
        // have a FIFO depth of at least 8 (see RM0433 Rev 7
        // Table 409.) but pick 4 as a conservative value.
        const FIFO_WORDS: usize = 4; // todo: 8?

        // Fill the first half of the write FIFO
        let len = write_words.len();
        let mut write = write_words.iter();
        for _ in 0..core::cmp::min(FIFO_WORDS, len) {
            self.send(*write.next().unwrap())?;
        }

        // Continue filling write FIFO and emptying read FIFO
        for word in write {
            let _ = self.exchange_duplex(*word)?;
        }

        // Dummy read from the read FIFO
        for _ in 0..core::cmp::min(FIFO_WORDS, len) {
            let _ = self.read_duplex()?;
        }

        Ok(())
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<(), SpiError> {
        // Depth of FIFO to use. All current SPI implementations
        // have a FIFO depth of at least 8 (see RM0433 Rev 7
        // Table 409.) but pick 4 as a conservative value.
        const FIFO_WORDS: usize = 4; // todo: 8?

        // Fill the first half of the write FIFO
        let len = words.len();
        for i in 0..core::cmp::min(FIFO_WORDS, len) {
            self.send(words[i])?;
        }

        for i in FIFO_WORDS..len + FIFO_WORDS {
            if i < len {
                // Continue filling write FIFO and emptying read FIFO
                let read_value = self.exchange_duplex(words[i])?;

                words[i - FIFO_WORDS] = read_value;
            } else {
                // Finish emptying the read FIFO
                words[i - FIFO_WORDS] = self.read_duplex()?;
            }
        }

        Ok(())
    }

    // todo: End H7xx HAL C+Ps

    /// Transmit data using DMA. See L44 RM, section 40.4.9: Communication using DMA.
    /// Note that the `channel` argument is unused on F3 and L4, since it is hard-coded,
    /// and can't be configured using the DMAMUX peripheral. (`dma::mux()` fn).
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub unsafe fn write_dma<D>(
        &mut self,
        buf: &[u8],
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // Static write and read buffers?
        let (ptr, len) = (buf.as_ptr(), buf.len());

        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        // todo: Accept u16 words too.
        // todo: Pri and Circular as args?

        // A DMA access is requested when the TXE or RXNE enable bit in the SPIx_CR2 register is
        // set. Separate requests must be issued to the Tx and Rx buffers.
        // In transmission, a DMA request is issued each time TXE is set to 1. The DMA then
        // writes to the SPIx_DR register.

        // When starting communication using DMA, to prevent DMA channel management raising
        // error events, these steps must be followed in order:
        //
        // 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is
        // used.
        // (N/A)

        // 2. Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = R::write_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);

        #[cfg(any(feature = "h5", feature = "h7"))]
        let periph_addr = &self.regs.txdr as *const _ as u32;
        #[cfg(not(any(feature = "h5", feature = "h7")))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        #[cfg(any(feature = "h5", feature = "h7"))]
        let num_data = len as u32;
        #[cfg(not(any(feature = "h5", feature = "h7")))]
        let num_data = len as u16;

        dma.cfg_channel(
            channel,
            periph_addr,
            ptr as u32,
            num_data,
            dma::Direction::ReadFromMem,
            dma::DataSize::S8,
            dma::DataSize::S8,
            channel_cfg,
        );

        // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
        #[cfg(not(feature = "h7"))]
        self.regs.cr2.modify(|_, w| w.txdmaen().set_bit());
        #[cfg(feature = "h7")]
        self.regs.cfg1.modify(|_, w| w.txdmaen().set_bit());

        // 4. Enable the SPI by setting the SPE bit.
        self.regs.cr1.modify(|_, w| w.spe().set_bit());
    }

    /// Receive data using DMA. See L44 RM, section 40.4.9: Communication using DMA.
    /// Note that the `channel` argument is unused on F3 and L4, since it is hard-coded,
    /// and can't be configured using the DMAMUX peripheral. (`dma::mux()` fn).
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub unsafe fn read_dma(
        &mut self,
        buf: &mut [u8],
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma_periph: dma::DmaPeriph,
    ) {
        // todo: Accept u16 words too.
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        #[cfg(not(any(feature = "h5", feature = "h7")))]
        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());
        #[cfg(any(feature = "h5", feature = "h7"))]
        self.regs.cfg1.modify(|_, w| w.rxdmaen().set_bit());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = R::read_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);

        #[cfg(any(feature = "h5", feature = "h7"))]
        let periph_addr = &self.regs.rxdr as *const _ as u32;
        #[cfg(not(any(feature = "h5", feature = "h7")))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        #[cfg(any(feature = "h5", feature = "h7"))]
        let num_data = len as u32;
        #[cfg(not(any(feature = "h5", feature = "h7")))]
        let num_data = len as u16;

        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    periph_addr,
                    ptr as u32,
                    num_data,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                );
            }
            #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    periph_addr,
                    ptr as u32,
                    num_data,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                );
            }
        }

        self.regs.cr1.modify(|_, w| w.spe().set_bit());
    }

    /// Transfer data from DMA; this is the basic reading API, using both write and read transfers:
    /// It performs a write with register data, and reads to a buffer.
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub unsafe fn transfer_dma(
        &mut self,
        buf_write: &[u8],
        buf_read: &mut [u8],
        channel_write: DmaChannel,
        channel_read: DmaChannel,
        channel_cfg_write: ChannelCfg,
        channel_cfg_read: ChannelCfg,
        dma_periph: dma::DmaPeriph,
        // dma: &mut Dma<D>,
    ) {
        // where
        // D: Deref<Target = dma_p::RegisterBlock>,
        // {
        // todo: Accept u16 words too.
        let (ptr_write, len_write) = (buf_write.as_ptr(), buf_write.len());
        let (ptr_read, len_read) = (buf_read.as_mut_ptr(), buf_read.len());

        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        // todo: DRY here, with `write_dma`, and `read_dma`.

        #[cfg(any(feature = "h5", feature = "h7"))]
        let periph_addr_write = &self.regs.txdr as *const _ as u32;
        #[cfg(any(feature = "h5", not(feature = "h7")))]
        let periph_addr_write = &self.regs.dr as *const _ as u32;

        #[cfg(any(feature = "h5", feature = "h7"))]
        let periph_addr_read = &self.regs.rxdr as *const _ as u32;
        #[cfg(any(feature = "h5", not(feature = "h7")))]
        let periph_addr_read = &self.regs.dr as *const _ as u32;

        #[cfg(any(feature = "h5", feature = "h7"))]
        let num_data_write = len_write as u32;
        #[cfg(any(feature = "h5", not(feature = "h7")))]
        let num_data_write = len_write as u16;

        #[cfg(any(feature = "h5", feature = "h7"))]
        let num_data_read = len_read as u32;
        #[cfg(any(feature = "h5", not(feature = "h7")))]
        let num_data_read = len_read as u16;

        // Be careful - order of enabling Rx and Tx may matter, along with other things like when we
        // enable the channels, and the SPI periph.
        #[cfg(not(any(feature = "h5", feature = "h7")))]
        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());
        #[cfg(any(feature = "h5", feature = "h7"))]
        self.regs.cfg1.modify(|_, w| w.rxdmaen().set_bit());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel_write = R::write_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel_read = R::read_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);
        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel_write,
                    periph_addr_write,
                    ptr_write as u32,
                    num_data_write,
                    dma::Direction::ReadFromMem,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg_write,
                );

                dma::cfg_channel(
                    &mut regs,
                    channel_read,
                    periph_addr_read,
                    ptr_read as u32,
                    num_data_read,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg_read,
                );
            }
            #[cfg(not(any(feature = "f3x4", feature = "g0", feature = "wb")))]
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel_write,
                    periph_addr_write,
                    ptr_write as u32,
                    num_data_write,
                    dma::Direction::ReadFromMem,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg_write,
                );

                dma::cfg_channel(
                    &mut regs,
                    channel_read,
                    periph_addr_read,
                    ptr_read as u32,
                    num_data_read,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg_read,
                );
            }
        }

        #[cfg(not(any(feature = "h5", feature = "h7")))]
        self.regs.cr2.modify(|_, w| w.txdmaen().set_bit());
        #[cfg(any(feature = "h5", feature = "h7"))]
        self.regs.cfg1.modify(|_, w| w.txdmaen().set_bit());

        self.regs.cr1.modify(|_, w| w.spe().set_bit());
    }

    /// Stop a DMA transfer. Stops the channel, and disables the `txdmaen` and `rxdmaen` bits.
    /// Run this after each transfer completes - you may wish to do this in an interrupt
    /// (eg DMA transfer complete) instead of blocking. `channel2` is an optional second channel
    /// to stop; eg if you have both a tx and rx channel.
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub fn stop_dma(
        &mut self,
        channel: DmaChannel,
        channel2: Option<DmaChannel>,
        // dma: &mut Dma<D>,
        dma_periph: dma::DmaPeriph,
    ) {
        // where
        // D: Deref<Target = dma_p::RegisterBlock>,
        // {
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

    #[cfg(not(any(feature = "h5", feature = "h7")))]
    /// Enable an interrupt. Note that unlike on other peripherals, there's no explicit way to
    /// clear these. RM: "Writing to the transmit data register always clears the TXE bit.
    /// The TXE flag is set by hardware."
    pub fn enable_interrupt(&mut self, interrupt_type: SpiInterrupt) {
        self.regs.cr2.modify(|_, w| match interrupt_type {
            SpiInterrupt::TxBufEmpty => w.txeie().set_bit(),
            SpiInterrupt::RxBufNotEmpty => w.rxneie().set_bit(),
            SpiInterrupt::Error => w.errie().set_bit(),
        });
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    /// Enable an interrupt.
    pub fn enable_interrupt(&mut self, interrupt_type: SpiInterrupt) {
        self.regs.ier.modify(|_, w| match interrupt_type {
            SpiInterrupt::NumberOfTransactionsReload => w.tserfie().set_bit(),
            SpiInterrupt::ModeFault => w.modfie().set_bit(),
            SpiInterrupt::Tifre => w.tifreie().set_bit(),
            SpiInterrupt::CrcError => w.crceie().set_bit(),
            SpiInterrupt::Overrun => w.ovrie().set_bit(),
            SpiInterrupt::Underrun => w.udrie().set_bit(),
            SpiInterrupt::Txtfie => w.txtfie().set_bit(),
            SpiInterrupt::EotSuspTxc => w.eotie().set_bit(),
            // SpiInterrupt::Dxp => w.dxpie().set_bit(),
            // SpiInterrupt::Txp => w.txpie().set_bit(),
            // SpiInterrupt::Rxp => w.rxpie().set_bit(),
            _ => w.eotie().set_bit(), // todo: PAC ommission?
        });
    }

    #[cfg(any(feature = "h5", feature = "h7"))]
    /// Clear an interrupt.
    pub fn clear_interrupt(&mut self, interrupt_type: SpiInterrupt) {
        self.regs.ifcr.write(|w| match interrupt_type {
            SpiInterrupt::NumberOfTransactionsReload => w.tserfc().set_bit(),
            SpiInterrupt::ModeFault => w.modfc().set_bit(),
            SpiInterrupt::Tifre => w.tifrec().set_bit(),
            SpiInterrupt::CrcError => w.crcec().set_bit(),
            SpiInterrupt::Overrun => w.ovrc().set_bit(),
            SpiInterrupt::Underrun => w.udrc().set_bit(),
            SpiInterrupt::Txtfie => w.txtfc().set_bit(),
            SpiInterrupt::EotSuspTxc => w.eotc().set_bit(),
            // SpiInterrupt::Dxp => w.dxpc().set_bit(),
            // SpiInterrupt::Txp => w.txpc().set_bit(),
            // SpiInterrupt::Rxp => w.rxpc().set_bit(),
            _ => w.eotc().set_bit(), // todo: PAC ommission?
        });
    }
}

#[cfg(feature = "embedded_hal")]
impl<R> FullDuplex<u8> for Spi<R>
where
    R: Deref<Target = pac::spi1::RegisterBlock> + RccPeriph,
{
    type Error = SpiError;

    fn read(&mut self) -> nb::Result<u8, SpiError> {
        match Spi::read(self) {
            Ok(r) => Ok(r),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), SpiError> {
        match Spi::write_one(self, byte) {
            Ok(r) => Ok(r),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

#[cfg(feature = "embedded_hal")]
impl<R> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<R> where
    R: Deref<Target = pac::spi1::RegisterBlock> + RccPeriph
{
}

#[cfg(feature = "embedded_hal")]
impl<R> embedded_hal::blocking::spi::write::Default<u8> for Spi<R> where
    R: Deref<Target = pac::spi1::RegisterBlock> + RccPeriph
{
}
