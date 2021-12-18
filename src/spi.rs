//! Support for the Serial Peripheral Interface (SPI) bus peripheral.
//! Provides APIs to configure, read, and write from
//! SPI, with blocking, nonblocking, and DMA functionality.

use core::{ops::Deref, ptr};

use cortex_m::interrupt::free;

#[cfg(feature = "embedded-hal")]
use embedded_hal::spi::FullDuplex;

use crate::{
    pac::{self, RCC},
    util::{DmaPeriph, RccPeriph},
};

#[cfg(all(feature = "g0", not(feature = "g0b0"), not(feature = "g0b1"), not(feature = "g0c1")))]
use crate::pac::dma as dma_p;
#[cfg(any(
    feature = "f3",
    feature = "l4",
    feature = "g4",
    feature = "h7",
    feature = "wb",
    feature = "wl",
    feature = "g0b0",
    feature = "g0b1",
    feature = "g0c1",
))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "f4", feature = "l5")))]
use crate::dma::{self, Dma, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

use cfg_if::cfg_if;

/// SPI error
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

/// Possible interrupt types. Enable these in CR2. Check and clear with SR. There is no explicit
/// way to clear these.
#[derive(Copy, Clone)]
pub enum SpiInterrupt {
    /// Tx buffer empty (TXEIE)
    TxBufEmpty,
    /// Rx buffer not empty (RXNEIE)
    RxBufNotEmpty,
    /// Error (ERRIE)
    Error,
}

/// Set the factor to divide the APB clock by to set baud rate. Sets `SPI_CR1` register, `BR` field.
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
/// Select the communication mode between.
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
    pub fn mode0() -> Self {
        Self {
            polarity: SpiPolarity::IdleLow,
            phase: SpiPhase::CaptureOnFirstTransition,
        }
    }

    /// Set Spi Mode 1: Idle low, capture on second transition.
    pub fn mode1() -> Self {
        Self {
            polarity: SpiPolarity::IdleLow,
            phase: SpiPhase::CaptureOnSecondTransition,
        }
    }

    /// Set Spi Mode 2: Idle high, capture on first transition.
    pub fn mode2() -> Self {
        Self {
            polarity: SpiPolarity::IdleHigh,
            phase: SpiPhase::CaptureOnFirstTransition,
        }
    }

    /// Set Spi Mode 3: Idle high, capture on second transition.
    pub fn mode3() -> Self {
        Self {
            polarity: SpiPolarity::IdleHigh,
            phase: SpiPhase::CaptureOnSecondTransition,
        }
    }
}

/// Initial configuration data for the SPI peripheral.
pub struct SpiConfig {
    /// SPI mode associated with Polarity and Phase. Defaults to Mode0: Idle low, capture on first transition.
    pub mode: SpiMode,
    pub comm_mode: SpiCommMode,
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
        Self {
            mode: SpiMode::mode0(),
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
    R: Deref<Target = pac::spi1::RegisterBlock> + DmaPeriph + RccPeriph,
{
    /// Initialize an SPI peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, cfg: SpiConfig, baud_rate: BaudRate) -> Self {
        free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };
            R::en_reset(rcc);
        });

        cfg_if! {
            if #[cfg(feature = "h7")] {
                  // Disable SS output
                regs.cfg2.write(|w| w.ssoe().disabled());

                regs.cfg1.modify(|_, w| {
                    w.mbr().bits(baud_rate as u8);
                    w.dsize().bits(cfg.data_size as u8)

                });

                // ssi: select slave = master mode
                regs.cr1.write(|w| w.ssi().slave_not_selected());

                // todo: Data size on H7.

                // todo: Come back to this.
                // Calculate the CS->transaction cycle delay bits.
                // let (start_cycle_delay, interdata_cycle_delay) = {
                //     let mut delay: u32 = (config.cs_delay * spi_freq as f32) as u32;
                //
                //     // If the cs-delay is specified as non-zero, add 1 to the delay cycles
                //     // before truncation to an integer to ensure that we have at least as
                //     // many cycles as required.
                //     if config.cs_delay > 0.0_f32 {
                //         delay += 1;
                //     }
                //
                //     if delay > 0xF {
                //         delay = 0xF;
                //     }
                //
                //     // If CS suspends while data is inactive, we also require an
                //     // "inter-data" delay.
                //     if cfg.suspend_when_inactive {
                //         (delay as u8, delay as u8)
                //     } else {
                //         (delay as u8, 0_u8)
                //     }
                // };


                // mstr: master configuration
                // lsbfrst: MSB first
                // comm: full-duplex
                // todo: Flesh this out.
                regs.cfg2.write(|w| {
                    w.cpha().bit(cfg.mode.phase as u8 != 0);
                        w.cpol().bit(cfg.mode.polarity as u8 != 0);
                        w.master().master();
                        w.lsbfrst().msbfirst()
                        // w.ssom().bit(config.suspend_when_inactive);
                        // w.ssm().bit(config.managed_cs == false);
                        // w.ssoe().bit(config.managed_cs == true);
                        // w.mssi().bits(start_cycle_delay);
                        // w.midi().bits(interdata_cycle_delay);
                        // w.ioswp().bit(config.swap_miso_mosi == true)
                        // w.comm().variant(communication_mode);
                });

                // spe: enable the SPI bus
                regs.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());
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
                    w.ssi().set_bit(); // todo?
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
        Spi { regs, cfg }
    }

    /// Change the SPI baud rate.
    pub fn reclock(&mut self, baud_rate: BaudRate) {
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        #[cfg(not(feature = "h7"))]
        self.regs.cr1.modify(|_, w| unsafe {
            w.br().bits(baud_rate as u8);
            w.spe().set_bit()
        });

        #[cfg(feature = "h7")]
        self.regs
            .cfg1
            .modify(|_, w| unsafe { w.mbr().bits(baud_rate as u8) });

        #[cfg(feature = "h7")]
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
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn read(&mut self) -> nb::Result<u8, Error> {
        let sr = self.regs.sr.read();

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let crce = sr.crce().bit_is_set();
                let not_empty = sr.rxp().bit_is_set();
            } else {
                let crce = sr.crcerr().bit_is_set();
                let not_empty = sr.rxne().bit_is_set();
            }
        }

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if crce {
            Err(nb::Error::Other(Error::Crc))
        } else if not_empty {
            #[cfg(feature = "h7")]
            // todo: note: H7 can support words beyond u8. (Can others too?)
            let result = unsafe { ptr::read_volatile(&self.regs.rxdr as *const _ as *const u8) };
            #[cfg(not(feature = "h7"))]
            let result = unsafe { ptr::read_volatile(&self.regs.dr as *const _ as *const u8) };
            Ok(result)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Write a single byte if available, or block until it's available.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn write_one(&mut self, byte: u8) -> nb::Result<(), Error> {
        let sr = self.regs.sr.read();

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let crce = sr.crce().bit_is_set();
                let rdy = sr.txp().bit_is_set();
                let rdy = sr.txp().bit_is_set();
            } else {
                let crce = sr.crcerr().bit_is_set();
                let rdy = sr.txe().bit_is_set();
            }
        }

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if crce {
            Err(nb::Error::Other(Error::Crc))
        } else if rdy {
            cfg_if! {
                if #[cfg(feature = "h7")] {
                    // todo: note: H7 can support words beyond u8. (Can others too?)
                    unsafe { ptr::write_volatile(&self.regs.txdr as *const _ as *mut u8, byte) };
                    // write CSTART to start a transaction in master mode
                    self.regs.cr1.modify(|_, w| w.cstart().started());
                }
                 else {
                    unsafe { ptr::write_volatile(&self.regs.dr as *const _ as *mut u8, byte) };
                }
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Write multiple bytes on the SPI line, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        for word in words {
            nb::block!(self.write_one(word.clone()))?;
            nb::block!(self.read())?;
        }

        Ok(())
    }

    /// Read multiple bytes to a buffer, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<(), Error> {
        for word in words.iter_mut() {
            nb::block!(self.write_one(word.clone()))?;
            *word = nb::block!(self.read())?;
        }

        Ok(())
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Transmit data using DMA. See L44 RM, section 40.4.9: Communication using DMA.
    /// Note that the `channel` argument has no effect on F3 and L4.
    pub unsafe fn write_dma<D>(&mut self, buf: &[u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // Static write and read buffers?
        let (ptr, len) = (buf.as_ptr(), buf.len());

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
        R::write_sel(dma);

        #[cfg(feature = "h7")]
        let periph_addr = &self.regs.txdr as *const _ as u32;
        #[cfg(not(feature = "h7"))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        dma.cfg_channel(
            channel,
            periph_addr,
            ptr as u32,
            len as u16,
            dma::Direction::ReadFromMem,
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );

        // atomic::compiler_fence(Ordering::Release);  // todo ?

        // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
        self.regs.cr2.modify(|_, w| w.txdmaen().set_bit());

        // 4. Enable the SPI by setting the SPE bit.
        self.regs.cr1.modify(|_, w| w.spe().set_bit());
        // (todo: Should be already set. Should we disable it at the top of this fn just in case?)
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Receive data using DMA. See L44 RM, section 40.4.9: Communication using DMA.
    /// Note taht the `channel` argument has no effect on F3 and L4.
    pub unsafe fn read_dma<D>(&mut self, buf: &mut [u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // todo: Accept u16 words too.
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        // atomic::compiler_fence(Ordering::Release);  // todo ?

        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = R::read_chan();
        #[cfg(feature = "l4")]
        R::read_sel(dma);

        #[cfg(feature = "h7")]
        let periph_addr = &self.regs.rxdr as *const _ as u32;
        #[cfg(not(feature = "h7"))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        dma.cfg_channel(
            channel,
            periph_addr,
            ptr as u32,
            len as u16,
            dma::Direction::ReadFromPeriph,
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );

        self.regs.cr1.modify(|_, w| w.spe().set_bit());

        // todo: Set rxne or something to start?
    }

    // todo: pub fn transfer_dma()?

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Stop a DMA transfer. Stops the channel, and disables the `txdmaen` and `rxdmaen` bits.
    /// Run this after each transfer completes - you may wish to do this in an interrupt
    /// (eg DMA transfer complete) instead of blocking.
    pub fn stop_dma<D>(&mut self, channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // (RM:) To close communication it is mandatory to follow these steps in order:
        // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.
        dma.stop(channel);
        // 2. Disable the SPI by following the SPI disable procedure:
        // self.disable(); // todo: This probably isn't required.
        // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
        // SPI_CR2 register, if DMA Tx and/or DMA Rx are used.
        self.regs.cr2.modify(|_, w| {
            w.txdmaen().clear_bit();
            w.rxdmaen().clear_bit()
        })
    }

    #[cfg(not(feature = "h7"))]
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
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> FullDuplex<u8> for Spi<R>
where
    R: Deref<Target = pac::spi1::RegisterBlock> + DmaPeriph + RccPeriph,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        Spi::read(self)
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        Spi::write_one(self, byte)
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<R> where
    R: Deref<Target = pac::spi1::RegisterBlock> + DmaPeriph + RccPeriph
{
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> embedded_hal::blocking::spi::write::Default<u8> for Spi<R> where
    R: Deref<Target = pac::spi1::RegisterBlock> + DmaPeriph + RccPeriph
{
}
