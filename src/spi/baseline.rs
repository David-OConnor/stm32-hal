use core::{ops::Deref, ptr};

use super::*;
use crate::{
    check_errors,
    pac::{self, RCC},
    util::RccPeriph,
    MAX_ITERS,
};

/// Possible interrupt types. Enable these in SPIx_CR2. Check and clear with SR. There is no explicit
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

/// These bits configure the data length for SPI transfers. Sets `SPI_CR2` register, `DS` field.
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

impl<R> Spi<R>
where
    R: Deref<Target = pac::spi1::RegisterBlock> + RccPeriph,
{
    /// Initialize an SPI peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, cfg: SpiConfig, baud_rate: BaudRate) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        R::en_reset(rcc);

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
        regs.cr2.modify(|_, w| {
            w.ssoe()
                .bit(cfg.slave_select == SlaveSelect::HardwareOutEnable)
        });

        #[cfg(not(feature = "f4"))]
        regs.cr2.modify(|_, w| unsafe {
            // a) Configure the DS[3:0] bits to select the data length for the transfer.
            w.ds().bits(cfg.data_size as u8);
            // b) Configure SSOE (Notes: 1 & 2 & 3).
            w.ssoe()
                .bit(cfg.slave_select == SlaveSelect::HardwareOutEnable);
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

        // todo: It sounds like you should enable and disable spi during writes, not on init!
        // todo: This lets you use hardware CS management, and seems to be teh way the RM
        // todo steers you towards regardless.

        Self { regs, cfg }
    }

    /// Change the SPI baud rate.
    pub fn reclock(&mut self, baud_rate: BaudRate) {
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        self.regs.cr1.modify(|_, w| unsafe {
            w.br().bits(baud_rate as u8);
            w.spe().set_bit()
        });
    }

    /// L44 RM, section 40.4.9: "Procedure for disabling the SPI"
    /// When SPI is disabled, it is mandatory to follow the disable procedures described in this
    /// paragraph. It is important to do this before the system enters a low-power mode when the
    /// peripheral clock is stopped. Ongoing transactions can be corrupted in this case. In some
    /// modes the disable procedure is the only way to stop continuous communication running.
    pub fn disable(&mut self) {
        // The correct disable procedure is (except when receive only mode is used):

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

    /// Read a single byte if available, or block until it's available.
    pub fn read(&mut self) -> Result<u8, SpiError> {
        check_errors!(self.regs.sr.read());

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
    pub fn write_one(&mut self, byte: u8) -> Result<(), SpiError> {
        check_errors!(self.regs.sr.read());

        let mut i = 0;
        while !self.regs.sr.read().txe().bit_is_set() {
            i += 1;
            if i >= MAX_ITERS {
                return Err(SpiError::Hardware);
            }
        }

        #[allow(invalid_reference_casting)]
        unsafe {
            ptr::write_volatile(&self.regs.dr as *const _ as *mut u8, byte)
        };

        Ok(())
    }

    /// Write multiple bytes on the SPI line, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn write(&mut self, words: &[u8]) -> Result<(), SpiError> {
        for word in words {
            self.write_one(*word)?;
            self.read()?;
        }

        Ok(())
    }

    /// Read multiple bytes to a buffer, blocking until complete.
    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<(), SpiError> {
        for word in words.iter_mut() {
            self.write_one(*word)?;
            *word = self.read()?;
        }

        Ok(())
    }

    /// An alternative transfer API, using separate read and write buffers.
    pub fn transfer_type2<'w>(
        &mut self,
        write_buf: &'w [u8],
        read_buf: &'w mut [u8],
    ) -> Result<(), SpiError> {
        for (i, word) in write_buf.iter().enumerate() {
            self.write_one(*word)?;
            if i < read_buf.len() {
                read_buf[i] = self.read()?;
            }
        }

        // for (i_read, word) in read_buf.iter().enumerate() {
        //     self.write_one(*word)?;
        //     read_buf[i_write] = self.read()?;
        //     println!("read: {}", read_buf[i_write]);
        // }

        // for (i_write, word) in write_buf.iter().enumerate() {
        //     self.write_one(*word)?;
        //     let i_read = i + write_buf.len();
        //
        //     if i_write >= write_buf.len() - 1 {
        //         // let i_read = i_write - write_buf.len() + 1;
        //         read_buf[i_read] = self.read()?;
        //         println!("read. i:{} v:{:?}", i_read, read_buf[i_read]);
        //         i_read += 1;
        //     }
        // }

        Ok(())
    }

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
