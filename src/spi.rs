//! Serial Peripheral Interface (SPI) bus. Implements traits from `embedded-hal`.

// Based on `stm32l4xx-hal` and `stm32h7xx-hal`.

use core::{ops::Deref, ptr};

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(not(feature = "g0"))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
use crate::dma::{self, Dma};

// todo: non-static buffers?
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use cfg_if::cfg_if;

// todo: More config options and enums?

// todo: Don't make EH the default API.

/// SPI error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

#[derive(Clone, Copy)]
pub enum SpiDevice {
    One,
    #[cfg(not(feature = "f3x4"))]
    Two,
    #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
    Three,
}

#[cfg(feature = "h7")]
#[derive(Copy, Clone)]
pub struct SpiConfig {
    pub mode: Mode,
    /// Specify that the SPI MISO/MOSI lines are swapped.
    ///
    /// Note:
    /// * This function updates the HAL peripheral to treat the pin provided in the MISO parameter
    /// as the MOSI pin and the pin provided in the MOSI parameter as the MISO pin.
    pub swap_miso_mosi: bool,
    /// Specify a delay between CS assertion and the beginning of the SPI transaction.
    ///
    /// Note:
    /// * This function introduces a delay on SCK from the initiation of the transaction. The delay
    /// is specified as a number of SCK cycles, so the actual delay may vary.
    ///
    /// Arguments:
    /// * `delay` - The delay between CS assertion and the start of the transaction in seconds.
    /// register for the output pin.
    pub cs_delay: f32,
    /// CS pin is automatically managed by the SPI peripheral.
    ///
    /// # Note
    /// SPI is configured in "endless transaction" mode, which means that the SPI CSn pin will
    /// assert when the first data is sent and will not de-assert.
    ///
    /// If CSn should be de-asserted between each data transfer, use `suspend_when_inactive()` as
    /// well.
    ///
    pub managed_cs: bool,
    /// Suspend a transaction automatically if data is not available in the FIFO.
    ///
    /// # Note
    /// This will de-assert CSn when no data is available for transmission and hardware is managing
    /// the CSn pin.
    pub suspend_when_inactive: bool,
    /// Select the communication mode of the SPI bus.
    pub communication_mode: CommunicationMode,
}

#[cfg(feature = "h7")]
impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        SpiConfig {
            mode,
            swap_miso_mosi: false,
            cs_delay: 0.0,
            managed_cs: false,
            suspend_when_inactive: false,
            communication_mode: CommunicationMode::FullDuplex,
        }
    }
}

#[cfg(feature = "h7")]
impl From<Mode> for Config {
    fn from(mode: Mode) -> Self {
        Self::new(mode)
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<S> {
    regs: S,
    device: SpiDevice,
}

impl<S> Spi<S>
where
    S: Deref<Target = pac::spi1::RegisterBlock>,
{
    /// Configures the SPI peripheral to operate in full duplex master mode
    pub fn new<C: ClockCfg>(
        regs: S,
        device: SpiDevice,
        mode: Mode,
        freq: u32,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        match device {
            SpiDevice::One => {
                #[cfg(not(feature = "f301"))] // todo: Not sure what's going on  here.
                rcc_en_reset!(apb2, spi1, rcc);
            }
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => {
                rcc_en_reset!(apb1, spi2, rcc);
            }
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            SpiDevice::Three => {
                cfg_if! {
                    // Note `sp3en` mixed with `spi3rst`; why we can't use the usual macro.
                    if #[cfg(any(feature = "l4x3", feature = "l5"))] {
                        rcc.apb1enr1.modify(|_, w| w.sp3en().set_bit());
                        rcc.apb1rstr1.modify(|_, w| w.spi3rst().set_bit());
                        rcc.apb1rstr1.modify(|_, w| w.spi3rst().clear_bit());
                    } else {
                        rcc_en_reset!(apb1, spi3, rcc);
                    }
                }
            }
        }

        cfg_if! {
            if #[cfg(feature = "h7")] {
                  // Disable SS output
                regs.cfg2.write(|w| w.ssoe().disabled());

                let config: Config = config.into();

                let spi_freq = freq;
                let spi_ker_ck = match Self::kernel_clk(clocks) {
                    Some(ker_hz) => ker_hz.0,
                    _ => panic!("$SPIX kernel clock not running!")
                };
                let mbr = match spi_ker_ck / spi_freq {
                    0 => unreachable!(),
                    1..=2 => MBR::DIV2,
                    3..=5 => MBR::DIV4,
                    6..=11 => MBR::DIV8,
                    12..=23 => MBR::DIV16,
                    24..=47 => MBR::DIV32,
                    48..=95 => MBR::DIV64,
                    96..=191 => MBR::DIV128,
                    _ => MBR::DIV256,
                };
                regs.cfg1.modify(|_, w| {
                    w.mbr()
                        .variant(mbr) // master baud rate
                });
                spi!(DSIZE, spi, $TY); // modify CFG1 for DSIZE

                // ssi: select slave = master mode
                regs.cr1.write(|w| w.ssi().slave_not_selected());

                // Calculate the CS->transaction cycle delay bits.
                let (start_cycle_delay, interdata_cycle_delay) = {
                    let mut delay: u32 = (config.cs_delay * spi_freq as f32) as u32;

                    // If the cs-delay is specified as non-zero, add 1 to the delay cycles
                    // before truncation to an integer to ensure that we have at least as
                    // many cycles as required.
                    if config.cs_delay > 0.0_f32 {
                        delay += 1;
                    }

                    if delay > 0xF {
                        delay = 0xF;
                    }

                    // If CS suspends while data is inactive, we also require an
                    // "inter-data" delay.
                    if config.suspend_when_inactive {
                        (delay as u8, delay as u8)
                    } else {
                        (delay as u8, 0_u8)
                    }
                };

                // The calculated cycle delay may not be more than 4 bits wide for the
                // configuration register.
                let communication_mode = match config.communication_mode {
                    CommunicationMode::Transmitter => COMM::TRANSMITTER,
                    CommunicationMode::Receiver => COMM::RECEIVER,
                    CommunicationMode::FullDuplex => COMM::FULLDUPLEX,
                };

                // mstr: master configuration
                // lsbfrst: MSB first
                // comm: full-duplex
                regs.cfg2.write(|w| {
                    w.cpha()
                        .bit(config.mode.phase ==
                             Phase::CaptureOnSecondTransition)
                        .cpol()
                        .bit(config.mode.polarity == Polarity::IdleHigh)
                        .master()
                        .master()
                        .lsbfrst()
                        .msbfirst()
                        .ssom()
                        .bit(config.suspend_when_inactive)
                        .ssm()
                        .bit(config.managed_cs == false)
                        .ssoe()
                        .bit(config.managed_cs == true)
                        .mssi()
                        .bits(start_cycle_delay)
                        .midi()
                        .bits(interdata_cycle_delay)
                        .ioswp()
                        .bit(config.swap_miso_mosi == true)
                        .comm()
                        .variant(communication_mode)
                });

                // spe: enable the SPI bus
                regs.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());
            } else {
                let fclk = match device {
                    SpiDevice::One => clocks.apb2(),
                    _ => clocks.apb1(),
                };

                let br = Self::compute_baud_rate(fclk, freq);

                // CPHA: phase
                // CPOL: polarity
                // MSTR: master mode
                // BR: 1 MHz
                // SPE: SPI disabled
                // LSBFIRST: MSB first
                // SSM: enable software slave management (NSS pin free for other uses)
                // SSI: set nss high = master mode
                // CRCEN: hardware CRC calculation disabled
                // BIDIMODE: 2 line unidirectional (full duplex)

                // L44 RM, section 40.4.7: Configuration of SPI
                // The configuration procedure is almost the same for master and slave. For specific mode
                // setups, follow the dedicated sections. When a standard communication is to be initialized,
                // perform these steps:

                // 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
                // (Handled in GPIO modules and user code)

                // 2. Write to the SPI_CR1 register:
                // todo: Should more of these be configurable vice hard set?)
                regs.cr1.modify(|_, w| unsafe {
                    // a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 4)
                    w.br().bits(br);
                    // b) Configure the CPOL and CPHA bits combination to define one of the four
                    // relationships between the data transfer and the serial clock (CPHA must be
                    // cleared in NSSP mode). (Note: 2 - except the case when CRC is enabled at TI
                    // mode).
                    w.cpol().bit(mode.polarity == Polarity::IdleHigh);
                    w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
                    // c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
                    // BIDIOE (RXONLY and BIDIMODE can't be set at the same time).
                    // (TODO: absent from L4X impl.)
                    // d) Configure the LSBFIRST bit to define the frame format (Note: 2).
                    w.lsbfirst().clear_bit();
                    // e) Configure the CRCL and CRCEN bits if CRC is needed (while SCK clock signal is
                    // at idle state).
                    w.crcen().clear_bit();
                    // f) Configure SSM and SSI (Notes: 2 & 3).
                    w.ssi().set_bit();
                    w.ssm().set_bit();
                    // g) Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on
                    // NSS if master is configured to prevent MODF error).
                    w.mstr().set_bit();
                    w.bidimode().clear_bit(); // todo?
                    w.spe().set_bit() // Enable SPI
                });

                // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                //        8-bit
                // DS: 8-bit data size
                // SSOE: Slave Select output disabled

                // 3. Write to SPI_CR2 register:
                #[cfg(feature = "f4")]
                regs.cr2.modify(|_, w| w.ssoe().clear_bit());

                #[cfg(not(feature = "f4"))]
                regs.cr2
                    .modify(|_, w| unsafe {
                        // a) Configure the DS[3:0] bits to select the data length for the transfer.
                        w.ds().bits(0b111);
                        // b) Configure SSOE (Notes: 1 & 2 & 3).
                        w.ssoe().clear_bit();
                        // e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the read
                        // access size for the SPIx_DR register.
                        w.frxth().set_bit()
                    });

                // c) Set the FRF bit if the TI protocol is required (keep NSSP bit cleared in TI mode).
                // d) Set the NSSP bit if the NSS pulse mode between two data units is required (keep
                // CHPA and TI bits cleared in NSSP mode).

                // f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
                // 4. Write to SPI_CRCPR register: Configure the CRC polynomial if needed.
                // 5. Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in
                // DMA registers if the DMA streams are used.
            }
        }
        Spi { regs, device }
    }

    #[cfg(not(feature = "h7"))]
    /// Change the baud rate of the SPI
    pub fn reclock<F, C: ClockCfg>(&mut self, freq: u32, clocks: C) {
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        let fclk = match self.device {
            SpiDevice::One => clocks.apb2(),
            _ => clocks.apb1(),
        };

        let br = Self::compute_baud_rate(fclk, freq);

        self.regs.cr1.modify(|_, w| {
            unsafe {
                w.br().bits(br);
            }
            w.spe().set_bit()
        });
    }

    fn compute_baud_rate(clocks: u32, freq: u32) -> u8 {
        match clocks / freq {
            0 => unreachable!(),
            1..=2 => 0b000,
            3..=5 => 0b001,
            6..=11 => 0b010,
            12..=23 => 0b011,
            24..=39 => 0b100,
            40..=95 => 0b101,
            96..=191 => 0b110,
            _ => 0b111,
        }
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    /// Enable use of DMA transmission for U[s]ART: (L44 RM, section 38.5.15)
    pub fn enable_dma<D>(&mut self, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // (see comments in below variant of this fn.)
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());
        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());
        self.regs.cr2.modify(|_, w| w.txdmaen().set_bit());
        self.regs.cr1.modify(|_, w| w.spe().set_bit());

        // todo: These are only valid for DMA1!
        #[cfg(feature = "l4")]
        match self.device {
            SpiDevice::One => {
                dma.channel_select(dma::DmaChannel::C3, 0b001); // Tx
                dma.channel_select(dma::DmaChannel::C2, 0b001); // Rx
            }
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => {
                dma.channel_select(dma::DmaChannel::C5, 0b001);
                dma.channel_select(dma::DmaChannel::C4, 0b001);
            }
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            SpiDevice::Three => {
                panic!(
                    "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github."
                )
            }
        };
        // Note that we need neither channel select, nor multiplex for F3.
    }

    #[cfg(any(feature = "l5", feature = "g0", feature = "g4"))]
    /// Enable use of DMA transmission for U[s]ART: (L44 RM, section 38.5.15)
    pub fn enable_dma<D>(
        &mut self,
        dma: &mut Dma<D>,
        chan_tx: dma::DmaChannel,
        chan_rx: dma::DmaChannel,
        mux: &mut pac::DMAMUX,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // RM:
        // When starting communication using DMA, to prevent DMA channel management raising
        // error events, these steps must be followed in order:

        // todo: Is disabling spi here required? Implied by "followed in order" above?
        self.regs.cr1.modify(|_, w| w.spe().clear_bit());

        // 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is
        // used.
        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());
        // 2. Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
        // todo?
        // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
        self.regs.cr2.modify(|_, w| w.txdmaen().set_bit());
        // 4. Enable the SPI by setting the SPE bit.
        self.regs.cr1.modify(|_, w| w.spe().set_bit());

        // todo: `disable_dma` or `stop_dma` function!
        // To close communication it is mandatory to follow these steps in order:
        // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.
        // 2. Disable the SPI by following the SPI disable procedure.
        // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
        // SPI_CR2 register, if DMA Tx and/or DMA Rx are used

        #[cfg(any(feature = "l5", feature = "g0", feature = "g4"))]
        // See G4 RM, Table 91.
        match self.device {
            SpiDevice::One => {
                dma.mux(chan_tx, dma::MuxInput::Spi1Tx as u8, mux); // Tx
                dma.mux(chan_rx, dma::MuxInput::Spi1Rx as u8, mux); // Rx
            }
            SpiDevice::Two => {
                dma.mux(chan_tx, dma::MuxInput::Spi2Tx as u8, mux);
                dma.mux(chan_rx, dma::MuxInput::Spi2Rx as u8, mux);
            }
            SpiDevice::Three => {
                dma.mux(chan_tx, dma::MuxInput::Spi3Tx as u8, mux);
                dma.mux(chan_rx, dma::MuxInput::Spi3Rx as u8, mux);
            }
        };

        // Note that we need neither channel select, nor multiplex for F3.
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Transmit data using DMA. See L44 RM, section 40.4.9: Communication using DMA
    pub fn write_dma<D, B>(&mut self, mut buf: B, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
        B: StaticWriteBuffer,
    {
        let (ptr, len) = unsafe { buf.write_buffer() };

        // A DMA access is requested when the TXE or RXNE enable bit in the SPIx_CR2 register is
        // set. Separate requests must be issued to the Tx and Rx buffers.
        // In transmission, a DMA request is issued each time TXE is set to 1. The DMA then
        // writes to the SPIx_DR register.

        // todo: This channel selection is confirmed for L4 only - check other families
        // todo DMA channel mapping

        // todo: Does this work with Muxing?
        // todo: Make these configurable with muxing on supported families.
        let channel = match self.device {
            SpiDevice::One => dma::DmaChannel::C3,
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => dma::DmaChannel::C5,
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            SpiDevice::Three => panic!(
                "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github."
            ),
        };

        #[cfg(feature = "h7")]
        let periph_addr = &self.regs.rxdr as *const _ as u32;
        #[cfg(not(feature = "h7"))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        dma.cfg_channel(
            channel,
            periph_addr,
            ptr as u32,
            len as u16, // (x2 per one of the examples??)
            dma::Priority::Medium, // todo: Pass pri as an arg?
            dma::Direction::ReadFromMem,
            dma::Circular::Disabled, // todo?
            dma::IncrMode::Disabled,
            dma::IncrMode::Enabled,
            dma::DataSize::S8,
            dma::DataSize::S8,
        );
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Receive data using DMA. See L44 RM, section 40.4.9: Communication using DMA
    pub fn read_dma<D, B>(&mut self, buf: B, dma: &mut Dma<D>)
    where
        B: StaticReadBuffer,
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = unsafe { buf.read_buffer() };

        // In reception, a DMA request is issued each time RXNE is set to 1. The DMA then reads
        // the SPIx_DR register.

        // todo: Make these configurable with muxing on supported families.
        // todo: This channel selection is confirmed for L4 only - check other families
        // todo DMA channel mapping
        let channel = match self.device {
            SpiDevice::One => dma::DmaChannel::C2,
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => dma::DmaChannel::C4,
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            SpiDevice::Three => panic!(
                "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github."
            ),
        };

        #[cfg(feature = "h7")]
        let periph_addr = &self.regs.rxdr as *const _ as u32;
        #[cfg(not(feature = "h7"))]
        let periph_addr = &self.regs.dr as *const _ as u32;

        dma.cfg_channel(
            channel,
            periph_addr,
            ptr as u32,
            len as u16, // (x2 per one of the examples??)
            dma::Priority::Medium, // todo: Pass pri as an arg?
            dma::Direction::ReadFromPeriph,
            dma::Circular::Disabled, // todo?

            dma::IncrMode::Disabled,
            dma::IncrMode::Enabled,
            dma::DataSize::S8,
            dma::DataSize::S8,
        );
    }
}

impl<S> FullDuplex<u8> for Spi<S>
where
    S: Deref<Target = pac::spi1::RegisterBlock>,
{
    type Error = Error;

    /// See L44 RM, section 40.4.9: Data transmission and reception procedures.
    fn read(&mut self) -> nb::Result<u8, Error> {
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

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
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
}

impl<S> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<S> where
    S: Deref<Target = pac::spi1::RegisterBlock>
{
}

impl<S> embedded_hal::blocking::spi::write::Default<u8> for Spi<S> where
    S: Deref<Target = pac::spi1::RegisterBlock>
{
}
