//! Serial Peripheral Interface (SPI) bus. Implements traits from `embedded-hal`.

use core::{
    ops::Deref,
    ptr,
    sync::atomic::{self, Ordering},
};

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(feature = "f3", feature = "l4", feature = "g4"))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
use crate::dma::{self, Dma, DmaChannel, DmaInput};

use cfg_if::cfg_if;

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

/// Possible interrupt types. Enable these in CR2. Check and clear with SR. Clear in ?
#[derive(Debug)]
pub enum SpiInterrupt {
    /// Tx buffer empty (TXEIE)
    TxBufEmpty,
    /// Rx buffer not empty (RXNEIE)
    RxBufNotEmpty,
    /// Error (ERRIE)
    Error,
}

#[derive(Clone, Copy)]
pub enum SpiDevice {
    One,
    #[cfg(not(feature = "f3x4"))]
    Two,
    #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
    Three,
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
/// Used for managing NSS / CS pin.
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

pub struct SpiConfig {
    pub mode: Mode,
    pub comm_mode: SpiCommMode,
    pub slave_select: SlaveSelect,
    // pub cs_delay: f32,
    // pub swap_miso_mosi: bool,
    // pub suspend_when_inactive: bool,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            mode: Mode {
                // todo: What's a good default mode? Mode 1?
                polarity: Polarity::IdleHigh,
                phase: Phase::CaptureOnFirstTransition,
            },
            comm_mode: SpiCommMode::FullDuplex,
            slave_select: SlaveSelect::Software,
        }
    }
}

/// Represents an Serial Peripheral Interface (SPI) peripheral.
pub struct Spi<S> {
    regs: S,
    device: SpiDevice,
    cfg: SpiConfig,
}

impl<S> Spi<S>
where
    S: Deref<Target = pac::spi1::RegisterBlock>,
{
    /// Configures the SPI peripheral to operate in full duplex master mode
    pub fn new<C: ClockCfg>(
        regs: S,
        device: SpiDevice,
        cfg: SpiConfig,
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
                    _ => panic!("$SPX kernel clock not running!")
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

                // L44 RM, section 40.4.7: Configuration of SPI
                // The configuration procedure is almost the same for master and slave. For specific mode
                // setups, follow the dedicated sections. When a standard communication is to be initialized,
                // perform these steps:

                // 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
                // (Handled in GPIO modules and user code)

                // 2. Write to the SPI_CR1 register:
                regs.cr1.modify(|_, w| unsafe {
                    // a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 4)
                    w.br().bits(br);
                    // b) Configure the CPOL and CPHA bits combination to define one of the four
                    // relationships between the data transfer and the serial clock (CPHA must be
                    // cleared in NSSP mode). (Note: 2 - except the case when CRC is enabled at TI
                    // mode).
                    w.cpol().bit(cfg.mode.polarity == Polarity::IdleHigh);
                    w.cpha().bit(cfg.mode.phase == Phase::CaptureOnSecondTransition);
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
                        w.ds().bits(0b111);
                        // b) Configure SSOE (Notes: 1 & 2 & 3).
                        w.ssoe().bit(cfg.slave_select == SlaveSelect::HardwareOutEnable);
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

            // todo: It sounds like you should enable and disable spi during writes, not on init!
            // todo: This lets you use hardware CS management, and seems to be teh way the RM
            // todo steers you towards regardless.
        }
        Spi { regs, device, cfg }
    }

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

    /// Compute value for the baud rate register (BR). BR is specified as
    /// fraction offpclk (eg apb1 or apb2). Use the divider that will provide
    /// a speed closest to the one requested.
    fn compute_baud_rate(clocks: u32, freq: u32) -> u8 {
        // todo: Check that this is the same across MCUs.
        match clocks / freq {
            0..=2 => 0b000,    // fpclk / 2
            3..=6 => 0b001,    // /4
            7..=11 => 0b010,   // /8
            12..=23 => 0b011,  // /16
            24..=39 => 0b100,  // /32
            40..=95 => 0b101,  // /64
            96..=191 => 0b110, // /128
            _ => 0b111,        // /256
        }
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

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Transmit data using DMA. See L44 RM, section 40.4.9: Communication using DMA.
    /// Note that the `channel` argument has no effect on F3 and L4.
    pub fn write_dma<D>(&mut self, buf: &[u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // Static write and read buffers?
        let (ptr, len) = (buf.as_ptr(), buf.len());

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
        let tx_channel = match self.device {
            SpiDevice::One => DmaInput::Spi1Tx.dma1_channel(),
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => DmaInput::Spi2Tx.dma1_channel(),
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            SpiDevice::Three => panic!(
                "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github."
            ),
        };

        // #[cfg(any(feature = "f3", feature = "l4"))]
        // let rx_channel = match self.device {
        //     SpiDevice::One => DmaInput::Spi1Rx.dma1_channel(),
        //     #[cfg(not(feature = "f3x4"))]
        //     SpiDevice::Two => DmaInput::Spi2Rx.dma1_channel(),
        //     #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
        //     SpiDevice::Three => panic!(
        //         "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
        //         or PR on Github."
        //     ),
        // };

        #[cfg(feature = "l4")]
        match self.device {
            SpiDevice::One => {
                dma.channel_select(DmaInput::Spi1Tx);
                // dma.channel_select(DmaInput::Spi1Rx);
            }
            SpiDevice::Two => {
                dma.channel_select(DmaInput::Spi2Tx);
                // dma.channel_select(DmaInput::Spi2Rx);
            }
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            _ => unimplemented!(),
        };

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
    pub fn read_dma<D>(&mut self, buf: &mut [u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        // atomic::compiler_fence(Ordering::Release);  // todo ?

        self.regs.cr2.modify(|_, w| w.rxdmaen().set_bit());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            SpiDevice::One => DmaInput::Spi1Rx.dma1_channel(),
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => DmaInput::Spi2Rx.dma1_channel(),
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            _ => panic!(
                "DMA on SPI3 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github."
            ),
        };

        #[cfg(feature = "l4")]
        match self.device {
            SpiDevice::One => dma.channel_select(DmaInput::Spi1Rx),
            #[cfg(not(feature = "f3x4"))]
            SpiDevice::Two => dma.channel_select(DmaInput::Spi2Rx),
            #[cfg(not(any(feature = "f3x4", feature = "f410", feature = "g0")))]
            _ => unimplemented!(),
        };

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

    /// Enable an interrupt
    pub fn enable_interrupt(&mut self, interrupt_type: SpiInterrupt) {
        match interrupt_type {
            SpiInterrupt::TxBufEmpty => self.regs.cr2.modify(|_, w| w.txeie().set_bit()),
            SpiInterrupt::RxBufNotEmpty => self.regs.cr2.modify(|_, w| w.rxneie().set_bit()),
            SpiInterrupt::Error => self.regs.cr2.modify(|_, w| w.errie().set_bit()),
        }
    }

    // // todo: Not sure how to clear SPI interrupts. No ICR, and SR is read only?
    // /// Clear an interrupt flag.
    // pub fn clear_interrupt(&mut self, interrupt_type: SpiInterrupt) {
    //     match interrupt_type {
    //         SpiInterrupt::TxBufEmpty => self.regs.cr2.modify(|_, w| w.txeie.set_bit()),
    //         SpiInterrupt::RxBufNotEmpty => self.regs.cr2.modify(|_, w| w.rxneie.set_bit()),
    //         SpiInterrupt::Error => self.regs.cr2.modify(|_, w| w.erreie.set_bit()),
    //     }
    // }
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
