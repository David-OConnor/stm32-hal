//! Serial Peripheral Interface (SPI) bus. Implements traits from `embedded-hal`.

// Based on `stm32l4xx-hal` and `stm32h7xx-hal`.

use core::{ops::Deref, ptr};

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};

use cfg_if::cfg_if;

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

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
                // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                //        8-bit
                // DS: 8-bit data size
                // SSOE: Slave Select output disabled
                #[cfg(feature = "f4")]
                regs.cr2.write(|w| w.ssoe().clear_bit());

                #[cfg(not(feature = "f4"))]
                regs.cr2
                    .write(|w| unsafe {
                        w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                    });

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
                regs.cr1.write(|w| unsafe {
                    w.cpha()
                        .bit(mode.phase == Phase::CaptureOnSecondTransition)
                        .cpol()
                        .bit(mode.polarity == Polarity::IdleHigh)
                        .mstr()
                        .set_bit()
                        .br()
                        .bits(br)
                        .spe()
                        .set_bit()
                        .lsbfirst()
                        .clear_bit()
                        .ssi()
                        .set_bit()
                        .ssm()
                        .set_bit()
                        .crcen()
                        .clear_bit()
                        .bidimode()
                        .clear_bit()
                });
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
}

impl<S> FullDuplex<u8> for Spi<S>
where
    S: Deref<Target = pac::spi1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let sr = self.regs.sr.read();

        // todo: DRY between H7 and non-H7 branches here
        cfg_if! {
            if #[cfg(feature = "h7")] {
                return Err(if sr.ovr().is_overrun() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().is_fault() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crce().is_error() {
                    nb::Error::Other(Error::Crc)
                } else if sr.rxp().is_not_empty() {
                    // NOTE(read_volatile) read only 1 byte (the
                    // svd2rust API only allows reading a
                    // half-word)
                    return Ok(unsafe {
                        ptr::read_volatile(
                            &self.spi.rxdr as *const _ as *const $TY,
                        )
                    });
                } else {
                    nb::Error::WouldBlock
                });
            } else {
                return Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.rxne().bit_is_set() {
                    // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                    // reading a half-word)
                    return Ok(unsafe {
                        ptr::read_volatile(&self.regs.dr as *const _ as *const u8)
                    });
                } else {
                    nb::Error::WouldBlock
                });
            }
        }
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        let sr = self.regs.sr.read();

        // todo: DRY between H7 and non-H7 branches here
        cfg_if! {
            if #[cfg(feature = "h7")] {
                return  Err(if sr.ovr().is_overrun() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().is_fault() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crce().is_error() {
                    nb::Error::Other(Error::Crc)
                } else if sr.txp().is_not_full() {
                    // NOTE(write_volatile) see note above
                    unsafe {
                        ptr::write_volatile(
                            &self.spi.txdr as *const _ as *mut $TY,
                            byte,
                        )
                    }
                    // write CSTART to start a transaction in
                    // master mode
                    self.spi.cr1.modify(|_, w| w.cstart().started());

                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                });
            } else {
                return Err(if sr.ovr().bit_is_set() {
                    nb::Error::Other(Error::Overrun)
                } else if sr.modf().bit_is_set() {
                    nb::Error::Other(Error::ModeFault)
                } else if sr.crcerr().bit_is_set() {
                    nb::Error::Other(Error::Crc)
                } else if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    unsafe { ptr::write_volatile(&self.regs.dr as *const _ as *mut u8, byte) }
                    return Ok(());
                } else {
                    nb::Error::WouldBlock
                });
            }
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
