//! This module allows for serial communication using the STM32 U(S)ART peripheral
//! It provides APIs to configure, read, and write from
//! U(S)ART, with blocking, nonblocking, and DMA functionality.

// todo: Synchronous mode.
// todo: Auto baud

// todo: Missing some features (like additional interrupts) on the USARTv3 peripheral . (L5, G etc)

use core::ops::Deref;

use cfg_if::cfg_if;

#[cfg(not(any(feature = "f4", feature = "l552", feature = "h5")))]
use crate::dma::{self, ChannelCfg, DmaChannel};
#[cfg(any(feature = "c0", feature = "h5"))]
use crate::pac::DMA as DMA1;
#[cfg(not(any(feature = "h5", feature = "c0")))]
use crate::pac::DMA1;
// use crate::util::{cr1, isr};
use crate::{
    clocks::Clocks,
    error::{Error, Result},
    pac::{self, RCC},
    util::{BaudPeriph, RccPeriph, bounded_loop, cr1, isr},
};

// todo: Prescaler (USART_PRESC) register on v3 (L5, G, H etc)

/// Serial error
#[non_exhaustive]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum UsartError {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// The number of stop bits. (USART_CR2, STOP)
pub enum StopBits {
    S1 = 0b00,
    S0_5 = 0b01,
    S2 = 0b10,
    S1_5 = 0b11,
}

#[derive(Clone, Copy, PartialEq)]
/// Parity control enable/disable, and even/odd selection (USART_CR1, PCE and PS)
pub enum Parity {
    EnabledEven,
    EnabledOdd,
    Disabled,
}

#[derive(Clone, Copy)]
/// The length of word to transmit and receive. (USART_CR1, M)
pub enum WordLen {
    W8,
    W9,
    W7,
}

impl WordLen {
    /// We use this function due to the M field being split into 2 separate bits.
    /// Returns M1 val, M0 val
    pub fn bits(&self) -> (u8, u8) {
        match self {
            Self::W8 => (0, 0),
            Self::W9 => (0, 1),
            Self::W7 => (1, 0),
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Set Oversampling16 or Oversampling8 modes.
pub enum OverSampling {
    O16 = 0,
    O8 = 1,
}

#[derive(Clone, Copy, PartialEq)]
pub enum IrdaMode {
    /// "IrDA mode disabled
    None,
    /// "IrDA SIR rx/tx enabled in 'normal' mode"
    Normal,
    /// "IrDA SIR 'low-power' mode"
    LowPower,
}

#[cfg(not(feature = "f4"))]
#[derive(Clone, Copy)]
/// The type of USART interrupt to configure. Reference the USART_ISR register.
pub enum UsartInterrupt {
    /// If the inner value of this is `Some`, its inner value will set
    /// the character to match on `enable_interrupt`. The option's value doesn't
    /// affect anything when stopping or clearing interrupts.
    CharDetect(Option<u8>),
    Cts,
    EndOfBlock,
    Idle,
    FramingError,
    LineBreak,
    Overrun,
    ParityError,
    ReadNotEmpty,
    ReceiverTimeout,
    #[cfg(not(any(feature = "f3", feature = "l4")))] // todo: PAC ommission?
    Tcbgt,
    TransmissionComplete,
    TransmitEmpty,
}

/// Configuration for U(S)ART. and LpUart. Can be used with default::Default.
pub struct UsartConfig {
    /// Word length. Defaults to 8-bits.
    pub word_len: WordLen,
    /// Stop bits: Defaults to 1.
    pub stop_bits: StopBits,
    /// Oversampling rate. Defaults to 16x. N/A for LpUart.
    pub oversampling: OverSampling,
    /// Enable or disable parity control. Defaults to disabled.
    pub parity: Parity,
    /// IrDA mode: Enables this protocol, which is used to communicate with IR devices.
    /// N/A for LpUart.
    pub irda_mode: IrdaMode,
    #[cfg(any(feature = "g4", feature = "h7"))]
    /// The first-in, first-out buffer is enabled. Defaults to enabled.
    pub fifo_enabled: bool,
    #[cfg(not(feature = "f4"))]
    /// Optionally, disable the overrun functionality. Defaults to `false`.
    pub overrun_disabled: bool,
}

impl Default for UsartConfig {
    fn default() -> Self {
        Self {
            word_len: WordLen::W8,
            stop_bits: StopBits::S1,
            oversampling: OverSampling::O16,
            parity: Parity::Disabled,
            irda_mode: IrdaMode::None,
            #[cfg(any(feature = "g4", feature = "h7"))]
            fifo_enabled: true,
            #[cfg(not(feature = "f4"))]
            overrun_disabled: false,
        }
    }
}

// Sep macros due to too many feature-gates.
// todo: Use functions?

// todo: Support fifo_disabled regs.

/// Represents the USART peripheral, for serial communications.
pub struct Usart<R> {
    pub regs: R,
    baud: u32,
    config: UsartConfig,
}

impl<R> Usart<R>
where
    R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph + BaudPeriph,
{
    /// Initialize a U(S)ART peripheral, including configuration register writes, and enabling and
    /// resetting its RCC peripheral clock. `baud` is the baud rate, in bytes-per-second.
    pub fn new(regs: R, baud: u32, config: UsartConfig, clock_cfg: &Clocks) -> Result<Self> {
        let rcc = unsafe { &(*RCC::ptr()) };
        R::en_reset(rcc);

        let mut usart = Self { regs, baud, config };

        // This should already be disabled on power up, but disable here just in case;
        // some bits can't be set with USART enabled.

        usart.disable()?;

        // Set up transmission. See L44 RM, section 38.5.2: "Character Transmission Procedures".
        // 1. Program the M bits in USART_CR1 to define the word length.

        let word_len_bits = usart.config.word_len.bits();
        cr1!(usart.regs).modify(|_, w| {
            w.over8().bit(usart.config.oversampling as u8 != 0);
            w.pce().bit(usart.config.parity != Parity::Disabled);
            cfg_if! {
                if #[cfg(not(any(feature = "f", feature = "wl")))] {
                    w.m1().bit(word_len_bits.0 != 0);
                    w.m0().bit(word_len_bits.1 != 0);
                    return w.ps().bit(usart.config.parity == Parity::EnabledOdd);
                } else if #[cfg(feature = "f")] {
                    // todo: QC this.
                    w.m().bit(word_len_bits.0 != 0);
                    return w.ps().bit(usart.config.parity == Parity::EnabledOdd);
                } else {
                    return w.ps().bit(usart.config.parity == Parity::EnabledOdd);
                }
            }
        });

        #[cfg(not(feature = "f4"))]
        usart
            .regs
            .cr3()
            .modify(|_, w| w.ovrdis().bit(usart.config.overrun_disabled));

        // Must be done before enabling.
        #[cfg(any(feature = "g4", feature = "h7"))]
        usart
            .regs
            .cr1()
            .modify(|_, w| w.fifoen().bit(usart.config.fifo_enabled));

        // 2. Select the desired baud rate using the USART_BRR register.
        usart.set_baud(baud, clock_cfg).ok();
        // 3. Program the number of stop bits in USART_CR2.
        usart
            .regs
            .cr2()
            .modify(|_, w| unsafe { w.stop().bits(usart.config.stop_bits as u8) });
        // 4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
        usart.enable()?;

        // 5. Select DMA enable (DMAT[R]] in USART_CR3 if multibuffer communication is to take
        // place. Configure the DMA register as explained in multibuffer communication.
        // (Handled in `read_dma()` and `write_dma()`)
        // 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
        // 6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
        // start bit.

        cr1!(usart.regs).modify(|_, w| {
            w.te().bit(true);
            w.re().bit(true)
        });

        match usart.config.irda_mode {
            // See G4 RM, section 37.5.18: USART IrDA SIR ENDEC block
            // " IrDA mode is selected by setting the IREN bit in the USART_CR3 register. In IrDA mode,
            // the following bits must be kept cleared:
            // • LINEN, STOP and CLKEN bits in the USART_CR2 register,
            IrdaMode::None => (),
            _ => {
                usart.regs.cr2().modify(|_, w| unsafe {
                    w.linen().clear_bit();
                    w.stop().bits(0);
                    w.clken().clear_bit()
                });

                // • SCEN and HDSEL bits in the USART_CR3 register."
                usart.regs.cr3().modify(|_, w| {
                    w.scen().clear_bit();
                    w.hdsel().clear_bit();
                    w.irlp().bit(usart.config.irda_mode == IrdaMode::LowPower);
                    w.iren().bit(true)
                });
            }
        }

        Ok(usart)
    }
}

impl<R> Usart<R>
where
    R: Deref<Target = pac::usart1::RegisterBlock> + BaudPeriph,
{
    /// Set the BAUD rate. Called during init, and can be called later to change BAUD
    /// during program execution.
    pub fn set_baud(&mut self, baud: u32, clock_cfg: &Clocks) -> Result<()> {
        let originally_enabled = cr1!(self.regs).read().ue().bit_is_set();

        if originally_enabled {
            cr1!(self.regs).modify(|_, w| w.ue().clear_bit());
            bounded_loop!(
                cr1!(self.regs).read().ue().bit_is_set(),
                Error::RegisterUnchanged
            );
        }

        // To set BAUD rate, see L4 RM section 38.5.4: "USART baud rate generation".
        // todo: This assumes the USART clock is APB1 or 2 depending on which USART.
        // todo: Take into account the selectable USART clock in both
        // todo util::baud implementation, and `clocks` module.
        let fclk = R::baud(clock_cfg);

        let usart_div = match self.config.oversampling {
            OverSampling::O16 => fclk / baud,
            OverSampling::O8 => 2 * fclk / baud,
        };

        // USARTDIV is an unsigned fixed point number that is coded on the USART_BRR register.
        // • When OVER8 = 0, BRR = USARTDIV.
        // • When OVER8 = 1
        // – BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right.
        // – BRR[3] must be kept cleared.
        // – BRR[15:4] = USARTDIV[15:4]
        // todo: BRR needs to be modified per the above if on oversampling 8.

        #[cfg(feature = "f4")]
        let div = usart_div as u16;
        #[cfg(not(feature = "f4"))]
        // let div = usart_div as u32;
        let div = usart_div;

        self.regs.brr().write(|w| unsafe { w.bits(div) });

        self.baud = baud;

        if originally_enabled {
            cr1!(self.regs).modify(|_, w| w.ue().bit(true));
        }

        Ok(())
    }
}

impl<R> Usart<R>
where
    R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph,
{
    /// Enable this U(S)ART peripheral.
    pub fn enable(&mut self) -> Result<()> {
        cr1!(self.regs).modify(|_, w| w.ue().bit(true));
        bounded_loop!(
            cr1!(self.regs).read().ue().bit_is_clear(),
            Error::RegisterUnchanged
        );

        Ok(())
    }

    /// Disable this U(S)ART peripheral.
    pub fn disable(&mut self) -> Result<()> {
        cr1!(self.regs).modify(|_, w| w.ue().clear_bit());
        bounded_loop!(
            cr1!(self.regs).read().ue().bit_is_set(),
            Error::RegisterUnchanged
        );
        Ok(())
    }

    /// Transmit data, as a sequence of u8. See L44 RM, section 38.5.2: "Character transmission procedure"
    pub fn write(&mut self, data: &[u8]) -> Result<()> {
        // todo: how does this work with a 9 bit words? Presumably you'd need to make `data`
        // todo take `&u16`.

        // 7. Write the data to send in the USART_TDR register (this clears the TXE bit). Repeat this
        // for each data to be transmitted in case of single buffer.

        for word in data {
            cfg_if! {
                if #[cfg(any(
                    feature = "h5",
                    feature = "c0",
                    feature = "g050", feature = "g051", feature = "g061")
                )] {
                    bounded_loop!(
                        isr!(self.regs).read().txfe().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                } else if #[cfg(feature = "f4")] {
                    bounded_loop!(
                        isr!(self.regs).read().tc().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                    bounded_loop!(
                        isr!(self.regs).read().txe().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                } else {
                    bounded_loop!(
                        isr!(self.regs).read().txe().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                }
            }

            // 8. After writing the last data into the USART_TDR register, wait until TC=1. This indicates
            // that the transmission of the last frame is complete. This is required for instance when
            // the USART is disabled or enters the Halt mode to avoid corrupting the last
            // transmission
            cfg_if! {
                if #[cfg(feature = "f4")] {
                    self.regs
                        .dr()
                        .modify(|_, w| unsafe { w.dr().bits(*word as u16) });
                    bounded_loop!(
                        self.regs.sr().read().tc().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                } else {
                    // Note: Per these PACs, TXFNF and TXE are on the same field, so this is actually
                    // checking txfnf if the fifo is enabled.
                    self.regs
                        .tdr()
                        .modify(|_, w| unsafe { w.tdr().bits(*word as u16) });
                }
            }
        }

        Ok(())
    }

    /// Write a single word, without waiting until ready for the next. Compared to the `write()` function, this
    /// does not block.
    pub fn write_one(&mut self, word: u8) {
        // todo: how does this work with a 9 bit words? Presumably you'd need to make `data`
        // todo take `&u16`.
        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
            self.regs
                .tdr()
                .modify(|_, w| unsafe { w.tdr().bits(word as u16) });
            } else {
                self.regs
                    .dr()
                    .modify(|_, w| unsafe { w.dr().bits(word as u16) });
            }
        }
    }

    /// Receive data into a u8 buffer. See L44 RM, section 38.5.3: "Character reception procedure"
    pub fn read(&mut self, buf: &mut [u8]) -> Result<()> {
        for i in 0..buf.len() {
            // Wait for the next bit

            cfg_if! {
                if #[cfg(any(
                    feature = "h5",
                    feature = "c0",
                    feature = "g050", feature = "g051", feature = "g061")
                )] {
                    bounded_loop!(
                        isr!(self.regs).read().rxfne().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                } else {
                    bounded_loop!(
                        isr!(self.regs).read().rxne().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                }
            }

            #[cfg(not(feature = "f4"))]
            {
                buf[i] = self.regs.rdr().read().rdr().bits() as u8;
            }
            #[cfg(feature = "f4")]
            {
                buf[i] = self.regs.dr().read().dr().bits() as u8;
            }
        }

        // When a character is received:
        // • The RXNE bit is set to indicate that the content of the shift register is transferred to the
        // RDR. In other words, data has been received and can be read (as well as its
        // associated error flags).
        // • An interrupt is generated if the RXNEIE bit is set.
        // • The error flags can be set if a frame error, noise or an overrun error has been detected
        // during reception. PE flag can also be set with RXNE.
        // • In multibuffer, RXNE is set after every byte received and is cleared by the DMA read of
        // the Receive data Register.
        // • In single buffer mode, clearing the RXNE bit is performed by a software read to the
        // USART_RDR register. The RXNE flag can also be cleared by writing 1 to the RXFRQ
        // in the USART_RQR register. The RXNE bit must be cleared before the end of the
        // reception of the next character to avoid an overrun error

        Ok(())
    }

    /// Read a single word, without waiting until ready for the next. Compared to the `read()` function, this
    /// does not block.
    pub fn read_one(&mut self) -> u8 {
        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
                self.regs.rdr().read().rdr().bits() as u8
            } else {
                self.regs.dr().read().dr().bits() as u8
            }
        }
    }

    #[cfg(not(any(feature = "f4", feature = "l552", feature = "h5")))]
    /// Transmit data using DMA. (L44 RM, section 38.5.15)
    /// Note that the `channel` argument is unused on F3 and L4, since it is hard-coded,
    /// and can't be configured using the DMAMUX peripheral. (`dma::mux()` fn).
    pub unsafe fn write_dma(
        &mut self,
        buf: &[u8],
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma_periph: dma::DmaPeriph,
    ) -> Result<()> {
        let (ptr, len) = (buf.as_ptr(), buf.len());

        // To map a DMA channel for USART transmission, use
        // the following procedure (x denotes the channel number):

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = R::write_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);

        let num_data = len as u32;

        // "DMA mode can be enabled for transmission by setting DMAT bit in the USART_CR3
        // register. Data is loaded from a SRAM area configured using the DMA peripheral (refer to
        // Section 11: Direct memory access controller (DMA) on page 295) to the USART_TDR
        // register whenever the TXE bit is set."
        self.regs.cr3().modify(|_, w| w.dmat().bit(true));

        // 6. Clear the TC flag in the USART_ISR register by setting the TCCF bit in the
        // USART_ICR register.
        self.regs.icr().write(|w| w.tccf().bit(true));

        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    // 1. Write the USART_TDR register address in the DMA control register to configure it as
                    // the destination of the transfer. The data is moved to this address from memory after
                    // each TXE event.
                    self.regs.tdr().as_ptr() as u32,
                    // 2. Write the memory address in the DMA control register to configure it as the source of
                    // the transfer. The data is loaded into the USART_TDR register from this memory area
                    // after each TXE event.
                    ptr as u32,
                    // 3. Configure the total number of bytes to be transferred to the DMA control register.
                    num_data,
                    dma::Direction::ReadFromMem,
                    // 4. Configure the channel priority in the DMA control register
                    // (Handled by `ChannelCfg::default())`
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                )
            }
            #[cfg(dma2)]
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    self.regs.tdr().as_ptr() as u32,
                    ptr as u32,
                    num_data,
                    dma::Direction::ReadFromMem,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                )
            }
        }

        // 5. Configure DMA interrupt generation after half/ full transfer as required by the
        // application.
        // (Handled in `cfg_channel`)

        // 7. Activate the channel in the DMA register.
        // When the number of data transfers programmed in the DMA Controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector.
        // (Handled in `cfg_channel`)

        // In transmission mode, once the DMA has written all the data to be transmitted (the TCIF flag
        // is set in the DMA_ISR register), the TC flag can be monitored to make sure that the USART
        // communication is complete. This is required to avoid corrupting the last transmission before
        // disabling the USART or entering Stop mode. Software must wait until TC=1. The TC flag
        // remains cleared during all data transfers and it is set by hardware at the end of transmission
        // of the last frame.
    }

    #[cfg(not(any(feature = "f4", feature = "l552", feature = "h5")))]
    /// Receive data using DMA. (L44 RM, section 38.5.15; G4 RM section 37.5.19.
    /// Note that the `channel` argument is unused on F3 and L4, since it is hard-coded,
    /// and can't be configured using the DMAMUX peripheral. (`dma::mux()` fn).
    pub unsafe fn read_dma(
        &mut self,
        buf: &mut [u8],
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma_periph: dma::DmaPeriph,
    ) -> Result<()> {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = R::read_chan();
        #[cfg(feature = "l4")]
        let mut dma_regs = unsafe { &(*DMA1::ptr()) }; // todo: Hardcoded DMA1
        #[cfg(feature = "l4")]
        R::write_sel(&mut dma_regs);

        let num_data = len as u32;

        // DMA mode can be enabled for reception by setting the DMAR bit in USART_CR3 register.
        self.regs.cr3().modify(|_, w| w.dmar().bit(true));

        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    // 1. Write the USART_RDR register address in the DMA control register to configure it as
                    // the source of the transfer. The data is moved from this address to the memory after
                    // each RXNE event.
                    self.regs.rdr().as_ptr() as u32,
                    // 2. Write the memory address in the DMA control register to configure it as the destination
                    // of the transfer. The data is loaded from USART_RDR to this memory area after each
                    // RXNE event.
                    ptr as u32,
                    // 3. Configure the total number of bytes to be transferred to the DMA control register.
                    num_data,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                )
            }
            #[cfg(dma2)]
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    channel,
                    self.regs.rdr().as_ptr() as u32,
                    ptr as u32,
                    num_data,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S8,
                    dma::DataSize::S8,
                    channel_cfg,
                )
            }
        }

        // 4. Configure the channel priority in the DMA control register
        // (Handled in cfg)

        // 5. Configure interrupt generation after half/ full transfer as required by the application.
        // (Handled by user code))

        // 6. Activate the channel in the DMA control register. (Handled by `cfg_channel` above).
        // When the number of data transfers programmed in the DMA Controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector.
        // (Handled in above fn call)

        // When the number of data transfers programmed in the DMA Controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector.
    }

    // /// Flush the transmit buffer.
    // pub fn flush(&self) {
    //     #[cfg(not(feature = "f4"))]
    //     while isr!(self.regs).read().tc().bit_is_clear() {}
    //     #[cfg(feature = "f4")]
    //     while self.regs.sr().read().tc().bit_is_clear() {}
    // }

    #[cfg(not(feature = "f4"))]
    /// Enable a specific type of interrupt. See G4 RM, Table 349: USART interrupt requests.
    /// If `Some`, the inner value of `CharDetect` sets the address of the char to match.
    /// If `None`, the interrupt is enabled without changing the char to match.
    pub fn enable_interrupt(&mut self, interrupt: UsartInterrupt) -> Result<()> {
        match interrupt {
            UsartInterrupt::CharDetect(char_wrapper) => {
                if let Some(char) = char_wrapper {
                    // Disable the UART to allow writing the `add` and `addm7` bits
                    cr1!(self.regs).modify(|_, w| w.ue().clear_bit());
                    bounded_loop!(
                        cr1!(self.regs).read().ue().bit_is_set(),
                        Error::RegisterUnchanged
                    );

                    // Enable character-detecting UART interrupt
                    cr1!(self.regs).modify(|_, w| w.cmie().bit(true));

                    // Allow an 8-bit address to be set in `add`.
                    self.regs.cr2().modify(|_, w| unsafe {
                        w.addm7().bit(true);
                        // Set the character to detect
                        w.add().bits(char)
                    });

                    cr1!(self.regs).modify(|_, w| w.ue().bit(true));
                    bounded_loop!(
                        cr1!(self.regs).read().ue().bit_is_clear(),
                        Error::RegisterUnchanged
                    );
                }

                cr1!(self.regs).modify(|_, w| w.cmie().bit(true));
            }
            UsartInterrupt::Cts => {
                self.regs.cr3().modify(|_, w| w.ctsie().bit(true));
            }
            UsartInterrupt::EndOfBlock => {
                cr1!(self.regs).modify(|_, w| w.eobie().bit(true));
            }
            UsartInterrupt::Idle => {
                cr1!(self.regs).modify(|_, w| w.idleie().bit(true));
            }
            UsartInterrupt::FramingError => {
                self.regs.cr3().modify(|_, w| w.eie().bit(true));
            }
            UsartInterrupt::LineBreak => {
                self.regs.cr2().modify(|_, w| w.lbdie().bit(true));
            }
            UsartInterrupt::Overrun => {
                self.regs.cr3().modify(|_, w| w.eie().bit(true));
            }
            UsartInterrupt::ParityError => {
                cr1!(self.regs).modify(|_, w| w.peie().bit(true));
            }
            UsartInterrupt::ReadNotEmpty => {
                #[cfg(feature = "h5")]
                cr1!(self.regs).modify(|_, w| w.rxfneie().bit(true));
                #[cfg(not(feature = "h5"))]
                cr1!(self.regs).modify(|_, w| w.rxneie().bit(true));
            }
            UsartInterrupt::ReceiverTimeout => {
                cr1!(self.regs).modify(|_, w| w.rtoie().bit(true));
            }
            #[cfg(not(any(feature = "f3", feature = "l4")))]
            UsartInterrupt::Tcbgt => {
                self.regs.cr3().modify(|_, w| w.tcbgtie().bit(true));
                self.regs.cr3().modify(|_, w| w.tcbgtie().bit(true));
            }
            UsartInterrupt::TransmissionComplete => {
                cr1!(self.regs).modify(|_, w| w.tcie().bit(true));
            }
            UsartInterrupt::TransmitEmpty => {
                #[cfg(feature = "h5")]
                cr1!(self.regs).modify(|_, w| w.txfeie().bit(true));
                #[cfg(not(feature = "h5"))]
                cr1!(self.regs).modify(|_, w| w.txeie().bit(true));
            }
        }
        Ok(())
    }

    #[cfg(not(feature = "f4"))]
    /// Disable a specific type of interrupt. See G4 RM, Table 349: USART interrupt requests.
    /// Note that the inner value of `CharDetect` doesn't do anything here.
    pub fn disable_interrupt(&mut self, interrupt: UsartInterrupt) {
        match interrupt {
            UsartInterrupt::CharDetect(_) => {
                cr1!(self.regs).modify(|_, w| w.cmie().clear_bit());
            }
            UsartInterrupt::Cts => {
                self.regs.cr3().modify(|_, w| w.ctsie().clear_bit());
            }
            UsartInterrupt::EndOfBlock => {
                cr1!(self.regs).modify(|_, w| w.eobie().clear_bit());
            }
            UsartInterrupt::Idle => {
                cr1!(self.regs).modify(|_, w| w.idleie().clear_bit());
            }
            UsartInterrupt::FramingError => {
                self.regs.cr3().modify(|_, w| w.eie().clear_bit());
            }
            UsartInterrupt::LineBreak => {
                self.regs.cr2().modify(|_, w| w.lbdie().clear_bit());
            }
            UsartInterrupt::Overrun => {
                self.regs.cr3().modify(|_, w| w.eie().clear_bit());
            }
            UsartInterrupt::ParityError => {
                cr1!(self.regs).modify(|_, w| w.peie().clear_bit());
            }
            UsartInterrupt::ReadNotEmpty => {
                #[cfg(feature = "h5")]
                cr1!(self.regs).modify(|_, w| w.rxfneie().clear_bit());
                #[cfg(not(feature = "h5"))]
                cr1!(self.regs).modify(|_, w| w.rxneie().clear_bit());
            }
            UsartInterrupt::ReceiverTimeout => {
                cr1!(self.regs).modify(|_, w| w.rtoie().clear_bit());
            }
            #[cfg(not(any(feature = "f3", feature = "l4")))]
            UsartInterrupt::Tcbgt => {
                self.regs.cr3().modify(|_, w| w.tcbgtie().clear_bit());
                self.regs.cr3().modify(|_, w| w.tcbgtie().clear_bit());
            }
            UsartInterrupt::TransmissionComplete => {
                cr1!(self.regs).modify(|_, w| w.tcie().clear_bit());
            }
            UsartInterrupt::TransmitEmpty => {
                #[cfg(feature = "h5")]
                cr1!(self.regs).modify(|_, w| w.txfeie().clear_bit());
                #[cfg(not(feature = "h5"))]
                cr1!(self.regs).modify(|_, w| w.txeie().clear_bit());
            }
        }
    }

    #[cfg(not(feature = "f4"))]
    /// Print the (raw) contents of the status register.
    pub fn read_status(&self) -> u32 {
        unsafe { isr!(self.regs).read().bits() }
    }

    #[cfg(not(feature = "f4"))]
    /// Clears the interrupt pending flag for a specific type of interrupt. Note that
    /// it can also clear error flags, like Overrun and framing errors. See G4 RM,
    /// Table 349: USART interrupt requests.
    /// Note that the inner value of `CharDetect` doesn't do anything here.
    pub fn clear_interrupt(&mut self, interrupt: UsartInterrupt) {
        match interrupt {
            UsartInterrupt::CharDetect(_) => self.regs.icr().write(|w| w.cmcf().bit(true)),
            UsartInterrupt::Cts => self.regs.icr().write(|w| w.ctscf().bit(true)),
            UsartInterrupt::EndOfBlock => self.regs.icr().write(|w| w.eobcf().bit(true)),
            UsartInterrupt::Idle => self.regs.icr().write(|w| w.idlecf().bit(true)),
            UsartInterrupt::FramingError => self.regs.icr().write(|w| w.fecf().bit(true)),
            UsartInterrupt::LineBreak => self.regs.icr().write(|w| w.lbdcf().bit(true)),
            UsartInterrupt::Overrun => self.regs.icr().write(|w| w.orecf().bit(true)),
            UsartInterrupt::ParityError => self.regs.icr().write(|w| w.pecf().bit(true)),
            UsartInterrupt::ReadNotEmpty => self.regs.rqr().write(|w| w.rxfrq().bit(true)),
            UsartInterrupt::ReceiverTimeout => self.regs.icr().write(|w| w.rtocf().bit(true)),
            #[cfg(not(any(feature = "f3", feature = "l4", feature = "h7")))]
            UsartInterrupt::Tcbgt => self.regs.icr().write(|w| w.tcbgtcf().bit(true)),
            #[cfg(feature = "h7")]
            UsartInterrupt::Tcbgt => self.regs.icr().write(|w| w.tcbgtcf().bit(true)),
            UsartInterrupt::TransmissionComplete => self.regs.icr().write(|w| w.tccf().bit(true)),
            UsartInterrupt::TransmitEmpty => self.regs.rqr().write(|w| w.txfrq().bit(true)),
        };
    }

    #[cfg(not(feature = "f4"))]
    /// Checks if a given status flag is set. Returns `true` if the status flag is set. Note that this preforms
    /// a read each time called. If checking multiple flags, this isn't optimal.
    pub fn check_status_flag(&mut self, flag: UsartInterrupt) -> bool {
        let status = isr!(self.regs).read();

        match flag {
            UsartInterrupt::CharDetect(_) => status.cmf().bit_is_set(),
            UsartInterrupt::Cts => status.cts().bit_is_set(),
            UsartInterrupt::EndOfBlock => status.eobf().bit_is_set(),
            UsartInterrupt::Idle => status.idle().bit_is_set(),
            UsartInterrupt::FramingError => status.fe().bit_is_set(),
            UsartInterrupt::LineBreak => status.lbdf().bit_is_set(),
            UsartInterrupt::Overrun => status.ore().bit_is_set(),
            UsartInterrupt::ParityError => status.pe().bit_is_set(),
            #[cfg(any(
                feature = "h5",
                feature = "c0",
                feature = "g050",
                feature = "g051",
                feature = "g061"
            ))]
            UsartInterrupt::ReadNotEmpty => status.rxfne().bit_is_set(),
            #[cfg(not(any(
                feature = "h5",
                feature = "c0",
                feature = "g050",
                feature = "g051",
                feature = "g061"
            )))]
            UsartInterrupt::ReadNotEmpty => status.rxne().bit_is_set(),
            UsartInterrupt::ReceiverTimeout => status.rtof().bit_is_set(),
            #[cfg(not(any(feature = "f3", feature = "l4")))]
            UsartInterrupt::Tcbgt => status.tcbgt().bit_is_set(),
            UsartInterrupt::TransmissionComplete => status.tc().bit_is_set(),
            #[cfg(any(
                feature = "h5",
                feature = "c0",
                feature = "g050",
                feature = "g051",
                feature = "g061"
            ))]
            UsartInterrupt::TransmitEmpty => status.txfe().bit_is_set(),
            #[cfg(not(any(
                feature = "h5",
                feature = "c0",
                feature = "g050",
                feature = "g051",
                feature = "g061"
            )))]
            UsartInterrupt::TransmitEmpty => status.txe().bit_is_set(),
        }
    }

    fn check_status(&mut self) -> core::result::Result<(), UsartError> {
        cfg_if! {
            if #[cfg(feature = "f4")] {
                let status = self.regs.sr().read();
            } else {
                let status = self.regs.isr().read();
            }
        }
        let mut result = if status.pe().bit_is_set() {
            Err(UsartError::Parity)
        } else if status.fe().bit_is_set() {
            Err(UsartError::Framing)
        } else if status.ore().bit_is_set() {
            Err(UsartError::Overrun)
        } else {
            Ok(())
        };

        #[cfg(not(any(
            feature = "wl",
            feature = "c0",
            feature = "g050",
            feature = "g051",
            feature = "g061"
        )))]
        if status.nf().bit_is_set() {
            result = Err(UsartError::Noise);
        }

        #[cfg(any(feature = "c0", feature = "g050", feature = "g051", feature = "g061"))]
        if status.ne().bit_is_set() {
            result = Err(UsartError::Noise);
        }

        if result.is_err() {
            // For F4, clear error flags by reading SR and DR
            // For others, clear error flags by reading ISR, clearing ICR, then reading RDR
            cfg_if! {
                if #[cfg(feature = "f4")] {
                    let _ = self.regs.dr().read();
                } else {
                    self.regs.icr().write(|w| {
                        w.pecf().bit(true);
                        w.fecf().bit(true);
                        #[cfg(not(any(feature = "c0", feature = "g050", feature = "g051", feature = "g061")))]
                        w.ncf().bit(true);
                        #[cfg(any(feature = "c0", feature = "g050", feature = "g051", feature = "g061"))]
                        w.necf().bit(true);
                        w.orecf().bit(true)
                    });
                    let _ = self.regs.rdr().read();
                }
            }
        }
        result
    }
}

#[cfg(feature = "embedded_hal")]
mod embedded_io_impl {
    use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};

    use super::*;

    impl<R> ErrorType for Usart<R> {
        type Error = crate::error::Error;
    }

    impl<R> Read for Usart<R>
    where
        R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph + BaudPeriph,
        Usart<R>: ReadReady,
    {
        fn read(&mut self, mut buf: &mut [u8]) -> core::result::Result<usize, Self::Error> {
            // Block until at least one byte can be read:
            while !self.read_ready()? {
                cortex_m::asm::nop();
            }

            let buf_len = buf.len();
            while !buf.is_empty() && self.read_ready()? {
                let (first, remaining) = buf.split_first_mut().unwrap();
                *first = self.read_one();
                buf = remaining;
            }
            Ok(buf_len - buf.len())
        }
    }

    impl<R> ReadReady for Usart<R>
    where
        R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph + BaudPeriph,
    {
        fn read_ready(&mut self) -> core::result::Result<bool, Self::Error> {
            self.check_status()?;
            cfg_if! {
                if #[cfg(any(
                    feature = "h5",
                    feature = "c0",
                    feature = "g050", feature = "g051", feature = "g061")
                )] {
                    let ready = self.regs.isr().read().rxfne().bit_is_set();
                } else if #[cfg(feature = "f4")] {
                    let ready = self.regs.sr().read().rxne().bit_is_set();
                } else {
                    let ready = self.regs.isr().read().rxne().bit_is_set();
                }
            };
            Ok(ready)
        }
    }

    impl<R> Write for Usart<R>
    where
        R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph + BaudPeriph,
    {
        fn write(&mut self, mut buf: &[u8]) -> core::result::Result<usize, Self::Error> {
            // Block until at least one byte can be written:
            while !self.write_ready()? {
                cortex_m::asm::nop();
            }

            let buf_len = buf.len();
            while !buf.is_empty() && self.write_ready()? {
                let (byte, remaining) = buf.split_first().unwrap();
                self.write_one(*byte);
                buf = remaining;
            }
            Ok(buf_len - buf.len())
        }

        fn flush(&mut self) -> core::result::Result<(), Self::Error> {
            #[cfg(not(feature = "f4"))]
            while isr!(self.regs).read().tc().bit_is_clear() {}
            #[cfg(feature = "f4")]
            while self.regs.sr().read().tc().bit_is_clear() {}
            Ok(())
        }
    }

    impl<R> WriteReady for Usart<R>
    where
        R: Deref<Target = pac::usart1::RegisterBlock> + RccPeriph + BaudPeriph,
    {
        fn write_ready(&mut self) -> core::result::Result<bool, Self::Error> {
            cfg_if! {
                if #[cfg(any(
                    feature = "h5",
                    feature = "c0",
                    feature = "g050", feature = "g051", feature = "g061")
                )] {
                    let ready = self.regs.isr().read().txfe().bit_is_set();
                } else if #[cfg(feature = "f4")] {
                    let ready = self.regs.sr().read().txe().bit_is_set();
                } else {
                    let ready = self.regs.isr().read().txe().bit_is_set();
                }
            };
            Ok(ready)
        }
    }
}
