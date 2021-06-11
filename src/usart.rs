//! This module allows for serial communication using the STM32 USART module.
//! Supports Embedded HAL `Read` and `Write` traits.

// todo: Synchronous mode.
// todo: Auto baud

// todo: Missing some features (like additional interrupts) on the USARTv3 peripheral . (L5, G etc)

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};
use core::ops::Deref;

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(feature = "f3", feature = "l4", feature = "g4", feature = "wb"))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
use crate::dma::{self, Dma, DmaChannel, DmaInput};

use embedded_hal::{
    blocking,
    serial::{Read, Write},
};

use cfg_if::cfg_if;

// todo: Prescaler (USART_PRESC) register on v3 (L5, G, H etc)

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
    fn bits(&self) -> (u8, u8) {
        match self {
            Self::W8 => (0, 0),
            Self::W9 => (0, 1),
            Self::W7 => (1, 0),
        }
    }
}

#[derive(Clone, Copy)]
/// Specify the Usart device to use. Used internally for setting the appropriate APB.
pub enum UsartDevice {
    One,
    #[cfg(not(feature = "wb"))]
    Two,
    #[cfg(not(any(
        feature = "f401",
        feature = "f410",
        feature = "f411",
        feature = "f412",
        feature = "f413",
        feature = "l4x1",
        feature = "g0",
        feature = "wb",
    )))]
    Three,
    // Four,  todo
    // Five,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Set Oversampling16 or Oversampling8 modes.
pub enum OverSampling {
    O16 = 0,
    O8 = 1,
}

#[cfg(not(feature = "f4"))]
#[derive(Clone, Copy)]
/// The type of USART interrupt to configure. Reference the USART_ISR register.
pub enum UsartInterrupt {
    CharDetect(u8),
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

pub struct UsartConfig {
    word_len: WordLen,
    stop_bits: StopBits,
    oversampling: OverSampling,
    parity: Parity,
}

impl Default for UsartConfig {
    fn default() -> Self {
        Self {
            word_len: WordLen::W8,
            stop_bits: StopBits::S1,
            oversampling: OverSampling::O16,
            parity: Parity::Disabled,
        }
    }
}

/// Represents the USART peripheral, for serial communications.
pub struct Usart<U> {
    regs: U,
    device: UsartDevice,
    pub baud: u32,
    pub config: UsartConfig,
}

impl<U> Usart<U>
where
    U: Deref<Target = pac::usart1::RegisterBlock>,
{
    pub fn new<C: ClockCfg>(
        regs: U,
        device: UsartDevice,
        baud: u32,
        config: UsartConfig,
        clock_cfg: &C,
        rcc: &mut RCC,
    ) -> Self {
        match device {
            UsartDevice::One => {
                rcc_en_reset!(apb2, usart1, rcc);
            }
            #[cfg(not(feature = "wb"))]
            UsartDevice::Two => {
                cfg_if! {
                    if #[cfg(not(feature = "f4"))] {
                        rcc_en_reset!(apb1, usart2, rcc);
                    } else {
                        // Notice the `usart` vs `uart` asymmetry.
                        rcc.apb1enr.modify(|_, w| w.usart2en().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.uart2rst().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.uart2rst().clear_bit());
                    }
                }
            }
            #[cfg(not(any(
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "f412",
                feature = "f413",
                feature = "l4x1",
                feature = "g0",
                feature = "wb",
            )))]
            UsartDevice::Three => {
                cfg_if! {
                    if #[cfg(not(feature = "f4"))] {
                        rcc_en_reset!(apb1, usart3, rcc);
                    } else {
                        rcc.apb1enr.modify(|_, w| w.usart3en().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.uart3rst().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.uart3rst().clear_bit());
                    }
                }
            } // UsartDevice::Four => {
              //     rcc_en_reset!(apb1, uart4, rcc);
              // }
              // UsartDevice::Five => {
              //     rcc_en_reset!(apb1, usart5, rcc);
              // }
        }

        let mut result = Self {
            regs,
            device,
            baud,
            config,
        };

        // This should already be disabled on power up, but disable here just in case;
        // some bits can't be set with USART enabled.
        result.regs.cr1.modify(|_, w| w.ue().clear_bit());
        while result.regs.cr1.read().ue().bit_is_set() {}

        #[cfg(not(any(feature = "f3", feature = "f4")))]
        let word_len_bits = result.config.word_len.bits();

        // Set up transmission. See L44 RM, section 38.5.2: "Character Transmission Procedures".
        // 1. Program the M bits in USART_CR1 to define the word length.

        let word_len_bits = result.config.word_len.bits();
        result.regs.cr1.modify(|_, w| {
            w.over8().bit(result.config.oversampling as u8 != 0);
            w.pce().bit(result.config.parity != Parity::Disabled);
            cfg_if! {
                if #[cfg(not(any(feature = "f3", feature = "f4")))] {
                    w.m1().bit(word_len_bits.0 != 0);
                    w.m0().bit(word_len_bits.1 != 0);
                    return w.ps().bit(result.config.parity == Parity::EnabledOdd);
                } else {
                    return w.ps().bit(result.config.parity == Parity::EnabledOdd);
                }
            }
        });

        // todo: Workaround due to a PAC bug, where M0 is missing.
        #[cfg(any(feature = "f3", feature = "f4"))]
        result.regs.cr1.write(|w| unsafe {
            w.bits(
                result.regs.cr1.read().bits()
                    | ((word_len_bits.0 as u32) << 28)
                    | ((word_len_bits.1 as u32) << 12),
            )
        });

        // 2. Select the desired baud rate using the USART_BRR register.
        result.set_baud(baud, clock_cfg);
        // 3. Program the number of stop bits in USART_CR2.
        result
            .regs
            .cr2
            .modify(|_, w| unsafe { w.stop().bits(result.config.stop_bits as u8) });
        // 4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
        result.regs.cr1.modify(|_, w| w.ue().set_bit());
        // 5. Select DMA enable (DMAT[R]] in USART_CR3 if multibuffer communication is to take
        // place. Configure the DMA register as explained in multibuffer communication.
        // (Handled in `read_dma()` and `write_dma()`)
        // 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
        // 6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
        // start bit.
        result.regs.cr1.modify(|_, w| {
            w.te().set_bit();
            w.re().set_bit()
        });

        result
    }

    /// Set the BAUD rate. Called during init, and can be called later to change BAUD
    /// during program execution.
    pub fn set_baud<C: ClockCfg>(&mut self, baud: u32, clock_cfg: &C) {
        let originally_enabled = self.regs.cr1.read().ue().bit_is_set();

        if originally_enabled {
            self.regs.cr1.modify(|_, w| w.ue().clear_bit());
            while self.regs.cr1.read().ue().bit_is_set() {}
        }

        // To set BAUD rate, see L4 RM section 38.5.4: "USART baud rate generation".
        let fclk = match self.device {
            UsartDevice::One => clock_cfg.apb2(),
            _ => clock_cfg.apb1(),
        };

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

        self.regs.brr.write(|w| unsafe { w.bits(usart_div as u32) });

        self.baud = baud;

        if originally_enabled {
            self.regs.cr1.modify(|_, w| w.ue().set_bit());
        }
    }

    /// Transmit data, as a sequence of u8.. See L44 RM, section 38.5.2: "Character transmission procedure"
    pub fn write(&mut self, data: &[u8]) {
        // 7. Write the data to send in the USART_TDR register (this clears the TXE bit). Repeat this
        // for each data to be transmitted in case of single buffer.

        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
                for word in data {
                    while self.regs.isr.read().txe().bit_is_clear() {}
                    // todo: how does this work with a 9 bit words? Presumably you'd need to make `data`
                    // todo take `&u16`.
                    self.regs
                        .tdr
                        .modify(|_, w| unsafe { w.tdr().bits(*word as u16) });
                }
                // 8. After writing the last data into the USART_TDR register, wait until TC=1. This indicates
                // that the transmission of the last frame is complete. This is required for instance when
                // the USART is disabled or enters the Halt mode to avoid corrupting the last
                // transmission
                while self.regs.isr.read().tc().bit_is_clear() {}
            } else {
                for word in data {
                    while self.regs.sr.read().txe().bit_is_clear() {}
                    self.regs
                        .dr
                        .modify(|_, w| unsafe { w.dr().bits(*word as u16) });

                    // // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                    // unsafe {
                    //     ptr::write_volatile(*self.regs.dr, word)
                    // }
                }
                while self.regs.sr.read().tc().bit_is_clear() {}
            }
        }
    }

    /// Receive data into a u8 buffer. See L44 RM, section 38.5.3: "Character reception procedure"
    pub fn read(&mut self, buf: &mut [u8]) {
        for i in 0..buf.len() {
            // Wait for the next bit
            cfg_if! {
                if #[cfg(not(feature = "f4"))] {
                    while self.regs.isr.read().rxne().bit_is_clear() {}
                    buf[i] = self.regs.rdr.read().rdr().bits() as u8;
                } else {
                    while self.regs.sr.read().rxne().bit_is_clear() {}
                    buf[i] = self.regs.dr.read().dr().bits() as u8;
                }
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
    }

    /// Read a single word, without confirming if it's ready. This is useful in async concepts,
    /// when you know word is ready to be read.
    pub fn read_one(&mut self) -> u8 {
        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
                self.regs.rdr.read().rdr().bits() as u8
            } else {
                 self.regs.dr.read().dr().bits() as u8
            }
        }
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Transmit data using DMA. (L44 RM, section 38.5.15)
    /// Note that the `channel` argument is only used on F3 and L4.
    pub fn write_dma<D>(&mut self, buf: &[u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_ptr(), buf.len());

        // To map a DMA channel for USART transmission, use
        // the following procedure (x denotes the channel number):

        // todo: This channel selection is confirmed for L4 only - check other families
        // todo DMA channel mapping
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            UsartDevice::One => DmaInput::Usart1Tx.dma1_channel(),
            #[cfg(not(feature = "wb"))]
            UsartDevice::Two => DmaInput::Usart2Tx.dma1_channel(),
            #[cfg(not(any(
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "f412",
                feature = "f413",
                feature = "l4x1",
                feature = "g0",
                feature = "wb",
            )))]
            UsartDevice::Three => DmaInput::Usart3Tx.dma1_channel(),
        };

        // todo: These are only valid for DMA1!
        #[cfg(feature = "l4")]
        match self.device {
            UsartDevice::One => dma.channel_select(DmaInput::Usart1Tx),
            #[cfg(not(feature = "wb"))]
            UsartDevice::Two => dma.channel_select(DmaInput::Usart2Tx),
            #[cfg(not(feature = "l4x1"))]
            UsartDevice::Three => dma.channel_select(DmaInput::Usart3Tx),
        }

        // Note that we need neither channel select, nor multiplex for F3.

        // todo: Pri and Circular as args?

        dma.cfg_channel(
            channel,
            // 1. Write the USART_TDR register address in the DMA control register to configure it as
            // the destination of the transfer. The data is moved to this address from memory after
            // each TXE event.
            &self.regs.tdr as *const _ as u32,
            // 2. Write the memory address in the DMA control register to configure it as the source of
            // the transfer. The data is loaded into the USART_TDR register from this memory area
            // after each TXE event.
            ptr as u32,
            // 3. Configure the total number of bytes to be transferred to the DMA control register.
            len as u16,
            dma::Direction::ReadFromMem,
            // 4. Configure the channel priority in the DMA control register
            // (Handled by `ChannelCfg::default())`
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );

        // 5. Configure DMA interrupt generation after half/ full transfer as required by the
        // application.
        // (Handled in `cfg_channel`)

        // 6. Clear the TC flag in the USART_ISR register by setting the TCCF bit in the
        // USART_ICR register.
        self.regs.icr.write(|w| w.tccf().set_bit());

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

        // (this RM quote isn't part of the steps and section the others above are)
        // "DMA mode can be enabled for transmission by setting DMAT bit in the USART_CR3
        // register. Data is loaded from a SRAM area configured using the DMA peripheral (refer to
        // Section 11: Direct memory access controller (DMA) on page 295) to the USART_TDR
        // register whenever the TXE bit is set."
        self.regs.cr3.modify(|_, w| w.dmat().set_bit());
    }

    #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
    /// Receive data using DMA. (L44 RM, section 38.5.15)
    /// Note that the `channel` argument is only used on F3 and L4.
    pub fn read_dma<D>(&mut self, buf: &mut [u8], channel: DmaChannel, dma: &mut Dma<D>)
    where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            UsartDevice::One => DmaInput::Usart1Rx.dma1_channel(),
            #[cfg(not(feature = "wb"))]
            UsartDevice::Two => DmaInput::Usart2Rx.dma1_channel(),
            #[cfg(not(any(
                feature = "f401",
                feature = "f410",
                feature = "f411",
                feature = "f412",
                feature = "f413",
                feature = "l4x1",
                feature = "g0",
                feature = "wb",
            )))]
            UsartDevice::Three => DmaInput::Usart3Rx.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match self.device {
            UsartDevice::One => dma.channel_select(DmaInput::Usart1Rx),
            UsartDevice::Two => dma.channel_select(DmaInput::Usart2Rx),
            #[cfg(not(feature = "l4x1"))]
            UsartDevice::Three => dma.channel_select(DmaInput::Usart3Rx),
        }

        // todo: Pri and Circular as args?

        dma.cfg_channel(
            channel,
            // 1. Write the USART_RDR register address in the DMA control register to configure it as
            // the source of the transfer. The data is moved from this address to the memory after
            // each RXNE event.
            &self.regs.tdr as *const _ as u32,
            // 2. Write the memory address in the DMA control register to configure it as the destination
            // of the transfer. The data is loaded from USART_RDR to this memory area after each
            // RXNE event.
            ptr as u32,
            // 3. Configure the total number of bytes to be transferred to the DMA control register.
            len as u16,
            dma::Direction::ReadFromPeriph,
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );

        self.regs.cr3.modify(|_, w| w.dmar().set_bit());

        // 5. Configure interrupt generation after half/ full transfer as required by the application.
        // todo (Let the user call `dma.setup_interrupt()`? But that requires they know
        // todo which channel of DMA usart tx is on.

        // 6. Activate the channel in the DMA control register.
        // When the number of data transfers programmed in the DMA Controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector.
        // (Handled in above fn call)

        // When the number of data transfers programmed in the DMA Controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector.
    }

    /// Flush the transmit buffer.
    pub fn flush(&self) {
        #[cfg(not(feature = "f4"))]
        while self.regs.isr.read().tc().bit_is_clear() {}
        #[cfg(feature = "f4")]
        while self.regs.sr.read().tc().bit_is_clear() {}
    }

    #[cfg(not(feature = "f4"))]
    /// Enable a specific type of interrupt.
    pub fn enable_interrupt(&mut self, interrupt: UsartInterrupt) {
        // Disable the UART to allow writing the `add` and `addm7` bits
        self.regs.cr1.modify(|_, w| w.ue().clear_bit());
        while self.regs.cr1.read().ue().bit_is_set() {}

        match interrupt {
            UsartInterrupt::CharDetect(char) => {
                // Enable character-detecting UART interrupt
                self.regs.cr1.modify(|_, w| w.cmie().set_bit());

                // Allow an 8-bit address to be set in `add`.
                self.regs.cr2.modify(|_, w| {
                    w.addm7().set_bit();
                    // Set the character to detect
                    cfg_if! {
                        if #[cfg(any(feature = "f3", feature = "l4", feature = "h7"))] {
                            w.add().bits(char)
                        } else { unsafe { // todo: Is this right, or backwards?
                            w.add0_3().bits(char & 0b1111);
                            w.add4_7().bits(char & (0b1111 << 4))
                        }}
                    }
                });
            }
            UsartInterrupt::Cts => {
                self.regs.cr3.modify(|_, w| w.ctsie().set_bit());
            }
            UsartInterrupt::EndOfBlock => {
                self.regs.cr1.modify(|_, w| w.eobie().set_bit());
            }
            UsartInterrupt::Idle => {
                self.regs.cr1.modify(|_, w| w.idleie().set_bit());
            }
            UsartInterrupt::FramingError => {
                self.regs.cr3.modify(|_, w| w.eie().set_bit());
            }
            UsartInterrupt::LineBreak => {
                self.regs.cr2.modify(|_, w| w.lbdie().set_bit());
            }
            UsartInterrupt::Overrun => {
                self.regs.cr3.modify(|_, w| w.eie().set_bit());
            }
            UsartInterrupt::ParityError => {
                self.regs.cr1.modify(|_, w| w.peie().set_bit());
            }
            UsartInterrupt::ReadNotEmpty => {
                self.regs.cr1.modify(|_, w| w.rxneie().set_bit());
            }
            UsartInterrupt::ReceiverTimeout => {
                self.regs.cr1.modify(|_, w| w.rtoie().set_bit());
            }
            #[cfg(not(any(feature = "f3", feature = "l4")))]
            UsartInterrupt::Tcbgt => {
                self.regs.cr3.modify(|_, w| w.tcbgtie().set_bit());
            }
            UsartInterrupt::TransmissionComplete => {
                self.regs.cr1.modify(|_, w| w.tcie().set_bit());
            }
            UsartInterrupt::TransmitEmpty => {
                self.regs.cr1.modify(|_, w| w.txeie().set_bit());
            }
        }

        self.regs.cr1.modify(|_, w| w.ue().set_bit());
    }

    #[cfg(not(feature = "f4"))]
    /// Clears the interrupt pending flag for a specific type of interrupt.
    pub fn clear_interrupt(&mut self, interrupt: UsartInterrupt) {
        match interrupt {
            UsartInterrupt::CharDetect(_) => self.regs.icr.write(|w| w.cmcf().set_bit()),
            UsartInterrupt::Cts => self.regs.icr.write(|w| w.ctscf().set_bit()),
            UsartInterrupt::EndOfBlock => self.regs.icr.write(|w| w.eobcf().set_bit()),
            UsartInterrupt::Idle => self.regs.icr.write(|w| w.idlecf().set_bit()),
            UsartInterrupt::FramingError => self.regs.icr.write(|w| w.fecf().set_bit()),
            UsartInterrupt::LineBreak => self.regs.icr.write(|w| w.lbdcf().set_bit()),
            UsartInterrupt::Overrun => self.regs.icr.write(|w| w.orecf().set_bit()),
            UsartInterrupt::ParityError => self.regs.icr.write(|w| w.pecf().set_bit()),
            UsartInterrupt::ReadNotEmpty => self.regs.rqr.write(|w| w.rxfrq().set_bit()),
            UsartInterrupt::ReceiverTimeout => self.regs.icr.write(|w| w.rtocf().set_bit()),
            #[cfg(not(any(feature = "f3", feature = "l4", feature = "h7")))]
            UsartInterrupt::Tcbgt => self.regs.icr.write(|w| w.tcbgtcf().set_bit()),
            #[cfg(feature = "h7")]
            UsartInterrupt::Tcbgt => self.regs.icr.write(|w| w.tcbgtc().set_bit()),
            UsartInterrupt::TransmissionComplete => self.regs.icr.write(|w| w.tccf().set_bit()),
            UsartInterrupt::TransmitEmpty => self.regs.rqr.write(|w| w.txfrq().set_bit()),
        }
    }
}

/// Serial error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

impl<U> Read<u8> for Usart<U>
where
    U: Deref<Target = pac::usart1::RegisterBlock>,
{
    type Error = Error;

    #[cfg(not(feature = "f4"))]
    fn read(&mut self) -> nb::Result<u8, Error> {
        let rxne = self.regs.isr.read().rxne().bit_is_set();

        if rxne {
            Ok(self.regs.rdr.read().rdr().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[cfg(feature = "f4")]
    fn read(&mut self) -> nb::Result<u8, Error> {
        let rxne = self.regs.sr.read().rxne().bit_is_set();

        if rxne {
            Ok(self.regs.dr.read().dr().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<U> Write<u8> for Usart<U>
where
    U: Deref<Target = pac::usart1::RegisterBlock>,
{
    type Error = Error;

    #[cfg(not(feature = "f4"))]
    fn write(&mut self, word: u8) -> nb::Result<(), Error> {
        let txe = self.regs.isr.read().txe().bit_is_set();

        if txe {
            self.regs
                .tdr
                .modify(|_, w| unsafe { w.tdr().bits(word as u16) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[cfg(feature = "f4")]
    fn write(&mut self, word: u8) -> nb::Result<(), Error> {
        let txe = self.regs.sr.read().txe().bit_is_set();

        if txe {
            self.regs
                .dr
                .modify(|_, w| unsafe { w.dr().bits(word as u16) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Error> {
        #[cfg(not(feature = "f4"))]
        let tc = self.regs.isr.read().tc().bit_is_set();
        #[cfg(feature = "f4")]
        let tc = self.regs.sr.read().tc().bit_is_set();

        if tc {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<U> blocking::serial::Write<u8> for Usart<U>
where
    U: Deref<Target = pac::usart1::RegisterBlock>,
{
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Error> {
        Usart::write(self, buffer);
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Error> {
        #[cfg(not(feature = "f4"))]
        while self.regs.isr.read().tc().bit_is_clear() {}
        #[cfg(feature = "f4")]
        while self.regs.sr.read().tc().bit_is_clear() {}

        Ok(())
    }
}
