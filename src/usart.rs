//! This module allows for serial communication using the STM32 USART module.

// todo: see if you can use this macro-less approach elsewhere. Currently used
// todo here and in I2c

// todo: Synchronous mode.
// todo: Auto baud

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

use embedded_hal::{
    blocking,
    serial::{Read, Write},
};

use core::ops::Deref;

use cfg_if::cfg_if;

// todo: Make a single interrupt function that takes this macro?
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

#[derive(Clone, Copy)]
#[repr(u8)]
/// The length of word to transmit and receive. (USART_CR1, M)
pub enum WordLen {
    W8 = 0b00,
    W9 = 0b01,
    W7 = 0b10,
}

impl WordLen {
    /// We use this function due to the M field being split into 2 separate bits.
    /// Returns M1 val, M0 val
    /// todo: Make sure this isn't backwards.
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
    Two,
    #[cfg(not(any(feature = "l4x1", feature = "g0")))]
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
    Tcbgt,
    TransmissionComplete,
    TransmitEmpty,
}

pub struct UsartConfig {
    word_len: WordLen,
    stop_bits: StopBits,
    oversampling: OverSampling,
}

impl Default for UsartConfig {
    fn default() -> Self {
        Self {
            word_len: WordLen::W8,
            stop_bits: StopBits::S1,
            oversampling: OverSampling::O16,
        }
    }
}

/// Represents the USART peripheral, for serial communications.
pub struct Usart<U> {
    regs: U,
    baud: u32,
    config: UsartConfig,
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
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        // todo: Hard set to usart 1 to get started
        match device {
            UsartDevice::One => {
                rcc_en_reset!(apb2, usart1, rcc);
            }
            UsartDevice::Two => {
                rcc_en_reset!(apb1, usart2, rcc);
            }
            #[cfg(not(any(feature = "l4x1", feature = "g0")))]
            UsartDevice::Three => {
                rcc_en_reset!(apb1, usart3, rcc);
            } // UsartDevice::Four => {
              //     rcc_en_reset!(apb1, uart4, rcc);
              // }
              // UsartDevice::Five => {
              //     rcc_en_reset!(apb1, usart5, rcc);
              // }
        }

        // This should already be disabled on power up, but disable here just in case:
        regs.cr1.modify(|_, w| w.ue().clear_bit());
        while regs.cr1.read().ue().bit_is_set() {}

        regs.cr1
            .modify(|_, w| w.over8().bit(config.oversampling as u8 != 0));

        // Set up transmission. See L44 RM, section 38.5.2: "Character Transmission Procedures".
        // 1. Program the M bits in USART_CR1 to define the word length.
        // todo: Make sure you aren't doing this backwards.

        cfg_if! {
            if #[cfg(feature = "f3")] {
                regs.cr1.modify(|_, w| w.m().bit(config.word_len as u8 != 0));
            } else {
                let word_len_bits = config.word_len.bits();
                regs.cr1.modify(|_, w| w.m1().bit(word_len_bits.0 != 0));
                regs.cr1.modify(|_, w| w.m0().bit(word_len_bits.1 != 0));
            }
        }

        // 2. Select the desired baud rate using the USART_BRR register.

        // To set BAUD rate, see L4 RM section 38.5.4: "USART baud rate generation".
        let fclk = match device {
            UsartDevice::One => clocks.apb2(),
            _ => clocks.apb1(),
        };

        // Oversampling by 16:
        let usart_div = match config.oversampling {
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

        regs.brr.write(|w| unsafe { w.bits(usart_div as u32) });
        // 3. Program the number of stop bits in USART_CR2.
        regs.cr2
            .modify(|_, w| unsafe { w.stop().bits(config.stop_bits as u8) });
        // 4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
        regs.cr1.modify(|_, w| w.ue().set_bit());
        // 5. Select DMA enable (DMAT) in USART_CR3 if multibuffer communication is to take
        // place. Configure the DMA register as explained in multibuffer communication.
        // todo?
        // 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
        // 6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
        // start bit.
        regs.cr1.modify(|_, w| {
            w.te().set_bit();
            w.re().set_bit()
        });

        Self { regs, baud, config }
    }

    /// Transmit data, as a sequence of u8.. See L44 RM, section 38.5.2: "Character transmission procedure"
    pub fn write(&mut self, data: &[u8]) {
        // 7. Write the data to send in the USART_TDR register (this clears the TXE bit). Repeat this
        // for each data to be transmitted in case of single buffer.
        // todo: Do we need to manually wait until txe = 1?
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
    }

    /// Receive data into a u8 buffer. See L44 RM, section 38.5.3: "Character reception procedure"
    pub fn read(&mut self, buf: &mut [u8]) {
        for i in 0..buf.len() {
            // Wait for the next bit
            while self.regs.isr.read().rxne().bit_is_clear() {}
            buf[i] = self.regs.rdr.read().rdr().bits() as u8;
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

    // /// Flush the transmit buffer.
    // pub fn flush(&mut self) {
    //
    // }

    /// Enable a specific type of interrupt.
    pub fn enable_interrupt(&mut self, interrupt_type: UsartInterrupt) {
        // Disable the UART to allow writing the `add` and `addm7` bits
        self.regs.cr1.modify(|_, w| w.ue().clear_bit());
        while self.regs.cr1.read().ue().bit_is_set() {}

        match interrupt_type {
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
                // self.regs.cr1.modify(|_, w| w.eie().set_bit()); // todo
            }
            UsartInterrupt::LineBreak => {
                self.regs.cr2.modify(|_, w| w.lbdie().set_bit());
            }
            UsartInterrupt::Overrun => {
                // self.regs.cr1.modify(|_, w| w.eie().set_bit()); // todo
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
            UsartInterrupt::Tcbgt => {
                // self.regs.cr3.modify(|_, w| w.tcbgtie().set_bit()); // todo?
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

    /// Clears the interrupt pending flag for a specific type of interrupt.
    pub fn clear_interrupt(&mut self, interrupt_type: UsartInterrupt) {
        match interrupt_type {
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
            UsartInterrupt::Tcbgt => {
                // self.regs.icr.write(|w| w.tcbgtcf().set_bit()) // todo ?
            }
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

    fn read(&mut self) -> nb::Result<u8, Error> {
        if self.regs.isr.read().rxne().bit_is_set() {
            Ok(self.regs.rdr.read().rdr().bits() as u8)
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

    fn write(&mut self, word: u8) -> nb::Result<(), Error> {
        if self.regs.isr.read().txe().bit_is_set() {
            self.regs
                .tdr
                .modify(|_, w| unsafe { w.tdr().bits(word as u16) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Error> {
        if self.regs.isr.read().tc().bit_is_set() {
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
        while self.regs.isr.read().tc().bit_is_clear() {}
        Ok(())
    }
}
