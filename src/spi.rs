//! Serial Peripheral Interface (SPI) bus.

// Based on `stm32l4xx-hal`.

use core::ptr;

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};

use paste::paste;

use crate::{
    pac::{RCC, SPI1, SPI2, SPI3},
    traits::ClockCfg,
};

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

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCK pin. This trait is sealed and cannot be implemented.
pub trait SckPin<SPI>: private::Sealed {}
/// MISO pin. This trait is sealed and cannot be implemented.
pub trait MisoPin<SPI>: private::Sealed {}
/// MOSI pin. This trait is sealed and cannot be implemented.
pub trait MosiPin<SPI>: private::Sealed {}

// #[derive(Clone, Copy)]
// pub enum SpiDevice {
//     One,
//     Two,
//     Three,
// }

// macro_rules! pins {
//     ($spi:ident, $af:ident, SCK: [$($sck:ident),*], MISO: [$($miso:ident),*], MOSI: [$($mosi:ident),*]) => {
//         $(
//             impl private::Sealed for $sck<Alternate<$af, Input<Floating>>> {}
//             impl SckPin<$spi> for $sck<Alternate<$af, Input<Floating>>> {}
//         )*
//         $(
//             impl private::Sealed for $miso<Alternate<$af, Input<Floating>>> {}
//             impl MisoPin<$spi> for $miso<Alternate<$af, Input<Floating>>> {}
//         )*
//         $(
//             impl private::Sealed for $mosi<Alternate<$af, Input<Floating>>> {}
//             impl MosiPin<$spi> for $mosi<Alternate<$af, Input<Floating>>> {}
//         )*
//     }
// }

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI> {
    spi: SPI,
}

macro_rules! hal {
    ($($SPIX:ident: ($spi:ident, $constructorname:ident, $apb:ident, $enr:ident, $rst:ident, $en:ident),)+) => {
        $(
            impl Spi<$SPIX> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $constructorname<C: ClockCfg>(
                    spi: $SPIX,
                    mode: Mode,
                    freq: u32,
                    clocks: &C,
                    rcc: &mut RCC,
                ) -> Self
                // where
                //     SCK: SckPin<$SPIX>,
                //     MISO: MisoPin<$SPIX>,
                //     MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX

                    // todo: this is similar to code we use in `timer.rs`.
                                        cfg_if::cfg_if! {
                        if #[cfg(feature = "f3")] {
                            paste! {
                                rcc.[<$apb enr>].modify(|_, w| w.$en().set_bit());
                                rcc.[<$apb rstr>].modify(|_, w| w.[<$spi rst>]().set_bit());
                                rcc.[<$apb rstr>].modify(|_, w| w.[<$spi rst>]().clear_bit());
                            }
                        } else if # [cfg(any(feature = "l4", feature = "l5"))] {
                            paste! {
                                // We use `$enr` and $rst, since we only add `1` after for apb1.
                                // This isn't required on f3.
                                rcc.[<$apb $enr>].modify(|_, w| w.$en().set_bit());
                                rcc.[<$apb $rst>].modify(|_, w| w.[<$spi rst>]().set_bit());
                                rcc.[<$apb $rst>].modify(|_, w| w.[<$spi rst>]().clear_bit());
                            }
                        }
                    }

                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    // DS: 8-bit data size
                    // SSOE: Slave Select output disabled
                    spi.cr2
                        .write(|w| unsafe {
                            w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                        });

                    paste! {
                        let br = Self::compute_baud_rate(clocks.[<$apb _timer>](), freq);
                    }

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
                    spi.cr1.write(|w| unsafe {
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

                    Spi { spi }
                }

                /// Change the baud rate of the SPI
                pub fn reclock<F, C: ClockCfg>(&mut self, freq: u32, clocks: C) {
                    self.spi.cr1.modify(|_, w| w.spe().clear_bit());
                    self.spi.cr1.modify(|_, w| {
                        paste!{
                            unsafe {w.br().bits(Self::compute_baud_rate(clocks.[<$apb _timer>](), freq));}
                            w.spe().set_bit()
                        }
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

            impl FullDuplex<u8> for Spi<$SPIX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX> {}

            impl embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX> {}
        )+
    }
}

#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6"
))]
hal! {
    SPI1: (spi1, spi1_unchecked, apb2, enr, rstr, spi1en),
}

#[cfg(any(feature = "f302", feature = "f303", feature = "f373", feature = "f3x4",))]
hal! {
    SPI1: (spi1, spi1_unchecked, apb2, enr, rstr, spi1en),
}

// #[cfg(any(feature = "l4", feature = "l5"))]
// hal! {
//     SPI1: (spi1, APB2, enr, rstr),
// }

#[cfg(any(feature = "f301", feature = "f302", feature = "f303", feature = "f373",))]
hal! {
    SPI2: (spi2, spi2_unchecked, apb1, enr1, rstr1, spi2en),
}

#[cfg(any(
    feature = "l4x1",
    feature = "l4x3",
    feature = "l4x4",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l552",
    feature = "l562",
))]
hal! {
    SPI2: (spi2, spi2_unchecked, apb1, enr1, rstr1, spi2en),
}

#[cfg(any(feature = "f301", feature = "f302", feature = "f303", feature = "f373",))]
hal! {
    SPI3: (spi3, spi3_unchecked, apb1, enr1, rstr1, spi3en),
}

#[cfg(any(feature = "l4x1", feature = "l4x4", feature = "l4x5", feature = "l4x6",))]
hal! {
    SPI3: (spi3, spi3_unchecked, apb1, enr1, rstr1, spi3en),
}

// The final argument of the macro appears to be needed due to a differene
// in the SPI3 L4 RM and SVDs: It's `sp3en` instead of `spi3en` for some variants.
#[cfg(any(feature = "l4x3", feature = "l552", feature = "l562"))]
hal! {
    SPI3: (spi3, spi3_unchecked, apb1, enr1, rstr1, sp3en),
}

// #[cfg(any(
//     feature = "stm32l4x1",
//     feature = "stm32l4x2",
//     feature = "stm32l4x5",
//     feature = "stm32l4x6",
// ))]
// pins!(SPI3, AF6,
//     SCK: [PB3, PC10],
//     MISO: [PB4, PC11],
//     MOSI: [PB5, PC12]);
//
// #[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
// pins!(SPI3, AF6, SCK: [PG9], MISO: [PG10], MOSI: [PG11]);

// #[cfg(any(
//     feature = "stm32l4x1",
//     feature = "stm32l4x3",
//     feature = "stm32l4x5",
//     feature = "stm32l4x6",
// ))]
// use crate::stm32::SPI2;

// #[cfg(any(
//     feature = "stm32l4x1",
//     feature = "stm32l4x3",
//     feature = "stm32l4x5",
//     feature = "stm32l4x6",
// ))]
// pins!(SPI2, AF5,
//     SCK: [PB13, PB10, PD1],
//     MISO: [PB14, PC2, PD3],
//     MOSI: [PB15, PC3, PD4]);
