//! This is an internal module that contains utility functionality used by other modules.

#[cfg(feature = "l4")]
use core::ops::Deref;

use cfg_if::cfg_if;

// todo: L5 has a PAC bug on CCR registers past 1.
#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::{self, Dma, DmaChannel, DmaInput};
#[cfg(any(feature = "f3", feature = "l4"))]
use crate::pac::DMA1;
#[cfg(feature = "l4")]
use crate::pac::dma1;
use crate::{
    clocks::Clocks,
    pac::{self, rcc::RegisterBlock},
};

#[cfg(not(any(
    feature = "f401",
    feature = "f411",
    feature = "f412",
    feature = "l412",
    feature = "wb",
    feature = "g0",
    feature = "c0",
)))]
cfg_if! {
    if #[cfg(any(feature = "f3", feature = "l412", feature = "g4", feature = "h7b3"))] {
        use crate::pac::DAC1;
    } else {
        use crate::pac::DAC as DAC1;
    }
}

cfg_if! {
    if #[cfg(any(feature = "g0", feature = "c0", feature = "wl"))] {
        use crate::pac::ADC as ADC1;

    } else {
        use crate::pac::ADC1;
    }
}

#[cfg(any(feature = "f3", feature = "l4",))]
use crate::pac::dma1 as dma_p;

// Used for while loops, to allow returning an error instead of hanging.
// todo: What should this be?
// Observation: On G4 at 170Mhz, 300,000 triggers a failure when erasing a flash page (prior to step 5),
// but 900,000 is fine.
pub(crate) const MAX_ITERS: u32 = 900_000;

/// DRY: Instead of infinitely busy-looping on some condition, we bound the number of iterations
/// and return a given error upon exceeding that bound.
macro_rules! bounded_loop {
    ($cond:expr, $err:expr, $iters:expr, $($content:tt)?) => {
        let mut iterations = 0;
        while $cond {
            iterations += 1;
            if iterations >= $iters {
                return Err($err);
            }
            $($content)?
        }
    };
    ($cond:expr, $err:expr, $content:tt) => {
        bounded_loop!($cond, $err, crate::util::MAX_ITERS, ($content));
    };
    ($cond:expr, $err:expr, $iters:literal) => {
        bounded_loop!($cond, $err, $iters, ())
    };
    ($cond:expr, $err:expr) => {
        bounded_loop!($cond, $err, crate::util::MAX_ITERS, ())
    };
}
pub(crate) use bounded_loop;

cfg_if! {
    if #[cfg(feature = "h5")] {
        macro_rules! cr1 {
            ($regs:expr) => {
                $regs.cr1_enabled()
            };
        }
    } else if #[cfg(not(feature = "h5"))] {
        macro_rules! cr1 {
            ($regs:expr) => {
                $regs.cr1()
            };
        }
    }
}

cfg_if! {
    if #[cfg(feature = "h5")] {
        macro_rules! isr {
            ($regs:expr) => {
                $regs.isr_enabled()
            };
        }
    } else if  #[cfg(feature = "f4")] {
        macro_rules! isr {
            ($regs:expr) => {
                $regs.sr()
            };
        }
    } else {
        macro_rules! isr {
            ($regs:expr) => {
                $regs.isr()
            };
        }
    }
}

pub(crate) use cr1;
pub(crate) use isr;

/// Enables and resets peripheral clocks on various RCC registers.
/// The first argument is a `apb1`, `ahb2` etc to specify the reg block. The second is something like
/// `tim1`, and the third is a `pac::RCC`.
macro_rules! rcc_en_reset {
    (apb1, $periph:expr, $rcc:expr) => {
        paste::paste! { cfg_if::cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4"))] {
                $rcc.apb1enr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apb1rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apb1rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else if #[cfg(any(feature = "l4", feature = "l5", feature = "g4", feature = "wb", feature = "wl"))] {
                $rcc.apb1enr1().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apb1rstr1().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apb1rstr1().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else if #[cfg(any(feature = "g0", feature = "c0"))] {
                $rcc.apbenr1().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apbrstr1().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apbrstr1().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else {  // H7
                $rcc.apb1lenr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apb1lrstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apb1lrstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
                // todo: apb1h equivs
            }
            // todo: apb1enr2 on L5? Currently we only use it with USB, which is handled in
            // todo `usb.rs`.
            // todo: apb1enr2
            // todo: f4 has apb1lenr as well as apb1enr
        }}
    };
    (apb2, $periph:expr, $rcc:expr) => {
        paste::paste! { cfg_if::cfg_if! {
            if #[cfg(any(feature = "g0", feature = "c0"))] {
                $rcc.apbenr2().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apbrstr2().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apbrstr2().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else {
                $rcc.apb2enr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.apb2rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.apb2rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
            }
        }}
    };
    (apb4, $periph:expr, $rcc:expr) => {
        paste::paste! {
            $rcc.apb4enr().modify(|_, w| w.[<$periph en>]().bit(true));
            $rcc.apb4rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
            $rcc.apb4rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
        }
    };
    (ahb1, $periph:expr, $rcc:expr) => {
        paste::paste! { cfg_if::cfg_if! {
            if #[cfg(feature = "f3")] {
                $rcc.ahbenr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else if #[cfg(any(feature = "g0", feature = "c0"))] {
                $rcc.ahbenr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
            } else {
                $rcc.ahb1enr().modify(|_, w| w.[<$periph en>]().bit(true));
                $rcc.ahb1rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
                $rcc.ahb1rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
            }
        }}
    };
    (ahb2, $periph:expr, $rcc:expr) => {
        paste::paste! {
            $rcc.ahb2enr().modify(|_, w| w.[<$periph en>]().bit(true));
            $rcc.ahb2rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
            $rcc.ahb2rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
        }
    };
    (ahb3, $periph:expr, $rcc:expr) => {
        paste::paste! {
            $rcc.ahb3enr().modify(|_, w| w.[<$periph en>]().bit(true));
            $rcc.ahb3rstr().modify(|_, w| w.[<$periph rst>]().bit(true));
            $rcc.ahb3rstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
        }
    };
    // C0's pac v 0.16 calls it this, although it's actually ahb4.
    (ahb, $periph:expr, $rcc:expr) => {
        paste::paste! {
            $rcc.ahbenr().modify(|_, w| w.[<$periph en>]().bit(true));
            $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().bit(true));
            $rcc.ahbrstr().modify(|_, w| w.[<$periph rst>]().clear_bit());
        }
    };
}

#[cfg(not(any(feature = "f", feature = "g0", feature = "c0", feature = "l")))]
macro_rules! rcc_en_reset_apb1enr2 {
    ($periph:expr, $rcc:expr) => {
        paste::paste! {
            $rcc.apb1enr2().modify(|_, w| w.[<$periph en>]().bit(true));
            $rcc.apb1rstr2().modify(|_, w| w.[<$periph rst>]().bit(true));
            $rcc.apb1rstr2().modify(|_, w| w.[<$periph rst>]().clear_bit());
        }
    };
}

pub(crate) use rcc_en_reset;

/// Uart only. Important: This assumes we use the default UART clock.
pub trait BaudPeriph {
    fn baud(clock_cfg: &Clocks) -> u32;
}

impl BaudPeriph for pac::USART1 {
    fn baud(clock_cfg: &Clocks) -> u32 {
        #[cfg(feature = "c0")]
        return clock_cfg.apb1();
        #[cfg(not(feature = "c0"))]
        return clock_cfg.apb2();
    }
}

#[cfg(not(any(feature = "wb", feature = "wl")))]
impl BaudPeriph for pac::USART2 {
    fn baud(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
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
    feature = "c0",
    feature = "wb",
    feature = "wl",
)))]
impl BaudPeriph for pac::USART3 {
    fn baud(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

cfg_if! {
    if #[cfg(any(feature = "l4x6", feature = "h7", feature = "f401", feature = "f407"))] {
        #[cfg(not(feature = "f401"))]
        impl BaudPeriph for pac::UART4 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb1()
            }
        }

        #[cfg(not(feature = "f401"))]
        impl BaudPeriph for pac::UART5 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb1()
            }
        }

        #[cfg(any(feature = "h7", feature = "f401", feature = "f407"))]
        impl BaudPeriph for pac::USART6 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb2()
            }
        }

        #[cfg(feature = "h7")]
        impl BaudPeriph for pac::UART7 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb1()
            }
        }

        #[cfg(feature = "h7")]
        impl BaudPeriph for pac::UART8 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb1()
            }
        }

        #[cfg(feature = "h735")]
        impl BaudPeriph for pac::UART9 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb2()
            }
        }

        #[cfg(feature = "h735")]
        impl BaudPeriph for pac::USART10 {
            fn baud(clock_cfg: &Clocks) -> u32 {
                clock_cfg.apb2()
            }
        }
    }
}

#[cfg(not(any(
    feature = "f",
    feature = "g0",
    feature = "c0",
    feature = "wl",
    feature = "l"
)))]
impl BaudPeriph for pac::LPUART1 {
    fn baud(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

// #[cfg(not(any(
//     feature = "f",
//     feature = "l",
//     feature = "g0",
//     feature = "h7",
//     feature = "wb",
//     feature = "wl",
//     feature = "g431",
//     // todo: Which families have LPUART2?
// )))]
// impl BaudPeriph for pac::LPUART2 {
//     fn baud(clock_cfg: &Clocks) -> u32 {
//         // todo: QC this; it's a guess.
//         clock_cfg.apb1()
//     }
// }

// todo: This trait is currently a one-off for adc, and isn't currently used.
pub trait VrefPeriph {
    fn vref(clock_cfg: &Clocks) -> u32;
}

impl VrefPeriph for ADC1 {
    fn vref(clock_cfg: &Clocks) -> u32 {
        #[cfg(feature = "c0")]
        return clock_cfg.apb1();
        #[cfg(not(feature = "c0"))]
        return clock_cfg.apb2();
    }
}

#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l412",
    feature = "l4x5",
    feature = "l4x6",
))]
impl VrefPeriph for pac::ADC2 {
    fn vref(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

#[cfg(all(feature = "g4", not(any(feature = "g431", feature = "g441"))))]
impl VrefPeriph for pac::ADC3 {
    fn vref(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

#[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))]
impl VrefPeriph for pac::ADC4 {
    fn vref(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

#[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))]
impl VrefPeriph for pac::ADC5 {
    fn vref(clock_cfg: &Clocks) -> u32 {
        clock_cfg.apb1()
    }
}

/// Used to provide peripheral-specific implementation for RCC enable/reset, and for F3 and L4,
/// DMA channel assignment.
pub trait RccPeriph {
    fn en_reset(rcc: &RegisterBlock);

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel;
    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel;
    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D);
    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D);
}

#[cfg(not(any(
    feature = "f401",
    feature = "f410",
    feature = "f411",
    feature = "g031",
    feature = "g041",
    feature = "g070",
    feature = "g030",
    feature = "g051",
    feature = "g061",
    feature = "c0",
    feature = "wb",
    feature = "wl"
)))]
impl RccPeriph for pac::TIM6 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, tim6, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

#[cfg(not(any(
    feature = "f301",
    feature = "f302",
    feature = "f401",
    feature = "f410",
    feature = "f411",
    feature = "g031",
    feature = "g041",
    feature = "g070",
    feature = "g030",
    feature = "g051",
    feature = "g061",
    feature = "c0",
    feature = "wb",
    feature = "wl",
    feature = "l412",
)))]
impl RccPeriph for pac::TIM7 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, tim7, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

impl RccPeriph for pac::I2C1 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, i2c1, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::I2c1Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::I2c1Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::I2c1Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::I2c1Tx);
    }
}

#[cfg(not(any(feature = "wb", feature = "f3x4", feature = "c011", feature = "c031")))]
impl RccPeriph for pac::I2C2 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, i2c2, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::I2c2Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::I2c2Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::I2c2Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::I2c2Tx);
    }
}

#[cfg(any(feature = "h7", feature = "wb"))]
impl RccPeriph for pac::I2C3 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, i2c3, rcc);
    }
}

#[cfg(feature = "h7")]
impl RccPeriph for pac::I2C4 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb4, i2c4, rcc);
    }
}

#[cfg(not(feature = "f301"))] // todo: Not sure what's going on  here.
impl RccPeriph for pac::SPI1 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb2, spi1, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Spi1Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Spi1Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi1Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi1Tx);
    }
}

#[cfg(not(any(
    feature = "f3x4",
    feature = "wb",
    feature = "wl",
    feature = "c011",
    feature = "c031",
    feature = "g061"
)))]
impl RccPeriph for pac::SPI2 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb1, spi2, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Spi2Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Spi2Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi2Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi2Tx);
    }
}

#[cfg(not(any(
    feature = "f3x4",
    feature = "f410",
    feature = "g0",
    feature = "c0",
    feature = "wb",
    feature = "wl",
    feature = "l412",
)))]
impl RccPeriph for pac::SPI3 {
    fn en_reset(rcc: &RegisterBlock) {
        cfg_if! {
            // Note `sp3en` mixed with `spi3rst`; why we can't use the usual macro.
            if #[cfg(feature = "l5")] {
                rcc.apb1enr1().modify(|_, w| w.sp3en().bit(true));
                rcc.apb1rstr1().modify(|_, w| w.spi3rst().bit(true));
                rcc.apb1rstr1().modify(|_, w| w.spi3rst().clear_bit());
            } else {
                rcc_en_reset!(apb1, spi3, rcc);
            }
        }
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Spi3Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Spi3Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi3Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Spi3Tx);
    }
}

#[cfg(feature = "h7")]
impl RccPeriph for pac::SPI4 {
    fn en_reset(rcc: &RegisterBlock) {
        cfg_if! {
            // Note `sp4en` mixed with `spi4rst`; why we can't use the usual macro.
            if #[cfg(feature = "l5")] {
                rcc.apb2enr1().modify(|_, w| w.sp4en().bit(true));
                rcc.apb2rstr1.modify(|_, w| w.spi4rst().bit(true));
                rcc.apb2rstr1.modify(|_, w| w.spi4rst().clear_bit());
            } else {
                rcc_en_reset!(apb2, spi4, rcc);
            }
        }
    }
}

#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "g0",
    feature = "c0",
    feature = "g4", // todo: G4 PAC issue re getting channel-specific reg blocks.
    feature = "h7b3",
    feature = "wl",
    feature = "l412",
)))]
impl RccPeriph for pac::SAI1 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb2, sai1, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

#[cfg(all(feature = "h7", not(feature = "h735")))]
impl RccPeriph for pac::SAI2 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb2, sai2, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

#[cfg(all(feature = "h7", not(feature = "h735")))]
impl RccPeriph for pac::SAI3 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb2, sai3, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

#[cfg(feature = "h7")]
impl RccPeriph for pac::SAI4 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb4, sai4, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
        unimplemented!()
    }
}

// #[cfg(any(feature = "g0c1", feature = "g4", feature = "h7"))]
// impl RccPeriph for pac::FDCAN {
//     #[cfg(feature = "g4")]
//     fn en_reset(rcc: &RegisterBlock) {
//         rcc_en_reset!(apb1, fdcan, rcc);
//     }
//
//     #[cfg(not(feature = "g4"))]
//     fn en_reset(rcc: &RegisterBlock) {
//         rcc_en_reset!(apb1, fdcan1, rcc);
//     }
//
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn read_chan() -> DmaChannel {
//         unimplemented!()
//     }
//
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn write_chan() -> DmaChannel {
//         unimplemented!()
//     }
//
//     #[cfg(feature = "l4")]
//     fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
//         unimplemented!()
//     }
//
//     #[cfg(feature = "l4")]
//     fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {
//         unimplemented!()
//     }
// }

impl RccPeriph for pac::USART1 {
    fn en_reset(rcc: &RegisterBlock) {
        rcc_en_reset!(apb2, usart1, rcc);
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Usart1Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Usart1Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart1Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart1Tx);
    }
}

#[cfg(not(any(feature = "wb", feature = "wl")))]
impl RccPeriph for pac::USART2 {
    fn en_reset(rcc: &RegisterBlock) {
        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
                rcc_en_reset!(apb1, usart2, rcc);
            } else {
                // `usart` vs `uart`
                rcc.apb1enr().modify(|_, w| w.usart2en().bit(true));
                rcc.apb1rstr().modify(|_, w| w.usart2rst().bit(true));
                rcc.apb1rstr().modify(|_, w| w.usart2rst().clear_bit());
            }
        }
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Usart2Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Usart2Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart2Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart2Tx);
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
    feature = "c0",
    feature = "wb",
    feature = "wl",
)))]
impl RccPeriph for pac::USART3 {
    fn en_reset(rcc: &RegisterBlock) {
        cfg_if! {
            if #[cfg(not(feature = "f4"))] {
                rcc_en_reset!(apb1, usart3, rcc);
            } else {
                rcc.apb1enr().modify(|_, w| w.usart3en().bit(true));
                rcc.apb1rstr().modify(|_, w| w.usart3rst().bit(true));
                rcc.apb1rstr().modify(|_, w| w.usart3rst().clear_bit());
            }
        }
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn read_chan() -> DmaChannel {
        DmaInput::Usart3Rx.dma1_channel()
    }

    #[cfg(any(feature = "f3", feature = "l4"))]
    fn write_chan() -> DmaChannel {
        DmaInput::Usart3Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart3Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Usart3Tx);
    }
}

cfg_if! {
    if #[cfg(any(feature = "l4x6", feature = "g473", feature = "g474", feature = "g483", feature = "g484", feature = "h7", feature = "f401", feature = "f407"))] {
        #[cfg(not(feature = "f401"))]
        impl RccPeriph for pac::UART4 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, uart4, rcc);
            }

             #[cfg(feature = "l4")]
            fn read_chan() -> DmaChannel {
                DmaInput::Uart4Rx.dma1_channel()
            }

            #[cfg(feature = "l4")]
            fn write_chan() -> DmaChannel {
                DmaInput::Uart4Tx.dma1_channel()
            }

            #[cfg(feature = "l4")]
            fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
                dma::channel_select(regs, DmaInput::Uart4Rx);
            }

            #[cfg(feature = "l4")]
            fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
                dma::channel_select(regs, DmaInput::Uart4Tx);
            }
        }

        #[cfg(not(feature = "f401"))]
        impl RccPeriph for pac::UART5 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, uart5, rcc);
            }

            #[cfg(feature = "l4")]
            fn read_chan() -> DmaChannel {
                DmaInput::Uart5Rx.dma1_channel()
            }

            #[cfg(feature = "l4")]
            fn write_chan() -> DmaChannel {
                DmaInput::Uart5Tx.dma1_channel()
            }

            #[cfg(feature = "l4")]
            fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
                dma::channel_select(regs, DmaInput::Uart5Rx);
            }

            #[cfg(feature = "l4")]
            fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
                dma::channel_select(regs, DmaInput::Uart5Tx);
            }
        }

        #[cfg(any(feature = "h7", feature = "f401", feature = "f407"))]
        impl RccPeriph for pac::USART6 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb2, usart6, rcc);
            }
        }

        #[cfg(feature = "h7")]
        impl RccPeriph for pac::UART7 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, uart7, rcc);
            }
        }

        #[cfg(feature = "h7")]
        impl RccPeriph for pac::UART8 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, uart8, rcc);
            }
        }

        #[cfg(feature = "h735")]
        impl RccPeriph for pac::UART9 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb2, uart9, rcc);
            }
        }

        #[cfg(feature = "h735")]
        impl RccPeriph for pac::USART10 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb2, usart10, rcc);
            }
        }
    }
}

#[cfg(not(any(
    feature = "f",
    feature = "g0",
    feature = "wl",
    feature = "l",
    feature = "c0"
)))]
impl RccPeriph for pac::LPUART1 {
    fn en_reset(rcc: &RegisterBlock) {
        #[cfg(not(feature = "h7"))]
        rcc_en_reset_apb1enr2!(lpuart1, rcc);

        #[cfg(feature = "h7")]
        rcc_en_reset!(apb4, lpuart1, rcc);
    }

    #[cfg(feature = "l4")]
    fn read_chan() -> DmaChannel {
        DmaInput::LPuart1Rx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn write_chan() -> DmaChannel {
        DmaInput::Lpuart1Tx.dma1_channel()
    }

    #[cfg(feature = "l4")]
    fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Lpuart1Rx);
    }

    #[cfg(feature = "l4")]
    fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
        dma::channel_select(regs, DmaInput::Lpuart1Tx);
    }
}

// #[cfg(not(any(
//     feature = "f",
//     feature = "l",
//     feature = "g0",
//     feature = "h7",
//     feature = "wb",
//     feature = "wl",
//     feature = "g431"
// )))]
// impl RccPeriph for pac::LPUART2 {
//     fn en_reset(rcc: &RegisterBlock) {
//         rcc_en_reset_apb1enr2!(lpuart2, rcc);
//     }

//     #[cfg(feature = "l4")]
//     fn read_chan() -> DmaChannel {
//         DmaInput::LPuart2Rx.dma1_channel()
//     }

//     #[cfg(feature = "l4")]
//     fn write_chan() -> DmaChannel {
//         DmaInput::Lpuart2Tx.dma1_channel()
//     }

//     #[cfg(feature = "l4")]
//     fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
//         dma::channel_select(regs, DmaInput::Lpuart2Rx);
//     }

//     #[cfg(feature = "l4")]
//     fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(regs: &mut D) {
//         dma::channel_select(regs, DmaInput::Lpuart2Tx);
//     }
// }

#[cfg(not(any(
    feature = "f401",
    feature = "f411",
    feature = "f412",
    feature = "wb",
    feature = "g0",
    feature = "c0",
    feature = "l412",
)))]
cfg_if! {
    if #[cfg(all(feature = "h7", not(feature = "h7b3")))] {
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, dac12, rcc);
            }
        }
    } else if #[cfg(feature = "f3")] {
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, dac1, rcc);
            }
            fn read_chan() -> DmaChannel {unimplemented!()}

            fn write_chan() -> DmaChannel {unimplemented!()}
        }

        #[cfg(any(feature = "f303", feature = "f373", feature = "f3x4"))]
        impl RccPeriph for pac::DAC2 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, dac2, rcc);
            }
            fn read_chan() -> DmaChannel {unimplemented!()}

            fn write_chan() -> DmaChannel {unimplemented!()}
        }
    } else if #[cfg(feature = "g4")] {
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(ahb2, dac1, rcc);
            }
        }

        impl RccPeriph for pac::DAC2 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(ahb2, dac2, rcc);
            }
        }

        impl RccPeriph for pac::DAC3 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(ahb2, dac3, rcc);
            }
        }

        impl RccPeriph for pac::DAC4 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(ahb2, dac4, rcc);
            }
        }
    } else if #[cfg(feature = "h5")] {
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                // todo: Should be DAC1 PAC-side.
                rcc_en_reset!(ahb2, dac12, rcc);
            }
        }
    } else if #[cfg(feature = "f4")] {
        // F4 only uses 1 enable, despite having 2 devices. (each with 1 channel)
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                rcc_en_reset!(apb1, dac, rcc);
            }
        }
    } else {
        impl RccPeriph for DAC1 {
            fn en_reset(rcc: &RegisterBlock) {
                #[cfg(feature = "wl")]
                rcc.apb1enr1().modify(|_, w| w.dac1en().bit(true));
                #[cfg(not(feature = "wl"))]
                rcc_en_reset!(apb1, dac1, rcc);
            }

            #[cfg(feature = "l4")]
            fn read_chan() -> DmaChannel {unimplemented!()}

            #[cfg(feature = "l4")]
            fn write_chan() -> DmaChannel {unimplemented!()}

            #[cfg(feature = "l4")]
            fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {unimplemented!()}

            #[cfg(feature = "l4")]
            fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(_regs: &mut D) {unimplemented!()}
        }
    }
}

// todo: APB1LR2 on L5, and AHB4 on H7. Fix it. (I2C4)
// I2cDevice::Four => {

// We currently only set up DAC1 DMA, and it's split by channels, not device.

// todo: Use thsi approach for USART and SAI. When you un-macro them, ADC and Timer as well.

// todo: ADC DMA on F3 and L4. Note that they're not currently set up as `RccPeripheral`,.

// #[cfg(any(feature = "f3", feature = "l4"))]
// impl DmaPeriph for ADC1 {
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn read_chan() -> DmaChannel {
//         DmaInput::Adc1.dma1_channel()
//     }
//
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn write_chan() -> DmaChannel {
//         unimplemented!()
//     }
//
//     #[cfg(feature = "l4")]
//     fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(dma: &mut Dma<D>) {
//         dma.channel_select(DmaInput::Adc1);
//     }
//
//     #[cfg(feature = "l4")]
//     fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(dma: &mut Dma<D>) {
//         unimplemented!()
//     }
// }
//
// #[cfg(any(
//     feature = "l4x1",
//     feature = "l4x2",
//     feature = "l412",
//     feature = "l4x5",
//     feature = "l4x6",
// ))]
// impl DmaPeriph for pac::ADC2 {
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn read_chan() -> DmaChannel {
//         DmaInput::Adc2.dma1_channel()
//     }
//
//     #[cfg(any(feature = "f3", feature = "l4"))]
//     fn write_chan() -> DmaChannel {
//         unimplemented!()
//     }
//
//     #[cfg(feature = "l4")]
//     fn read_sel<D: Deref<Target = dma1::RegisterBlock>>(dma: &mut Dma<D>) {
//         dma.channel_select(DmaInput::Adc2);
//     }
//
//     #[cfg(feature = "l4")]
//     fn write_sel<D: Deref<Target = dma1::RegisterBlock>>(dma: &mut Dma<D>) {
//         unimplemented!()
//     }
// }

// L4 and F3 only have DMA on ADC 1 and 2.
