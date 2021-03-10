// Based on `stm32l4`.

//! API for the ADC (Analog to Digital Converter)
//!
//! # Examples
//!
//! Check out [examles/adc.rs].
//!
//! It can be built for the STM32F3Discovery running
//! `cargo build --example adc --features=stm32f303xc`
//!
//! [examples/adc.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/adc.rs

use cortex_m::asm;
use embedded_hal::adc::{Channel, OneShot};

use crate::traits::ClockCfg;

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l552",
    feature = "l562",
))]
use crate::{
    pac::{ADC1, ADC2},
};

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "f373",
))]
use crate::{
    pac::ADC1,
};

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "f302",
    feature = "f303",
    feature = "f3x4",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm3",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
))]
use crate::{
    pac::{ADC1, ADC1_2, ADC2},
};

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "f301",
))]
use crate::{
    pac::{ADC1, ADC1_2},
};

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "f303",
    feature = "f3x4",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm3",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
))]
use crate::{
    pac::{ADC3, ADC3_4, ADC4},
};

const MAX_ADVREGEN_STARTUP_US: u32 = 10;


/// Analog Digital Converter Peripheral
// TODO: Remove `pub` from the register block once all functionalities are implemented.
// Leave it here until then as it allows easy access to the registers.
pub struct Adc<ADC> {
    /// ADC Register
    pub rb: ADC,
    ckmode: ClockMode,
    operation_mode: Option<OperationMode>,
}

/// ADC sampling time
///
/// Each channel can be sampled with a different sample time.
/// There is always an overhead of 13 ADC clock cycles.
/// E.g. For Sampletime T_19 the total conversion time (in ADC clock cycles) is
/// 13 + 19 = 32 ADC Clock Cycles
pub enum SampleTime {
    /// 1.5 ADC clock cycles
    T_1,
    /// 2.5 ADC clock cycles
    T_2,
    /// 4.5 ADC clock cycles
    T_4,
    /// 7.5 ADC clock cycles
    T_7,
    /// 19.5 ADC clock cycles
    T_19,
    /// 61.5 ADC clock cycles
    T_61,
    /// 181.5 ADC clock cycles
    T_181,
    /// 601.5 ADC clock cycles
    T_601,
}

impl Default for SampleTime {
    /// T_1 is also the reset value.
    fn default() -> Self {
        SampleTime::T_1
    }
}

impl SampleTime {
    /// Conversion to bits for SMP
    fn bitcode(&self) -> u8 {
        match self {
            SampleTime::T_1 => 0b000,
            SampleTime::T_2 => 0b001,
            SampleTime::T_4 => 0b010,
            SampleTime::T_7 => 0b011,
            SampleTime::T_19 => 0b100,
            SampleTime::T_61 => 0b101,
            SampleTime::T_181 => 0b110,
            SampleTime::T_601 => 0b111,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
/// ADC operation mode
// TODO: Implement other modes (DMA, Differential,…)
pub enum OperationMode {
    /// OneShot Mode
    OneShot,
}

#[derive(Clone, Copy, PartialEq)]
/// ADC CkMode
// TODO: Add ASYNCHRONOUS mode
pub enum ClockMode {
    // /// Use Kernel Clock adc_ker_ck_input divided by PRESC. Asynchronous to AHB clock
    // ASYNCHRONOUS = 0,
    /// Use AHB clock rcc_hclk3. In this case rcc_hclk must equal sys_d1cpre_ck
    SyncDiv1 = 1,
    /// Use AHB clock rcc_hclk3 divided by 2
    SyncDiv2 = 2,
    /// Use AHB clock rcc_hclk3 divided by 4
    SyncDiv4 = 4,
}

impl Default for ClockMode {
    fn default() -> Self {
        Self::SyncDiv2
    }
}

// // ADC3_2 returns a pointer to a adc1_2 type, so this from is ok for both.
// impl From<CkMode> for CKMODE_A {
//     fn from(ckmode: CkMode) -> Self {
//         match ckmode {
//             //CkMode::ASYNCHRONOUS => CKMODE_A::ASYNCHRONOUS,
//             CkMode::SYNCDIV1 => CKMODE_A::SYNCDIV1,
//             CkMode::SYNCDIV2 => CKMODE_A::SYNCDIV2,
//             CkMode::SYNCDIV4 => CKMODE_A::SYNCDIV4,
//         }
//     }
// }

/// ADC data register alignment
pub enum Align {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl Default for Align {
    fn default() -> Self {
        Align::Right
    }
}

// impl From<Align> for ALIGN_A {
//     fn from(align: Align) -> ALIGN_A {
//         match align {
//             Align::Right => ALIGN_A::RIGHT,
//             Align::Left => ALIGN_A::LEFT,
//         }
//     }
// }

// /// Maps pins to ADC Channels.
// macro_rules! adc_pins {
//     ($ADC:ident, $($pin:ty => $chan:expr),+ $(,)*) => {
//         $(
//             impl Channel<$ADC> for $pin {
//                 type ID = u8;
//
//                 fn channel() -> u8 { $chan }
//             }
//         )+
//     };
// }

// # ADC1 Pin/Channel mapping
// ## f303

// #[cfg(feature = "stm32f303")]
// adc_pins!(ADC1,
//     gpioa::PA0<Analog> => 1,
//     gpioa::PA1<Analog> => 2,
//     gpioa::PA2<Analog> => 3,
//     gpioa::PA3<Analog> => 4,
//     gpioc::PC0<Analog> => 6,
//     gpioc::PC1<Analog> => 7,
//     gpioc::PC2<Analog> => 8,
//     gpioc::PC3<Analog> => 9,
// );

// #[cfg(any(feature = "stm32f303x6", feature = "stm32f303x8"))]
// adc_pins!(ADC1,
//     gpiob::PB0<Analog> => 11,
//     gpiob::PB1<Analog> => 12,
//     gpiob::PB13<Analog> => 13,
// );

// #[cfg(any(
//     feature = "stm32f303xb",
//     feature = "stm32f303xc",
//     feature = "stm32f303xd",
//     feature = "stm32f303xe",
// ))]
// adc_pins!(ADC1,
//     gpiof::PF4<Analog> => 5,
//     gpiof::PF2<Analog> => 10,
// );

// # ADC2 Pin/Channel mapping
// ## f303

// #[cfg(feature = "stm32f303")]
// adc_pins!(ADC2,
//     gpioa::PA4<Analog> => 1,
//     gpioa::PA5<Analog> => 2,
//     gpioa::PA6<Analog> => 3,
//     gpioa::PA7<Analog> => 4,
//     gpioc::PC4<Analog> => 5,
//     gpioc::PC0<Analog> => 6,
//     gpioc::PC1<Analog> => 7,
//     gpioc::PC2<Analog> => 8,
//     gpioc::PC3<Analog> => 9,
//     gpioc::PC5<Analog> => 11,
//     gpiob::PB2<Analog> => 12,
// );

// #[cfg(any(feature = "stm32f303x6", feature = "stm32f303x8"))]
// adc_pins!(ADC2,
//     gpiob::PB12<Analog> => 13,
//     gpiob::PB14<Analog> => 14,
//     gpiob::PB15<Analog> => 15,
// );

// #[cfg(any(
//     feature = "stm32f303xb",
//     feature = "stm32f303xc",
//     feature = "stm32f303xd",
//     feature = "stm32f303xe",
// ))]
// adc_pins!(ADC2,
//     gpiof::PF2<Analog> => 10,
// );

// # ADC3 Pin/Channel mapping
// ## f303
//
// #[cfg(any(
//     feature = "stm32f303xb",
//     feature = "stm32f303xc",
//     feature = "stm32f303xd",
//     feature = "stm32f303xe",
// ))]
// adc_pins!(ADC3,
//     gpiob::PB1<Analog> => 1,
//     gpioe::PE9<Analog> => 2,
//     gpioe::PE13<Analog> => 3,
//     // There is no ADC3 Channel #4
//     gpiob::PB13<Analog> => 5,
//     gpioe::PE8<Analog> => 6,
//     gpiod::PD10<Analog> => 7,
//     gpiod::PD11<Analog> => 8,
//     gpiod::PD12<Analog> => 9,
//     gpiod::PD13<Analog> => 10,
//     gpiod::PD14<Analog> => 11,
//     gpiob::PB0<Analog> => 12,
//     gpioe::PE7<Analog> => 13,
//     gpioe::PE10<Analog> => 14,
//     gpioe::PE11<Analog> => 15,
//     gpioe::PE12<Analog> => 16,
// );

// # ADC4 Pin/Channel mapping
// ## f303
//
// #[cfg(any(
//     feature = "stm32f303xb",
//     feature = "stm32f303xc",
//     feature = "stm32f303xd",
//     feature = "stm32f303xe",
// ))]
// adc_pins!(ADC4,
//     gpioe::PE14<Analog> => 1,
//     gpioe::PE15<Analog> => 2,
//     gpiob::PB12<Analog> => 3,
//     gpiob::PB14<Analog> => 4,
//     gpiob::PB15<Analog> => 5,
//     gpioe::PE8<Analog> => 6,
//     gpiod::PD10<Analog> => 7,
//     gpiod::PD11<Analog> => 8,
//     gpiod::PD12<Analog> => 9,
//     gpiod::PD13<Analog> => 10,
//     gpiod::PD14<Analog> => 11,
//     gpiod::PD8<Analog> => 12,
//     gpiod::PD9<Analog> => 13,
// );

// Abstract implementation of ADC functionality
// Do not use directly. See adc12_hal for a applicable Macro.
// TODO: Extend/generalize beyond f303
macro_rules! adc_hal {
    ($(
            $ADC:ident: ($adcx:ident, $ADC_COMMON:ident),
    )+) => {
        $(
            impl Adc<$ADC> {

                /// Init a new ADC
                ///
                /// Enables the clock, performs a calibration and enables the ADC
                ///
                /// # Panics
                /// If one of the following occurs:
                /// * the clocksetting is not well defined.
                /// * the clock was already enabled with a different setting
                ///
                pub fn $adcx<C: ClockCfg>(
                    rb: $ADC,
                    adc_common : &mut $ADC_COMMON,
                    ahb: &mut AHB,
                    ckmode: CkMode,
                    clocks: &C,
                ) -> Self {
                    let mut this_adc = Self {
                        rb,
                        clocks,
                        ckmode,
                        operation_mode: None,
                    };
                    if !(this_adc.clocks_welldefined(clocks)) {
                        crate::panic!("Clock settings not well defined");
                    }
                    if !(this_adc.enable_clock(ahb, adc_common)){
                        crate::panic!("Clock already enabled with a different setting");
                    }
                    this_adc.set_align(Align::default());
                    this_adc.calibrate(clocks);   // todo: Cal causes freeze! TS commented out.
                    // Reference Manual: "ADEN bit cannot be set during ADCAL=1
                    // and 4 ADC clock cycle after the ADCAL
                    // bit is cleared by hardware."
                    // this_adc.wait_adc_clk_cycles(4);
                    asm::delay(ckmode as u32 * 4); // todo attempted fix???
                    this_adc.enable();

                    this_adc
                }

                /// Software can use CkMode::SYNCDIV1 only if
                /// hclk and sysclk are the same. (see reference manual 15.3.3)
                fn clocks_welldefined<C: ClockCfg>(&self, clocks: &C) -> bool {
                    if (self.ckmode == CkMode::SYNCDIV1) {
                        clocks.hclk() == clocks.sysclk()
                    } else {
                        true
                    }
                }

                /// sets up adc in one shot mode for a single channel
                pub fn setup_oneshot(&mut self) {
                    self.rb.cr.modify(|_, w| w.adstp().stop());
                    self.rb.isr.modify(|_, w| w.ovr().clear());

                    self.rb.cfgr.modify(|_, w| w
                        .cont().single()
                        .ovrmod().preserve()
                    );

                    self.set_sequence_len(1);

                    self.operation_mode = Some(OperationMode::OneShot);
                }

                fn set_sequence_len(&mut self, len: u8) {
                    crate::assert!(len - 1 < 16, "ADC sequence length must be in 1..=16");
                    self.rb.sqr1.modify(|_, w| w.l().bits(len - 1));
                }

                fn set_align(&self, align: Align) {
                    self.rb.cfgr.modify(|_, w| w.align().variant(align.into()));
                }

                fn enable(&mut self) {
                    self.rb.cr.modify(|_, w| w.aden().enable());
                    while self.rb.isr.read().adrdy().is_not_ready() {}
                }

                fn disable(&mut self) {
                    self.rb.cr.modify(|_, w| w.addis().disable());
                }

                /// Calibrate according to 15.3.8 in the Reference Manual
                fn calibrate<C: ClockCfg>(&mut self, clocks: &C) {
                    if !self.rb.cr.read().advregen().is_enabled() {
                        self.advregen_enable();
                        self.wait_advregen_startup(clocks);
                    }

                    self.disable();

                    self.rb.cr.modify(|_, w| w
                        .adcaldif().single_ended()
                        .adcal()   .calibration());

                    while self.rb.cr.read().adcal().is_calibration() {}
                }

                fn advregen_enable(&mut self){
                    // need to go through intermediate first
                    self.rb.cr.modify(|_, w| w.advregen().intermediate());
                    self.rb.cr.modify(|_, w| w.advregen().enabled());
                }

                /// wait for the advregen to startup.
                ///
                /// This is based on the MAX_ADVREGEN_STARTUP_US of the device.
                fn wait_advregen_startup<C: ClockCfg>(&self, clocks: &C) {
                    // Prevents crashes. Not sure why / when it started showing up.
                    // https://github.com/rust-embedded/cortex-m/pull/328
                    let mut delay = (MAX_ADVREGEN_STARTUP_US * 1_000_000) / clocks.sysclk();
                    if delay < 2 {
                        delay = 2;
                    }
                    asm::delay(delay);
                }

                /// busy ADC read
                fn convert_one(&mut self, chan: u8) -> u16 {
                    self.ensure_oneshot();
                    self.set_chan_smps(chan, SampleTime::default());
                    self.select_single_chan(chan);

                    self.rb.cr.modify(|_, w| w.adstart().start());
                    while self.rb.isr.read().eos().is_not_complete() {}
                    self.rb.isr.modify(|_, w| w.eos().clear());
                    return self.rb.dr.read().rdata().bits();
                }

                fn ensure_oneshot(&mut self) {
                    if self.operation_mode != Some(OperationMode::OneShot) {
                        self.setup_oneshot();
                    }
                }

                /// This should only be invoked with the defined channels for the particular
                /// device. (See Pin/Channel mapping above)
                fn select_single_chan(&self, chan: u8) {
                    self.rb.sqr1.modify(|_, w|
                        // NOTE(unsafe): chan is the x in ADCn_INx
                        unsafe { w.sq1().bits(chan) }
                    );
                }

                /// Note: only allowed when ADSTART = 0
                // TODO: there are boundaries on how this can be set depending on the hardware.
                fn set_chan_smps(&self, chan: u8, smp: SampleTime) {
                    match chan {
                        1 => self.rb.smpr1.modify(|_, w| w.smp1().bits(smp.bitcode())),
                        2 => self.rb.smpr1.modify(|_, w| w.smp2().bits(smp.bitcode())),
                        3 => self.rb.smpr1.modify(|_, w| w.smp3().bits(smp.bitcode())),
                        4 => self.rb.smpr1.modify(|_, w| w.smp4().bits(smp.bitcode())),
                        5 => self.rb.smpr1.modify(|_, w| w.smp5().bits(smp.bitcode())),
                        6 => self.rb.smpr1.modify(|_, w| w.smp6().bits(smp.bitcode())),
                        7 => self.rb.smpr1.modify(|_, w| w.smp7().bits(smp.bitcode())),
                        8 => self.rb.smpr1.modify(|_, w| w.smp8().bits(smp.bitcode())),
                        9 => self.rb.smpr1.modify(|_, w| w.smp9().bits(smp.bitcode())),
                        11 => self.rb.smpr2.modify(|_, w| w.smp10().bits(smp.bitcode())),
                        12 => self.rb.smpr2.modify(|_, w| w.smp12().bits(smp.bitcode())),
                        13 => self.rb.smpr2.modify(|_, w| w.smp13().bits(smp.bitcode())),
                        14 => self.rb.smpr2.modify(|_, w| w.smp14().bits(smp.bitcode())),
                        15 => self.rb.smpr2.modify(|_, w| w.smp15().bits(smp.bitcode())),
                        16 => self.rb.smpr2.modify(|_, w| w.smp16().bits(smp.bitcode())),
                        17 => self.rb.smpr2.modify(|_, w| w.smp17().bits(smp.bitcode())),
                        18 => self.rb.smpr2.modify(|_, w| w.smp18().bits(smp.bitcode())),
                        _ => crate::unreachable!(),
                    };
                }

            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC>
            where
                WORD: From<u16>,
                PIN: Channel<$ADC, ID = u8>,
                {
                    type Error = ();

                    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                        let res = self.convert_one(PIN::channel());
                        return Ok(res.into());
                    }
                }
        )+
    }
}

// Macro to implement ADC functionallity for ADC1 and ADC2
// TODO: Extend/differentiate beyond f303.
macro_rules! adc12_hal {
    ($(
            $ADC:ident: ($adcx:ident),
    )+) => {
        $(
            impl Adc<$ADC> {
                /// Returns true iff
                ///     the clock can be enabled with the given settings
                ///  or the clock was already enabled with the same settings
                fn enable_clock(&self, ahb: &mut AHB, adc_common: &mut ADC1_2) -> bool {
                    if ahb.enr().read().adc12en().is_enabled() {
                        return (adc_common.ccr.read().ckmode().variant() == self.ckmode.into());
                    }
                    ahb.enr().modify(|_, w| w.adc12en().enabled());
                    adc_common.ccr.modify(|_, w| w
                        .ckmode().variant(self.ckmode.into())
                    );
                    true
                }
            }
            adc_hal! {
                $ADC: ($adcx, ADC1_2),
            }
        )+
    }
}

// Macro to implement ADC functionallity for ADC3 and ADC4
// TODO: Extend/differentiate beyond f303.
#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
macro_rules! adc34_hal {
    ($(
            $ADC:ident: ($adcx:ident),
    )+) => {
        $(
            impl Adc<$ADC> {
                /// Returns true iff
                ///     the clock can be enabled with the given settings
                ///  or the clock was already enabled with the same settings
                fn enable_clock(&self, ahb: &mut AHB, adc_common: &mut ADC3_4) -> bool {
                    if ahb.enr().read().adc34en().is_enabled() {
                        return (adc_common.ccr.read().ckmode().variant() == self.ckmode.into());
                    }
                    ahb.enr().modify(|_, w| w.adc34en().enabled());
                    adc_common.ccr.modify(|_, w| w
                        .ckmode().variant(self.ckmode.into())
                    );
                    true
                }
            }
            adc_hal! {
                $ADC: ($adcx, ADC3_4),
            }
        )+
    }
}

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
adc12_hal! {
    ADC1: (adc1),
    ADC2: (adc2),
}
#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
adc34_hal! {
    ADC3: (adc3),
    ADC4: (adc4),
}
