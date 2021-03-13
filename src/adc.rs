//! Based on `stm32f3xx-hal`.

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

use crate::{pac::RCC, traits::ClockCfg};

use paste::paste;

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm3",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
))]
use crate::pac::{ADC1, ADC1_2};

#[cfg(any(
    feature = "f302",
    feature = "f303",
))]
use crate::pac::ADC2;

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(
    feature = "f303",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm3",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
))]
use crate::pac::{ADC3, ADC3_4, ADC4};

#[cfg(not(feature = "f3"))]
use crate::pac::ADC_COMMON;

#[cfg(any(
    feature = "l4x3",
))]
use crate::pac::{ADC1};

#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x5",
    feature = "l4x6",
))]
use crate::pac::{ADC1, ADC2};

#[cfg(any(feature = "l4x5", feature = "l4x6"))]
use crate::pac::ADC3;

#[cfg(feature = "l5")]
use crate::pac::ADC;

const MAX_ADVREGEN_STARTUP_US: u32 = 10;

/// https://github.com/rust-embedded/embedded-hal/issues/267
/// We are simulating an enum due to how the `embedded-hal` trait is set up.
pub mod AdcChannel {
    pub struct C1;
    pub struct C2;
    pub struct C3;
    pub struct C4;
    pub struct C5;
    pub struct C6;
    pub struct C7;
    pub struct C8;
    pub struct C9;
    pub struct C10;
    pub struct C11;
    pub struct C12;
    pub struct C13;
    pub struct C14;
    pub struct C15;
    pub struct C16;
    pub struct C17;
    pub struct C18;
}

#[derive(Clone, Copy)]
enum AdcNum {
    One,
    Two,
    Three,
    Four,
}

/// Analog Digital Converter Peripheral
pub struct Adc<ADC> {
    /// ADC Register
    regs: ADC,
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
#[repr(u8)]
pub enum ClockMode {
    // /// Use Kernel Clock adc_ker_ck_input divided by PRESC. Asynchronous to AHB clock
    ASYNC = 0b00,
    /// Use AHB clock rcc_hclk3. In this case rcc_hclk must equal sys_d1cpre_ck
    SyncDiv1 = 0b01,
    /// Use AHB clock rcc_hclk3 divided by 2
    SyncDiv2 = 0b10,
    /// Use AHB clock rcc_hclk3 divided by 4
    SyncDiv4 = 0b11,
}

impl Default for ClockMode {
    fn default() -> Self {
        Self::SyncDiv2
    }
}

/// ADC data register alignment
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Align {
    /// Right alignment of output data
    Right = 0,
    /// Left alignment of output data
    Left = 1,
}

impl Default for Align {
    fn default() -> Self {
        Align::Right
    }
}

// Abstract implementation of ADC functionality
macro_rules! hal {
    ($ADC:ident, $ADC_COMMON:ident, $adc:ident, $adc_num:expr) => {
        impl Adc<$ADC> {
            paste! {
                /// Init a new ADC
                ///
                /// Enables the clock, performs a calibration and enables the ADC
                ///
                /// # Panics
                /// If one of the following occurs:
                /// * the clocksetting is not well defined.
                /// * the clock was already enabled with a different setting
                ///
                pub fn [<new_ $adc _unchecked>]<C: ClockCfg>(
                    regs: $ADC,
                    adc_common : &mut $ADC_COMMON,
                    ckmode: ClockMode,
                    clocks: &C,
                    rcc: &mut RCC,
                ) -> Self {
                    let mut this_adc = Self {
                        regs,
                        ckmode,
                        operation_mode: None,
                    };
                    if !(this_adc.clocks_welldefined(clocks)) {
                        panic!("Clock settings not well defined");
                    }
                    if !(this_adc.enable_clock(adc_common, rcc)){
                        panic!("Clock already enabled with a different setting");
                    }
                    this_adc.set_align(Align::default());
                    this_adc.calibrate(clocks);
                    // Reference Manual: "ADEN bit cannot be set during ADCAL=1
                    // and 4 ADC clock cycle after the ADCAL
                    // bit is cleared by hardware."
                    // this_adc.wait_adc_clk_cycles(4);
                    asm::delay(ckmode as u32 * 4);
                    this_adc.enable();

                    this_adc
                }
            }

             fn enable_clock(&self, common_regs: &mut $ADC_COMMON, rcc: &mut RCC) -> bool {
                 // `common_regs` is the same as `self.regs` for non-f3. On f3, it's a diff block,
                 // eg `adc12`.
                cfg_if::cfg_if! {
                    if #[cfg(any(feature = "f3"))] {
                        match $adc_num {
                            AdcNum::One | AdcNum::Two => {
                                #[cfg(any(feature = "f301"))]
                                if rcc.ahbenr.read().adc1en().is_enabled() {
                                    return (common_regs.ccr.read().ckmode().bits() == self.ckmode as u8);
                                }
                                #[cfg(any(feature = "f301"))]
                                rcc.ahbenr.modify(|_, w| w.adc1en().set_bit());

                                #[cfg(not(any(feature = "f301")))]
                                if rcc.ahbenr.read().adc12en().is_enabled() {
                                    return (common_regs.ccr.read().ckmode().bits() == self.ckmode as u8);
                                }
                                #[cfg(not(any(feature = "f301")))]
                                rcc.ahbenr.modify(|_, w| w.adc12en().set_bit());
                            }
                            AdcNum::Three | AdcNum::Four => {
                                #[cfg(not(any(feature = "f301", feature = "f302")))]
                                if rcc.ahbenr.read().adc34en().is_enabled() {
                                    return (common_regs.ccr.read().ckmode().bits() == self.ckmode as u8);
                                }
                                #[cfg(not(any(feature = "f301", feature = "f302")))]
                                rcc.ahbenr.modify(|_, w| w.adc34en().set_bit());
                            }

                        }

                    } else {
                        if rcc.ahb2enr.read().adcen().bit_is_set() {
                            return (common_regs.ccr.read().ckmode().bits() == self.ckmode as u8);
                        }
                        rcc.ahb2enr.modify(|_, w| w.adcen().set_bit());
                    }
                }

                common_regs.ccr.modify(|_, w| unsafe { w
                    .ckmode().bits(self.ckmode as u8)
                });
                true
            }

            /// Software can use ClockMode::SyncDiv1 only if
            /// hclk and sysclk are the same. (see reference manual 15.3.3)
            fn clocks_welldefined<C: ClockCfg>(&self, clocks: &C) -> bool {
                if (self.ckmode == ClockMode::SyncDiv1) {
                    clocks.hclk() == clocks.sysclk()
                } else {
                    true
                }
            }

            /// sets up adc in one shot mode for a single channel
            pub fn setup_oneshot(&mut self) {
                self.regs.cr.modify(|_, w| w.adstp().set_bit());
                self.regs.isr.modify(|_, w| w.ovr().clear_bit());

                self.regs.cfgr.modify(|_, w| w
                    .cont().clear_bit()  // single conversion mode.
                    .ovrmod().clear_bit()  // preserve DR data
                );

                self.set_sequence_len(1);

                self.operation_mode = Some(OperationMode::OneShot);
            }

            fn set_sequence_len(&mut self, len: u8) {
                if len - 1 >= 16 {
                    panic!("ADC sequence length must be in 1..=16")
                }

                // typo
                cfg_if::cfg_if! {
                    if #[cfg(any(feature = "l4x1", feature = "l4x2", feature = "l4x3", feature = "l4x5"))] {
                        self.regs.sqr1.modify(|_, w| unsafe { w.l3().bits(len - 1) });
                    } else {
                        self.regs.sqr1.modify(|_, w| unsafe { w.l().bits(len - 1) });
                    }
                }
            }

            fn set_align(&self, align: Align) {
                self.regs.cfgr.modify(|_, w| w.align().bit(align as u8 != 0));
            }

            fn enable(&mut self) {
                self.regs.cr.modify(|_, w| w.aden().set_bit());  // Enable
                while self.regs.isr.read().adrdy().bit_is_clear() {}  // Wait until ready
            }

            fn disable(&mut self) {
                self.regs.cr.modify(|_, w| w.addis().set_bit()); // Disable
            }

            /// Calibrate according to 15.3.8 in the Reference Manual
            fn calibrate<C: ClockCfg>(&mut self, clocks: &C) {
                let enabled;
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        enabled = self.regs.cr.read().advregen().bits() == 1;
                    } else {
                        enabled = self.regs.cr.read().advregen().bit_is_set();
                    }
                }
                if !enabled {
                    self.advregen_enable();
                    self.wait_advregen_startup(clocks);
                }

                self.disable();

                self.regs.cr.modify(|_, w| w
                    .adcaldif().clear_bit()  // single ended. // todo: Cal differential option!
                    .adcal().set_bit()); // start calibration.

                while self.regs.cr.read().adcal().bit_is_set() {}
            }

            fn advregen_enable(&mut self){
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        // need to go through intermediate first
                        self.regs.cr.modify(|_, w| w.advregen().intermediate());
                        self.regs.cr.modify(|_, w| w.advregen().enabled());  // Enable voltage regulator.
                    } else {
                        self.regs.cr.modify(|_, w| w.advregen().set_bit());  // Enable voltage regulator.
                    }
                }

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

                self.regs.cr.modify(|_, w| w.adstart().set_bit());  // Start
                while self.regs.isr.read().eos().bit_is_clear() {}  // wait until complete.
                self.regs.isr.modify(|_, w| w.eos().set_bit());  // Clear
                return self.regs.dr.read().bits() as u16;  // todo make sure you don't need rdata field.
            }

            fn ensure_oneshot(&mut self) {
                if self.operation_mode != Some(OperationMode::OneShot) {
                    self.setup_oneshot();
                }
            }

            /// This should only be invoked with the defined channels for the particular
            /// device. (See Pin/Channel mapping above)
            fn select_single_chan(&self, chan: u8) {
                self.regs.sqr1.modify(|_, w|
                    // NOTE(unsafe): chan is the x in ADCn_INx
                    // Channel as u8 is the ADC channel to use.
                    unsafe { w.sq1().bits(chan) }
                );
            }

            /// Note: only allowed when ADSTART = 0
            // TODO: there are boundaries on how this can be set depending on the hardware.
            fn set_chan_smps(&self, chan: u8, smp: SampleTime) {
                // Channel as u8 is the ADC channel to use.
                unsafe {
                    match chan {
                        1 => self.regs.smpr1.modify(|_, w| w.smp1().bits(smp.bitcode())),
                        2 => self.regs.smpr1.modify(|_, w| w.smp2().bits(smp.bitcode())),
                        3 => self.regs.smpr1.modify(|_, w| w.smp3().bits(smp.bitcode())),
                        4 => self.regs.smpr1.modify(|_, w| w.smp4().bits(smp.bitcode())),
                        5 => self.regs.smpr1.modify(|_, w| w.smp5().bits(smp.bitcode())),
                        6 => self.regs.smpr1.modify(|_, w| w.smp6().bits(smp.bitcode())),
                        7 => self.regs.smpr1.modify(|_, w| w.smp7().bits(smp.bitcode())),
                        8 => self.regs.smpr1.modify(|_, w| w.smp8().bits(smp.bitcode())),
                        9 => self.regs.smpr1.modify(|_, w| w.smp9().bits(smp.bitcode())),
                        11 => self.regs.smpr2.modify(|_, w| w.smp10().bits(smp.bitcode())),
                        12 => self.regs.smpr2.modify(|_, w| w.smp12().bits(smp.bitcode())),
                        13 => self.regs.smpr2.modify(|_, w| w.smp13().bits(smp.bitcode())),
                        14 => self.regs.smpr2.modify(|_, w| w.smp14().bits(smp.bitcode())),
                        15 => self.regs.smpr2.modify(|_, w| w.smp15().bits(smp.bitcode())),
                        16 => self.regs.smpr2.modify(|_, w| w.smp16().bits(smp.bitcode())),
                        17 => self.regs.smpr2.modify(|_, w| w.smp17().bits(smp.bitcode())),
                        18 => self.regs.smpr2.modify(|_, w| w.smp18().bits(smp.bitcode())),
                        _ => unreachable!(),
                    };
                }
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

        // todo: This is so janky. There has to be a better way.
        impl Channel<$ADC> for AdcChannel::C1 {
            type ID = u8;
            fn channel() -> u8 { 1 }
        }
        impl Channel<$ADC> for AdcChannel::C2 {
            type ID = u8;
            fn channel() -> u8 { 2 }
        }
        impl Channel<$ADC> for AdcChannel::C3 {
            type ID = u8;
            fn channel() -> u8 { 3}
        }
        impl Channel<$ADC> for AdcChannel::C4 {
            type ID = u8;
            fn channel() -> u8 { 4 }
        }
        impl Channel<$ADC> for AdcChannel::C5 {
            type ID = u8;
            fn channel() -> u8 { 5 }
        }
        impl Channel<$ADC> for AdcChannel::C6 {
            type ID = u8;
            fn channel() -> u8 { 6 }
        }
        impl Channel<$ADC> for AdcChannel::C7 {
            type ID = u8;
            fn channel() -> u8 { 7 }
        }
        impl Channel<$ADC> for AdcChannel::C8 {
            type ID = u8;
            fn channel() -> u8 { 8 }
        }
        impl Channel<$ADC> for AdcChannel::C9 {
            type ID = u8;
            fn channel() -> u8 { 9 }
        }
        impl Channel<$ADC> for AdcChannel::C10 {
            type ID = u8;
            fn channel() -> u8 { 10 }
        }
        impl Channel<$ADC> for AdcChannel::C11 {
            type ID = u8;
            fn channel() -> u8 { 11 }
        }
        impl Channel<$ADC> for AdcChannel::C12 {
            type ID = u8;
            fn channel() -> u8 { 12 }
        }
        impl Channel<$ADC> for AdcChannel::C13 {
            type ID = u8;
            fn channel() -> u8 { 13 }
        }
        impl Channel<$ADC> for AdcChannel::C14 {
            type ID = u8;
            fn channel() -> u8 { 14 }
        }
        impl Channel<$ADC> for AdcChannel::C15 {
            type ID = u8;
            fn channel() -> u8 { 15 }
        }
        impl Channel<$ADC> for AdcChannel::C16 {
            type ID = u8;
            fn channel() -> u8 { 16 }
        }
        impl Channel<$ADC> for AdcChannel::C17 {
            type ID = u8;
            fn channel() -> u8 { 17 }
        }
        impl Channel<$ADC> for AdcChannel::C18 {
            type ID = u8;
            fn channel() -> u8 { 18 }
        }
    }
}

// todo: l and h. and rest of f3

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
))]
hal!(ADC1, ADC1_2, adc1, AdcNum::One);

#[cfg(any(feature = "f302", feature = "f303",))]
hal!(ADC2, ADC1_2, adc2, AdcNum::Two);

#[cfg(any(feature = "f303"))]
hal!(ADC3, ADC3_4, adc3, AdcNum::Three);

#[cfg(any(feature = "f303"))]
hal!(ADC4, ADC3_4, adc4, AdcNum::Four);

#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6",
))]
hal!(ADC1, ADC_COMMON, adc1, AdcNum::One);

#[cfg(any(feature = "l4x1", feature = "l4x2", feature = "l4x5", feature = "l4x6",))]
hal!(ADC2, ADC_COMMON, adc2, AdcNum::Two);

#[cfg(any(feature = "l4x5", feature = "l4x6",))]
hal!(ADC3, ADC_COMMON, adc3, AdcNum::Three);

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "l5",
    ))] {
        // todo: How many channels does L5 have? This isn't type checked
        hal!(ADC, ADC_COMMON, adc, AdcNum:One);  // Todo: Channel 2. Fight through that chan imp to do it.
    }
}
