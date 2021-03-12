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

use crate::{
    pac::{ADC1, RCC},
    traits::ClockCfg,
};

use paste::paste;

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
use crate::pac::{adc1::cfgr::ALIGN_A, adc1_2::ccr::CKMODE_A, ADC1_2, ADC2};

// todo: what other features support ADC3 and 4? Trim this down as you get errors
#[cfg(any(feature = "f301",))]
use crate::pac::ADC1_2;

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
use crate::pac::{ADC3, ADC3_4, ADC4};

const MAX_ADVREGEN_STARTUP_US: u32 = 10;

#[derive(Clone, Copy)]
enum AdcNum {
    One,
    Two,
    Three,
    Four,
}

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
    ($ADC:ident, $adc:ident, $ADC_COMMON:ident, $adc_num:expr) => {
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
                rb: $ADC,
                adc_common : &mut $ADC_COMMON,
                ckmode: ClockMode,
                clocks: &C,
                rcc: &mut RCC,
            ) -> Self {
                let mut this_adc = Self {
                    rb,
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

             fn enable_clock(&self, adc_common: &mut $ADC_COMMON, rcc: &mut RCC) -> bool {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        match $adc_num {
                            AdcNum::One | AdcNum::Two => {
                                if rcc.ahbenr.read().adc12en().is_enabled() {
                                    return (adc_common.ccr.read().ckmode().bits() == self.ckmode as u8);
                                }
                                rcc.ahbenr.modify(|_, w| w.adc12en().set_bit());
                            }
                            AdcNum::Three | AdcNum::Four => {
                                if rcc.ahbenr.read().adc34en().is_enabled() {
                                    return (adc_common.ccr.read().ckmode().bits() == self.ckmode as u8);
                                }
                                rcc.ahbenr.modify(|_, w| w.adc34en().set_bit());
                            }

                        }

                    } else {
                        if rcc.ahb2enr.read().adcen().is_enabled() {
                            return (adc_common.ccr.read().ckmode().bits() == self.ckmode as u8);
                        }
                        rcc.ahb2enr.modify(|_, w| w.adcen().set_bit());
                    }
                }

                adc_common.ccr.modify(|_, w| w
                    .ckmode().bits(self.ckmode as u8)
                );
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
                if len - 1 >= 16 {
                    panic!("ADC sequence length must be in 1..=16")
                }
                self.rb.sqr1.modify(|_, w| w.l().bits(len - 1));
            }

            fn set_align(&self, align: Align) {
                self.rb.cfgr.modify(|_, w| w.align().bit(align as u8 != 0));
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
                    _ => unreachable!(),
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
    }
}

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
hal!(ADC1, adc1, ADC1_2, AdcNum::One);

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
hal!(ADC2, adc2, ADC1_2, AdcNum::Two);

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
hal!(ADC3, adc3, ADC3_4, AdcNum::Three);

#[cfg(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
))]
hal!(ADC4, adc4, ADC3_4, AdcNum::Four);
