//! API for the ADC (Analog to Digital Converter)

use cortex_m::asm;
use embedded_hal::adc::{Channel, OneShot};

use crate::{
    pac::{self, RCC},
    traits::ClockCfg,
};

use cfg_if::cfg_if;
use paste::paste;

const MAX_ADVREGEN_STARTUP_US: u32 = 10;

/// https://github.com/rust-embedded/embedded-hal/issues/267
/// We are simulating an enum due to how the `embedded-hal` trait is set up.
/// This will be fixed in a future version of EH.
#[allow(non_snake_case)]
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
    pub struct C19;
    pub struct C20;
}

#[derive(Clone, Copy)]
enum AdcNum {
    One,
    Two,
    Three,
    Four,
}

/// Analog to Digital converter peripheral
pub struct Adc<ADC> {
    /// ADC Register
    regs: ADC,
    ckmode: ClockMode,
    operation_mode: OperationMode,
    // Most families use u8 values for calibration, but H7 uses u16.
    cal_single_ended: Option<u16>, // Stored calibration value for single-ended
    cal_differential: Option<u16>, // Stored calibration value for differential
}

// todo: Adc sampling time below depends on the STM32 family. Eg the numbers below
// todo are wrong for L4, but the idea is the same.
/// ADC sampling time
///
/// Each channel can be sampled with a different sample time.
/// There is always an overhead of 13 ADC clock cycles.
/// E.g. For Sampletime T_19 the total conversion time (in ADC clock cycles) is
/// 13 + 19 = 32 ADC Clock Cycles
/// [derive(Clone, Copy)]
#[repr(u8)]
pub enum SampleTime {
    /// 1.5 ADC clock cycles
    T1 = 0b000,
    /// 2.5 ADC clock cycles
    T2 = 0b001,
    /// 4.5 ADC clock cycles
    T4 = 0b010,
    /// 7.5 ADC clock cycles
    T7 = 0b011,
    /// 19.5 ADC clock cycles
    T19 = 0b100,
    /// 61.5 ADC clock cycles
    T61 = 0b101,
    /// 181.5 ADC clock cycles
    T181 = 0b110,
    /// 601.5 ADC clock cycles
    T601 = 0b111,
}

impl Default for SampleTime {
    /// T_1 is also the reset value.
    fn default() -> Self {
        SampleTime::T1
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select single-ended, or differential inputs. Sets bits in the ADC[x]_DIFSEL register.
pub enum InputType {
    SingleEnded = 0,
    Differential = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// ADC operation mode
// TODO: Implement other modes
pub enum OperationMode {
    /// OneShot Mode
    OneShot = 0,
    Continuous = 1,
}

// #[cfg(any(feature = "l4", feature = "l5"))]
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// /// ADC Clock mode
// pub enum ClockMode {
//     // todo
//     Sysclock = 0b00,
//     PllSai1 = 0b01,
// }
//
// #[cfg(any(feature = "l4", feature = "l5"))]
// impl Default for ClockMode {
//     fn default() -> Self {
//         Self::Sysclock
//     }
// }

// todo: Clock mode for other MCUs!

// #[cfg(any(feature = "f3"))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// ADC Clock mode
pub enum ClockMode {
    /// Use Kernel Clock adc_ker_ck_input divided by PRESC. Asynchronous to AHB clock
    ASYNC = 0b00,
    /// Use AHB clock rcc_hclk3. In this case rcc_hclk must equal sys_d1cpre_ck
    SyncDiv1 = 0b01,
    /// Use AHB clock rcc_hclk3 divided by 2
    SyncDiv2 = 0b10,
    /// Use AHB clock rcc_hclk3 divided by 4
    SyncDiv4 = 0b11,
}

// #[cfg(any(feature = "f3"))]
impl Default for ClockMode {
    fn default() -> Self {
        Self::SyncDiv2
    }
}

#[cfg(not(feature = "h7"))]
/// ADC data register alignment
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Align {
    /// Right alignment of output data
    Right = 0,
    /// Left alignment of output data
    Left = 1,
}

#[cfg(not(feature = "h7"))]
impl Default for Align {
    fn default() -> Self {
        Align::Right
    }
}

#[cfg(feature = "h7")]
/// ADC data register alignment
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Align {
    NoShift = 0,
    L1 = 1,
    L2 = 2,
    L3 = 3,
    L4 = 4,
    L5 = 5,
    L6 = 6,
    L7 = 7,
    L8 = 8,
    L9 = 9,
    L10 = 10,
    L11 = 11,
    L12 = 12,
    L13 = 13,
    L14 = 14,
    L15 = 15,
}

#[cfg(feature = "h7")]
impl Default for Align {
    fn default() -> Self {
        Align::NoShift
    }
}

// Abstract implementation of ADC functionality
macro_rules! hal {
    ($ADC:ident, $ADC_COMMON:ident, $adc:ident, $adc_num:expr) => {
        impl Adc<pac::$ADC> {
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
                    regs: pac::$ADC,
                    adc_common : &mut pac::$ADC_COMMON,
                    ckmode: ClockMode,
                    clocks: &C,
                    rcc: &mut RCC,
                ) -> Self {
                    let mut this_adc = Self {
                        regs,
                        ckmode,
                        operation_mode: OperationMode::OneShot,
                        cal_single_ended: None,
                        cal_differential: None,
                    };

                    this_adc.enable_clock(adc_common, rcc);
                    this_adc.set_align(Align::default());

                    this_adc.advregen_enable(clocks);

                    this_adc.calibrate(InputType::SingleEnded, clocks);
                    this_adc.calibrate(InputType::Differential, clocks);
                    // Reference Manual: "ADEN bit cannot be set during ADCAL=1
                    // and 4 ADC clock cycle after the ADCAL
                    // bit is cleared by hardware."
                    asm::delay(ckmode as u32 * 4);
                    this_adc.enable();

                    this_adc.setup_oneshot(); // todo: Setup Continuous

                    this_adc
                }
            }

            /// Enable the ADC clock, and set the clock mode.
            fn enable_clock(&self, common_regs: &mut pac::$ADC_COMMON, rcc: &mut RCC) {
                 // `common_regs` is the same as `self.regs` for non-f3. On f3, it's a diff block,
                 // eg `adc12`.
                cfg_if! {
                    if #[cfg(any(feature = "f3", feature = "f4"))] {
                        match $adc_num {
                            AdcNum::One | AdcNum::Two => {
                                rcc.ahbenr.modify(|_, w| w.adc12en().set_bit());
                                rcc.ahbrstr.modify(|_, w| w.adc12rst().set_bit());
                                rcc.ahbrstr.modify(|_, w| w.adc12rst().clear_bit());
                            }
                            AdcNum::Three | AdcNum::Four => {
                                rcc.ahbenr.modify(|_, w| w.adc34en().set_bit());
                                rcc.ahbrstr.modify(|_, w| w.adc34rst().set_bit());
                                rcc.ahbrstr.modify(|_, w| w.adc34rst().clear_bit());
                            }
                        }
                    } else if #[cfg(feature = "h7")] {
                        match $adc_num {
                            AdcNum::One | AdcNum::Two => {
                                rcc.ahb1enr.modify(|_, w| w.adc12en().set_bit());
                                rcc.ahb1rstr.modify(|_, w| w.adc12rst().set_bit());
                                rcc.ahb1rstr.modify(|_, w| w.adc12rst().clear_bit());
                            }
                            AdcNum::Three | AdcNum::Four => {
                                rcc.ahb4enr.modify(|_, w| w.adc3en().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.adc3rst().set_bit());
                                rcc.ahb4rstr.modify(|_, w| w.adc3rst().clear_bit());
                            }
                        }

                    } else {
                        rcc.ahb2enr.modify(|_, w| w.adcen().set_bit());
                        rcc.ahb2rstr.modify(|_, w| w.adcrst().set_bit());
                        rcc.ahb2rstr.modify(|_, w| w.adcrst().clear_bit());
                    }
                }

                // todo: This should be the same with H7, per RM. Likely a PAC bug.
                #[cfg(not(feature = "h7"))]
                common_regs.ccr.modify(|_, w| unsafe { w.ckmode().bits(self.ckmode as u8) });
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

                self.operation_mode = OperationMode::OneShot;
            }

            pub fn set_sequence_len(&mut self, len: u8) {
                if len - 1 >= 16 {
                    panic!("ADC sequence length must be in 1..=16")
                }

                // typo
                cfg_if! {
                    if #[cfg(any(feature = "l4x1", feature = "l4x2", feature = "l4x3", feature = "l4x5"))] {
                        self.regs.sqr1.modify(|_, w| unsafe { w.l3().bits(len - 1) });
                    } else {
                        self.regs.sqr1.modify(|_, w| unsafe { w.l().bits(len - 1) });
                    }
                }
            }

            pub fn set_align(&self, align: Align) {
                #[cfg(feature = "h7")]
                self.regs.cfgr2.modify(|_, w| w.lshift().bits(align as u8));

                #[cfg(not(feature = "h7"))]
                self.regs.cfgr.modify(|_, w| w.align().bit(align as u8 != 0));
            }

            /// Enable the ADC.
            /// ADEN=1 enables the ADC. The flag ADRDY will be set once the ADC is ready for
            /// operation.
            fn enable(&mut self) {
                // 1. Clear the ADRDY bit in the ADC_ISR register by writing ‘1’.
                self.regs.isr.modify(|_, w| w.adrdy().clear_bit());
                // 2. Set ADEN=1.
                self.regs.cr.modify(|_, w| w.aden().set_bit());  // Enable
                // 3. Wait until ADRDY=1 (ADRDY is set after the ADC startup time). This can be done
                // using the associated interrupt (setting ADRDYIE=1).
                while self.regs.isr.read().adrdy().bit_is_clear() {}  // Wait until ready
                // 4. Clear the ADRDY bit in the ADC_ISR register by writing ‘1’ (optional).
                self.regs.isr.modify(|_, w| w.adrdy().clear_bit());
            }

            /// Disable the ADC.
            /// ADDIS=1 disables the ADC. ADEN and ADDIS are then automatically cleared by
            /// hardware as soon as the analog ADC is effectively disabled
            fn disable(&mut self) {
                // 1. Check that both ADSTART=0 and JADSTART=0 to ensure that no conversion is
                // ongoing. If required, stop any regular and injected conversion ongoing by setting
                // ADSTP=1 and JADSTP=1 and then wait until ADSTP=0 and JADSTP=0.
                self.stop_conversions();

                // 2. Set ADDIS=1.
                self.regs.cr.modify(|_, w| w.addis().set_bit()); // Disable

                // 3. If required by the application, wait until ADEN=0, until the analog
                // ADC is effectively disabled (ADDIS will automatically be reset once ADEN=0)
                // ( We're skipping this)
            }

            /// If any conversions are in progress, stop them. This is a step listed in the RMs
            /// for disable, and calibration procedures. See L4 RM: 16.4.17.
            /// When the ADSTP bit is set by software, any ongoing regular conversion is aborted with
            /// partial result discarded (ADC_DR register is not updated with the current conversion).
            /// When the JADSTP bit is set by software, any ongoing injected conversion is aborted with
            /// partial result discarded (ADC_JDRy register is not updated with the current conversion).
            /// The scan sequence is also aborted and reset (meaning that relaunching the ADC would
            /// restart a new sequence).
            pub fn stop_conversions(&mut self) {
                // The software can decide to stop regular conversions ongoing by setting ADSTP=1 and
                // injected conversions ongoing by setting JADSTP=1.
                // Stopping conversions will reset the ongoing ADC operation. Then the ADC can be
                // reconfigured (ex: changing the channel selection or the trigger) ready for a new operation.
                let cr_val = self.regs.cr.read();
                if cr_val.adstart().bit_is_set() || self.regs.cr.read().jadstart().bit_is_set() {
                    self.regs.cr.modify(|_, w| {
                        w.adstp().set_bit();
                        w.jadstp().set_bit()
                    })
                }
                while self.regs.cr.read().adstart().bit_is_set() || self.regs.cr.read().jadstart().bit_is_set() {}
            }

            /// Check if the ADC is enabled.
            fn is_enabled(&self) -> bool {
                self.regs.cr.read().aden().bit_is_set()
            }

            fn is_advregen_enabled(&self) -> bool {
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        self.regs.cr.read().advregen().bits() == 1
                    } else {
                        self.regs.cr.read().advregen().bit_is_set()
                    }
                }
            }

            /// Enable the voltage regulator, and exit deep sleep mode (some MCUs)
            pub fn advregen_enable<C: ClockCfg>(&mut self, clocks: &C){
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        // `F303 RM, 15.3.6:
                        // 1. Change ADVREGEN[1:0] bits from ‘10’ (disabled state, reset state) into ‘00’.
                        // 2. Change ADVREGEN[1:0] bits from ‘00’ into ‘01’ (enabled state).
                        self.regs.cr.modify(|_, w| unsafe { w.advregen().bits(0b00)});
                        self.regs.cr.modify(|_, w| unsafe { w.advregen().bits(0b01)});
                    } else {
                        // L443 RM, 16.4.6:
                        // By default, the ADC is in Deep-power-down mode where its supply is internally switched off
                        // to reduce the leakage currents (the reset state of bit DEEPPWD is 1 in the ADC_CR
                        // register).
                        // To start ADC operations, it is first needed to exit Deep-power-down mode by setting bit
                        // DEEPPWD=0.
                        self.regs.cr.modify(|_, w| w.deeppwd().clear_bit());  // Exit deep sleep mode.
                        self.regs.cr.modify(|_, w| w.advregen().set_bit());  // Enable voltage regulator.
                    }
                }

                self.wait_advregen_startup(clocks)
            }

            /// Disable power, eg to save power in low power modes. Inferred from RM,
            /// we should run this before entering `STOP` mode, in conjunction with with
            /// disabling the ADC.
            pub fn advregen_disable(&mut self){
                cfg_if! {
                    if #[cfg(feature = "f3")] {
                        // `F303 RM, 15.3.6:
                        // 1. Change ADVREGEN[1:0] bits from ‘01’ (enabled state) into ‘00’.
                        // 2. Change ADVREGEN[1:0] bits from ‘00’ into ‘10’ (disabled state)
                        self.regs.cr.modify(|_, w| unsafe { w.advregen().bits(0b00) });
                        self.regs.cr.modify(|_, w| unsafe { w.advregen().bits(0b10) });
                    } else {
                        // L4 RM, 16.4.6: Writing DEEPPWD=1 automatically disables the ADC voltage
                        // regulator and bit ADVREGEN is automatically cleared.
                        // When the internal voltage regulator is disabled (ADVREGEN=0), the internal analog
                        // calibration is kept.
                        // In ADC Deep-power-down mode (DEEPPWD=1), the internal analog calibration is lost and
                        // it is necessary to either relaunch a calibration or re-apply the calibration factor which was
                        // previously saved (
                        self.regs.cr.modify(|_, w| w.deeppwd().set_bit());
                        // todo: We could offer an option to disable advregen without setting deeppwd,
                        // todo, which would keep calibration.
                    }
                }
            }

            /// Wait for the advregen to startup.
            ///
            /// This is based on the MAX_ADVREGEN_STARTUP_US of the device.
            fn wait_advregen_startup<C: ClockCfg>(&self, clocks: &C) {
                let mut delay = (MAX_ADVREGEN_STARTUP_US * 1_000_000) / clocks.sysclk();
                // https://github.com/rust-embedded/cortex-m/pull/328
                if delay < 2 {  // Work around a bug in cortex-m.
                    delay = 2;
                }
                asm::delay(delay);
            }

            /// Calibrate. See L4 RM, 16.5.8, or F404 RM, section 15.3.8.
            /// Stores calibration values, which can be re-inserted later,
            /// eg after entering ADC deep sleep mode, or MCU STANDBY or VBAT.
            pub fn calibrate<C: ClockCfg>(&mut self, input_type: InputType, clocks: &C) {
                // 1. Ensure DEEPPWD=0, ADVREGEN=1 and that ADC voltage regulator startup time has
                // elapsed.
                if !self.is_advregen_enabled() {
                    self.advregen_enable(clocks);
                }

                let was_enabled = self.is_enabled();
                // Calibration can only be initiated when the ADC is disabled (when ADEN=0).
                // 2. Ensure that ADEN=0
                if was_enabled {
                    self.disable();
                }

                self.regs.cr.modify(|_, w| w
                    // RM:
                    // The calibration factor to be applied for single-ended input conversions is different from the
                    // factor to be applied for differential input conversions:
                    // • Write ADCALDIF=0 before launching a calibration which will be applied for singleended input conversions.
                    // • Write ADCALDIF=1 before launching a calibration which will be applied for differential
                    // input conversions.
                    // 3. Select the input mode for this calibration by setting ADCALDIF=0 (single-ended input)
                    // or ADCALDIF=1 (differential input).
                    .adcaldif().bit(input_type as u8 != 0)
                    // The calibration is then initiated by software by setting bit ADCAL=1.
                    // 4. Set ADCAL=1.
                    .adcal().set_bit()); // start calibration.

                // ADCAL bit stays at 1 during all the
                // calibration sequence. It is then cleared by hardware as soon the calibration completes. At
                // this time, the associated calibration factor is stored internally in the analog ADC and also in
                // the bits CALFACT_S[6:0] or CALFACT_D[6:0] of ADC_CALFACT register (depending on
                // single-ended or differential input calibration)
                // 5. Wait until ADCAL=0.
                while self.regs.cr.read().adcal().bit_is_set() {}

                // 6. The calibration factor can be read from ADC_CALFACT register.
                match input_type {
                    InputType::SingleEnded => {
                        let val = self.regs.calfact.read().calfact_s().bits();
                        #[cfg(not(feature = "h7"))]
                        let val = val as u16;
                        self.cal_single_ended = Some(val);
                    }
                    InputType::Differential => {
                         let val = self.regs.calfact.read().calfact_d().bits();
                         #[cfg(not(feature = "h7"))]
                         let val = val as u16;
                         self.cal_differential = Some(val);
                    }
                }

                if was_enabled {
                    self.enable();
                }
            }

            /// Insert a previously-saved calibration value into the ADC.
            /// Se L4 RM, 16.4.8.
            pub fn inject_calibration(&mut self) {
                // 1. Ensure ADEN=1 and ADSTART=0 and JADSTART=0 (ADC enabled and no
                // conversion is ongoing).
                if !self.is_enabled() {
                    self.enable();
                }
                self.stop_conversions();


                // 2. Write CALFACT_S and CALFACT_D with the new calibration factors.
                if let Some(cal) = self.cal_single_ended {
                    #[cfg(not(feature = "h7"))]
                    let cal = cal as u8;
                    self.regs.calfact.modify(|_, w| unsafe { w.calfact_s().bits(cal) });
                }
                if let Some(cal) = self.cal_differential {
                    #[cfg(not(feature = "h7"))]
                    let cal = cal as u8;
                    self.regs.calfact.modify(|_, w| unsafe { w.calfact_d().bits(cal) });
                }

                // 3. When a conversion is launched, the calibration factor will be injected into the analog
                // ADC only if the internal analog calibration factor differs from the one stored in bits
                // CALFACT_S for single-ended input channel or bits CALFACT_D for differential input
                // channel.
            }

            /// Select single-ended, or differential conversions for a given channel.
            pub fn set_input_type(&mut self, channel: u8, input_type: InputType) {
                // L44 RM, 16.4.7:
                // Channels can be configured to be either single-ended input or differential input by writing
                // into bits DIFSEL[15:1] in the ADC_DIFSEL register. This configuration must be written while
                // the ADC is disabled (ADEN=0). Note that DIFSEL[18:16,0] are fixed to single ended
                // channels and are always read as 0.
                let was_enabled = self.is_enabled();
                if was_enabled {
                    self.disable();
                }

                // todo: Figure out how you do the bit shift math here; we don't have individual fields.
                // difsel!(self.regs, channel, input_type);
                // self.regs.difsel.write(|w| w.difsel_1_15r().bits(input_type as u8));

                if was_enabled {
                    self.enable();
                }
            }

            /// Start a conversion: Either a single measurement, or continuous conversions.
            /// See L4 RM 16.4.15 for details.
            pub fn start_conversion(&mut self, chan: u8, mode: OperationMode) {
                // Set continuous or differential mode.
                self.regs.cfgr.modify(|_, w| w.cont().bit(mode as u8 != 0));
                self.select_channel(chan);

                // L4 RM: In Single conversion mode, the ADC performs once all the conversions of the channels.
                // This mode is started with the CONT bit at 0 by either:
                // • Setting the ADSTART bit in the ADC_CR register (for a regular channel)
                // • Setting the JADSTART bit in the ADC_CR register (for an injected channel)
                // • External hardware trigger event (for a regular or injected channel)
                // (Here, we assume a regular channel)
                self.regs.cr.modify(|_, w| w.adstart().set_bit());  // Start

                // After the regular sequence is complete, after each conversion is complete,
                // the EOC (end of regular conversion) flag is set.
                // After the regular sequence is complete: The EOS (end of regular sequence) flag is set.
                // (We're ignoring eoc, since this module doesn't currently support sequences)
                while self.regs.isr.read().eos().bit_is_clear() {}  // wait until complete.
            }

            /// Read data from a conversion. In OneShot mode, this will generally be run right
            /// after `start_conversion`.
            pub fn read_result(&mut self) -> u16 {
                self.regs.dr.read().bits() as u16
            }

            /// Select the channel to sample. Note that this register allows setting a sequence,
            /// but for now, we only support converting one channel at a time.
            /// (Single-ended or differential, but not a sequence.)
            fn select_channel(&self, chan: u8) {
                // Channel as u8 is the ADC channel to use.
                self.regs.sqr1.modify(|_, w| unsafe { w.sq1().bits(chan) });
            }

            /// Select the sample time for a given channel.
            pub fn set_sample_time(&mut self, chan: u8, smp: SampleTime) {
                // Channel is the ADC channel to use.

                // RM: Note: only allowed when ADSTART = 0 and JADSTART = 0.
                self.stop_conversions();
                unsafe {
                    match chan {
                        1 => self.regs.smpr1.modify(|_, w| w.smp1().bits(smp as u8)),
                        2 => self.regs.smpr1.modify(|_, w| w.smp2().bits(smp as u8)),
                        3 => self.regs.smpr1.modify(|_, w| w.smp3().bits(smp as u8)),
                        4 => self.regs.smpr1.modify(|_, w| w.smp4().bits(smp as u8)),
                        5 => self.regs.smpr1.modify(|_, w| w.smp5().bits(smp as u8)),
                        6 => self.regs.smpr1.modify(|_, w| w.smp6().bits(smp as u8)),
                        7 => self.regs.smpr1.modify(|_, w| w.smp7().bits(smp as u8)),
                        8 => self.regs.smpr1.modify(|_, w| w.smp8().bits(smp as u8)),
                        9 => self.regs.smpr1.modify(|_, w| w.smp9().bits(smp as u8)),
                        11 => self.regs.smpr2.modify(|_, w| w.smp10().bits(smp as u8)),
                        12 => self.regs.smpr2.modify(|_, w| w.smp12().bits(smp as u8)),
                        13 => self.regs.smpr2.modify(|_, w| w.smp13().bits(smp as u8)),
                        14 => self.regs.smpr2.modify(|_, w| w.smp14().bits(smp as u8)),
                        15 => self.regs.smpr2.modify(|_, w| w.smp15().bits(smp as u8)),
                        16 => self.regs.smpr2.modify(|_, w| w.smp16().bits(smp as u8)),
                        17 => self.regs.smpr2.modify(|_, w| w.smp17().bits(smp as u8)),
                        18 => self.regs.smpr2.modify(|_, w| w.smp18().bits(smp as u8)),
                        // 19 => self.regs.smpr2.modify(|_, w| w.smp19().bits(smp as u8)),
                        // 20 => self.regs.smpr2.modify(|_, w| w.smp20().bits(smp as u8)),
                        _ => unreachable!(),
                    };
                }
            }
        }

        impl<WORD, PIN> OneShot<pac::$ADC, WORD, PIN> for Adc<pac::$ADC>
        where
            WORD: From<u16>,
            PIN: Channel<pac::$ADC, ID = u8>,
            {
                type Error = ();

                fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                    self.start_conversion(PIN::channel(), OperationMode::OneShot);
                    return Ok(self.read_result().into());
                }
        }

        // todo: This is so janky. There has to be a better way.
        impl Channel<pac::$ADC> for AdcChannel::C1 {
            type ID = u8;
            fn channel() -> u8 { 1 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C2 {
            type ID = u8;
            fn channel() -> u8 { 2 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C3 {
            type ID = u8;
            fn channel() -> u8 { 3}
        }
        impl Channel<pac::$ADC> for AdcChannel::C4 {
            type ID = u8;
            fn channel() -> u8 { 4 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C5 {
            type ID = u8;
            fn channel() -> u8 { 5 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C6 {
            type ID = u8;
            fn channel() -> u8 { 6 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C7 {
            type ID = u8;
            fn channel() -> u8 { 7 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C8 {
            type ID = u8;
            fn channel() -> u8 { 8 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C9 {
            type ID = u8;
            fn channel() -> u8 { 9 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C10 {
            type ID = u8;
            fn channel() -> u8 { 10 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C11 {
            type ID = u8;
            fn channel() -> u8 { 11 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C12 {
            type ID = u8;
            fn channel() -> u8 { 12 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C13 {
            type ID = u8;
            fn channel() -> u8 { 13 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C14 {
            type ID = u8;
            fn channel() -> u8 { 14 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C15 {
            type ID = u8;
            fn channel() -> u8 { 15 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C16 {
            type ID = u8;
            fn channel() -> u8 { 16 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C17 {
            type ID = u8;
            fn channel() -> u8 { 17 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C18 {
            type ID = u8;
            fn channel() -> u8 { 18 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C19 {
            type ID = u8;
            fn channel() -> u8 { 19 }
        }
        impl Channel<pac::$ADC> for AdcChannel::C20 {
            type ID = u8;
            fn channel() -> u8 { 20 }
        }
    }
}

#[cfg(any(feature = "f301", feature = "f302", feature = "f303",))]
hal!(ADC1, ADC1_2, adc1, AdcNum::One);

#[cfg(any(feature = "f302", feature = "f303",))]
hal!(ADC2, ADC1_2, adc2, AdcNum::Two);

#[cfg(any(feature = "f303"))]
hal!(ADC3, ADC3_4, adc3, AdcNum::Three);

#[cfg(any(feature = "f303"))]
hal!(ADC4, ADC3_4, adc4, AdcNum::Four);

#[cfg(any(feature = "l4"))]
hal!(ADC1, ADC_COMMON, adc1, AdcNum::One);

// todo: ADC 1 vs 2 on L5? L5 supports up to 2 ADCs, so I'm not sure what's going on here.
#[cfg(any(feature = "l5"))]
hal!(ADC, ADC_COMMON, adc1, AdcNum::One);

#[cfg(any(feature = "l4x1", feature = "l4x2", feature = "l4x5", feature = "l4x6",))]
hal!(ADC2, ADC_COMMON, adc2, AdcNum::Two);

#[cfg(any(feature = "l4x5", feature = "l4x6",))]
hal!(ADC3, ADC_COMMON, adc3, AdcNum::Three);

// todo: Beyond ADC1 for H7.
#[cfg(any(
    feature = "h743",
    feature = "h743v",
    feature = "h747cm4",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
))]
hal!(ADC1, ADC1, adc1, AdcNum::One);
