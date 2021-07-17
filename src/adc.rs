//! Provides functionality for the ADC (Analog to Digital Converter).

use cortex_m::{asm, interrupt::free};

#[cfg(feature = "embedded-hal")]
use embedded_hal::adc::{Channel, OneShot};

use core::{ops::Deref, ptr};

use crate::{
    clocks::Clocks,
    pac::{self, RCC},
    rcc_en_reset,
};

use cfg_if::cfg_if;
use paste::paste;

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(feature = "f3", feature = "l4", feature = "g4"))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel, DmaInput};

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

#[derive(Clone, Copy, PartialEq)]
pub enum AdcDevice {
    One,
    Two,
    Three,
    Four,
    #[cfg(feature = "g4")]
    Five,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// See L44 RM, section 16.5: ADC interrupts
pub enum AdcInterrupt {
    Ready,
    EndOfConversion,
    EnfOfSequence,
    EndofConversionInjected,
    EndOfSequenceInjected,
    Watchdog1,
    Watchdog2,
    Watchdog3,
    EndOfSamplingPhase,
    Overrun,
    InjectedOverflow,
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
pub enum OperationMode {
    /// OneShot Mode
    OneShot = 0,
    Continuous = 1,
}

// todo: Check the diff ways of configuring clock; i don't think teh enum below covers all.(?)

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// ADC Clock mode
/// (L44 RM, Section 16.4.3) The input clock is the same for the three ADCs and can be selected between two different
/// clock sources (see Figure 40: ADC clock scheme):
/// 1. The ADC clock can be a specific clock source. It can be derived from the following
/// clock sources:
/// – The system clock
/// – PLLSAI1 (single ADC implementation)
/// Refer to RCC Section for more information on how to generate ADC dedicated clock.
/// To select this scheme, bits CKMODE[1:0] of the ADCx_CCR register must be reset.
/// 2. The ADC clock can be derived from the AHB clock of the ADC bus interface, divided by
/// a programmable factor (1, 2 or 4). In this mode, a programmable divider factor can be
/// selected (/1, 2 or 4 according to bits CKMODE[1:0]).
/// To select this scheme, bits CKMODE[1:0] of the ADCx_CCR register must be different
/// from “00”.
pub enum ClockMode {
    // Use Kernel Clock adc_ker_ck_input divided by PRESC. Asynchronous to AHB clock
    Async = 0b00,
    /// Use AHB clock rcc_hclk3.
    /// "For option 2), a prescaling factor of 1 (CKMODE[1:0]=01) can be used only if the AHB
    /// prescaler is set to 1 (HPRE[3:0] = 0xxx in RCC_CFGR register)."
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

/// Represents an Analog to Digital Converter (ADC) peripheral.
pub struct Adc<R> {
    /// ADC Register
    regs: R,
    // Note: We don't own the common regs; pass them mutably where required, since they may be used
    // by a different ADC.
    device: AdcDevice,
    ckmode: ClockMode,
    operation_mode: OperationMode,
    // Most families use u8 values for calibration, but H7 uses u16.
    cal_single_ended: Option<u16>, // Stored calibration value for single-ended
    cal_differential: Option<u16>, // Stored calibration value for differential
    vdda_calibrated: f32,
}

// Abstract implementation of ADC functionality
macro_rules! hal {
    ($ADC:ident, $ADC_COMMON:ident, $adc:ident, $rcc_num:tt) => {
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
                pub fn [<new_ $adc>](
                    regs: pac::$ADC,
                    device: AdcDevice,
                    common_regs : &mut pac::$ADC_COMMON,
                    ckmode: ClockMode,
                    clocks: &Clocks,
                ) -> Self {
                    let mut result = Self {
                        regs,
                        device,
                        ckmode,
                        operation_mode: OperationMode::OneShot,
                        cal_single_ended: None,
                        cal_differential: None,
                        vdda_calibrated: 0.
                    };

                    result.enable_clock(common_regs);
                    result.set_align(Align::default());

                    result.advregen_enable(clocks);

                    result.calibrate(InputType::SingleEnded, clocks);
                    result.calibrate(InputType::Differential, clocks);

                    // Reference Manual: "ADEN bit cannot be set during ADCAL=1
                    // and 4 ADC clock cycle after the ADCAL
                    // bit is cleared by hardware."
                    let adc_per_cpu_cycles = match result.ckmode {
                        ClockMode::Async => unimplemented!(),
                        ClockMode::SyncDiv1 => 1,
                        ClockMode::SyncDiv2 => 2,
                        ClockMode::SyncDiv4 => 4,
                    };
                    asm::delay(adc_per_cpu_cycles * 4);
                    result.enable();

                    result.setup_oneshot(); // todo: Setup Continuous

                    // Set up VDDA only after the ADC is otherwise enabled.
                    result.setup_vdda(common_regs, clocks);

                    result
                }
            }

            /// Enable the ADC clock, and set the clock mode.
            fn enable_clock(&self, regs_common: &mut pac::$ADC_COMMON) {
            // todo: Consider merging this code into `new`, as we do in other modules.
                free(|cs| {
                    let mut rcc = unsafe { &(*RCC::ptr()) };

                    paste! {
                        cfg_if! {
                            if #[cfg(any(feature = "f3", feature = "h7"))] {
                                rcc_en_reset!(ahb1, [<adc $rcc_num>], rcc);
                            } else if #[cfg(feature = "f4")] {
                                rcc_en_reset!(2, [<adc $rcc_num>], rcc);
                            } else if #[cfg(any(feature = "h7"))] {
                            // todo: 1 and 2 are on ahb1enr etc. 3 is on ahb4. 3 won't work here.
                                rcc_en_reset!(ahb1, [<adc $rcc_num>], rcc);
                            } else if #[cfg(any(feature = "g4"))] {
                                rcc_en_reset!(ahb2, [<adc $rcc_num>], rcc);
                            } else {  // ie L4, L5, G0(?)
                                rcc_en_reset!(ahb2, adc, rcc);
                            }
                        }
                    }
                });

                regs_common.ccr.modify(|_, w| unsafe { w.ckmode().bits(self.ckmode as u8) });

            }

            /// sets up adc in one shot mode for a single channel
            pub fn setup_oneshot(&mut self) {
                self.regs.cr.modify(|_, w| w.adstp().set_bit());
                self.regs.isr.modify(|_, w| w.ovr().clear_bit());

                self.regs.cfgr.modify(|_, w| w
                    .cont().clear_bit()  // single conversion mode.
                    // RM:
                    // • OVRMOD=0: The overrun event preserves the data register from being overrun: the
                    // old data is maintained and the new conversion is discarded and lost. If OVR remains at
                    // 1, any further conversions will occur but the result data will be also discarded.
                    // • OVRMOD=1: The data register is overwritten with the last conversion result and the
                    // previous unread data is lost. If OVR remains at 1, any further conversions will operate
                    // normally and the ADC_DR register will always contain the latest converted data.
                    .ovrmod().clear_bit()  // preserve DR data
                );

                self.set_sequence_len(1);

                self.operation_mode = OperationMode::OneShot;
            }

            pub fn set_sequence_len(&mut self, len: u8) {
                if len - 1 >= 16 {
                    panic!("ADC sequence length must be in 1..=16")
                }

                self.regs.sqr1.modify(|_, w| unsafe { w.l().bits(len - 1) });
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
                self.regs.isr.modify(|_, w| w.adrdy().set_bit());
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
                    });

                    while self.regs.cr.read().adstart().bit_is_set() || self.regs.cr.read().jadstart().bit_is_set() {}
                }
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
            pub fn advregen_enable(&mut self, clocks: &Clocks){
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
            fn wait_advregen_startup(&self, clocks: &Clocks) {
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
            pub fn calibrate(&mut self, input_type: InputType, clocks: &Clocks) {
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

                // Note that we don't use the `difsel` PAC accessor here; not required,
                // and would require some feature gating due to differences in field names among some
                // variants. Would also requires shifting `channel` by 1 more.
                let val = self.regs.difsel.read().bits();
                self.regs.difsel.write(|w| unsafe { w.bits(
                    val | ((input_type as u32) << channel)
                )});

                if was_enabled {
                    self.enable();
                }
            }

            /// Select a sequence to sample, by inputting a single channel and position.
            pub fn set_sequence(&mut self, chan: u8, position: u8) {
                match position {
                    1 => self.regs.sqr1.modify(|_, w| unsafe { w.sq1().bits(chan) }),
                    2 => self.regs.sqr1.modify(|_, w| unsafe { w.sq2().bits(chan) }),
                    3 => self.regs.sqr1.modify(|_, w| unsafe { w.sq3().bits(chan) }),
                    4 => self.regs.sqr1.modify(|_, w| unsafe { w.sq4().bits(chan) }),
                    5 => self.regs.sqr2.modify(|_, w| unsafe { w.sq5().bits(chan) }),
                    6 => self.regs.sqr2.modify(|_, w| unsafe { w.sq6().bits(chan) }),
                    7 => self.regs.sqr2.modify(|_, w| unsafe { w.sq7().bits(chan) }),
                    8 => self.regs.sqr2.modify(|_, w| unsafe { w.sq8().bits(chan) }),
                    9 => self.regs.sqr2.modify(|_, w| unsafe { w.sq9().bits(chan) }),
                    10 => self.regs.sqr3.modify(|_, w| unsafe { w.sq10().bits(chan) }),
                    11 => self.regs.sqr3.modify(|_, w| unsafe { w.sq11().bits(chan) }),
                    12 => self.regs.sqr3.modify(|_, w| unsafe { w.sq12().bits(chan) }),
                    13 => self.regs.sqr3.modify(|_, w| unsafe { w.sq13().bits(chan) }),
                    14 => self.regs.sqr3.modify(|_, w| unsafe { w.sq14().bits(chan) }),
                    15 => self.regs.sqr4.modify(|_, w| unsafe { w.sq15().bits(chan) }),
                    16 => self.regs.sqr4.modify(|_, w| unsafe { w.sq16().bits(chan) }),
                    _ => panic!("Sequence out of bounds. Only 16 positions are available."),
                }
            }

            /// Select the sample time for a given channel.
            pub fn set_sample_time(&mut self, chan: u8, smp: SampleTime) {
                // Channel is the ADC channel to use.

                // RM: Note: only allowed when ADSTART = 0 and JADSTART = 0.
                self.stop_conversions();
                unsafe {
                    match chan {
                        #[cfg(not(feature = "f3"))]
                        0 => self.regs.smpr1.modify(|_, w| w.smp0().bits(smp as u8)),
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

            /// Set up the internal voltage reference, to improve conversion from reading
            /// to voltage accuracy. See L44 RM, section 16.4.34: "Monitoring the internal voltage reference"
            fn setup_vdda(&mut self, regs_common: &mut pac::$ADC_COMMON, clocks: &Clocks) {
                // todo: Using ints intead of floats here and in read_voltage, for use on Cortex M0s?
                // todo: May need to use mv.
                regs_common.ccr.modify(|_, w| w.vrefen().set_bit());
                // "Table 24. Embedded internal voltage reference" states that it takes a maximum of 12 us
                // to stabilize the internal voltage reference, we wait a little more.
                let mut delay = (15_000_000) / clocks.sysclk();
                // https://github.com/rust-embedded/cortex-m/pull/328
                if delay < 2 {  // Work around a bug in cortex-m.
                    delay = 2;
                }
                asm::delay(delay);


                // (From L4xx-hal) "Table 24. Embedded internal voltage reference" states that the sample time needs to be
                // at a minimum 4 us. With 640.5 ADC cycles we have a minimum of 8 us at 80 MHz, leaving
                // some headroom.
                // todo: Reset existing sample time after this measurement?
                // todo: This may not work properly on variants clocked faster than 80Mhz(?)

                // RM: It is possible to monitor the internal voltage reference (VREFINT) to have a reference point for
                // evaluating the ADC VREF+ voltage level.
                // The internal voltage reference is internally connected to the input channel 0 of the ADC1
                // (ADC1_INP0).

                // Regardless of which ADC we're on, we take this reading using ADC1.
                #[cfg(not(any(feature = "l5", feature = "g4")))]
                let vref_reading = if self.device != AdcDevice::One {
                    // todo: What if ADC1 is alreayd enabled and configured differently?
                    // todo: Either way, if you're also using ADC1, this will screw things up⋅.

                    // If we're currently using ADC1 (and this is a different ADC), skip this step for now;
                    // VDDA will be wrong,
                    // and all readings using voltage conversion will be wrong.
                    if unsafe { (*pac::ADC1::ptr()).cr.read().aden().bit_is_set() } {
                        // todo: Take an ADC1 reading if this is the case.
                        return
                    }

                    let mut dp = unsafe { pac::Peripherals::steal() };

                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            let mut common_regs = dp.ADC1_2;
                        } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                            let mut common_regs = dp.ADC_COMMON;
                        } else {
                            let mut common_regs = dp.ADC12_COMMON;
                        }
                    }

                    let mut adc1 = Adc::new_adc1(
                        dp.ADC1,
                        AdcDevice::One,
                        &mut common_regs,
                        ClockMode::default(),
                        clocks,
                    );
                    adc1.set_sample_time(0, SampleTime::T601);
                    let reading = adc1.read(0);

                    // Disable ADC1 and its clock.
                    adc1.disable();
                    // todo: Disable the ADC1 clock. (Note it's on 3 diff regs depending on family)

                    reading
                } else {
                    self.set_sample_time(0, SampleTime::T601);
                    self.read(0)
                };

                #[cfg(feature = "l5")]
                let vref_reading = 0.; // todo handle this! Just take a reading off the only ADC.

                #[cfg(feature = "g4")]
                let vref_reading = 0.; // todo handle this! Weird macro issue (chicken+egg?)

                // self.set_sample_time(0, old_sample_time);
                regs_common.ccr.modify(|_, w| w.vrefen().clear_bit());


                // The VDDA power supply voltage applied to the microcontroller may be subject to variation or
                // not precisely known. The embedded internal voltage reference (VREFINT) and its calibration
                // data acquired by the ADC during the manufacturing process at VDDA = 3.0 V can be used to
                // evaluate the actual VDDA voltage level.
                // The following formula gives the actual VDDA voltage supplying the device:
                // VDDA = 3.0 V x VREFINT_CAL / VREFINT_DATA
                // where:
                // • VREFINT_CAL is the VREFINT calibration value
                // • VREFINT_DATA is the actual VREFINT output value converted by ADC

                // todo: This address may be different on different MCUs, even within the same family!!
                let vrefint_cal: u16 = unsafe { ptr::read_volatile(&*(0x1FFF_75AA as *const _)) };

                self.vdda_calibrated = 3. * vrefint_cal as f32 / vref_reading as f32
            }

            /// Convert a raw measurement into a voltage in Volts, using the calibrated VDDA.
            /// See RM0394, section 16.4.34
            pub fn reading_to_voltage(&self, reading: u16) -> f32 {
                // RM:
                // Converting a supply-relative ADC measurement to an absolute voltage value
                // The ADC is designed to deliver a digital value corresponding to the ratio between the analog
                // power supply and the voltage applied on the converted channel. For most application use
                // cases, it is necessary to convert this ratio into a voltage independent of VDDA. For
                // applications where VDDA is known and ADC converted values are right-aligned you can use
                // the following formula to get this absolute value:

                // V_CHANNELx = V_DDA / FULL_SCALE x ADCx_DATA

                // Where:
                // • VREFINT_CAL is the VREFINT calibration value
                // • ADC_DATA is the value measured by the ADC on channel x (right-aligned)
                // • VREFINT_DATA is the actual VREFINT output value converted by the ADC
                // • FULL_SCALE is the maximum digital value of the ADC output. For example with 12-bit
                // resolution, it will be 212 − 1 = 4095 or with 8-bit resolution, 28 − 1 = 255
                // todo: Pass vdda here, or to teh struct?
                // todo: FULL_SCALE will be different for 16-bit. And differential?
                self.vdda_calibrated / 4_096. * reading as f32
            }

            /// Start a conversion: Either a single measurement, or continuous conversions.
            /// See L4 RM 16.4.15 for details.
            pub fn start_conversion(&mut self, sequence: &[u8], mode: OperationMode) {
                // Set continuous or one-shot mode.
                self.regs.cfgr.modify(|_, w| w.cont().bit(mode as u8 != 0));
                // todo: You should call this elsewhere, once, to prevent unneded reg writes.
                for (i, channel) in sequence.iter().enumerate() {
                    self.set_sequence(*channel, i as u8 + 1); // + 1, since sequences start at 1.
                }

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
                // todo: read_volatile to be more efficient due to only reading 16 bits?
                self.regs.dr.read().bits() as u16
            }

            /// Take a single reading, in OneShot mode
            pub fn read(&mut self, channel: u8) -> u16 {
                self.start_conversion(&[channel], OperationMode::OneShot);
                self.read_result()
            }

            // todo: fn read_voltage, using vrefint and L4xx-hal style calibration?

            #[cfg(not(any(feature = "g0", feature = "h7", feature = "f4", feature = "l5")))]
            /// Take a one shot reading, using DMA. See L44 RM, 16.4.27: "DMA one shot mode".
            /// Note that the `channel` argument is only used on F3 and L4.
            pub unsafe fn read_dma<D>(&mut self, buf: &mut [u16], adc_channel: u8, dma_channel: DmaChannel, dma: &mut Dma<D>)
            where
                D: Deref<Target = dma_p::RegisterBlock>,
            {

                let (ptr, len) = (buf.as_mut_ptr(), buf.len());
                // The software is allowed to write (dmaen and dmacfg) only when ADSTART=0 and JADSTART=0 (which
                // ensures that no conversion is ongoing)
                // Todo: Should these settings be handled in `init`?
                self.stop_conversions();
                self.regs.cfgr.modify(|_, w| {
                    w.dmacfg().clear_bit(); // (one shot mode)  todo: Circular mode option; dmacfg=1.
                    w.dmaen().set_bit()
                });

                // L44 RM, Table 41. "DMA1 requests for each channel
                // todo: DMA2 support.
                #[cfg(any(feature = "f3", feature = "l4"))]
                let channel = match self.device {
                    AdcDevice::One => DmaInput::Adc1.dma1_channel(),
                    AdcDevice::Two => DmaInput::Adc2.dma1_channel(),
                    _ => panic!("DMA on ADC beyond 2 is not supported. If it is for your MCU, please submit an issue \
                or PR on Github.")
                };

                #[cfg(feature = "l4")]
                match self.device {
                    AdcDevice::One => dma.channel_select(DmaInput::Adc1),
                    AdcDevice::Two => dma.channel_select(DmaInput::Adc2),
                    _ => unimplemented!(),
                }

                self.set_sequence(adc_channel, 1);
                // todo: Support sequences.

                self.regs.cr.modify(|_, w| w.adstart().set_bit());  // Start

                // Since converted channel values are stored into a unique data register, it is useful to use
                // DMA for conversion of more than one channel. This avoids the loss of the data already
                // stored in the ADC_DR register.
                // When the DMA mode is enabled (DMAEN bit set to 1 in the ADC_CFGR register in single
                // ADC mode or MDMA different from 0b00 in dual ADC mode), a DMA request is generated
                // after each conversion of a channel. This allows the transfer of the converted data from the
                // ADC_DR register to the destination location selected by the software.
                // Despite this, if an overrun occurs (OVR=1) because the DMA could not serve the DMA
                // transfer request in time, the ADC stops generating DMA requests and the data
                // corresponding to the new conversion is not transferred by the DMA. Which means that all
                // the data transferred to the RAM can be considered as valid.
                // Depending on the configuration of OVRMOD bit, the data is either preserved or overwritten
                // (refer to Section : ADC overrun (OVR, OVRMOD)).
                // The DMA transfer requests are blocked until the software clears the OVR bit.
                // Two different DMA modes are proposed depending on the application use and are
                // configured with bit DMACFG of the ADC_CFGR register in single ADC mode, or with bit
                // DMACFG of the ADC_CCR register in dual ADC mode:
                // • DMA one shot mode (DMACFG=0).
                // This mode is suitable when the DMA is programmed to transfer a fixed number of data.
                // • DMA circular mode (DMACFG=1)
                // This mode is suitable when programming the DMA in circular mode.


                // In [DMA one shot mode], the ADC generates a DMA transfer request each time a new conversion data
                // is available and stops generating DMA requests once the DMA has reached the last DMA
                // transfer (when DMA_EOT interrupt occurs - refer to DMA paragraph) even if a conversion
                // has been started again.
                // When the DMA transfer is complete (all the transfers configured in the DMA controller have
                // been done):
                // • The content of the ADC data register is frozen.
                // • Any ongoing conversion is aborted with partial result discarded.
                // • No new DMA request is issued to the DMA controller. This avoids generating an
                // overrun error if there are still conversions which are started.
                // • Scan sequence is stopped and reset.
                // • The DMA is stopped.

                dma.cfg_channel(
                    dma_channel,
                    &self.regs.dr as *const _ as u32,
                    ptr as u32,
                    len as u16,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S16,
                    dma::DataSize::S16,
                    Default::default(),
                );
            }

            /// Enable a specific type of ADC interrupt.
            pub fn enable_interrupt(&mut self, interrupt: AdcInterrupt) {
                self.regs.ier.modify(|_, w| match interrupt {
                    AdcInterrupt::Ready => w.adrdyie().set_bit(),
                    AdcInterrupt::EndOfConversion => w.eocie().set_bit(),
                    AdcInterrupt::EnfOfSequence => w.eosie().set_bit(),
                    AdcInterrupt::EndofConversionInjected => w.jeocie().set_bit(),
                    AdcInterrupt::EndOfSequenceInjected => w.jeosie().set_bit(),
                    AdcInterrupt::Watchdog1 => w.awd1ie().set_bit(),
                    AdcInterrupt::Watchdog2 => w.awd2ie().set_bit(),
                    AdcInterrupt::Watchdog3 => w.awd3ie().set_bit(),
                    AdcInterrupt::EndOfSamplingPhase => w.eosmpie().set_bit(),
                    AdcInterrupt::Overrun => w.ovrie().set_bit(),
                    AdcInterrupt::InjectedOverflow => w.jqovfie().set_bit(),
                });
            }

            /// Clear an interrupt flag of the specified type. Consider running this in the
            /// corresponding ISR.
            pub fn clear_interrupt(&mut self, interrupt: AdcInterrupt) {
                self.regs.isr.modify(|_, w| match interrupt {
                    AdcInterrupt::Ready =>  w.adrdy().set_bit(),
                    AdcInterrupt::EndOfConversion =>  w.eoc().set_bit(),
                    AdcInterrupt::EnfOfSequence =>  w.eos().set_bit(),
                    AdcInterrupt::EndofConversionInjected =>  w.jeoc().set_bit(),
                    AdcInterrupt::EndOfSequenceInjected =>  w.jeos().set_bit(),
                    AdcInterrupt::Watchdog1 =>  w.awd1().set_bit(),
                    AdcInterrupt::Watchdog2 =>  w.awd2().set_bit(),
                    AdcInterrupt::Watchdog3 =>  w.awd3().set_bit(),
                    AdcInterrupt::EndOfSamplingPhase =>  w.eosmp().set_bit(),
                    AdcInterrupt::Overrun =>  w.ovr().set_bit(),
                    AdcInterrupt::InjectedOverflow =>  w.jqovf().set_bit(),
                });
                // match interrupt {
                //     AdcInterrupt::Ready => self.regs.icr.write(|_w| w.adrdy().set_bit()),
                //     AdcInterrupt::EndOfConversion => self.regs.icr.write(|w| w.eoc().set_bit()),
                //     AdcInterrupt::EnfOfSequence => self.regs.icr.write(|_w| w.eos().set_bit()),
                //     AdcInterrupt::EndofConversionInjected => self.regs.icr.write(|_w| w.jeoc().set_bit()),
                //     AdcInterrupt::EndOfSequenceInjected => self.regs.icr.write(|_w| w.jeos().set_bit()),
                //     AdcInterrupt::Watchdog1 => self.regs.icr.write(|_w| w.awd1().set_bit()),
                //     AdcInterrupt::Watchdog2 => self.regs.icr.write(|_w| w.awd2().set_bit()),
                //     AdcInterrupt::Watchdog3 => self.regs.icr.write(|_w| w.awd3().set_bit()),
                //     AdcInterrupt::EndOfSamplingPhase => self.regs.icr.write(|_w| w.eosmp().set_bit()),
                //     AdcInterrupt::Overrun => self.regs.icr.write(|_w| w.ovr().set_bit()),
                //     AdcInterrupt::InjectedOverflow => self.regs.icr.write(|_w| w.jqovf().set_bit()),
                // }
            }
        }

        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl<WORD, PIN> OneShot<pac::$ADC, WORD, PIN> for Adc<pac::$ADC>
        where
            WORD: From<u16>,
            PIN: Channel<pac::$ADC, ID = u8>,
            {
                type Error = ();

                fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                    Ok(Self::read(self, PIN::channel()).into())
                }
        }

        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        // todo: This mess is due to how EH implements the OneShot trait.
        impl Channel<pac::$ADC> for AdcChannel::C1 {
            type ID = u8;
            fn channel() -> u8 { 1 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C2 {
            type ID = u8;
            fn channel() -> u8 { 2 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C3 {
            type ID = u8;
            fn channel() -> u8 { 3}
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C4 {
            type ID = u8;
            fn channel() -> u8 { 4 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C5 {
            type ID = u8;
            fn channel() -> u8 { 5 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C6 {
            type ID = u8;
            fn channel() -> u8 { 6 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C7 {
            type ID = u8;
            fn channel() -> u8 { 7 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C8 {
            type ID = u8;
            fn channel() -> u8 { 8 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C9 {
            type ID = u8;
            fn channel() -> u8 { 9 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C10 {
            type ID = u8;
            fn channel() -> u8 { 10 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C11 {
            type ID = u8;
            fn channel() -> u8 { 11 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C12 {
            type ID = u8;
            fn channel() -> u8 { 12 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C13 {
            type ID = u8;
            fn channel() -> u8 { 13 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C14 {
            type ID = u8;
            fn channel() -> u8 { 14 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C15 {
            type ID = u8;
            fn channel() -> u8 { 15 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C16 {
            type ID = u8;
            fn channel() -> u8 { 16 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C17 {
            type ID = u8;
            fn channel() -> u8 { 17 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C18 {
            type ID = u8;
            fn channel() -> u8 { 18 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C19 {
            type ID = u8;
            fn channel() -> u8 { 19 }
        }
        #[cfg(feature = "embedded-hal")]
        #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
        impl Channel<pac::$ADC> for AdcChannel::C20 {
            type ID = u8;
            fn channel() -> u8 { 20 }
        }
    }
}

#[cfg(any(feature = "f301", feature = "f302", feature = "f303",))]
hal!(ADC1, ADC1_2, adc1, 12);

#[cfg(any(feature = "f302", feature = "f303",))]
hal!(ADC2, ADC1_2, adc2, 12);

#[cfg(any(feature = "f303"))]
hal!(ADC3, ADC3_4, adc3, 34);

#[cfg(any(feature = "f303"))]
hal!(ADC4, ADC3_4, adc4, 34);

#[cfg(any(feature = "l4"))]
hal!(ADC1, ADC_COMMON, adc1, _);

#[cfg(any(
    feature = "l4x1",
    feature = "l4x2",
    feature = "l412",
    feature = "l4x5",
    feature = "l4x6",
))]
hal!(ADC2, ADC_COMMON, adc2, _);

#[cfg(any(feature = "l4x5", feature = "l4x6",))]
hal!(ADC3, ADC_COMMON, adc3, _);

// todo: ADC 1 vs 2 on L5? L5 supports up to 2 ADCs, so I'm not sure what's going on here.
#[cfg(any(feature = "l5"))]
hal!(ADC, ADC_COMMON, adc1, _);

// todo Implement ADC3 on H7. The issue is the enable / reset being on ahb4.
cfg_if! {
    if #[cfg(feature = "h7")] {
        hal!(ADC1, ADC12_COMMON, adc1, 12);
        hal!(ADC2, ADC12_COMMON, adc2, 12);
        // hal!(ADC3, ADC3_COMMON, adc3, 3);
    }
}

#[cfg(feature = "h7")]

cfg_if! {
    if #[cfg(feature = "g4")] {
        hal!(ADC1, ADC12_COMMON, adc1, 12);
        hal!(ADC2, ADC12_COMMON, adc2, 12);
    }
}

#[cfg(all(feature = "g4", not(any(feature = "g431", feature = "g441"))))]
hal!(ADC3, ADC345_COMMON, adc3, 345);

cfg_if! {
    if #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))] {
        hal!(ADC4, ADC345_COMMON, adc4, 345);
        hal!(ADC5, ADC345_COMMON, adc5, 345);
    }
}

// todo F4 as (depending on variant?) ADC 1, 2, 3
