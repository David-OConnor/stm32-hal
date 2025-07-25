//! Provides support for basic timer functionality. Includes initialization, interrupts,
//! and PWM features. Also supports capture compare, output compare, burst DMA, and getting the current uptime using
//! an overflowing wrapper. (In seconds, milliseconds, or microseconds)
//!
//! Low-power timers (LPTIM) and high-presolution timers (HRTIM) are not yet supported.

// todo: WB and WL should support pwm features

#[cfg(feature = "monotonic")]
use core;
#[cfg(not(any(
    feature = "f401",
    feature = "f410",
    feature = "f411",
    feature = "f413",
    feature = "g031",
    feature = "g041",
    feature = "g070",
    feature = "g030",
    feature = "wb",
    feature = "wl"
)))]
use core::ops::Deref;
use core::{
    sync::atomic::{AtomicU32, Ordering},
    time::Duration,
};

use cfg_if::cfg_if;
use num_traits::float::FloatCore; // To round floats.
use paste::paste;
#[cfg(feature = "monotonic")]
use rtic_monotonic::Monotonic;

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;
#[cfg(not(any(feature = "f4", feature = "l552")))]
use crate::dma::{self, ChannelCfg, DmaChannel};
#[cfg(feature = "c0")]
use crate::pac::DMA as DMA1;
#[cfg(not(feature = "c0"))]
use crate::pac::DMA1;
#[cfg(not(any(
    feature = "f401",
    feature = "f410",
    feature = "f411",
    feature = "f413",
    feature = "g031",
    feature = "g041",
    feature = "g070",
    feature = "g030",
    feature = "g051",
    feature = "c0",
    feature = "wb",
    feature = "wl"
)))]
use crate::util::RccPeriph;
// todo: LPTIM (low-power timers) and HRTIM (high-resolution timers). And Advanced control functionality
use crate::{
    clocks::Clocks,
    error::{Error, Result},
    instant::Instant,
    pac::{self, RCC},
    util::rcc_en_reset,
};

// This `TICK_OVERFLOW_COUNT` must be incremented in firmware in the timer's update interrupt.
pub static TICK_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

// todo: Low power timer enabling etc. eg on L4, RCC_APB1Enr1().LPTIM1EN

#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum TimerError {
    /// Used for when attempting to set a timer period that is out of range.
    ValueError,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// This bit-field selects the trigger input to be used to synchronize the counter.
/// Sets SMCR register, TS field.
pub enum InputTrigger {
    ///Internal Trigger 0 (ITR0)
    Internal0 = 0b00000,
    Internal1 = 0b00001,
    Internal2 = 0b00010,
    Internal3 = 0b00011,
    /// TI1 Edge Detector (TI1F_ED)
    Ti1Edge = 0b00100,
    FilteredTimerInput1 = 0b00101,
    FilteredTimerInput2 = 0b00110,
    ExternalTriggerInput = 0b00111,
    Internal4 = 0b01000,
    Internal5 = 0b01001,
    Internal6 = 0b01010,
    Internal7 = 0b01011,
    Internal8 = 0b01100,
    Internal9 = 0b01101,
    Internal10 = 0b01110,
    Internal11 = 0b01111,
    Internal12 = 0b10000,
    Internal13 = 0b10001,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// When external signals are selected the active edge of the trigger signal (TRGI) is linked to
/// the polarity selected on the external input (see Input Control register and Control Register
/// description. Sets SMCR register, SMS field.
pub enum InputSlaveMode {
    /// Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal
    /// clock
    Disabled = 0b0000,
    /// Encoder mode 1 - Counter counts up/down on TI1FP1 edge depending on TI2FP2
    /// level
    Encoder1 = 0b0001,
    /// Encoder mode 2 - Counter counts up/down on TI2FP2 edge depending on TI1FP1
    /// level.
    Encoder2 = 0b0010,
    /// Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges
    /// depending on the level of the other input.
    Encoder3 = 0b0011,
    /// Reset mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter
    /// and generates an update of the registers.
    Reset = 0b0100,
    /// Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The
    /// counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of
    /// the counter are controlled.
    Gated = 0b0101,
    /// Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not
    /// reset). Only the start of the counter is controlled.
    Trigger = 0b0110,
    /// External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    ExternalClock1 = 0b0111,
    /// Combined reset + trigger mode - Rising edge of the selected trigger input (TRGI)
    /// reinitializes the counter, generates an update of the registers and starts the counter.
    CombinedResetTrigger = 0b1000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// These bits allow selected information to be sent in master mode to slave timers for
/// synchronization (TRGO). Sets CR2 register, MMS field.
pub enum MasterModeSelection {
    /// Tthe UG bit from the TIMx_EGR register is used as trigger output (TRGO). If the
    /// reset is generated by the trigger input (slave mode controller configured in reset mode) then
    /// the signal on TRGO is delayed compared to the actual reset.
    Reset = 0b000,
    /// the Counter Enable signal CNT_EN is used as trigger output (TRGO). It is
    /// useful to start several timers at the same time or to control a window in which a slave timer is
    /// enable. The Counter Enable signal is generated by a logic AND between CEN control bit
    /// and the trigger input when configured in gated mode. When the Counter Enable signal is
    /// controlled by the trigger input, there is a delay on TRGO, except if the master/slave mode is
    /// selected (see the MSM bit description in TIMx_SMCR register).
    Enable = 0b001,
    /// The update event is selected as trigger output (TRGO). For instance a master
    /// timer can then be used as a prescaler for a slave timer.
    Update = 0b010,
    /// Compare Pulse - The trigger output send a positive pulse when the CC1IF flag is to be
    /// set (even if it was already high), as soon as a capture or a compare match occurred.
    /// (TRGO).
    ComparePulse = 0b011,
    /// OC1REF signal is used as trigger output (TRGO)
    Compare1 = 0b100,
    /// OC2REF signal is used as trigger output (TRGO)
    Compare2 = 0b101,
    /// OC3REF signal is used as trigger output (TRGO)
    Compare3 = 0b110,
    /// OC4REF signal is used as trigger output (TRGO)
    Compare4 = 0b111,
}

/// Timer interrupt
pub enum TimerInterrupt {
    /// Update interrupt can be used for a timeout. DIER UIE to set, ... to clear
    Update,
    /// Trigger. DIER TIE to set, ... to clear
    Trigger,
    /// Capture/Compare. CC1IE to set, ... to clear
    CaptureCompare1,
    /// Capture/Compare. CC2IE to set, ... to clear
    CaptureCompare2,
    /// Capture/Compare. CC3IE to set, ... to clear
    CaptureCompare3,
    /// Capture/Compare. CC4IE to set, ... to clear
    CaptureCompare4,
    /// Update DMA. DIER UDE to set, ... to clear
    UpdateDma,
    /// Drigger. TDE to set, ... to clear
    TriggerDma,
    /// Capture/Compare. CC1DE to set, ... to clear
    CaptureCompare1Dma,
    /// Capture/Compare. CC2DE to set, ... to clear
    CaptureCompare2Dma,
    /// Capture/Compare. CC3DE to set, ... to clear
    CaptureCompare3Dma,
    /// Capture/Compare. CC4DE to set, ... to clear
    CaptureCompare4Dma,
}

/// Output alignment. Sets `TIMx_CR1` register, `CMS` field.
#[derive(Clone, Copy)]
pub enum Alignment {
    /// Edge-aligned mode. The counter counts up or down depending on the direction bit
    /// (DIR).
    Edge = 0b00,
    /// Center-aligned mode 1. The counter counts up and down alternatively. Output compare
    /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
    /// only when the counter is counting down.
    Center1 = 0b01,
    /// Center-aligned mode 2. The counter counts up and down alternatively. Output compare
    /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
    /// only when the counter is counting up.
    Center2 = 0b10,
    /// Center-aligned mode 3. The counter counts up and down alternatively. Output compare
    /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
    /// both when the counter is counting up or down.
    Center3 = 0b11,
}

/// Timer channel
#[derive(Clone, Copy)]
pub enum TimChannel {
    C1,
    C2,
    C3,
    #[cfg(not(feature = "wl"))]
    C4,
}

/// Timer count direction. Defaults to `Up`.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CountDir {
    Up = 0,
    Down = 1,
}

/// Capture/Compare selection.
/// This field defines the direction of the channel (input/output) as well as the used input.
/// It affects the TIMx_CCMR1 register, CCxS fields. Note that the signifiders of the input sources
/// varies depending on the channel. For example, the one labeled `InputTi1` here is always the associated
/// channel, while `InputTi2` is 2 for ch1, 1 for ch2, 4 for ch3, and 3 for ch4.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CaptureCompare {
    Output = 0b00,
    InputTi1 = 0b01,
    InputTi2 = 0b10,
    InputTrc = 0b11,
}

/// Capture/Compare output polarity. Defaults to `ActiveHigh` in hardware. Sets TIMx_CCER register,
/// CCxP and CCXNP fields.
#[derive(Clone, Copy)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

impl Polarity {
    /// For use with `set_bit()`.
    fn bit(&self) -> bool {
        match self {
            Self::ActiveHigh => false,
            Self::ActiveLow => true,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// See F303 ref man, section 21.4.7. H745 RM, section 41.4.8. Sets TIMx_CCMR1 register, OC1M field.
/// These bits define the behavior of the output reference signal OC1REF from which OC1 and
/// OC1N are derived. OC1REF is active high whereas OC1 and OC1N active level depends
/// on CC1P and CC1NP bits.
pub enum OutputCompare {
    /// Frozen - The comparison between the output compare register TIMx_CCR1 and the
    /// counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing
    /// base).
    Frozen = 0b0000,
    /// Set channel 1 to active level on match. OC1REF signal is forced high when the
    /// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
    Active = 0b0001,
    /// Set channel 1 to inactive level on match. OC1REF signal is forced low when the
    /// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
    /// 0011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
    Inactive = 0b0010,
    /// tim_oc1ref toggles when TIMx_CNT=TIMx_CCR1.
    Toggle = 0b0011,
    /// Force inactive level - OC1REF is forced low.
    ForceInactive = 0b0100,
    /// Force active level - OC1REF is forced high.
    ForceActive = 0b0101,
    /// PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
    /// else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
    /// TIMx_CNT>TIMx_CCR1 else active (OC1REF=1).
    Pwm1 = 0b0110,
    /// PWM mode 2 - In upcounting, channel 1 is inactive as long as
    /// TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
    /// TIMx_CNT>TIMx_CCR1 else inactive.
    Pwm2 = 0b0111,
    /// Retriggerable OPM mode 1 - In up-counting mode, the channel is active until a trigger
    /// event is detected (on TRGI signal). Then, a comparison is performed as in PWM mode 1
    /// and the channels becomes inactive again at the next update. In down-counting mode, the
    /// channel is inactive until a trigger event is detected (on TRGI signal). Then, a comparison is
    /// performed as in PWM mode 1 and the channels becomes inactive again at the next update.
    RetriggerableOpmMode1 = 0b1000,
    /// Retriggerable OPM mode 2 - In up-counting mode, the channel is inactive until a
    /// trigger event is detected (on TRGI signal). Then, a comparison is performed as in PWM
    /// mode 2 and the channels becomes inactive again at the next update. In down-counting
    /// mode, the channel is active until a trigger event is detected (on TRGI signal). Then, a
    /// comparison is performed as in PWM mode 1 and the channels becomes active again at the
    /// next update.
    RetriggerableOpmMode2 = 0b1001,
    /// Combined PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
    /// OC1REFC is the logical OR between OC1REF and OC2REF.
    CombinedPwm1 = 0b1100,
    /// Combined PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
    /// OC1REFC is the logical AND between OC1REF and OC2REF.
    CombinedPwm2 = 0b1101,
    /// Asymmetric PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
    /// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
    /// down.
    AsymmetricPwm1 = 0b1110,
    /// Asymmetric PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
    /// /// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
    /// down
    AsymmetricPwm2 = 0b1111,
}

/// Update Request source. This bit is set and cleared by software to select the UEV event sources.
/// Sets `TIMx_CR1` register, `URS` field.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UpdateReqSrc {
    /// Any of the following events generate an update interrupt or DMA request.
    /// These events can be:
    /// – Counter overflow/underflow
    /// – Setting the UG bit
    /// – Update generation through the slave mode controller
    Any = 0,
    /// Only counter overflow/underflow generates an update interrupt or DMA request.
    OverUnderFlow = 1,
}

/// Capture/Compaer DMA selection.
/// Sets `TIMx_CR2` register, `CCDS` field.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CaptureCompareDma {
    /// CCx DMA request sent when CCx event occur
    Ccx = 0,
    /// CCx DMA request sent when update event occurs
    Update = 1,
}

/// Initial configuration data for Timer peripherals.
#[derive(Clone)]
pub struct TimerConfig {
    /// If `one_pulse_mode` is true, the counter stops counting at the next update event
    /// (clearing the bit CEN). If false, Counter is not stopped at update event. Defaults to false.
    /// Sets `TIMx_CR` register, `OPM` field.
    pub one_pulse_mode: bool,
    /// Update request source. Ie, counter overflow/underflow only, or any. defaults to any.
    pub update_request_source: UpdateReqSrc,
    /// Set `true` to buffer the preload. Useful when changing period and duty while the timer is running.
    /// Default to false.
    pub auto_reload_preload: bool,
    /// Select center or edge alignment. Defaults to edge.
    pub alignment: Alignment,
    /// Sets when CCx DMA requests occur. Defaults to on CCx event.
    pub capture_compare_dma: CaptureCompareDma,
    /// Timer counting direction. Defaults to up.
    pub direction: CountDir,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            one_pulse_mode: false,
            update_request_source: UpdateReqSrc::Any,
            auto_reload_preload: false,
            alignment: Alignment::Edge,
            capture_compare_dma: CaptureCompareDma::Ccx,
            direction: CountDir::Up,
        }
    }
}

/// Represents a General Purpose or Advanced Control timer.
pub struct Timer<TIM> {
    /// Register block for the specific timer.
    pub regs: TIM,
    /// Our config stucture, for configuration that is written to the timer hardware on initialization
    /// via the constructor.
    pub cfg: TimerConfig,
    /// Associated timer clock speed in Hz.
    clock_speed: u32,
    // #[cfg(feature = "monotonic")]
    // /// Used to indicate the timer has expired, and running time counts (eg the `time_elapsed()` method) properly
    // /// increment.
    // pub wrap_count: u32,
    // #[cfg(feature = "monotonic")]
    /// Updated in the constructor and `set_freq` fns. Used for mapping timer ticks to time (eg in
    /// seconds, us etc)
    ns_per_tick: u64,
    period: f32, // In seconds. Used for overflow tracking. Updated when `ns_per_tick` is.
}

macro_rules! make_timer {
    ($TIMX:ident, $tim:ident, $apb:expr, $res:ident) => {
        impl Timer<pac::$TIMX> {
            paste! {
                /// Initialize a DFSDM peripheral, including  enabling and resetting
                /// its RCC peripheral clock.
                pub fn [<new_ $tim>](regs: pac::$TIMX, freq: f32, cfg: TimerConfig, clocks: &Clocks) -> Self {
                    let rcc = unsafe { &(*RCC::ptr()) };

                    // `freq` is in Hz.
                    rcc_en_reset!([<apb $apb>], $tim, rcc);

                    #[cfg(not(feature = "c0"))]
                    let clock_speed = clocks.[<apb $apb _timer>]();
                    #[cfg(feature = "c0")]
                    let clock_speed = clocks.apb1_timer(); // C0 only has one APB

                    regs.cr1().modify(|_, w| {
                        #[cfg(not(feature = "f373"))]
                        w.opm().bit(cfg.one_pulse_mode);
                        w.urs().bit(cfg.update_request_source as u8 != 0);
                        w.arpe().bit(cfg.auto_reload_preload)
                    });

                    #[cfg(not(any(feature = "f373", feature = "c0")))]
                    regs.cr2().modify(|_, w| {
                        w.ccds().bit(cfg.capture_compare_dma as u8 != 0)
                    });

                    let mut result = Timer {
                        clock_speed,
                        cfg,
                        regs,
                        // #[cfg(feature = "monotonic")]
                        // wrap_count: 0,
                        // #[cfg(feature = "monotonic")]
                        ns_per_tick: 0, // set below
                        period: 0., // set below.
                    };

                    result.set_freq(freq).ok();
                    #[cfg(not(feature = "c0"))]
                    result.set_dir();

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register. This also clears the interrupt flag,
                    // which the EGER update will generate.
                    result.reinitialize();

                    result
                }
            }

            /// Enable a specific type of Timer interrupt.
            pub fn enable_interrupt(&mut self, interrupt: TimerInterrupt) {
                match interrupt {
                    TimerInterrupt::Update => self.regs.dier().modify(|_, w| w.uie().bit(true)),
                    // todo: Only DIER is in PAC, or some CCs. PAC BUG? Only avail on some timers/MCUs?
                    // TimerInterrupt::Trigger => self.regs.dier().modify(|_, w| w.tie().bit(true)),
                    // TimerInterrupt::CaptureCompare1 => self.regs.dier().modify(|_, w| w.cc1ie().bit(true)),
                    // TimerInterrupt::CaptureCompare2 => self.regs.dier().modify(|_, w| w.cc2ie().bit(true)),
                    // TimerInterrupt::CaptureCompare3 => self.regs.dier().modify(|_, w| w.cc3ie().bit(true)),
                    // TimerInterrupt::CaptureCompare4 => self.regs.dier().modify(|_, w| w.cc4ie().bit(true)),
                    #[cfg(not(feature = "f3"))] // todo: Not working on some variants
                    TimerInterrupt::UpdateDma => self.regs.dier().modify(|_, w| w.ude().bit(true)),
                    // TimerInterrupt::TriggerDma => self.regs.dier().modify(|_, w| w.tde().bit(true)),
                    // TimerInterrupt::CaptureCompare1Dma => self.regs.dier().modify(|_, w| w.cc1de().bit(true)),
                    // TimerInterrupt::CaptureCompare2Dma => self.regs.dier().modify(|_, w| w.ccd2de().bit(true)),
                    // TimerInterrupt::CaptureCompare3Dma => self.regs.dier().modify(|_, w| w.cc3de().bit(true)),
                    // TimerInterrupt::CaptureCompare4Dma => self.regs.dier().modify(|_, w| w.cc4de().bit(true)),
                    _ => unimplemented!("TODO TEMP PROBLEMS"),
                };
            }

            /// Disable a specific type of Timer interrupt.
            pub fn disable_interrupt(&mut self, interrupt: TimerInterrupt) {
                match interrupt {
                    TimerInterrupt::Update => self.regs.dier().modify(|_, w| w.uie().clear_bit()),
                    // todo: Only DIER is in PAC, or some CCs. PAC BUG? Only avail on some timers/MCUs?
                    // TimerInterrupt::Trigger => self.regs.dier().modify(|_, w| w.tie().clear_bit()),
                    // TimerInterrupt::CaptureCompare1 => self.regs.dier().modify(|_, w| w.cc1ie().clear_bit()),
                    // TimerInterrupt::CaptureCompare2 => self.regs.dier().modify(|_, w| w.cc2ie().clear_bit()),
                    // TimerInterrupt::CaptureCompare3 => self.regs.dier().modify(|_, w| w.cc3ie().clear_bit()),
                    // TimerInterrupt::CaptureCompare4 => self.regs.dier().modify(|_, w| w.cc4ie().clear_bit()),
                    #[cfg(not(feature = "f3"))] // todo: Not working on some variants
                    TimerInterrupt::UpdateDma => self.regs.dier().modify(|_, w| w.ude().clear_bit()),
                    // TimerInterrupt::TriggerDma => self.regs.dier().modify(|_, w| w.tde().clear_bit()),
                    // TimerInterrupt::CaptureCompare1Dma => self.regs.dier().modify(|_, w| w.cc1de().clear_bit()),
                    // TimerInterrupt::CaptureCompare2Dma => self.regs.dier().modify(|_, w| w.ccd2de().clear_bit()),
                    // TimerInterrupt::CaptureCompare3Dma => self.regs.dier().modify(|_, w| w.cc3de().clear_bit()),
                    // TimerInterrupt::CaptureCompare4Dma => self.regs.dier().modify(|_, w| w.cc4de().clear_bit()),
                    _ => unimplemented!("TODO TEMP PROBLEMS"),
                };
            }

            /// Clears interrupt associated with this timer.
            ///
            /// If the interrupt is not cleared, it will immediately retrigger after
            /// the ISR has finished. For examlpe, place this at the top of your timer's
            /// interrupt handler.
            pub fn clear_interrupt(&mut self, interrupt: TimerInterrupt) {
                // Note that unlike other clear interrupt functions, for this, we clear the bit instead
                // of setting it. Due to the way our SVDs are set up not working well with this atomic clear,
                // we need to make sure we write 1s to the rest of the bits.
                // todo: Overcapture flags for each CC? DMA interrupts?
                #[cfg(feature = "c0")]
                let bits = 0xffff;
                #[cfg(not(feature = "c0"))]
                let bits = 0xffff_ffff;

                unsafe {
                    match interrupt {
                        TimerInterrupt::Update => self
                            .regs
                            .sr()
                            .write(|w| w.bits(bits).uif().clear_bit()),
                        // todo: Only DIER is in PAC, or some CCs. PAC BUG? Only avail on some timers?
                        // TimerInterrupt::Trigger => self.regs.sr().write(|w| w.bits(bits).tif().clear_bit()),
                        // TimerInterrupt::CaptureCompare1 => self.regs.sr().write(|w| w.bits(bits).cc1if().clear_bit()),
                        // TimerInterrupt::CaptureCompare2 => self.regs.sr().write(|w| w.bits(bits).cc2if().clear_bit()),
                        // TimerInterrupt::CaptureCompare3 => self.regs.sr().write(|w| w.bits(bits).cc3if().clear_bit()),
                        // TimerInterrupt::CaptureCompare4 => self.regs.sr().write(|w| w.bits(bits).cc4if().clear_bit()),
                        _ => unimplemented!(
                            "Clearing DMA flags is unimplemented using this function."
                        ),
                    };
                }
            }

            /// Enable (start) the timer.
            pub fn enable(&mut self) {
                self.regs.cr1().modify(|_,w| w.cen().bit(true));
            }

            /// Disable (stop) the timer.
            pub fn disable(&mut self) {
                self.regs.cr1().modify(|_, w| w.cen().clear_bit());
            }

            /// Check if the timer is enabled.
            pub fn is_enabled(&self) -> bool {
                self.regs.cr1().read().cen().bit_is_set()
            }

            #[cfg(not(feature = "c0"))]
            /// Print the (raw) contents of the status register.
            pub fn read_status(&self) -> u32 {
                unsafe { self.regs.sr().read().bits() }
            }

            #[cfg(feature = "c0")]
            /// Print the (raw) contents of the status register.
            pub fn read_status(&self) -> u16 {
                unsafe { self.regs.sr().read().bits().try_into().unwrap() }
            }

            /// Set the timer frequency, in Hz. Overrides the period or frequency set
            /// in the constructor.
            pub fn set_freq(&mut self, mut freq: f32) -> Result<()> {
                assert!(freq > 0.);
                // todo: Take into account the `timxsw` bit in RCC CFGR3, which may also
                // todo require an adjustment to freq.
                match self.cfg.alignment {
                    Alignment::Edge => (),
                    _ => freq *= 2.,
                }

                let (psc, arr) = calc_freq_vals(freq, self.clock_speed)?;

                self.regs.arr().write(|w| unsafe { w.bits(arr.into()) });
                self.regs.psc().write(|w| unsafe { w.bits(psc.into()) });

                // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
                // period = (PSC+1)*(ARR+1) / TIMclk
                // Calculate this based on our actual ARR and PSC values; don't use
                // the requested frequency or period.
                let arr_f32 = arr as f32;
                let period_secs = (psc as f32 + 1.) * ( arr_f32 + 1.) / self.clock_speed as f32;
                self.ns_per_tick = (period_secs / arr_f32 * 1_000_000_000.) as u64;
                self.period = period_secs;

                Ok(())
            }

            /// Set the timer period, in seconds. Overrides the period or frequency set
            /// in the constructor.
            pub fn set_period(&mut self, period: f32) -> Result<()> {
                assert!(period > 0.);
                self.set_freq(1. / period)
            }

            /// Set the auto-reload register value. Used for adjusting frequency.
            pub fn set_auto_reload(&mut self, arr: u32) {
                // todo: Could be u16 or u32 depending on timer resolution,
                // todo but this works for now.
                #[cfg(not(feature = "c0"))]
                self.regs.arr().write(|w| unsafe { w.bits(arr.into()) });
                #[cfg(feature = "c0")]
                self.regs.arr().write(|w| unsafe { w.bits((arr as u32).try_into().unwrap()) });
            }

            /// Set the prescaler value. Used for adjusting frequency.
            pub fn set_prescaler(&mut self, psc: u16) {
                self.regs.psc().write(|w| unsafe { w.bits(psc.into()) });
            }

            /// Reset the countdown; set the counter to 0.
            pub fn reset_count(&mut self) {
                self.regs.cnt().write(|w| unsafe { w.bits(0) });
            }

            /// UIF Flag in SR register is set when CNT reg is overflow / underflow
            ///
            pub fn get_uif(&self ) -> bool {
                self.regs.sr().read().uif().bit_is_set()
            }

            pub fn clear_uif(&mut self) {
                #[cfg(feature = "c0")]
                let bits = 0xffff;
                #[cfg(not(feature = "c0"))]
                let bits = 0xffff_ffff;

                unsafe {
                    self.regs
                        .sr()
                        .write(|w| w.bits(bits).uif().clear_bit());
                }
            }

            /// Re-initialize the counter and generates an update of the registers. Note that the prescaler
            /// counter is cleared too (anyway the prescaler ratio is not affected). The counter is cleared.
            /// When changing timer frequency (or period) via PSC, you may need to run this. Alternatively, change
            /// the freq in an update ISR.
            /// Note from RM, PSC reg: PSC contains the value to be loaded in the active prescaler
            /// register at each update event
            /// (including when the counter is cleared through UG bit of TIMx_EGR register or through
            /// trigger controller when configured in “reset mode”).'
            /// If you're doing something where the updates can wait a cycle, this isn't required. (eg PWM
            /// with changing duty period).
            pub fn reinitialize(&mut self) {
                self.regs.egr().write(|w| w.ug().bit(true));
                self.clear_interrupt(TimerInterrupt::Update);
            }

            /// Read the current counter value.
            pub fn read_count(&self) -> u32 {
                // todo: This depends on resolution. We read the whole
                // todo res and pass a u32 just in case.
                // self.regs.cnt().read().cnt().bits()
                self.regs.cnt().read().bits()
            }


            /// Enables PWM output for a given channel and output compare, with an initial duty cycle, in Hz.
            pub fn enable_pwm_output(
                &mut self,
                channel: TimChannel,
                compare: OutputCompare,
                duty: f32,
            ) {
                self.set_capture_compare_output(channel, CaptureCompare::Output);
                self.set_preload(channel, true);
                self.set_output_compare(channel, compare);
                self.set_duty(channel, (self.get_max_duty() as f32 * duty) as $res);
                self.enable_capture_compare(channel);
            }

            /// Return the integer associated with the maximum duty period.
            pub fn get_max_duty(&self) -> $res {
                #[cfg(feature = "g0")]
                return self.regs.arr().read().bits().try_into().unwrap();
                #[cfg(not(feature = "g0"))]
                self.regs.arr().read().arr().bits().try_into().unwrap()
            }

             /// See G4 RM, section 29.4.24: Dma burst mode. "The TIMx timers have the capability to
             /// generate multiple DMA requests upon a single event.
             /// The main purpose is to be able to re-program part of the timer multiple times without
             /// software overhead, but it can also be used to read several registers in a row, at regular
             /// intervals." This may be used to create arbitrary waveforms by modifying the CCR register
             /// (base address = 13-16, for CCR1-4), or for implementing duty-cycle based digital protocols.
            #[cfg(not(any(feature = "g0", feature = "f", feature = "l552", feature = "l4")))]
            pub unsafe fn write_dma_burst(
                &mut self,
                buf: &[u16],
                base_address: u8,
                burst_len: u8,
                dma_channel: DmaChannel,
                channel_cfg: ChannelCfg,
                ds_32_bits: bool,
                dma_periph: dma::DmaPeriph,
            ) -> Result<()> {
                // Note: F3 and L4 are unsupported here, since I'm not sure how to select teh
                // correct Timer channel.

                // todo: Should we disable the timer here?

                let (ptr, len) = (buf.as_ptr(), buf.len());

                // todo: For F3 and L4, manually set channel using PAC for now. Currently
                // todo we don't have a way here to pick the timer. Could do it with a new macro arg.

                // L44 RM, Table 41. "DMA1 requests for each channel"
                // #[cfg(any(feature = "f3", feature = "l4"))]
                // let dma_channel = match tim_channel {
                //     // SaiChannel::A => DmaInput::Sai1A.dma1_channel(),
                // };
                //
                // #[cfg(feature = "l4")]
                // match tim_channel {
                //     SaiChannel::B => dma.channel_select(DmaInput::Sai1B),
                // };

                // RM:
                // This example is for the case where every CCRx register has to be updated once. If every
                // CCRx register is to be updated twice for example, the number of data to transfer should be
                // 6. Let's take the example of a buffer in the RAM containing data1, data2, data3, data4, data5
                // and data6. The data is transferred to the CCRx registers as follows: on the first update DMA
                // request, data1 is transferred to CCR2, data2 is transferred to CCR3, data3 is transferred to
                // CCR4 and on the second update DMA request, data4 is transferred to CCR2, data5 is
                // transferred to CCR3 and data6 is transferred to CCR4.

                // 1. Configure the corresponding DMA channel as follows:
                // –DMA channel peripheral address is the DMAR register address
                let periph_addr = unsafe { &self.regs.dmar() as *const _ as u32 };
                // –DMA channel memory address is the address of the buffer in the RAM containing
                // the data to be transferred by DMA into CCRx registers.

                // Number of data to transfer is our buffer len number of registers we're editing, x
                // number of half-words written to each reg.
                let num_data = len as u32;

                // 2.
                // Configure the DCR register by configuring the DBA and DBL bit fields as follows:
                // DBL = 3 transfers, DBA = 0xE.

                // The DBL[4:0] bits in the TIMx_DCR register set the DMA burst length. The timer recognizes
                // a burst transfer when a read or a write access is done to the TIMx_DMAR address), i.e. the
                // number of transfers (either in half-words or in bytes).
                // The DBA[4:0] bits in the TIMx_DCR registers define the DMA base address for DMA
                // transfers (when read/write access are done through the TIMx_DMAR address). DBA is
                // defined as an offset starting from the address of the TIMx_CR1 register:
                // Example:
                // 00000: TIMx_CR1
                // 00001: TIMx_CR2
                // 00010: TIMx_SMCR
                self.regs.dcr().modify(|_, w| unsafe {
                    w.dba().bits(base_address);
                    w.dbl().bits(burst_len as u8 - 1)
                });

                // 3. Enable the TIMx update DMA request (set the UDE bit in the DIER register).
                // note: Leaving this to application code for now.
                // self.enable_interrupt(TimerInterrupt::UpdateDma);

                // 4. Enable TIMx
                self.enable();

                // 5. Enable the DMA channel
                match dma_periph {
                    dma::DmaPeriph::Dma1 => {
                        let mut regs = unsafe { &(*DMA1::ptr()) };
                        dma::cfg_channel(
                            &mut regs,
                            dma_channel,
                            periph_addr,
                            ptr as u32,
                            num_data,
                            dma::Direction::ReadFromMem,
                            // Note: This may only be relevant if modifying a reg that changes for 32-bit
                            // timers, like AAR and CCRx
                            if ds_32_bits { dma::DataSize::S32} else { dma::DataSize::S16 },
                            dma::DataSize::S16,
                            channel_cfg,
                        )?;
                    }
                    #[cfg(dma2)]
                    dma::DmaPeriph::Dma2 => {
                        let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                        dma::cfg_channel(
                            &mut regs,
                            dma_channel,
                            periph_addr,
                            ptr as u32,
                            num_data,
                            dma::Direction::ReadFromMem,
                            // Note: This may only be relevant if modifying a reg that changes for 32-bit
                            // timers, like AAR and CCRx
                            if ds_32_bits { dma::DataSize::S32} else { dma::DataSize::S16 },
                            dma::DataSize::S16,
                            channel_cfg,
                        )?;
                    }
                }
                Ok(())
            }

            #[cfg(not(any(feature = "g0", feature = "f", feature = "l552", feature = "l4")))]
            pub unsafe fn read_dma_burst(
                // todo: Experimenting with input capture.
                &mut self,
                buf: &mut [u16],
                base_address: u8,
                burst_len: u8,
                dma_channel: DmaChannel,
                channel_cfg: ChannelCfg,
                ds_32_bits: bool,
                dma_periph: dma::DmaPeriph,
            ) -> Result<()> {
                let (ptr, len) = (buf.as_mut_ptr(), buf.len());

                let periph_addr = unsafe { &self.regs.dmar() as *const _ as u32 };

                let num_data = len as u32;

                self.regs.dcr().modify(|_, w| unsafe {
                    w.dba().bits(base_address);
                    w.dbl().bits(burst_len as u8 - 1)
                });

                self.enable();

                match dma_periph {
                    dma::DmaPeriph::Dma1 => {
                        #[cfg(not(feature = "c0"))]
                        let mut regs = unsafe { &(*pac::DMA1::ptr()) };
                        #[cfg(feature = "c0")]
                        let mut regs = unsafe { &(*pac::DMA::ptr()) };

                        dma::cfg_channel(
                            &mut regs,
                            dma_channel,
                            periph_addr,
                            ptr as u32,
                            num_data,
                            dma::Direction::ReadFromPeriph,
                            // Note: This may only be relevant if modifying a reg that changes for 32-bit
                            // timers, like AAR and CCRx
                            if ds_32_bits { dma::DataSize::S32} else { dma::DataSize::S16 },
                            dma::DataSize::S16,
                            channel_cfg,
                        )?;
                    }
                    #[cfg(dma2)]
                    dma::DmaPeriph::Dma2 => {
                        let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                        dma::cfg_channel(
                            &mut regs,
                            dma_channel,
                            periph_addr,
                            ptr as u32,
                            num_data,
                            dma::Direction::ReadFromPeriph,
                            // Note: This may only be relevant if modifying a reg that changes for 32-bit
                            // timers, like AAR and CCRx
                            if ds_32_bits { dma::DataSize::S32} else { dma::DataSize::S16 },
                            dma::DataSize::S16,
                            channel_cfg,
                        )?;
                    }
                }
                Ok(())
            }

            /// Get the time elapsed since the start of the timer, taking overflow wraps into account.
            ///
            /// Important: the value returned here will only be correct if the ARR and PSC are set
            /// only using the constructor, `set_freq`, or `set_period` methods.
            pub fn now(&mut self) -> Instant {
                // let wrap_count = self.wrap_count;
                let wrap_count = TICK_OVERFLOW_COUNT.load(Ordering::Acquire);

                let ticks = (self.read_count() as u64 + wrap_count as u64 *
                    self.get_max_duty() as u64);
                let count_ns = ticks as i128 * self.ns_per_tick as i128;

                Instant::new(count_ns)
            }

            pub fn elapsed(&mut self, since: Instant) -> Duration {
                self.now() - since
            }
        }

        #[cfg(feature = "monotonic")]
        impl Monotonic for Timer<pac::$TIMX> {
            type Instant = Instant;
            type Duration = core::time::Duration;

            const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

            fn now(&mut self) -> Self::Instant {
                self.time_elapsed()
            }

            /// We use the compare 1 channel for this.
            /// todo: Support wrapping?
            fn set_compare(&mut self, instant: Self::Instant) {
                self.regs
                    .ccr1()
                    .write(|w| unsafe { w.ccr().bits(((instant.count_ns as f32 / self.ns_per_tick) as u16).into()) });
            }

            /// We use the compare 1 channel for this.
            /// todo: Support wrapping?
            fn clear_compare_flag(&mut self) {
                self.regs.sr().modify(|_, w| w.cc1if().clear_bit());
            }

            fn zero() -> Self::Instant {
                Instant::default()
            }

            unsafe fn reset(&mut self) {
                self.reset_count();
                TICK_OVERFLOW_COUNT.store(0, Ordering::Release);
            }

            fn on_interrupt(&mut self) {
                // self.wrap_count += 1;
                TICK_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            fn enable_timer(&mut self) {
                self.enable();
            }
            fn disable_timer(&mut self) {
                self.disable();
            }

            /// Print the (raw) contents of the status register.
            pub fn read_status(&self) -> u32 {
                unsafe { self.regs.isr().read().bits() }
            }
        }

        // cfg_if! {
        //     if #[cfg(feature = "embedded_hal")] {
        //         // use embedded_hal::{
        //         //     blocking::delay::{DelayMs, DelayUs},
        //         //     timer::CountDown,
        //         // };
        //         // use embedded_time::{rate::Hertz, duration};
        //         // use void::Void;
        //     }
        // }

        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayMs<u32> for Timer<pac::$TIMX> {
        //     fn delay_ms(&mut self, ms: u32) {
        //         let ms_to_s = ms as f32 / 1_000.;
        //         self.set_freq(1. / (ms_to_s)).ok();
        //         self.reset_count();
        //         self.enable();
        //         while self.get_uif() == false {}
        //         self.clear_uif();
        //         self.disable();
        //     }
        // }
        //
        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayMs<u16> for Timer<pac::$TIMX> {
        //     fn delay_ms(&mut self, ms: u16) {
        //         self.delay_ms(ms as u32)
        //     }
        // }
        //
        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayMs<u8> for Timer<pac::$TIMX> {
        //     fn delay_ms(&mut self, ms: u8) {
        //         self.delay_ms(ms as u32)
        //     }
        // }
        //
        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayUs<u32> for Timer<pac::$TIMX> {
        //     fn delay_us(&mut self, us: u32) {
        //         let us_to_s = us as f32 / 1_000_000.;
        //         self.set_freq(1. / (us_to_s)).ok();
        //         self.reset_count();
        //         self.enable();
        //         while self.get_uif() == false {}
        //         self.clear_uif();
        //         self.disable();
        //     }
        // }
        //
        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayUs<u16> for Timer<pac::$TIMX> {
        //     fn delay_us(&mut self, us: u16) {
        //         self.delay_us(us as u32);
        //     }
        // }
        //
        // #[cfg(feature = "embedded_hal")]
        // // #[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
        // impl DelayUs<u8> for Timer<pac::$TIMX> {
        //     fn delay_us(&mut self, us: u8) {
        //         self.delay_us(us as u32);
        //     }
        // }

        // /// Implementation of the embedded-hal CountDown trait
        // /// To use Countdown it is prefered to configure new timer in Oneshot mode :
        // ///
        // ///     Example :
        // ///     llet tim16_conf = TimerConfig {
        // ///             one_pulse_mode: true,
        // ///             ..Default::default()
        // ///         };
        // ///
        // ///     Creation of timer. Here the freq arg value is not important, the freq
        // ///         will be changed for each CountDown start timer depends on delay wanted.
        // ///
        // /// let mut tim16: Timer<TIM16> = Timer::new_tim16(dp.TIM16, 1000., tim16_conf, &clocks);
        // ///
        // ///
        // #[cfg(feature = "embedded_hal")]
        // impl CountDown for Timer<pac::$TIMX>
        // {
        //     type Time = duration::Generic<u32>;
        //
        //     fn start<T>(&mut self, timeout: T)
        //         where
        //             T: Into<Self::Time>,
        //     {
        //
        //         //Disable timer and clear flag uif
        //         self.disable();
        //         self.clear_uif();
        //
        //         //Get timeout
        //         let timeout: Self::Time = timeout.into();
        //
        //         let denom = timeout.scaling_factor().denominator();
        //
        //         //Prevent a division by zero
        //         if *denom != 0 {
        //             //Convert time to seconds
        //             let time_to_s = timeout.integer() as f32 / *denom as f32;
        //
        //             //Configure timer
        //             self.set_freq(1. / (time_to_s)).ok();
        //             self.reset_count();
        //             //Update event to setup prescale and reload registers
        //             self.reinitialize();
        //             self.clear_uif();
        //
        //             //start the timer
        //             self.enable();
        //         }
        //
        //     }
        //
        //     /// Wait until the timer has elapsed
        //     /// and than clear the event.
        //     fn wait(&mut self) -> nb::Result<(), Void> {
        //         if !self.get_uif() {
        //             Err(nb::Error::WouldBlock)
        //         } else {
        //             self.clear_uif();
        //             self.disable();
        //             Ok(())
        //         }
        //     }
        // }
    }
}

// We use macros to support the varying number of capture compare channels available on
// different timers.
// Note that there's lots of DRY between these implementations.
macro_rules! cc_4_channels {
    ($TIMX:ident, $res:ident) => {
        impl Timer<pac::$TIMX> {
            /// Function that allows us to set direction only on timers that have this option.
            pub fn set_dir(&mut self) {
                self.regs
                    .cr1()
                    .modify(|_, w| w.dir().bit(self.cfg.direction as u8 != 0));
                self.regs
                    .cr1()
                    .modify(|_, w| unsafe { w.cms().bits(self.cfg.alignment as u8) });
            }

            /// Set up input capture, eg for PWM input.
            /// L4 RM, section 26.3.8. H723 RM, section 43.3.7.
            /// Note: Does not handle TISEL (timer input selection register - you must do this manually
            /// using the PAC.
            #[cfg(not(any(
                feature = "f",
                feature = "l4x5",
                feature = "l5",
                feature = "g0",
                feature = "wb"
            )))]
            pub fn set_input_capture(
                &mut self,
                channel: TimChannel,
                mode: CaptureCompare,
                // trigger: InputTrigger,
                // slave_mode: InputSlaveMode,
                ccp: Polarity,
                ccnp: Polarity,
            ) {
                // 2. Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1
                // register.
                self.set_capture_compare_input(channel, mode);

                // 1. Select the proper TI1x source (internal or external) with the TI1SEL[3:0] bits in the
                // TIMx_TISEL register.
                // Leaving it at its default value of 0 selects the timer input, which we'll hard-code
                // for now.
                // let tisel = 0b0000;

                // 3.
                // Program the needed input filter duration in relation with the signal connected to the
                // timer (when the input is one of the tim_tix (ICxF bits in the TIMx_CCMRx register). Let’s
                // imagine that, when toggling, the input signal is not stable during at must 5 internal clock
                // cycles. We must program a filter duration longer than these 5 clock cycles. We can
                // validate a transition on tim_ti1 when 8 consecutive samples with the new level have
                // been detected (sampled at fDTS frequency). Then write IC1F bits to 0011 in the
                // TIMx_CCMR1 register.
                let filter = 0b0011; // todo experimenting.

                match channel {
                    TimChannel::C1 => {
                        // self.regs.tisel.modify(|_, w| unsafe { w.ti1sel().bits(tisel) });

                        // 3. Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter
                        // clear): write the CC1P and CC1NP bits to ‘0’ (active on rising edge).
                        // (Note: We could use the `set_polarity` and `set_complementary_polarity` methods, but
                        // this allows us to combine them in a single reg write.)
                        self.regs.ccer().modify(|_, w| {
                            w.cc1p().bit(ccp.bit());
                            w.cc1np().bit(ccnp.bit())
                        });

                        // 5.
                        // Program the input prescaler. In our example, we wish the capture to be performed at
                        // each valid transition, so the prescaler is disabled (write IC1PS bits to 00 in the
                        // TIMx_CCMR1 register).
                        self.regs.ccmr1_input().modify(|_, w| unsafe {
                            // todo: PAC ommission?
                            // w.ic1psc().bits(0b00);
                            w.ic1f().bits(filter)
                        });
                    }
                    TimChannel::C2 => {
                        // self.regs.tisel.modify(|_, w| unsafe { w.ti2sel().bits(tisel) });

                        self.regs.ccer().modify(|_, w| {
                            w.cc2p().bit(ccp.bit());
                            w.cc2np().bit(ccnp.bit())
                        });

                        self.regs.ccmr1_input().modify(|_, w| unsafe {
                            w.ic2psc().bits(0b00);
                            w.ic2f().bits(filter)
                        });
                    }
                    TimChannel::C3 => {
                        // self.regs.tisel.modify(|_, w| unsafe { w.ti3sel().bits(tisel) });

                        self.regs.ccer().modify(|_, w| {
                            w.cc3p().bit(ccp.bit());
                            w.cc3np().bit(ccnp.bit())
                        });

                        self.regs.ccmr2_input().modify(|_, w| unsafe {
                            w.ic3psc().bits(0b00);
                            w.ic3f().bits(filter)
                        });
                    }
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => {
                        // self.regs.tisel.modify(|_, w| unsafe { w.ti4sel().bits(tisel) });

                        self.regs.ccer().modify(|_, w| {
                            #[cfg(not(any(feature = "f4", feature = "l4")))]
                            w.cc4np().bit(ccnp.bit());
                            w.cc4p().bit(ccp.bit())
                        });

                        self.regs.ccmr2_input().modify(|_, w| unsafe {
                            w.ic4psc().bits(0b00);
                            w.ic4f().bits(filter)
                        });
                    }
                }

                // todo: SMCR: Set in the Input PWM settings, but not normal input capture (?)
                // 6. Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
                // (TI1FP1 selected).
                // self.regs.smcr().modify(|_, w| unsafe {
                //     w.ts().bits(trigger as u8);
                //     // 7. Configure the slave mode controller in reset mode: write the SMS bits to 0100 in the
                //     // TIMx_SMCR register.
                //     w.sms().bits(slave_mode as u8)
                // });

                // 6. Enable capture from the counter into the capture register by setting the CC1E bit in the
                // TIMx_CCER register.
                self.enable_capture_compare(channel);

                // 7.If needed, enable the related interrupt request by setting the CC1IE bit in the
                // TIMx_DIER register, and/or the DMA request by setting the CC1DE bit in the
                // TIMx_DIER register.
            }

            // todo: more advanced PWM modes. Asymmetric, combined, center-aligned etc.

            /// Set Output Compare Mode. See docs on the `OutputCompare` enum.
            pub fn set_output_compare(&mut self, channel: TimChannel, mode: OutputCompare) {
                match channel {
                    TimChannel::C1 => {
                        self.regs.ccmr1_output().modify(|_, w| unsafe {
                            #[cfg(not(any(feature = "f", feature = "l5", feature = "wb")))]
                            w.oc1m_3().bit((mode as u8) >> 3 != 0);
                            w.oc1m().bits((mode as u8) & 0b111)
                        });
                    }
                    TimChannel::C2 => {
                        self.regs.ccmr1_output().modify(|_, w| unsafe {
                            #[cfg(not(any(feature = "f", feature = "l5", feature = "wb")))]
                            w.oc2m_3().bit((mode as u8) >> 3 != 0);
                            w.oc2m().bits((mode as u8) & 0b111)
                        });
                    }
                    TimChannel::C3 => {
                        self.regs.ccmr2_output().modify(|_, w| unsafe {
                            #[cfg(not(any(feature = "f", feature = "l5", feature = "wb")))]
                            w.oc3m_3().bit((mode as u8) >> 3 != 0);
                            w.oc3m().bits((mode as u8) & 0b111)
                        });
                    }
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => {
                        self.regs.ccmr2_output().modify(|_, w| unsafe {
                            #[cfg(not(any(
                                feature = "f3",
                                feature = "f4",
                                feature = "l5",
                                feature = "wb",
                                feature = "h7"
                            )))]
                            w.oc4m_3().bit((mode as u8) >> 3 != 0);
                            w.oc4m().bits((mode as u8) & 0b111)
                        });
                    }
                }
            }

            /// Return the set duty period for a given channel. Divide by `get_max_duty()`
            /// to find the portion of the duty cycle used.
            pub fn get_duty(&self, channel: TimChannel) -> $res {
                cfg_if! {
                    if #[cfg(any(feature = "wb", feature = "wl"))] {
                        (match channel {
                            TimChannel::C1 => self.regs.ccr1().read().bits(),
                            TimChannel::C2 => self.regs.ccr2().read().bits(),
                            TimChannel::C3 => self.regs.ccr3().read().bits(),
                            #[cfg(not(feature = "wl"))]
                            TimChannel::C4 => self.regs.ccr4().read().bits(),
                        }) as $res
                    } else {
                        match channel {
                            TimChannel::C1 => self.regs.ccr1().read().ccr().bits().into(),
                            TimChannel::C2 => self.regs.ccr2().read().ccr().bits().into(),
                            TimChannel::C3 => self.regs.ccr3().read().ccr().bits().into(),
                            #[cfg(not(feature = "wl"))]
                            TimChannel::C4 => self.regs.ccr4().read().ccr().bits().into(),
                        }
                    }
                }
            }

            /// Set the duty cycle, as a portion of ARR (`get_max_duty()`). Note that this
            /// needs to be re-run if you change ARR at any point.
            pub fn set_duty(&mut self, channel: TimChannel, duty: $res) {
                unsafe {
                    match channel {
                        TimChannel::C1 => self
                            .regs
                            .ccr1()
                            .write(|w| w.ccr().bits(duty.try_into().unwrap())),
                        TimChannel::C2 => self
                            .regs
                            .ccr2()
                            .write(|w| w.ccr().bits(duty.try_into().unwrap())),
                        TimChannel::C3 => self
                            .regs
                            .ccr3()
                            .write(|w| w.ccr().bits(duty.try_into().unwrap())),
                        #[cfg(not(feature = "wl"))]
                        TimChannel::C4 => self
                            .regs
                            .ccr4()
                            .write(|w| w.ccr().bits(duty.try_into().unwrap())),
                    };
                }
            }

            /// Set timer alignment to Edge, or one of 3 center modes.
            /// STM32F303 ref man, section 21.4.1:
            /// Bits 6:5 CMS: Center-aligned mode selection
            /// 00: Edge-aligned mode. The counter counts up or down depending on the direction bit
            /// (DIR).
            /// 01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
            /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
            /// only when the counter is counting down.
            /// 10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
            /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
            /// only when the counter is counting up.
            /// 11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
            /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
            /// both when the counter is counting up or down.
            pub fn set_alignment(&mut self, alignment: Alignment) {
                self.regs
                    .cr1()
                    .modify(|_, w| unsafe { w.cms().bits(alignment as u8) });
                self.cfg.alignment = alignment;
            }

            /// Set output polarity. See docs on the `Polarity` enum.
            pub fn set_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1p().bit(polarity.bit())),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2p().bit(polarity.bit())),
                    TimChannel::C3 => self.regs.ccer().modify(|_, w| w.cc3p().bit(polarity.bit())),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self.regs.ccer().modify(|_, w| w.cc4p().bit(polarity.bit())),
                };
            }

            /// Set complementary output polarity. See docs on the `Polarity` enum.
            pub fn set_complementary_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccer()
                        .modify(|_, w| w.cc1np().bit(polarity.bit())),
                    TimChannel::C2 => self
                        .regs
                        .ccer()
                        .modify(|_, w| w.cc2np().bit(polarity.bit())),
                    TimChannel::C3 => self
                        .regs
                        .ccer()
                        .modify(|_, w| w.cc3np().bit(polarity.bit())),
                    #[cfg(not(any(feature = "f4", feature = "wl", feature = "l4")))]
                    TimChannel::C4 => self
                        .regs
                        .ccer()
                        .modify(|_, w| w.cc4np().bit(polarity.bit())),
                    #[cfg(any(feature = "f4", feature = "wl", feature = "l4"))] // PAC ommission
                    _ => panic!(),
                };
            }
            /// Disables capture compare on a specific channel.
            pub fn disable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().clear_bit()),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2e().clear_bit()),
                    TimChannel::C3 => self.regs.ccer().modify(|_, w| w.cc3e().clear_bit()),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self.regs.ccer().modify(|_, w| w.cc4e().clear_bit()),
                };
            }

            /// Enables capture compare on a specific channel.
            pub fn enable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().bit(true)),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2e().bit(true)),
                    TimChannel::C3 => self.regs.ccer().modify(|_, w| w.cc3e().bit(true)),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self.regs.ccer().modify(|_, w| w.cc4e().bit(true)),
                };
            }

            /// Set Capture Compare mode in output mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_output(
                &mut self,
                channel: TimChannel,
                mode: CaptureCompare,
            ) {
                // Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
                self.disable_capture_compare(channel);

                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    TimChannel::C2 => self
                        .regs
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc2s().bits(mode as u8) }),
                    TimChannel::C3 => self
                        .regs
                        .ccmr2_output()
                        .modify(unsafe { |_, w| w.cc3s().bits(mode as u8) }),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self
                        .regs
                        .ccmr2_output()
                        .modify(unsafe { |_, w| w.cc4s().bits(mode as u8) }),
                };
            }

            /// Set Capture Compare mode in input mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_input(&mut self, channel: TimChannel, mode: CaptureCompare) {
                // Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
                self.disable_capture_compare(channel);

                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_input()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    TimChannel::C2 => self
                        .regs
                        .ccmr1_input()
                        .modify(unsafe { |_, w| w.cc2s().bits(mode as u8) }),
                    TimChannel::C3 => self
                        .regs
                        .ccmr2_input()
                        .modify(unsafe { |_, w| w.cc3s().bits(mode as u8) }),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self
                        .regs
                        .ccmr2_input()
                        .modify(unsafe { |_, w| w.cc4s().bits(mode as u8) }),
                };
            }

            /// Set preload mode.
            /// OC1PE: Output Compare 1 preload enable
            /// 0: Preload register on TIMx_CCR1 disabled. TIMx_CCR1 can be written at anytime, the
            /// new value is taken in account immediately.
            /// 1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload
            /// register. TIMx_CCR1 preload value is loaded in the active register at each update event.
            /// Note: 1: These bits can not be modified as long as LOCK level 3 has been programmed
            /// (LOCK bits in TIMx_BDTR register) and CC1S=’00’ (the channel is configured in
            /// output).
            /// 2: The PWM mode can be used without validating the preload register only in one
            /// pulse mode (OPM bit set in TIMx_CR1 register). Else the behavior is not guaranteed.
            ///
            /// Setting preload is required to enable PWM.
            pub fn set_preload(&mut self, channel: TimChannel, value: bool) {
                match channel {
                    TimChannel::C1 => self.regs.ccmr1_output().modify(|_, w| w.oc1pe().bit(value)),
                    TimChannel::C2 => self.regs.ccmr1_output().modify(|_, w| w.oc2pe().bit(value)),
                    TimChannel::C3 => self.regs.ccmr2_output().modify(|_, w| w.oc3pe().bit(value)),
                    #[cfg(not(feature = "wl"))]
                    TimChannel::C4 => self.regs.ccmr2_output().modify(|_, w| w.oc4pe().bit(value)),
                };

                // "As the preload registers are transferred to the shadow registers only when an update event
                // occurs, before starting the counter, you have to initialize all the registers by setting the UG
                // bit in the TIMx_EGR register."
                self.reinitialize();
            }
        }
    };
}

#[cfg(any(feature = "g0", feature = "g4", feature = "c0"))]
macro_rules! cc_2_channels {
    ($TIMX:ident, $res:ident) => {
        impl Timer<pac::$TIMX> {
            /// Function that allows us to set direction only on timers that have this option.
            fn set_dir(&mut self) {
                // self.regs.cr1().modify(|_, w| w.dir().bit(self.cfg.direction as u8 != 0));
            }

            // todo: more advanced PWM modes. Asymmetric, combined, center-aligned etc.

            /// Set up input capture, eg for PWM input.
            /// L4 RM, section 26.3.8. H723 RM, section 43.3.7.
            /// Note: Does not handle TISEL (timer input selection register - you must do this manually
            /// using the PAC.
            #[cfg(not(any(feature = "f", feature = "l4x5", feature = "l5", feature = "g0")))]
            pub fn set_input_capture(
                &mut self,
                channel: TimChannel,
                mode: CaptureCompare,
                // trigger: InputTrigger,
                // slave_mode: InputSlaveMode,
                ccp: Polarity,
                ccnp: Polarity,
            ) {
                self.set_capture_compare_input(channel, mode);

                let filter = 0b00;

                match channel {
                    TimChannel::C1 => {
                        self.regs.ccer().modify(|_, w| {
                            w.cc1p().bit(ccp.bit());
                            w.cc1np().bit(ccnp.bit())
                        });

                        self.regs.ccmr1_input().modify(|_, w| unsafe {
                            // w.ic1psc().bits(0b00);
                            w.ic1f().bits(filter)
                        });
                    }
                    TimChannel::C2 => {
                        #[cfg(not(feature = "c0"))]
                        self.regs.ccer().modify(|_, w| {
                            w.cc2p().bit(ccp.bit());
                            w.cc2np().bit(ccnp.bit())
                        });

                        self.regs.ccmr1_input().modify(|_, w| unsafe {
                            w.ic2psc().bits(0b00);
                            w.ic2f().bits(filter)
                        });
                    }
                    _ => panic!()
                }

                // self.regs.smcr().modify(|_, w| unsafe {
                //     w.ts().bits(trigger as u8);
                //     w.sms().bits(slave_mode as u8)
                // });

                self.enable_capture_compare(channel);
            }

            /// Set Output Compare Mode. See docs on the `OutputCompare` enum.
            pub fn set_output_compare(&mut self, channel: TimChannel, mode: OutputCompare) {
                match channel {
                    TimChannel::C1 => {
                       self.regs.ccmr1_output().modify(|_, w| unsafe {
                        #[cfg(not(any(feature = "f4", feature = "l5", feature = "wb")))]
                           w.oc1m_3().bit((mode as u8) >> 3 != 0);
                           w.oc1m().bits((mode as u8) & 0b111)

                        });
                    }
                    TimChannel::C2 => {
                      self.regs.ccmr1_output().modify(|_, w| unsafe {
                        #[cfg(not(any(feature = "f4", feature = "l5", feature = "wb")))]
                          w.oc2m_3().bit((mode as u8) >> 3 != 0);
                          w.oc2m().bits((mode as u8) & 0b111)

                        });
                    }
                    _ => panic!()
                }
            }

            /// Return the set duty period for a given channel. Divide by `get_max_duty()`
            /// to find the portion of the duty cycle used.
            pub fn get_duty(&self, channel: TimChannel) -> $res {
                cfg_if! {
                    if #[cfg(feature = "g0")] {
                        match channel {
                            TimChannel::C1 => self.regs.ccr1().read().bits().try_into().unwrap(),
                            TimChannel::C2 => self.regs.ccr2().read().bits().try_into().unwrap(),
                            _ => panic!()
                        }
                    } else if #[cfg(any(feature = "wb", feature = "wl", feature = "l5"))] {
                        match channel {
                            TimChannel::C1 => self.regs.ccr1().read().ccr1().bits(),
                            TimChannel::C2 => self.regs.ccr2().read().ccr2().bits(),
                            _ => panic!()
                        }
                    } else {
                        match channel {
                            TimChannel::C1 => self.regs.ccr1().read().ccr().bits().try_into().unwrap(),
                            TimChannel::C2 => self.regs.ccr2().read().ccr().bits().try_into().unwrap(),
                            _ => panic!()
                        }
                    }
                }
            }

            /// Set the duty cycle, as a portion of ARR (`get_max_duty()`). Note that this
            /// needs to be re-run if you change ARR at any point.
            pub fn set_duty(&mut self, channel: TimChannel, duty: $res) {
                cfg_if! {
                    if #[cfg(feature = "g0")] {
                        match channel {
                            // TimChannel::C1 => self.regs.ccr1().write(|w| w.ccr1().bits(duty.try_into().unwrap())),
                            // TimChannel::C2 => self.regs.ccr2().write(|w| w.ccr2().bits(duty.try_into().unwrap())),
                            _ => panic!()
                        };
                    } else if #[cfg(any(feature = "wb", feature = "wl", feature = "l5"))] {
                        unsafe {
                            match channel {
                                TimChannel::C1 => self.regs.ccr1().write(|w| w.ccr1().bits(duty.try_into().unwrap())),
                                TimChannel::C2 => self.regs.ccr2().write(|w| w.ccr2().bits(duty.try_into().unwrap())),
                                _ => panic!()
                            };
                        }
                    } else {
                        unsafe {
                            match channel {
                                TimChannel::C1 => self.regs.ccr1().write(|w| w.ccr().bits(duty.try_into().unwrap())),
                                TimChannel::C2 => self.regs.ccr2().write(|w| w.ccr().bits(duty.try_into().unwrap())),
                                _ => panic!()
                            };
                        }
                    }
                }
            }

            /// Set output polarity. See docs on the `Polarity` enum.
            pub fn set_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1p().bit(polarity.bit())),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2p().bit(polarity.bit())),
                    _ => panic!()
                };
            }

            /// Set complementary output polarity. See docs on the `Polarity` enum.
            pub fn set_complementary_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1np().bit(polarity.bit())),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2np().bit(polarity.bit())),
                    _ => panic!()
                };
            }
            /// Disables capture compare on a specific channel.
            pub fn disable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().clear_bit()),
                    #[cfg(not(feature = "c0"))]
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2e().clear_bit()),
                    _ => panic!()
                };
            }

            /// Enables capture compare on a specific channel.
            pub fn enable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().bit(true)),
                    TimChannel::C2 => self.regs.ccer().modify(|_, w| w.cc2e().bit(true)),
                    _ => panic!()
                };
            }

            /// Set Capture Compare mode in output mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_output(&mut self, channel: TimChannel, mode: CaptureCompare) {
                self.disable_capture_compare(channel);

                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    #[cfg(not(feature = "c0"))]
                    TimChannel::C2 => self
                        .regs
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc2s().bits(mode as u8) }),
                    _ => panic!()
                };
            }

            /// Set Capture Compare mode in input mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_input(&mut self, channel: TimChannel, mode: CaptureCompare) {
                self.disable_capture_compare(channel);

                match channel {
                    // Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_input()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),

                    #[cfg(not(feature = "c0"))]
                    TimChannel::C2 => self
                        .regs
                        .ccmr1_input()
                        .modify(unsafe { |_, w| w.cc2s().bits(mode as u8) }),
                        _ => panic!()
                };
            }

            /// Set preload mode.
            /// OC1PE: Output Compare 1 preload enable
            /// 0: Preload register on TIMx_CCR1 disabled. TIMx_CCR1 can be written at anytime, the
            /// new value is taken in account immediately.
            /// 1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload
            /// register. TIMx_CCR1 preload value is loaded in the active register at each update event.
            /// Note: 1: These bits can not be modified as long as LOCK level 3 has been programmed
            /// (LOCK bits in TIMx_BDTR register) and CC1S=’00’ (the channel is configured in
            /// output).
            /// 2: The PWM mode can be used without validating the preload register only in one
            /// pulse mode (OPM bit set in TIMx_CR1 register). Else the behavior is not guaranteed.
            ///
            /// Setting preload is required to enable PWM.
            pub fn set_preload(&mut self, channel: TimChannel, value: bool) {
                match channel {
                    TimChannel::C1 => self.regs.ccmr1_output().modify(|_, w| w.oc1pe().bit(value)),
                    #[cfg(not(feature = "c0"))]
                    TimChannel::C2 => self.regs.ccmr1_output().modify(|_, w| w.oc2pe().bit(value)),
                    _ => panic!()
                };

                // "As the preload registers are transferred to the shadow registers only when an update event
                // occurs, before starting the counter, you have to initialize all the registers by setting the UG
                // bit in the TIMx_EGR register."
                self.reinitialize();
            }

        }
    }
}

macro_rules! cc_1_channel {
    ($TIMX:ident, $res:ident) => {
        impl Timer<pac::$TIMX> {
            /// Function that allows us to set direction only on timers that have this option.
            fn set_dir(&mut self) {} // N/A with these 1-channel timers.

            // todo: more advanced PWM modes. Asymmetric, combined, center-aligned etc.

            /// Set up input capture, eg for PWM input.
            /// L4 RM, section 26.3.8. H723 RM, section 43.3.7.
            /// Note: Does not handle TISEL (timer input selection register - you must do this manually
            /// using the PAC.
            #[cfg(not(any(feature = "f", feature = "l4x5", feature = "l5", feature = "g0")))]
            pub fn set_input_capture(
                &mut self,
                channel: TimChannel,
                mode: CaptureCompare,
                // trigger: InputTrigger,
                // slave_mode: InputSlaveMode,
                ccp: Polarity,
                ccnp: Polarity,
            ) {
                self.set_capture_compare_input(channel, mode);

                let filter = 0b00;

                match channel {
                    TimChannel::C1 => {
                        self.regs.ccer().modify(|_, w| {
                            w.cc1p().bit(ccp.bit());
                            w.cc1np().bit(ccnp.bit())
                        });

                        self.regs.ccmr1_input().modify(|_, w| unsafe {
                            w.ic1psc().bits(0b00);
                            w.ic1f().bits(filter)
                        });
                    }
                    _ => panic!(),
                };

                // todo?
                // self.regs.smcr().modify(|_, w| unsafe {
                //     w.ts().bits(trigger as u8);
                //     w.sms().bits(slave_mode as u8)
                // });

                self.enable_capture_compare(channel);
            }

            /// Set Output Compare Mode. See docs on the `OutputCompare` enum.
            pub fn set_output_compare(&mut self, channel: TimChannel, mode: OutputCompare) {
                match channel {
                    TimChannel::C1 => {
                        #[cfg(not(feature = "g070"))] // todo: PAC bug?
                        self.regs.ccmr1_output().modify(|_, w| unsafe {
                            // todo: L5/WB is probably due to a PAC error. Has oc1m_2.
                            #[cfg(not(any(
                                feature = "f",
                                feature = "l4",
                                feature = "l5",
                                feature = "wb",
                                feature = "g0"
                            )))]
                            w.oc1m_3().bit((mode as u8) >> 3 != 0);
                            w.oc1m().bits((mode as u8) & 0b111)
                        });
                    }
                    _ => panic!(),
                };
            }

            /// Return the set duty period for a given channel. Divide by `get_max_duty()`
            /// to find the portion of the duty cycle used.
            pub fn get_duty(&self, channel: TimChannel) -> $res {
                match channel {
                    TimChannel::C1 => self.regs.ccr1().read().ccr().bits().try_into().unwrap(),
                    _ => panic!(),
                }
            }

            /// Set the duty cycle, as a portion of ARR (`get_max_duty()`). Note that this
            /// needs to be re-run if you change ARR at any point.
            pub fn set_duty(&mut self, channel: TimChannel, duty: $res) {
                unsafe {
                    match channel {
                        TimChannel::C1 => self
                            .regs
                            .ccr1()
                            .write(|w| w.ccr().bits(duty.try_into().unwrap())),
                        _ => panic!(),
                    };
                }
            }

            /// Set output polarity. See docs on the `Polarity` enum.
            pub fn set_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1p().bit(polarity.bit())),
                    _ => panic!(),
                };
            }

            /// Set complementary output polarity. See docs on the `Polarity` enum.
            pub fn set_complementary_polarity(&mut self, channel: TimChannel, polarity: Polarity) {
                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccer()
                        .modify(|_, w| w.cc1np().bit(polarity.bit())),
                    _ => panic!(),
                };
            }
            /// Disables capture compare on a specific channel.
            pub fn disable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().clear_bit()),
                    _ => panic!(),
                };
            }

            /// Enables capture compare on a specific channel.
            pub fn enable_capture_compare(&mut self, channel: TimChannel) {
                match channel {
                    TimChannel::C1 => self.regs.ccer().modify(|_, w| w.cc1e().bit(true)),
                    _ => panic!(),
                };
            }

            /// Set Capture Compare mode in output mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_output(
                &mut self,
                channel: TimChannel,
                mode: CaptureCompare,
            ) {
                self.disable_capture_compare(channel);

                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    _ => panic!(),
                };
            }

            /// Set Capture Compare mode in input mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare_input(&mut self, channel: TimChannel, mode: CaptureCompare) {
                self.disable_capture_compare(channel);

                match channel {
                    TimChannel::C1 => self
                        .regs
                        .ccmr1_input()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    _ => panic!(),
                };
            }

            /// Set preload mode.
            /// OC1PE: Output Compare 1 preload enable
            /// 0: Preload register on TIMx_CCR1 disabled. TIMx_CCR1 can be written at anytime, the
            /// new value is taken in account immediately.
            /// 1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload
            /// register. TIMx_CCR1 preload value is loaded in the active register at each update event.
            /// Note: 1: These bits can not be modified as long as LOCK level 3 has been programmed
            /// (LOCK bits in TIMx_BDTR register) and CC1S=’00’ (the channel is configured in
            /// output).
            /// 2: The PWM mode can be used without validating the preload register only in one
            /// pulse mode (OPM bit set in TIMx_CR1 register). Else the behavior is not guaranteed.
            ///
            /// Setting preload is required to enable PWM.
            pub fn set_preload(&mut self, channel: TimChannel, value: bool) {
                match channel {
                    TimChannel::C1 => self.regs.ccmr1_output().modify(|_, w| w.oc1pe().bit(value)),
                    _ => panic!(),
                };

                // "As the preload registers are transferred to the shadow registers only when an update event
                // occurs, before starting the counter, you have to initialize all the registers by setting the UG
                // bit in the TIMx_EGR register."
                self.reinitialize();
            }
        }
    };
}

/// Calculate values required to set the timer frequency: `PSC` and `ARR`. This can be
/// used for initial timer setup, or changing the value later. If used in performance-sensitive
/// code or frequently, set ARR and PSC directly instead of using this.
fn calc_freq_vals(freq: f32, clock_speed: u32) -> Result<(u16, u16)> {
    // `period` and `clock_speed` are both in Hz.

    // PSC and ARR range: 0 to 65535
    // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
    // APB1 (pclk1) is used by Tim2, 3, 4, 6, 7.
    // APB2 (pclk2) is used by Tim8, 15-20 etc.

    // We need to factor the right-hand-side of the above equation
    // into integers. There are likely clever algorithms available to do this.
    // Some examples: https://cp-algorithms.com/algebra/factorization.html
    // We've chosen something that attempts to maximize ARR, for precision when
    // setting duty cycle. Alternative approaches might involve setting a frequency closest to the
    // requested one.

    // If you work with pure floats, there are an infinite number of solutions: Ie for any value of PSC,
    // you can find an ARR to solve the equation.
    // The actual values are integers that must be between 0 and 65_536
    // Different combinations will result in different amounts of rounding error.

    let max_val = 65_535.;
    let rhs = clock_speed as f32 / freq;

    let psc = ((rhs - 1.) / (1 << 16) as f32).round();
    let arr = rhs / (psc + 1.) - 1.;

    if arr > max_val || psc > max_val {
        return Err(Error::TimerError(TimerError::ValueError));
    }

    Ok((psc as u16, arr as u16))
}

cfg_if! {
    if #[cfg(not(any(
        feature = "f401",
        feature = "f410",
        feature = "f411",
        feature = "f413",
        feature = "g031",
        feature = "g041",
        feature = "g070",
        feature = "g030",
        feature = "g051",
        feature = "c0",
        feature = "wb",
        feature = "wl"
    )))]  {
        /// Represents a Basic timer, used primarily to trigger the onboard DAC. Eg Tim6 or Tim7.
        pub struct BasicTimer<R> {
            pub regs: R,
            clock_speed: u32,
        }

        impl<R> BasicTimer<R>
            where
                R: Deref<Target = pac::tim6::RegisterBlock> + RccPeriph,
        {
            /// Initialize a Basic timer, including  enabling and resetting
            /// its RCC peripheral clock.
            pub fn new(
                regs: R,
                freq: f32,
                clock_cfg: &Clocks,
            ) -> Self {
                let rcc = unsafe { &(*RCC::ptr()) };
                R::en_reset(rcc);

                // Self { regs, config, clock_speed: clocks.apb1_timer()  }
                let mut result = Self { regs, clock_speed: clock_cfg.apb1_timer()  };

                result.set_freq(freq).ok();
                result
            }

            // todo: These fns are DRY from GP timer code!

            /// Enable the timer.
            pub fn enable(&mut self) {
                self.regs.cr1().modify(|_, w| w.cen().bit(true));
            }

            /// Disable the timer.
            pub fn disable(&mut self) {
                self.regs.cr1().modify(|_, w| w.cen().clear_bit());
            }

            /// Check if the timer is enabled.
            pub fn is_enabled(&self) -> bool {
                self.regs.cr1().read().cen().bit_is_set()
            }

            /// Set the timer period, in seconds. Overrides the period or frequency set
            /// in the constructor.  If changing pe riod frequently, don't use this method, as
            /// it has computational overhead: use `set_auto_reload` and `set_prescaler` methods instead.
            pub fn set_period(&mut self, time: f32) -> Result<() > {
                assert!(time > 0.);
                self.set_freq(1. / time)
            }

            /// Set the timer frequency, in Hz. Overrides the period or frequency set
            /// in the constructor. If changing frequency frequently, don't use this method, as
            /// it has computational overhead: use `set_auto_reload` and `set_prescaler` methods instead.
            pub fn set_freq(&mut self, freq: f32) -> Result<() > {
                assert!(freq > 0.);

                let (psc, arr) = calc_freq_vals(freq, self.clock_speed)?;

                self.regs.arr().write(|w| unsafe { w.bits(arr.into()) });
                self.regs.psc().write(|w| unsafe { w.bits(psc.into()) });

                Ok(())
            }

            /// Return the integer associated with the maximum duty period.
            pub fn get_max_duty(&self) -> u32 {
                return self.regs.arr().read().bits()
            }

            /// Set the auto-reload register value. Used for adjusting frequency.
            pub fn set_auto_reload(&mut self, arr: u16) {
                self.regs.arr().write(|w| unsafe { w.bits(arr.into()) });
            }

            /// Set the prescaler value. Used for adjusting frequency.
            pub fn set_prescaler(&mut self, psc: u16) {
                self.regs.psc().write(|w| unsafe { w.bits(psc.into()) });
            }

            /// Reset the count; set the counter to 0.
            pub fn reset_count(&mut self) {
                self.regs.cnt().write(|w| unsafe { w.bits(0) });
            }

            /// Read the current counter value.
            pub fn read_count(&self) -> u16 {
                #[cfg(feature = "l5")]
                return self.regs.cnt().read().bits() as u16;
                #[cfg(not(feature = "l5"))]
                self.regs.cnt().read().cnt().bits()
            }

            /// Allow selected information to be sent in master mode to slave timers for
            /// synchronization (TRGO).
            pub fn set_mastermode(&self, mode: MasterModeSelection) {
                self.regs.cr2().modify(|_, w| unsafe { w.mms().bits(mode as u8) });
            }
        }
    }
}

/// A freestanding function that does not require access to a `Timer` struct. Clears the Update interrupt.
pub fn clear_update_interrupt(tim_num: u8) {
    unsafe {
        let periphs = pac::Peripherals::steal();

        let bits = 0xffff_ffff;

        // Note: `.try_into().unwrap()` is for C0.

        match tim_num {
            #[cfg(not(any(feature = "f373")))]
            1 => {
                periphs.TIM1.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(not(any(
                feature = "f410",
                feature = "g070",
                feature = "g030",
                feature = "g031",
                feature = "g050",
                feature = "g061",
                feature = "c011",
                feature = "c031",
            )))]
            2 => {
                periphs
                    .TIM2
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            #[cfg(not(any(
                feature = "f301",
                feature = "l4x1",
                // feature = "l412",
                feature = "l4x3",
                feature = "l412",
                feature = "f410",
                feature = "wb",
                feature = "wl"
            )))]
            3 => {
                periphs
                    .TIM3
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            #[cfg(not(any(
                feature = "f301",
                feature = "f3x4",
                feature = "f410",
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "g0",
                feature = "c0",
                feature = "wb",
                feature = "wl"
            )))]
            4 => {
                periphs.TIM4.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(any(
                feature = "f373",
                feature = "l4x5",
                feature = "l4x6",
                feature = "l5",
                feature = "h5",
                feature = "h7",
                feature = "g473",
                feature = "g474",
                feature = "g483",
                feature = "g484",
                all(feature = "f4", not(feature = "f410")),
            ))]
            5 => {
                periphs.TIM5.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(any(
                feature = "f303",
                feature = "l4x5",
                feature = "l4x6",
                feature = "l562",
                feature = "h5",
                feature = "h7",
            ))]
            8 => {
                periphs.TIM8.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(any(feature = "h5",))]
            12 => {
                periphs.TIM12.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(any(feature = "h5",))]
            13 => {
                periphs.TIM13.sr().write(|w| w.bits(bits).uif().clear_bit());
            }
            #[cfg(any(feature = "h5", feature = "c0",))]
            14 => {
                periphs
                    .TIM14
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            #[cfg(not(any(
                feature = "f4",
                feature = "g031",
                feature = "g031",
                feature = "g041",
                feature = "g030",
                feature = "g051",
                feature = "g061",
                feature = "wb",
                feature = "wl",
                feature = "c0",
            )))]
            15 => {
                periphs
                    .TIM15
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            #[cfg(not(any(feature = "f4",)))]
            16 => {
                periphs
                    .TIM16
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            #[cfg(not(any(
                feature = "l4x1",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "f4",
            )))]
            17 => {
                periphs
                    .TIM17
                    .sr()
                    .write(|w| w.bits(bits.try_into().unwrap()).uif().clear_bit());
            }
            _ => unimplemented!(),
        }
    };
}

// /// Experimental approach where we set frequency without taking ownership.
// pub fn set_freq(timer: TimerNum, mut freq: f32) -> Result<(), ValueError> {
//
// }

// todo: Non-macro refactor base timer reg blocks:

// GP 32-bit: Tim2
// 2, 3, 4, 5

// GP 16-bit:
// 15, 16, 17 // (9-14 on F4) 14 on G0

// Basic:
// 6, 7

// Advanced: 1/8/20

#[cfg(not(any(feature = "f373")))]
make_timer!(TIM1, tim1, 2, u16);

#[cfg(not(any(feature = "f373", feature = "g0", feature = "g4")))]
cc_4_channels!(TIM1, u16);
// todo: PAC error?
// TIM1 on G4 is nominally 16-bits, but has ~20 bits on ARR, with PAC showing 32 bits?
#[cfg(any(feature = "g0", feature = "g4"))]
cc_2_channels!(TIM1, u16);

cfg_if! {
    if #[cfg(not(any(
        feature = "f410",
        feature = "g070",
        feature = "l5", // todo PAC bug?
        feature = "wb55", // todo PAC bug?
        feature = "g030",
        feature = "g031",
        feature = "g050",
        feature = "g061",
        feature = "g031",
        feature = "c011",
        feature = "c031",
    )))] {
        make_timer!(TIM2, tim2, 1, u32);
        cc_4_channels!(TIM2, u32);
    }
}

// todo: Note. G4, for example, has TIM2 and 5 as 32-bit, and TIM3 and 4 as 16-bit per RM,
// todo: But PAC shows different.
cfg_if! {
    if #[cfg(not(any(
        feature = "f301",
        feature = "l4x1",
        // feature = "l412",
        feature = "l5", // todo PAC bug?
        feature = "l4x3",
        feature = "l412",
        feature = "f410",
        feature = "wb",
        feature = "wl",
        // feature = "c0",
    )))] {
        make_timer!(TIM3, tim3, 1, u32);
        cc_4_channels!(TIM3, u32);
    }
}

// #[cfg(feature = "c0")]
// make_timer!(TIM3, tim3, 1, u16);
// #[cfg(feature = "c0")]
// cc_4_channels!(TIM3, u16);

cfg_if! {
    if #[cfg(not(any(
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4x1",
        feature = "l4x2",
        feature = "l412",
        feature = "l4x3",
        feature = "g0",
        feature = "c0",
        feature = "wb",
        feature = "wl"
    )))] {
        make_timer!(TIM4, tim4, 1, u32);
        cc_4_channels!(TIM4, u32);
    }
}

cfg_if! {
    if #[cfg(any(
       feature = "f373",
       feature = "l4x5",
       feature = "l4x6",
       feature = "l562",
       feature = "h5",
       feature = "h7",
       feature = "g473",
       feature = "g474",
       feature = "g483",
       feature = "g484",
       all(feature = "f4", not(feature = "f410")),
   ))] {
        make_timer!(TIM5, tim5, 1, u32);
        cc_4_channels!(TIM5, u32);
   }
}

cfg_if! {
    if #[cfg(any(
        feature = "f303",
        feature = "l4x5",
        feature = "l4x6",
        feature = "l562",
        feature = "h5",
        feature = "h7",
    ))] {
        make_timer!(TIM8, tim8, 2, u16);
        // todo: Some issues with field names or something on l562 here.
        #[cfg(not(feature = "l5"))] // PAC bug.
        cc_4_channels!(TIM8, u16);
        #[cfg(feature = "l5")] // PAC bug.
        cc_1_channel!(TIM8, u16);
    }
}

// todo: G4 should be 16-bits for TIM8. Why does the PAC use 32?
cfg_if! {
    if #[cfg(feature = "g4")] {
        make_timer!(TIM8, tim8, 2, u32);
        cc_4_channels!(TIM8, u32);
    }
}

cfg_if! {
    if #[cfg(feature = "h5")] {
        make_timer!(TIM12, tim12, 1, u32);
        cc_2_channels!(TIM12, u32);

        make_timer!(TIM13, tim13, 1, u32);
        cc_2_channels!(TIM13, u32);

        make_timer!(TIM14, tim14, 1, u32);
        cc_2_channels!(TIM14, u32);
    }
}

// #[cfg(feature = "c0")]
// make_timer!(TIM14, tim14, 1, u32);
// #[cfg(feature = "c0")]
// cc_2_channels!(TIM14, u16);

cfg_if! {
    if #[cfg(not(any(
        feature = "f4",
        feature = "g031",
        feature = "g031",
        feature = "g041",
        feature = "g030",
        feature = "g051",
        feature = "g061",
        feature = "wb",
        feature = "wl",
      // todo: Tim15 is available on c091/02, but I don't see a PAC for that.
        feature = "c0",
    )))] {
        make_timer!(TIM15, tim15, 2, u16);
        // todo: TIM15 on some variant has 2 channels (Eg H7). On others, like L4x3, it appears to be 1.
        cc_1_channel!(TIM15, u16);
    }
}

#[cfg(not(any(feature = "f4", feature = "c0")))]
make_timer!(TIM16, tim16, 2, u16);
#[cfg(not(any(feature = "f4", feature = "c0")))]
cc_1_channel!(TIM16, u16);

#[cfg(feature = "c0")]
make_timer!(TIM16, tim16, 2, u32);
#[cfg(feature = "c0")]
cc_1_channel!(TIM16, u32);

cfg_if! {
    if #[cfg(not(any(
        feature = "l4x1",
        feature = "l4x2",
        feature = "l412",
        feature = "l4x3",
        feature = "f4",
        // feature = "c0"
    )))] {
        make_timer!(TIM17, tim17, 2, u16);
        cc_1_channel!(TIM17, u16);
    }
}

// cfg_if! {
//     if #[cfg(any(
//         feature = "c0"
//     ))] {
//         make_timer!(TIM17, tim17, 2, u32);
//         cc_1_channel!(TIM17, u32);
//     }
// }

// { todo: tim18
//     TIM18: (tim18, apb2, enr, rstr),
// },

cfg_if! {
    if #[cfg(any(feature = "f373"))] {
        make_timer!(TIM12, tim12, 1, u16);
        make_timer!(TIM13, tim13, 1, u16);
        make_timer!(TIM14, tim14, 1, u16);
        make_timer!(TIM19, tim19, 2, u16);

        cc_1_channel!(TIM12, u16);
        cc_1_channel!(TIM13, u16);
        cc_1_channel!(TIM14, u16);
        cc_1_channel!(TIM19, u16);
    }
}

// todo: G4 (maybe not all variants?) have TIM20.
#[cfg(any(feature = "f303"))]
make_timer!(TIM20, tim20, 2, u16);
#[cfg(any(feature = "f303"))]
cc_4_channels!(TIM20, u16);

// todo: Remove the final "true/false" for adv ctrl. You need a sep macro like you do for ccx_channel!.
