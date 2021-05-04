//! Timers. Includes initialization, countdown functionality, interrupts, and PWM features.

use num_traits::float::Float;

use embedded_hal::timer::{CountDown, Periodic};

use void::Void;

// todo: LPTIM (low-power timers)

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

use cfg_if::cfg_if;
use paste::paste;

// todo: Low power timer enabling etc. eg on L4, RCC_APB1ENR1.LPTIM1EN

#[derive(Clone, Copy, Debug)]
/// Used for when attempting to set a timer period that is out of range.
pub struct ValueError {}

/// Hardware timers
pub struct Timer<TIM> {
    clock_speed: u32, // Associated timer clock speed in Hz.
    tim: TIM,         // Register block for the specific timer.
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

/// Output alignment
#[derive(Clone, Copy)]
pub enum Alignment {
    Edge,
    Center1,
    Center2,
    Center3,
}

// todo: TimerChannel, to avoid input conflicts?
/// Timer channel
#[derive(Clone, Copy)]
pub enum Channel {
    One,
    Two,
    Three,
    Four,
}

/// Timer count direction
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CountDir {
    Up = 0,
    Down = 1,
}

/// Capture/Compare selection.
/// This bit-field defines the direction of the channel (input/output) as well as the used input.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CaptureCompare {
    Output = 0b00,
    InputTi1 = 0b01,
    InputTi2 = 0b10,
    InputTrc = 0b11,
}

/// Capture/Compare output polarity. Defaults to `ActiveHigh` in hardware.
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
/// See F303 ref man, section 21.4.7.
/// These bits define the behavior of the output reference signal OC1REF from which OC1 and
/// OC1N are derived. OC1REF is active high whereas OC1 and OC1N active level depends
/// on CC1P and CC1NP bits.
/// 0000: Frozen - The comparison between the output compare register TIMx_CCR1 and the
/// counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing
/// base).
/// 0001: Set channel 1 to active level on match. OC1REF signal is forced high when the
/// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
/// 0010: Set channel 1 to inactive level on match. OC1REF signal is forced low when the
/// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
/// 0011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
/// 0100: Force inactive level - OC1REF is forced low.
/// 0101: Force active level - OC1REF is forced high.
/// 0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
/// else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
/// TIMx_CNT>TIMx_CCR1 else active (OC1REF=1).
/// 0111: PWM mode 2 - In upcounting, channel 1 is inactive as long as
/// TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
/// TIMx_CNT>TIMx_CCR1 else inactive.
/// 1000: Retriggerable OPM mode 1 - In up-counting mode, the channel is active until a trigger
/// event is detected (on TRGI signal). Then, a comparison is performed as in PWM mode 1
/// and the channels becomes inactive again at the next update. In down-counting mode, the
/// channel is inactive until a trigger event is detected (on TRGI signal). Then, a comparison is
/// performed as in PWM mode 1 and the channels becomes inactive again at the next update.
/// 1001: Retriggerable OPM mode 2 - In up-counting mode, the channel is inactive until a
/// trigger event is detected (on TRGI signal). Then, a comparison is performed as in PWM
/// mode 2 and the channels becomes inactive again at the next update. In down-counting
/// mode, the channel is active until a trigger event is detected (on TRGI signal). Then, a
/// comparison is performed as in PWM mode 1 and the channels becomes active again at the
/// next update.
/// 1010: Reserved,
/// 1011: Reserved,
/// 1100: Combined PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
/// OC1REFC is the logical OR between OC1REF and OC2REF.
/// 1101: Combined PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
/// OC1REFC is the logical AND between OC1REF and OC2REF.
/// 1110: Asymmetric PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
/// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
/// down.
/// 1111: Asymmetric PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
/// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
/// down
pub enum OutputCompare {
    // In our current implementation, the left bit here is ignored due to how
    // the `ocxm` fields are split between left most, and right three bits.
    // see `left_fit()` method below.
    Frozen = 0b0000,
    Active = 0b0001,
    Inactive = 0b0010,
    ForceInactive = 0b0100,
    ForceActive = 0b0101,
    Pwm1 = 0b0110,
    Pwm2 = 0b0111,
    RetriggerableOpmMode1 = 0b1000,
    RetriggerableOpmMode2 = 0b1001,
    CombinedPwm1 = 0b1100,
    CombinedPwm2 = 0b1101,
    AsymmetricPwm1 = 0b1110,
    AsymmetricPwm2 = 0b1111,
}

impl OutputCompare {
    /// A workaround due to the `ccmrx_output.ocym` fields being split into
    /// the left most, and first 3.
    /// Get the left bit, as a boolean. For the right three, we just
    /// parse the variant as a u8, and the left bit is ignored when setting
    /// in the 3-bit field.
    pub fn left_bit(&self) -> bool {
        matches!(
            self,
            Self::RetriggerableOpmMode1
                | Self::RetriggerableOpmMode2
                | Self::CombinedPwm1
                | Self::CombinedPwm2
                | Self::AsymmetricPwm1
                | Self::AsymmetricPwm2
        )
    }
}

macro_rules! hal {
    ($TIMX:ident, $tim:ident, $apb:expr) => {
        impl Periodic for Timer<pac::$TIMX> {}

        impl CountDown for Timer<pac::$TIMX> {
            type Time = f32;

            fn start<T>(&mut self, timeout: T)
            where
                T: Into<f32>,  // Hz.
            {
                self.disable();

                self.set_freq(timeout.into()).ok();

                // Trigger an update event to load the prescaler value to the clock
                // NOTE(write): uses all bits in this register.
                self.tim.egr.write(|w| w.ug().set_bit());
                // The above line raises an update event which will indicate
                // that the timer is already finished. Since this is not the case,
                // it should be cleared
                self.clear_update_interrupt_flag();

                // start counter
                self.enable();
            }

            fn wait(&mut self) -> nb::Result<(), Void> {
                if self.tim.sr.read().uif().bit_is_clear() {
                    Err(nb::Error::WouldBlock)
                } else {
                    self.clear_update_interrupt_flag();
                    Ok(())
                }
            }
        }

        impl Timer<pac::$TIMX> {
            paste! {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn [<new_ $tim>]<C: ClockCfg>(tim: pac::$TIMX, freq: f32, clocks: &C, rcc: &mut RCC) -> Self {
                    // `freq` is in Hz.
                    rcc_en_reset!([<apb $apb>], $tim, rcc);

                    let clock_speed = match $apb {
                        1 => clocks.apb1_timer(),
                        _ => clocks.apb2_timer(),
                    };
                    let mut timer = Timer { clock_speed, tim };

                    timer.set_freq(freq).ok();

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    timer.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    timer.clear_update_interrupt_flag();

                    timer
                }
            }
            /// Starts listening for an `event`. Used to enable interrupts.
            /// Starts listening for an `event`. Used to enable interrupts.
            pub fn listen(&mut self, event: Event) {
                match event {
                    // Enable update event interrupt
                    Event::TimeOut => self.tim.dier.write(|w| w.uie().set_bit()),
                }
            }

            /// Enable the timer.
            pub fn enable(&mut self) {
                self.tim.cr1.modify(|_, w| w.cen().set_bit());
            }

            /// Disable the timer.
            pub fn disable(&mut self) {
                self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            }

            /// Check if the timer is enabled.
            pub fn is_enabled(&self) -> bool {
                self.tim.cr1.read().cen().bit_is_set()
            }

            /// Clears interrupt associated with this timer.
            ///
            /// If the interrupt is not cleared, it will immediately retrigger after
            /// the ISR has finished. For examlpe, place this at the top of your timer's
            /// interrupt handler.
            pub fn clear_interrupt(&mut self) {
                self.tim.sr.write(|w| w.uif().clear_bit());
            }

            /// Stops listening for an `xcevent`. Used to disable interrupts.
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    // Disable update event interrupt
                    Event::TimeOut => self.tim.dier.write(|w| w.uie().clear_bit()),
                }
            }

            /// Clears Update Interrupt Flag
            pub fn clear_update_interrupt_flag(&mut self) {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
            }

            /// Releases the TIM peripheral
            pub fn free(mut self) -> pac::$TIMX {
                self.disable();
                self.tim
            }

            /// Set the timer frequency, in Hz. Overrides the period or frequency set
            /// in the constructor. If you use `center` aligned PWM, make sure to
            /// enter twice the freq you normally would.
            pub fn set_freq(&mut self, freq: f32) -> Result<(), ValueError> {
               // todo: Take into account settings like Center alignment, and
               // todo the `tim1sw` bit in RCC CFGR3, which change how the
               // todo freq behaves. Center alignment halves the frequency;
               // todo: Double `freq` here to compensate.
               let (psc, arr) = calc_freq_vals(freq, self.clock_speed)?;

                self.tim.arr.write(|w| unsafe { w.bits(arr.into()) });
                self.tim.psc.write(|w| unsafe { w.bits(psc.into()) });

                Ok(())
            }

            /// Set the auto-reload register value. Used for adjusting frequency.
            pub fn set_auto_reload(&mut self, arr: u32) {
                // todo: Could be u16 or u32 depending on timer resolution,
                // todo but this works for now.
                self.tim.arr.write(|w| unsafe { w.bits(arr.into()) });
            }

            /// Set the prescaler value. Used for adjusting frequency.
            pub fn set_prescaler(&mut self, psc: u16) {
                self.tim.psc.write(|w| unsafe { w.bits(psc.into()) });
            }

            /// Reset the countdown; set the counter to 0.
            pub fn reset_countdown(&mut self) {
                self.tim.cnt.write(|w| unsafe { w.bits(0) });
            }
        }
    }
}

/// Calculate values required to set the timer frequency: `PSC` and `ARR`. This can be
/// used for initial timer setup, or changing the value later.
fn calc_freq_vals(freq: f32, clock_speed: u32) -> Result<(u16, u16), ValueError> {
    // `period` and `clock_speed` are both in Hz.

    // PSC and ARR range: 0 to 65535
    // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
    // APB1 (pclk1) is used by Tim2, 3, 4, 6, 7.
    // APB2 (pclk2) is used by Tim8, 15-20 etc.

    // We need to factor the right-hand-side of the above equation (`rhs` variable)
    // into integers. There are likely clever algorithms available to do this.
    // Some examples: https://cp-algorithms.com/algebra/factorization.html
    // We've chosen something quick to write, and with sloppy precision;
    // should be good enough for most cases.

    // - If you work with pure floats, there are an infinite number of solutions: Ie for any value of PSC, you can find an ARR to solve the equation.
    // - The actual values are integers that must be between 0 and 65_536
    // - Different combinations will result in different amounts of rounding errors. Ideally, we pick the one with the lowest rounding error.
    // - The aboveapproach sets PSC and ARR always equal to each other.
    // This results in concise code, is computationally easy, and doesn't limit
    // the maximum period. There will usually be solutions that have a smaller rounding error.

    let max_val = 65_535;
    let rhs = clock_speed as f32 / freq;

    let arr = rhs.sqrt().round() as u16 - 1;
    let psc = arr;

    if arr > max_val || psc > max_val {
        return Err(ValueError {});
    }

    Ok((psc, arr))
}

macro_rules! pwm_features {
    ($TIMX:ident, $res:ident) => {
        impl Timer<pac::$TIMX> {
            /// Enables basic PWM output
            pub fn enable_pwm_output(
                &mut self,
                channel: Channel,
                compare: OutputCompare,
                dir: CountDir,
                duty: f32,
            ) {
                self.set_preload(channel, true);
                self.set_output_compare(channel, compare);
                self.set_duty(channel, (self.get_max_duty() as f32 * duty) as $res);
                self.tim.cr1.modify(|_, w| w.dir().bit(dir as u8 != 0));
                self.enable_capture_compare(channel);
            }

            /// Enables basic PWM input. TODO: Doesn't work yet.
            /// L4 RM, section 26.3.8
            pub fn enable_pwm_input(
                &mut self,
                channel: Channel,
                compare: OutputCompare,
                dir: CountDir,
                duty: f32,
            ) {
                // todo: These instruction sare specifically for TI1
                // 1. Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1
                // register (TI1 selected).
                // self.tim.ccmr1.modify(|_, w| w.cc1s().bit(0b01));

                // 2. Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter
                // clear): write the CC1P and CC1NP bits to ‘0’ (active on rising edge).
                // self.tim.ccmr1.modify(|_, w| {
                //     w.cc1p().bits(0b00);
                //     w.cc1np().bits(0b00)
                // });
                // 3. Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1
                // register (TI1 selected).
                // self.tim.ccmr2.modify(|_, w| w.cc2s().bit(0b10));

                // 4. Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P
                // and CC2NP bits to CC2P/CC2NP=’10’ (active on falling edge).
                // self.tim.ccr2.modify(|_, w| {
                //     w.cc2p().bits(0b10);
                //     w.cc2np().bits(0b10)
                // });

                // 5. Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
                // (TI1FP1 selected).
                // self.tim.smcr.modify(|_, w| w.ts().bits(0b101));

                // 6. Configure the slave mode controller in reset mode: write the SMS bits to 0100 in the
                // TIMx_SMCR register.
                // self.tim.smcr.modify(|_, w| w.sms().bits(0b0100));

                // 7. Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register.
                // self.tim.ccer.modify(|_, w| {
                //     w.cc1e().set_bit();
                //     w.cc2e().set_bit()
                // });
            }

            // todo: more advanced PWM modes. Asymmetric, combined, center-aligned etc.

            /// Set Output Compare Mode. See docs on the `OutputCompare` enum.
            pub fn set_output_compare(&mut self, channel: Channel, mode: OutputCompare) {
                match channel {
                    Channel::One => {
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| unsafe { w.oc1m().bits(mode as u8) });
                        // todo: Confirm other platforms handle everything using `oc1m`, and don't
                        // todo need the `oc1m_3` equiv. L5 and 4?
                        #[cfg(any(feature = "f302", feature = "f303"))]
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| w.oc1m_3().bit(mode.left_bit()));
                    }
                    Channel::Two => {
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| unsafe { w.oc1m().bits(mode as u8) });
                        #[cfg(any(feature = "f302", feature = "f303"))] // todo see note above
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| w.oc1m_3().bit(mode.left_bit()));
                    }
                    Channel::Three => {
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| unsafe { w.oc1m().bits(mode as u8) });
                        #[cfg(any(feature = "f302", feature = "f303"))] // todo see note above
                        self.tim
                            .ccmr1_output()
                            .modify(|_, w| w.oc1m_3().bit(mode.left_bit()));
                    }
                    Channel::Four => {
                        self.tim
                            .ccmr2_output()
                            .modify(|_, w| unsafe { w.oc4m().bits(mode as u8) });
                        #[cfg(any(feature = "f302", feature = "f303"))] // todo see note above
                        self.tim
                            .ccmr2_output()
                            .modify(|_, w| w.oc4m_3().bit(mode.left_bit()));
                    }
                }
            }

            /// Return the set duty period for a given channel. Divide by `get_max_duty()`
            /// to find the portion of the duty cycle used.
            pub fn get_duty(&self, channel: Channel) -> $res {
                cfg_if! {
                    if #[cfg(feature = "g0")] {
                        match channel {
                            Channel::One => self.tim.ccr1.read().bits(),
                            Channel::Two => self.tim.ccr2.read().bits(),
                            Channel::Three => self.tim.ccr3.read().bits(),
                            Channel::Four => self.tim.ccr4.read().bits(),
                        }
                    } else if #[cfg(feature = "g4")] {
                        match channel {
                            Channel::One => self.tim.ccr1.read().ccr1().bits(),
                            Channel::Two => self.tim.ccr2.read().ccr2().bits(),
                            Channel::Three => self.tim.ccr3.read().ccr3().bits(),
                            Channel::Four => self.tim.ccr4.read().ccr4().bits(),
                        }
                    } else {
                        match channel {
                            Channel::One => self.tim.ccr1.read().ccr().bits(),
                            Channel::Two => self.tim.ccr2.read().ccr().bits(),
                            Channel::Three => self.tim.ccr3.read().ccr().bits(),
                            Channel::Four => self.tim.ccr4.read().ccr().bits(),
                        }
                    }
                }
            }

            /// Set the duty cycle, as a portion of `get_max_duty()`.
            pub fn set_duty(&mut self, channel: Channel, duty: $res) {
                cfg_if! {
                    if #[cfg(feature = "g0")] {
                        match channel {
                            Channel::One => self.tim.ccr1.read().bits(),
                            Channel::Two => self.tim.ccr2.read().bits(),
                            Channel::Three => self.tim.ccr3.read().bits(),
                            Channel::Four => self.tim.ccr4.read().bits(),
                        };
                    } else if #[cfg(feature = "g4")] {
                        unsafe {
                            match channel {
                                Channel::One => self.tim.ccr1.write(|w| w.ccr1().bits(duty)),
                                Channel::Two => self.tim.ccr2.write(|w| w.ccr2().bits(duty)),
                                Channel::Three => self.tim.ccr3.write(|w| w.ccr3().bits(duty)),
                                Channel::Four => self.tim.ccr4.write(|w| w.ccr4().bits(duty)),
                            }
                        }
                    } else {
                        match channel {
                            Channel::One => self.tim.ccr1.write(|w| w.ccr().bits(duty)),
                            Channel::Two => self.tim.ccr2.write(|w| w.ccr().bits(duty)),
                            Channel::Three => self.tim.ccr3.write(|w| w.ccr().bits(duty)),
                            Channel::Four => self.tim.ccr4.write(|w| w.ccr().bits(duty)),
                        }
                    }
                }
            }

            /// Return the integer associated with the maximum duty period.
            /// todo: Duty could be u16 for low-precision timers.
            pub fn get_max_duty(&self) -> $res {
                #[cfg(feature = "g0")]
                return self.tim.arr.read().bits();
                #[cfg(not(feature = "g0"))]
                return self.tim.arr.read().arr().bits();
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
                let word = match alignment {
                    Alignment::Edge => 0b00,
                    Alignment::Center1 => 0b01,
                    Alignment::Center2 => 0b10,
                    Alignment::Center3 => 0b11,
                };
                self.tim.cr1.modify(|_, w| unsafe { w.cms().bits(word) });
            }

            /// Set output polarity. See docs on the `Polarity` enum.
            pub fn set_polarity(&mut self, channel: Channel, polarity: Polarity) {
                match channel {
                    Channel::One => self.tim.ccer.modify(|_, w| w.cc1p().bit(polarity.bit())),
                    Channel::Two => self.tim.ccer.modify(|_, w| w.cc2p().bit(polarity.bit())),
                    Channel::Three => self.tim.ccer.modify(|_, w| w.cc3p().bit(polarity.bit())),
                    Channel::Four => self.tim.ccer.modify(|_, w| w.cc4p().bit(polarity.bit())),
                }
            }

            /// Set complementary output polarity. See docs on the `Polarity` enum.
            pub fn set_complementary_polarity(&mut self, channel: Channel, polarity: Polarity) {
                match channel {
                    Channel::One => self.tim.ccer.modify(|_, w| w.cc1np().bit(polarity.bit())),
                    Channel::Two => self.tim.ccer.modify(|_, w| w.cc2np().bit(polarity.bit())),
                    Channel::Three => self.tim.ccer.modify(|_, w| w.cc3np().bit(polarity.bit())),
                    Channel::Four => self.tim.ccer.modify(|_, w| w.cc4np().bit(polarity.bit())),
                }
            }
            /// Disables capture compare on a specific channel.
            pub fn disable_capture_compare(&mut self, channel: Channel) {
                match channel {
                    Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().clear_bit()),
                    Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().clear_bit()),
                    Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().clear_bit()),
                    Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().clear_bit()),
                }
            }

            /// Enables capture compare on a specific channel.
            pub fn enable_capture_compare(&mut self, channel: Channel) {
                match channel {
                    Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().set_bit()),
                    Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().set_bit()),
                    Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().set_bit()),
                    Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().set_bit()),
                }
            }

            /// Set Capture Compare Mode. See docs on the `CaptureCompare` enum.
            pub fn set_capture_compare(&mut self, channel: Channel, mode: CaptureCompare) {
                match channel {
                    // Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
                    Channel::One => self
                        .tim
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc1s().bits(mode as u8) }),
                    Channel::Two => self
                        .tim
                        .ccmr1_output()
                        .modify(unsafe { |_, w| w.cc2s().bits(mode as u8) }),
                    Channel::Three => self
                        .tim
                        .ccmr2_output()
                        .modify(unsafe { |_, w| w.cc3s().bits(mode as u8) }),
                    Channel::Four => self
                        .tim
                        .ccmr2_output()
                        .modify(unsafe { |_, w| w.cc4s().bits(mode as u8) }),
                }
            }

            /// Set auto reload preloader; useful when changing period and duty mid-run.
            pub fn set_auto_reload_preload(&mut self, mode: bool) {
                self.tim.cr1.modify(|_, w| w.arpe().bit(mode));
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
            pub fn set_preload(&mut self, channel: Channel, value: bool) {
                match channel {
                    Channel::One => self.tim.ccmr1_output().modify(|_, w| w.oc1pe().bit(value)),
                    Channel::Two => self.tim.ccmr1_output().modify(|_, w| w.oc2pe().bit(value)),
                    Channel::Three => self.tim.ccmr2_output().modify(|_, w| w.oc3pe().bit(value)),
                    Channel::Four => self.tim.ccmr2_output().modify(|_, w| w.oc4pe().bit(value)),
                }

                // "As the preload registers are transferred to the shadow registers only when an update event
                // occurs, before starting the counter, you have to initialize all the registers by setting the UG
                // bit in the TIMx_EGR register."
                self.tim.egr.write(|w| w.ug().set_bit()); // Update
            }
        }
    };
}

// We only implement `pwm_features` for general purpose timers. Perhaps we should implement
// for advanced-control timers too.

#[cfg(not(any(feature = "f373")))]
hal!(TIM1, tim1, 2);
#[cfg(not(any(
    feature = "f373",
    feature = "f4",
    feature = "l4",
    feature = "l5",
    feature = "g0"
)))]
pwm_features!(TIM1, u16);

#[cfg(any(feature = "g0"))]
pwm_features!(TIM1, u32);

cfg_if! {
    if #[cfg(not(any(
        feature = "f410",
        feature = "g070",
    )))] {
        hal!(TIM2, tim2, 1);
    }
}

// todo: G4 has tim2, and it's 32-bit, but there may be a PAC error here; pac expects arr to be 16 bit.
#[cfg(feature = "g4")]
pwm_features!(TIM2, u16);

#[cfg(not(any(feature = "l5", feature = "g070", feature = "g4", feature = "f410")))]
pwm_features!(TIM2, u32);

#[cfg(not(any(feature = "f301", feature = "l4x1", feature = "l4x3", feature = "f410",)))]
hal!(TIM3, tim3, 1);

#[cfg(not(any(
    feature = "l4x1",
    feature = "l4x3",
    feature = "l5",
    feature = "f410",
    feature = "g0"
)))]
pwm_features!(TIM3, u16);

#[cfg(feature = "g0")]
pwm_features!(TIM3, u32);

cfg_if! {
    if #[cfg(not(any(
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4x1",
        feature = "l4x2",
        feature = "l4x3",
        feature = "l552",
        feature = "g0",
    )))] {
        hal!(TIM4, tim4, 1);
    }
}

cfg_if! {
    if #[cfg(not(any(
        feature = "f301",
        feature = "f3x4",
        feature = "f410",
        feature = "l4x1",
        feature = "l4x2",
        feature = "l4x3",
        feature = "l5",
        feature = "g0",
    )))] {
        pwm_features!(TIM4, u16);
    }
}

cfg_if! {
    if #[cfg(any(
       feature = "f373",
       feature = "l4x5",
       feature = "l4x6",
       feature = "l562",
       feature = "h7",
       all(feature = "f4", not(feature = "f410")),
   ))] {
        hal!(TIM5, tim5, 1);
   }
}

cfg_if! {
    if #[cfg(any(
       feature = "f373",
       feature = "l4x5",
       feature = "l4x6",
       feature = "h7",
       all(feature = "f4", not(feature = "f410")),
   ))] {
        pwm_features!(TIM5, u32);
   }
}

cfg_if! {
    if #[cfg(not(any(
        feature = "f401",
        feature = "f410",
        feature = "f411",
        feature = "g031",
        feature = "g041",
        feature = "g070",
        feature = "g030"
    )))] {
        hal!(TIM6, tim6, 1);
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
    feature = "g030"
)))]
hal!(TIM7, tim7, 1);

#[cfg(any(
    feature = "f303",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l562",
    feature = "g4"
))]
hal!(TIM8, tim8, 2);

// Todo: the L5 PAC has an address error on TIM15 - remove it until solved.
#[cfg(not(any(
    feature = "l5",
    feature = "f4",
    feature = "g031",
    feature = "g031",
    feature = "g041",
    feature = "g030"
)))]
hal!(TIM15, tim15, 2);

#[cfg(not(feature = "f4"))]
hal!(TIM16, tim16, 2);

cfg_if! {
    if #[cfg(not(any(
        feature = "l4x1",
        feature = "l4x2",
        feature = "l4x3",
        feature = "f4",
    )))] {
        hal!(TIM17, tim17, 2);
    }
}

// { todo: tim18
//     TIM18: (tim18, apb2, enr, rstr),
// },

cfg_if! {
    if #[cfg(any(feature = "f373"))] {
        hal!(TIM12, tim12, 1);
        hal!(TIM13, tim13, 1);
        hal!(TIM14, tim14, 1);
        hal!(TIM19, tim19, 2);
    }
}

#[cfg(any(feature = "f303"))]
hal!(TIM20, tim20, 2);
