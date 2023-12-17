//! Support for the Real Time Clock (RTC) peripheral.
//! For more details, see
//! [ST AN4759](https:/www.st.com%2Fresource%2Fen%2Fapplication_note%2Fdm00226326-using-the-hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-stm32-microcontrollers-stmicroelectronics.pdf&usg=AOvVaw3PzvL2TfYtwS32fw-Uv37h)

//! Uses [Chrono](https://docs.rs/chrono) for dates and times.

use core::convert::TryInto;

use cfg_if::cfg_if;
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};

use crate::pac::{EXTI, PWR, RCC, RTC};

// todo: QC use of ICSR vice SR and ISR wherever used in this module!

/// RTC Clock source.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum RtcClockSource {
    /// 01: LSE oscillator clock used as RTC clock
    Lse = 0b01,
    /// 10: LSI oscillator clock used as RTC clock
    Lsi = 0b10,
    /// 11: HSE oscillator clock divided by 32 used as RTC clock
    Hse = 0b11,
}

/// RTC error type
#[derive(Debug)]
pub enum Error {
    /// Invalid input error
    InvalidInputData,
}

/// See ref man, section 27.6.3, or AN4769, section 2.4.2.
/// To be used with WakeupPrescaler
#[derive(Clone, Copy, Debug)]
enum WakeupDivision {
    Sixteen,
    Eight,
    Four,
    Two,
}

/// See AN4759, table 13.
#[derive(Clone, Copy, Debug)]
enum ClockConfig {
    One(WakeupDivision),
    Two,
    Three,
}

/// Interrupt event
pub enum Event {
    WakeupTimer,
    AlarmA,
    AlarmB,
    Timestamp,
}

pub enum Alarm {
    AlarmA,
    AlarmB,
}

impl From<Alarm> for Event {
    fn from(a: Alarm) -> Self {
        match a {
            Alarm::AlarmA => Event::AlarmA,
            Alarm::AlarmB => Event::AlarmB,
        }
    }
}

/// Represents a Real Time Clock (RTC) peripheral.
pub struct Rtc {
    /// RTC Peripheral register definition
    regs: RTC,
    config: RtcConfig,
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Configuration data for the RTC.
pub struct RtcConfig {
    /// RTC clock source. Defaults to LSI (Low speed internal oscillator)
    pub clock_source: RtcClockSource,
    /// Asynchronous prescaler factor
    /// This is the asynchronous division factor:
    /// ck_apre frequency = RTCCLK frequency/(PREDIV_A+1)
    /// ck_apre drives the subsecond register. Defaults to 127.
    pub async_prescaler: u8,
    /// Synchronous prescaler factor
    /// This is the synchronous division factor:
    /// ck_spre frequency = ck_apre frequency/(PREDIV_S+1)
    /// ck_spre must be 1Hz. Defaults to 255.
    pub sync_prescaler: u16,
    /// Bypass LSE output - eg if you're using a self-powered external oscillator. This
    /// saves power, and lets you use the LSE output pin as a GPIO.
    pub bypass_lse_output: bool,
}

impl Default for RtcConfig {
    /// LSI with prescalers assuming 32.768 kHz.
    /// Raw sub-seconds in 1/256.
    fn default() -> Self {
        RtcConfig {
            clock_source: RtcClockSource::Lsi,
            async_prescaler: 127,
            sync_prescaler: 255,
            bypass_lse_output: false,
        }
    }
}

impl Rtc {
    /// Initialize the RTC, including configuration register writes.
    pub fn new(regs: RTC, config: RtcConfig) -> Self {
        let mut result = Self { regs, config };

        // Enable the peripheral clock for communication
        // You must enable the `pwren()` bit before making RTC register writes, or they won't stay
        // set. Enable the backup interface by setting PWREN

        // Note that unlock other RCC enableing processes, there's no corresponding reset
        // field here.

        // See L4 RM, `Backup domain access` section.
        let rcc = unsafe { &(*RCC::ptr()) };
        let pwr = unsafe { &(*PWR::ptr()) };

        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4"))] {
                rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
                pwr.cr.read(); // read to allow the pwr clock to enable
                pwr.cr.modify(|_, w| w.dbp().set_bit());
                while pwr.cr.read().dbp().bit_is_clear() {}
            } else if #[cfg(any(feature = "l4", feature = "l5", feature = "g4", feature = "l412", feature = "wb", feature = "wl"))] {
                // 1. Enable the power interface clock by setting the PWREN bits in the Section 6.4.18:
                // APB1 peripheral clock enable register 1 (RCC_APB1ENR1)
                #[cfg(not(any(feature = "wb", feature = "wl")))]
                rcc.apb1enr1.modify(|_, w| {
                    w.pwren().set_bit();
                    w.rtcapben().set_bit()
                });
                #[cfg(any(feature = "wb", feature = "wl"))]
                rcc.apb1enr1.modify(|_, w| w.rtcapben().set_bit());

                rcc.apb1smenr1.modify(|_, w| w.rtcapbsmen().set_bit());  // In sleep and stop modes.
                pwr.cr1.read(); // Read to allow the pwr clock to enable
                // 2. Set the DBP bit in the Power control register 1 (PWR_CR1) to enable access to the
                // backup domain
                pwr.cr1.modify( | _, w| w.dbp().set_bit()); // Unlock the backup domain
                while pwr.cr1.read().dbp().bit_is_clear() {}
            } else if #[cfg(any(feature = "g0"))] {
                rcc.apbenr1.modify(|_, w| {
                    w.pwren().set_bit();
                    w.rtcapben().set_bit()
                });
                rcc.apbsmenr1.modify(|_, w| w.rtcapbsmen().set_bit());  // In sleep and stop modes.
                pwr.cr1.read();
                pwr.cr1.modify( | _, w| w.dbp().set_bit());
                while pwr.cr1.read().dbp().bit_is_clear() {}
            } else if #[cfg(feature = "h5")] {
                rcc.apb3enr.modify(|_, w| w.rtcapben().set_bit());
                rcc.apb3lpenr.modify(|_, w| w.rtcapblpen().set_bit());  // In sleep and stop modes.
                pwr.dbpcr.read(); // read to allow the pwr clock to enable // todo??
                pwr.dbpcr.modify( | _, w| w.dbp().set_bit());
                while pwr.dbpcr.read().dbp().bit_is_clear() {}
            } else { // eg h7
                rcc.apb4enr.modify(|_, w| w.rtcapben().set_bit());
                rcc.apb4lpenr.modify(|_, w| w.rtcapblpen().set_bit());  // In sleep and stop modes.
                pwr.cr1.read(); // read to allow the pwr clock to enable
                pwr.cr1.modify( | _, w| w.dbp().set_bit());
                while pwr.cr1.read().dbp().bit_is_clear() {}
            }
        }

        // Set up the LSI or LSE as required.
        match config.clock_source {
            RtcClockSource::Lsi => {
                cfg_if! {
                    if #[cfg(feature = "wb")] {
                    // todo: LSI2?
                        rcc.csr.modify(|_, w| w.lsi1on().set_bit());
                        while rcc.csr.read().lsi1rdy().bit_is_clear() {}
                    } else if #[cfg(feature = "h5")] {
                        rcc.bdcr.modify(|_, w| w.lsion().set_bit());
                        while rcc.bdcr.read().lsirdy().bit_is_clear() {}
                    } else {
                        rcc.csr.modify(|_, w| w.lsion().set_bit());
                        while rcc.csr.read().lsirdy().bit_is_clear() {}
                    }
                }
            }
            RtcClockSource::Lse => {
                // Can only set lsebyp when lse is off, so do this as a separate step.
                rcc.bdcr
                    .modify(|_, w| w.lsebyp().bit(config.bypass_lse_output));
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());
                while rcc.bdcr.read().lserdy().bit_is_clear() {}
            }
            _ => (),
        }

        rcc.bdcr.modify(|_, w| {
            // 3. Select the RTC clock source in the Backup domain control register (RCC_BDCR).
            unsafe { w.rtcsel().bits(result.config.clock_source as u8) };
            // 4. Enable the RTC clock by setting the RTCEN [15] bit in the Backup domain control
            // register (RCC_BDCR)
            w.rtcen().set_bit()
        });

        result.edit_regs(false, |regs| {
            regs.cr.modify(
                |_, w| {
                    unsafe {
                        w.fmt()
                            .clear_bit() // 24hr
                            .osel()
                            /*
                                00: Output disabled
                                01: Alarm A output enabled
                                10: Alarm B output enabled
                                11: Wakeup output enabled
                            */
                            .bits(0b00)
                            .pol()
                            .clear_bit()
                    }
                }, // pol high
            );

            regs.prer.modify(|_, w| unsafe {
                w.prediv_s().bits(config.sync_prescaler);
                w.prediv_a().bits(config.async_prescaler)
            });
        });

        result
    }

    /// Sets calendar clock to 24 hr format
    pub fn set_24h_fmt(&mut self) {
        self.edit_regs(true, |regs| regs.cr.modify(|_, w| w.fmt().clear_bit()));
    }

    /// Sets calendar clock to 12 hr format
    pub fn set_12h_fmt(&mut self) {
        self.edit_regs(true, |regs| regs.cr.modify(|_, w| w.fmt().set_bit()));
    }

    /// Reads current hour format selection
    pub fn is_24h_fmt(&self) -> bool {
        !self.regs.cr.read().fmt().bit()
    }

    // /// Setup the alarm. See AN4759, section 2.3.1.
    // /// `sleep_time` is in ms. `Table 8` desribes these steps.
    // pub fn set_alarm(&mut self, exti: &mut EXTI) {
    // note: STM3241x and 42x have diff addresses, and are PAC incompatible!
    //     exti.imr1.modify(|_, w| w.mr18().unmasked());
    //     exti.rtsr1.modify(|_, w| w.tr18().set_bit());
    //     exti.ftsr1.modify(|_, w| w.tr18().clear_bit());
    //
    //     self.edit_regs(false, |regs| {
    //         regs.cr.modify(|_, w| w.alrae().clear_bit());
    //
    //         while regs.cr.read().alrae().bit_is_set() {}
    //
    //         // todo: Set the alarm time. This function will be broken until this is accomplished.
    //         // self.regs.alrmar.modify(|_, w| unsafe {});
    //
    //         regs.cr.modify(|_, w| w.alrae().set_bit());
    //         while regs.cr.read().alrae().bit_is_clear() {}
    //     })
    // }

    /// Helper fn, to do the important bits of setting the interval, with
    /// the registers already unlocked.
    fn set_wakeup_interval_inner(&mut self, sleep_time: f32) {
        // Program the value into the wakeup timer
        // Set WUT[15:0] in RTC_WUTR register. For RTC3 the user must also program
        // WUTOCLR bits.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo check ref man register table

        // See notes reffed below about WUCKSEL. We choose one of 3 "modes" described in AN4759 based
        // on sleep time. If in the overlap area, choose the lower (more precise) mode.
        // These all assume a 1hz `ck_spre`.
        let lfe_freq = match self.config.clock_source {
            RtcClockSource::Lse => 32_768.,
            RtcClockSource::Lsi => 40_000.,
            RtcClockSource::Hse => 250_000., // Assuming 8Mhz HSE, which may not be the case
        };

        // sleep_time = (1/lfe_freq) * div * (wutr + 1)
        // res = 1/lfe_freq * div
        // sleep_time = res * WUTR = 1/lfe_freq * div * (wutr + 1)
        // wutr = sleep_time * lfe_freq / div - 1

        let clock_cfg;
        let wutr;

        if sleep_time >= 0.00012207 && sleep_time < 32. {
            let division;
            let div;
            if sleep_time < 4. {
                division = WakeupDivision::Two; // Resolution: 61.035µs
                div = 2.;
            } else if sleep_time < 8. {
                division = WakeupDivision::Four; // Resolution: 122.08µs
                div = 4.;
            } else if sleep_time < 16. {
                division = WakeupDivision::Eight; // Resolution: 244.141
                div = 8.;
            } else {
                division = WakeupDivision::Sixteen; // Resolution: 488.281
                div = 16.;
            }
            clock_cfg = ClockConfig::One(division);
            wutr = sleep_time * lfe_freq / div - 1.
        } else if sleep_time < 65_536. {
            // 32s to 18 hours (This mode goes 1s to 18 hours; we use Config1 for the overlap)
            clock_cfg = ClockConfig::Two;
            wutr = sleep_time; // This works out conveniently!
        } else if sleep_time < 131_072. {
            // 18 to 36 hours
            clock_cfg = ClockConfig::Three;
            wutr = sleep_time - 65_537.;
        } else {
            panic!("Wakeup period must be between 0122.07µs and 36 hours.")
        }

        self.regs
            .wutr
            .modify(|_, w| unsafe { w.wut().bits(wutr as u16) });

        // Select the desired clock source. Program WUCKSEL[2:0] bits in RTC_CR register.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo: Check register docs and see what to set here.

        // See AN4759, Table 13. RM, 27.3.6

        // When ck_spre frequency is 1Hz, this allows to achieve a wakeup time from 1 s to
        // around 36 hours with one-second resolution. This large programmable time range is
        // divided in 2 parts:
        // – from 1s to 18 hours when WUCKSEL [2:1] = 10
        // – and from around 18h to 36h when WUCKSEL[2:1] = 11. In this last case 216 is
        // added to the 16-bit counter current value.When the initialization sequence is
        // complete (see Programming the wakeup timer on page 781), the timer starts
        // counting down.When the wakeup function is enabled, the down-counting remains
        // active in low-power modes. In addition, when it reaches 0, the WUTF flag is set in
        // the RTC_ISR register, and the wakeup counter is automatically reloaded with its
        // reload value (RTC_WUTR register value).
        let word = match clock_cfg {
            ClockConfig::One(division) => match division {
                WakeupDivision::Sixteen => 0b000,
                WakeupDivision::Eight => 0b001,
                WakeupDivision::Four => 0b010,
                WakeupDivision::Two => 0b011,
            },
            // for 2 and 3, what does `x` mean in the docs? Best guess is it doesn't matter.
            ClockConfig::Two => 0b100,   // eg 1s to 18h.
            ClockConfig::Three => 0b110, // eg 18h to 36h
        };

        // 000: RTC/16 clock is selected
        // 001: RTC/8 clock is selected
        // 010: RTC/4 clock is selected
        // 011: RTC/2 clock is selected
        // 10x: ck_spre (usually 1 Hz) clock is selected
        // 11x: ck_spre (usually 1 Hz) clock is selected and 216 is added to the WUT counter value
        self.regs
            .cr
            .modify(|_, w| unsafe { w.wucksel().bits(word) });
    }

    #[cfg(not(feature = "f373"))]
    /// Setup periodic auto-wakeup interrupts. See ST AN4759, Table 11, and more broadly,
    /// section 2.4.1. See also reference manual, section 27.5.
    /// In addition to running this function, set up the interrupt handling function by
    /// adding the line `make_rtc_interrupt_handler!(RTC_WKUP);` somewhere in the body
    /// of your program.
    /// `sleep_time` is in ms.
    pub fn set_wakeup(&mut self, sleep_time: f32) {
        // Configure and enable the EXTI line corresponding to the Wakeup timer even in
        // interrupt mode and select the rising edge sensitivity.
        // Sleep time is in seconds.  See L4 RM, Table 47 to see that exti line 20 is the RTC wakeup
        // timer. This appears to be the case for all families.

        // L4 RM, 5.3.11: To wakeup from Stop mode with an RTC wakeup event, it is necessary to:
        // • Configure the EXTI Line 20 to be sensitive to rising edge
        // • Configure the RTC to generate the RTC alarm

        let exti = unsafe { &(*EXTI::ptr()) };

        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "l4"))] {
                exti.imr1.modify(|_, w| w.mr20().unmasked());
                exti.rtsr1.modify(|_, w| w.tr20().set_bit());
                exti.ftsr1.modify(|_, w| w.tr20().clear_bit());
            } else if #[cfg(feature = "f4")] {
                exti.imr.modify(|_, w| w.mr20().unmasked());
                exti.rtsr.modify(|_, w| w.tr20().set_bit());
                exti.ftsr.modify(|_, w| w.tr20().clear_bit());
            } else if #[cfg(feature = "g4")]{
                exti.imr1.modify(|_, w| w.im20().unmasked());
                exti.rtsr1.modify(|_, w| w.rt20().set_bit());
                exti.ftsr1.modify(|_, w| w.ft20().clear_bit());
            } else if #[cfg(any(feature = "l5", feature = "g0", feature = "wb", feature = "wl", feature = "h5"))] {
                // exti.imr1.modify(|_, w| w.mr20().unmasked());
                // exti.rtsr1.modify(|_, w| w.rt20().set_bit());
                // exti.ftsr1.modify(|_, w| w.ft20().clear_bit());

           } else if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
                exti.c1imr1.modify(|_, w| w.mr20().unmasked());
                exti.rtsr1.modify(|_, w| w.tr20().set_bit());
                exti.ftsr1.modify(|_, w| w.tr20().clear_bit());
           } else { // H7
                exti.cpuimr1.modify(|_, w| w.mr20().unmasked());
                exti.rtsr1.modify(|_, w| w.tr20().set_bit());
                exti.ftsr1.modify(|_, w| w.tr20().clear_bit());
            }
        }

        // We can't use the `edit_regs` abstraction here due to being unable to call a method
        // in the closure.
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });

        // Disable the wakeup timer. Clear WUTE bit in RTC_CR register
        self.regs.cr.modify(|_, w| w.wute().clear_bit());

        // Ensure access to Wakeup auto-reload counter and bits WUCKSEL[2:0] is allowed.
        // Poll WUTWF until it is set in RTC_ISR (RTC2)/RTC_ICSR (RTC3) (May not be avail on F3)
        cfg_if! {
            if #[cfg(any(feature = "l5", feature = "g0", feature = "g4", feature = "l412", feature = "wl", feature = "h5"))] {
                while self.regs.icsr.read().wutwf().bit_is_clear() {}
            } else {
                while self.regs.isr.read().wutwf().bit_is_clear() {}
            }
        }

        self.set_wakeup_interval_inner(sleep_time);
        // Re-enable the wakeup timer. Set WUTE bit in RTC_CR register.
        // The wakeup timer restarts counting down.
        self.regs.cr.modify(|_, w| w.wute().set_bit());

        // Enable the wakeup timer interrupt.
        self.regs.cr.modify(|_, w| w.wutie().set_bit());

        cfg_if! {
            if #[cfg(any(feature = "l412", feature = "l5", feature = "g0", feature = "g4", feature = "l412", feature = "wl", feature = "h5"))] {
                self.regs.scr.write(|w| w.cwutf().set_bit());
            } else {
                self.regs.isr.modify(|_, w| w.wutf().clear_bit());
            }
        }

        self.regs.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    /// Enable the wakeup timer.
    pub fn enable_wakeup(&mut self) {
        unsafe {
            self.regs.wpr.write(|w| w.bits(0xCA));
            self.regs.wpr.write(|w| w.bits(0x53));
            self.regs.cr.modify(|_, w| w.wute().set_bit());
            self.regs.wpr.write(|w| w.bits(0xFF));
        }
    }

    /// Disable the wakeup timer.
    pub fn disable_wakeup(&mut self) {
        unsafe {
            self.regs.wpr.write(|w| w.bits(0xCA));
            self.regs.wpr.write(|w| w.bits(0x53));
            self.regs.cr.modify(|_, w| w.wute().clear_bit());
            self.regs.wpr.write(|w| w.bits(0xFF));
        }
    }

    /// Change the sleep time for the auto wakeup, after it's been set up.
    /// Sleep time is in MS. Major DRY from `set_wakeup`.
    pub fn set_wakeup_interval(&mut self, sleep_time: f32) {
        // `sleep_time` is in seconds.
        // See comments in `set_auto_wakeup` for what these writes do.

        // We can't use the `edit_regs` abstraction here due to being unable to call a method
        // in the closure.
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });

        let started_enabled = self.regs.cr.read().wute().bit_is_set();
        if started_enabled {
            self.regs.cr.modify(|_, w| w.wute().clear_bit());
        }

        cfg_if! {
            if #[cfg(any(feature = "l5", feature = "g0", feature = "g4", feature = "l412", feature = "wl", feature = "h5"))] {
                while self.regs.icsr.read().wutwf().bit_is_clear() {}
            } else {
                while self.regs.isr.read().wutwf().bit_is_clear() {}
            }
        }

        self.set_wakeup_interval_inner(sleep_time);

        if started_enabled {
            self.regs.cr.modify(|_, w| w.wute().set_bit());
        }

        self.regs.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    /// Clears the wakeup flag. Must be cleared manually after every RTC wakeup.
    /// Alternatively, you could call this in the RTC wakeup interrupt handler.
    pub fn clear_wakeup_flag(&mut self) {
        self.edit_regs(false, |regs| {
            regs.cr.modify(|_, w| w.wute().clear_bit());

            cfg_if! {
                if #[cfg(any(feature = "l412", feature = "l5", feature = "g0", feature = "g4", feature = "l412", feature = "wl", feature = "h5"))] {
                    regs.scr.write(|w| w.cwutf().set_bit());
                } else {
                    // Note that we clear this by writing 0, which isn't
                    // the standard convention, eg in other families, and
                    // other peripherals.
                    regs.isr.modify(|_, w| w.wutf().clear_bit());
                }
            }

            regs.cr.modify(|_, w| w.wute().set_bit());
        });
    }

    /// this function is used to disable write protection when modifying an RTC register.
    /// It also optionally handles the additional step required to set a clock or calendar
    /// value.
    fn edit_regs<F>(&mut self, init_mode: bool, mut closure: F)
    where
        F: FnMut(&mut RTC),
    {
        // Disable write protection
        // This is safe, as we're only writin the correct and expected values.
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });

        // todo: L4 has ICSR and ISR regs. Maybe both for backwards compat?

        cfg_if! {
             if #[cfg(any(feature = "l5", feature = "g0", feature = "g4", feature = "l412", feature = "wl", feature = "h5"))] {
                 // Enter init mode if required. This is generally used to edit the clock or calendar,
                 // but not for initial enabling steps.
                 if init_mode && self.regs.icsr.read().initf().bit_is_clear() {
                     // are we already in init mode?
                     self.regs.icsr.modify(|_, w| w.init().set_bit());
                     while self.regs.icsr.read().initf().bit_is_clear() {} // wait to return to init state
                 }

                 // Edit the regs specified in the closure, now that they're writable.
                 closure(&mut self.regs);

                 if init_mode {
                     self.regs.icsr.modify(|_, w| w.init().clear_bit()); // Exits init mode
                     while self.regs.icsr.read().initf().bit_is_set() {}
                 }
            // } else if #[cfg(feature = "wl")] {
            //     if init_mode && self.regs.isr.read().initf().bit_is_clear() {
            //         self.regs.icsr.modify(|_, w| w.init().set_bit());
            //         while self.regs.icsr.read().initf().bit_is_clear() {} // wait to return to init state
            //     }

            //     closure(&mut self.regs);

            //     if init_mode {
            //         self.regs.icsr.modify(|_, w| w.init().clear_bit()); // Exits init mode
            //         while self.regs.sr.read().initf().bit_is_set() {}
            //     }
            } else {
                 if init_mode && self.regs.isr.read().initf().bit_is_clear() {
                     self.regs.isr.modify(|_, w| w.init().set_bit());
                     while self.regs.isr.read().initf().bit_is_clear() {} // wait to return to init state
                 }

                 closure(&mut self.regs);

                 if init_mode {
                     self.regs.isr.modify(|_, w| w.init().clear_bit()); // Exits init mode
                     while self.regs.isr.read().initf().bit_is_set() {}
                 }
             }
        }

        // Re-enable write protection.
        // This is safe, as the field accepts the full range of 8-bit values.
        self.regs.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    /// set time using NaiveTime (ISO 8601 time without timezone)
    /// Hour format is 24h
    pub fn set_time(&mut self, time: &NaiveTime) -> Result<(), Error> {
        self.set_24h_fmt();
        let (ht, hu) = bcd2_encode(time.hour())?;
        let (mnt, mnu) = bcd2_encode(time.minute())?;
        let (st, su) = bcd2_encode(time.second())?;

        self.edit_regs(true, |regs| {
            regs.tr.write(|w| unsafe {
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mnt);
                w.mnu().bits(mnu);
                w.st().bits(st);
                w.su().bits(su);
                w.pm().clear_bit()
            })
        });

        Ok(())
    }

    /// Set the seconds component of the RTC's current time.
    pub fn set_seconds(&mut self, seconds: u8) -> Result<(), Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (st, su) = bcd2_encode(seconds as u32)?;
        self.edit_regs(true, |regs| {
            regs.tr
                .modify(|_, w| unsafe { w.st().bits(st).su().bits(su) })
        });

        Ok(())
    }

    /// Set the minutes component of the RTC's current time.
    pub fn set_minutes(&mut self, minutes: u8) -> Result<(), Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (mnt, mnu) = bcd2_encode(minutes as u32)?;
        self.edit_regs(true, |regs| {
            regs.tr
                .modify(|_, w| unsafe { w.mnt().bits(mnt).mnu().bits(mnu) })
        });

        Ok(())
    }

    /// Set the hours component of the RTC's current time.
    pub fn set_hours(&mut self, hours: u8) -> Result<(), Error> {
        let (ht, hu) = bcd2_encode(hours as u32)?;

        self.edit_regs(true, |regs| {
            regs.tr
                .modify(|_, w| unsafe { w.ht().bits(ht).hu().bits(hu) })
        });

        Ok(())
    }

    /// Set the weekday component of the RTC's current date.
    pub fn set_weekday(&mut self, weekday: u8) -> Result<(), Error> {
        if !(1..=7).contains(&weekday) {
            return Err(Error::InvalidInputData);
        }
        self.edit_regs(true, |regs| {
            regs.dr.modify(|_, w| unsafe { w.wdu().bits(weekday) })
        });

        Ok(())
    }

    /// Set the day component of the RTC's current date.
    pub fn set_day(&mut self, day: u8) -> Result<(), Error> {
        if !(1..=31).contains(&day) {
            return Err(Error::InvalidInputData);
        }
        let (dt, du) = bcd2_encode(day as u32)?;
        self.edit_regs(true, |regs| {
            regs.dr
                .modify(unsafe { |_, w| w.dt().bits(dt).du().bits(du) })
        });

        Ok(())
    }

    /// Set the month component of the RTC's current date.
    pub fn set_month(&mut self, month: u8) -> Result<(), Error> {
        if !(1..=12).contains(&month) {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(month as u32)?;
        self.edit_regs(true, |regs| {
            regs.dr
                .modify(|_, w| unsafe { w.mt().bit(mt > 0).mu().bits(mu) })
        });

        Ok(())
    }

    /// Set the year component of the RTC's current date.
    pub fn set_year(&mut self, year: u16) -> Result<(), Error> {
        if !(1970..=2038).contains(&year) {
            // todo: Is this right?
            return Err(Error::InvalidInputData);
        }
        let (yt, yu) = bcd2_encode(year as u32 - 2_000)?;
        // todo RTC is 2000 based ? Not sure best way to handle this.
        self.edit_regs(true, |regs| {
            regs.dr
                .modify(|_, w| unsafe { w.yt().bits(yt).yu().bits(yu) })
        });

        Ok(())
    }

    /// Set the date using NaiveDate (ISO 8601 calendar date without timezone).
    /// WeekDay is set using the `set_weekday` method
    pub fn set_date(&mut self, date: &NaiveDate) -> Result<(), Error> {
        if date.year() < 1970 {
            // todo: Is this right?
            return Err(Error::InvalidInputData);
        }

        let (yt, yu) = bcd2_encode((date.year() - 2_000) as u32)?;
        let (mt, mu) = bcd2_encode(date.month())?;
        let (dt, du) = bcd2_encode(date.day())?;

        self.edit_regs(true, |regs| {
            regs.dr.write(|w| unsafe {
                w.dt().bits(dt);
                w.du().bits(du);
                w.mt().bit(mt > 0);
                w.mu().bits(mu);
                w.yt().bits(yt);
                w.yu().bits(yu)
            })
        });

        Ok(())
    }

    /// Set the current datetime.
    pub fn set_datetime(&mut self, date: &NaiveDateTime) -> Result<(), Error> {
        if date.year() < 1970 {
            // todo is this right?
            return Err(Error::InvalidInputData);
        }

        self.set_24h_fmt();
        let (yt, yu) = bcd2_encode((date.year() - 2_000) as u32)?;
        let (mt, mu) = bcd2_encode(date.month())?;
        let (dt, du) = bcd2_encode(date.day())?;

        let (ht, hu) = bcd2_encode(date.hour())?;
        let (mnt, mnu) = bcd2_encode(date.minute())?;
        let (st, su) = bcd2_encode(date.second())?;

        self.edit_regs(true, |regs| {
            regs.dr.write(|w| unsafe {
                w.dt().bits(dt);
                w.du().bits(du);
                w.mt().bit(mt > 0);
                w.mu().bits(mu);
                w.yt().bits(yt);
                w.yu().bits(yu)
            })
        });

        self.edit_regs(true, |regs| {
            regs.tr.write(|w| unsafe {
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mnt);
                w.mnu().bits(mnu);
                w.st().bits(st);
                w.su().bits(su);
                w.pm().clear_bit()
            })
        });

        Ok(())
    }

    /// Get the seconds component of the current time.
    pub fn get_seconds(&mut self) -> u8 {
        let tr = self.regs.tr.read();
        bcd2_decode(tr.st().bits(), tr.su().bits()) as u8
    }

    /// Get the minutes component of the current time.
    pub fn get_minutes(&mut self) -> u8 {
        let tr = self.regs.tr.read();
        bcd2_decode(tr.mnt().bits(), tr.mnu().bits()) as u8
    }

    /// Get the hours component of the current time.
    pub fn get_hours(&mut self) -> u8 {
        let tr = self.regs.tr.read();
        bcd2_decode(tr.ht().bits(), tr.hu().bits()) as u8
    }

    /// Get the current time.
    pub fn get_time(&mut self) -> NaiveTime {
        NaiveTime::from_hms_opt(
            self.get_hours().into(),
            self.get_minutes().into(),
            self.get_seconds().into(),
        )
        .unwrap()
    }

    /// Get the weekday component of the current date.
    pub fn get_weekday(&mut self) -> u8 {
        let dr = self.regs.dr.read();
        bcd2_decode(dr.wdu().bits(), 0x00) as u8
    }

    /// Get the day component of the current date.
    pub fn get_day(&mut self) -> u8 {
        let dr = self.regs.dr.read();
        bcd2_decode(dr.dt().bits(), dr.du().bits()) as u8
    }

    /// Get the month component of the current date.
    pub fn get_month(&mut self) -> u8 {
        let dr = self.regs.dr.read();
        let mt: u8 = if dr.mt().bit() { 1 } else { 0 };
        bcd2_decode(mt, dr.mu().bits()) as u8
    }

    /// Get the year component of the current date.
    pub fn get_year(&mut self) -> u16 {
        let dr = self.regs.dr.read();
        (bcd2_decode(dr.yt().bits(), dr.yu().bits()) + 2000) as u16
    }

    /// Get the current date.
    pub fn get_date(&mut self) -> NaiveDate {
        NaiveDate::from_ymd_opt(
            self.get_year().into(),
            self.get_month().into(),
            self.get_day().into(),
        )
        .unwrap()
    }

    /// Get the current datetime.
    pub fn get_datetime(&mut self) -> NaiveDateTime {
        NaiveDate::from_ymd_opt(
            self.get_year().into(),
            self.get_month().into(),
            self.get_day().into(),
        )
        .unwrap()
        .and_hms_opt(
            self.get_hours().into(),
            self.get_minutes().into(),
            self.get_seconds().into(),
        )
        .unwrap()
    }
}

// Two 32-bit registers (RTC_TR and RTC_DR) contain the seconds, minutes, hours (12- or 24-hour format), day (day
// of week), date (day of month), month, and year, expressed in binary coded decimal format
// (BCD). The sub-seconds value is also available in binary format.
//
// The following helper functions encode into BCD format from integer and
// decode to an integer from a BCD value respectively.
fn bcd2_encode(word: u32) -> Result<(u8, u8), Error> {
    let l = match (word / 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };
    let r = match (word % 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };

    Ok((l, r))
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    (fst * 10 + snd).into()
}
