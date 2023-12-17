use cfg_if::cfg_if;

use crate::{
    clocks::RccError,
    pac::{self, FLASH, RCC},
    util::rcc_en_reset,
};

cfg_if! {
   if #[cfg(feature = "f3")] {
       #[derive(Clone, Copy)]
        /// The clocks source input used by the PLL.
        /// Note that this corresponds to Bits 16:15: Applicable only to some models,
        ///303xB/C etc use only bit 16, with bit 15 at reset value (0?) but it's equiv. 303xD/E and xE use bits 16:15.
        pub enum PllSrc {
            // todo: This is wrong for F303 xD and xE! These have 2 additional fields,
            // todo and setting HSE here will actually use HSI, but without the div2!
            HsiDiv2,
            Hse(u32),  // Freq in Hz
        }

        impl PllSrc {
            /// Required instead of u8 repr due to numerical value on non-uniform discrim being experimental.
            /// (ie, can't set on `Pll(Pllsrc)`.
            fn bits(&self) -> u8 {
                match self {
                    Self::HsiDiv2 => 0,
                    Self::Hse(_) => 1,
                }
            }
       }

       // #[cfg(feature = "f303xE")]
       // pub enum PllSrc {
       //      HsiDiv2,
       //      Hsi,
       //      Hse(u32),  // Freq in Hz
       //  }
       //
       // #[cfg(feature = "f303xE")]
       // impl PllSrc {
       //     /// Required instead of u8 repr due to numerical value on non-uniform discrim being experimental.
       //     /// (ie, can't set on `Pll(Pllsrc)`.
       //     fn bits(&self) -> u8 {
       //         match self {
       //             Self::HsiDiv2 => 0b00,
       //             Self::Hsi(_) => 0b01,
       //             Self::Hse(_) => 0b10,
       //         }
       //     }
       // }

   } else if #[cfg(feature = "f4")] {
           #[derive(Clone, Copy)]
            /// The clocks source input used by the PLL.
            pub enum PllSrc {
                Hsi,
                Hse(u32),
            }

            impl PllSrc {
                /// Required instead of u8 repr due to numerical value on non-uniform discrim being experimental.
                /// (ie, can't set on `Pll(Pllsrc)`.
                fn bits(&self) -> u8 {
                    match self {
                        Self::Hsi => 0,
                        Self::Hse(_) => 1,
                    }
                }
           }
   }
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    Hsi,
    Hse(u32), // freq in Mhz
    Pll(PllSrc),
    // #[cfg(feature = "f4")]
    // Pllr(PllSrc), // Not available on all variants.
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi => 0b00,
            Self::Hse(_) => 0b01,
            Self::Pll(_) => 0b10,
            // #[cfg(feature = "f4")]
            // Self::Pllr(_) => 0b11,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2. Scales the input source before the PLL.
pub enum Prediv {
    Div1 = 0b0000,
    Div2 = 0b0001,
    Div3 = 0b0010,
    Div4 = 0b0011,
    Div5 = 0b0100,
    Div6 = 0b0101,
    Div7 = 0b0110,
    Div8 = 0b0111,
    Div9 = 0b1000,
    Div10 = 0b1001,
    Div11 = 0b1010,
    Div12 = 0b1011,
    Div13 = 0b1100,
    Div14 = 0b1101,
    Div15 = 0b1110,
    Div16 = 0b1111,
}

#[cfg(feature = "f3")]
impl Prediv {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
            Self::Div5 => 5,
            Self::Div6 => 6,
            Self::Div7 => 7,
            Self::Div8 => 8,
            Self::Div9 => 9,
            Self::Div10 => 10,
            Self::Div11 => 11,
            Self::Div12 => 12,
            Self::Div13 => 13,
            Self::Div14 => 14,
            Self::Div15 => 15,
            Self::Div16 => 16,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PllMul {
    Mul2 = 0b0000,
    Mul3 = 0b0001,
    Mul4 = 0b0010,
    Mul5 = 0b0011,
    Mul6 = 0b0100,
    Mul7 = 0b0101,
    Mul8 = 0b0110,
    Mul9 = 0b0111,
    Mul10 = 0b1000,
    Mul11 = 0b1001,
    Mul12 = 0b1010,
    Mul13 = 0b1011,
    Mul14 = 0b1100,
    Mul15 = 0b1101,
    Mul16 = 0b1110,
}

#[cfg(feature = "f3")]
impl PllMul {
    pub fn value(&self) -> u8 {
        match self {
            Self::Mul2 => 2,
            Self::Mul3 => 3,
            Self::Mul4 => 4,
            Self::Mul5 => 5,
            Self::Mul6 => 6,
            Self::Mul7 => 7,
            Self::Mul8 => 8,
            Self::Mul9 => 9,
            Self::Mul10 => 10,
            Self::Mul11 => 11,
            Self::Mul12 => 12,
            Self::Mul13 => 13,
            Self::Mul14 => 14,
            Self::Mul15 => 15,
            Self::Mul16 => 16,
        }
    }
}

#[cfg(feature = "f4")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Pllp {
    Div2 = 0b00,
    Div4 = 0b01,
    Div6 = 0b10,
    Div8 = 0b11,
}

#[cfg(feature = "f4")]
impl Pllp {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div6 => 6,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Division factor for the AHB clock. Also known as AHB Prescaler.
pub enum HclkPrescaler {
    Div1 = 0b0000,
    Div2 = 0b1000,
    Div4 = 0b1001,
    Div8 = 0b1010,
    Div16 = 0b1011,
    Div64 = 0b1100,
    Div128 = 0b1101,
    Div256 = 0b1110,
    Div512 = 0b1111,
}

impl HclkPrescaler {
    pub fn value(&self) -> u16 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
            Self::Div512 => 512,
        }
    }
}

// f3 uses 0 - 2 only. F4 uses up to 7.
#[derive(Clone, Copy)]
#[repr(u8)]
/// Represents Flash wait states in the FLASH_ACR register.
enum WaitState {
    W0 = 0,
    W1 = 1,
    W2 = 2,
    #[cfg(feature = "f4")]
    W3 = 3,
    #[cfg(feature = "f4")]
    W4 = 4,
    #[cfg(feature = "f4")]
    W5 = 5,
    #[cfg(feature = "f4")]
    W6 = 6,
    #[cfg(feature = "f4")]
    W7 = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// For use with `RCC_APBPPRE1`, and `RCC_APBPPRE2`. Ie, low-speed and high-speed prescalers respectively.
pub enum ApbPrescaler {
    Div1 = 0b000,
    Div2 = 0b100,
    Div4 = 0b101,
    Div8 = 0b110,
    Div16 = 0b111,
}

impl ApbPrescaler {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UsbPrescaler {
    Div1_5 = 0,
    Div1 = 1,
}

#[cfg(feature = "f3")]
impl UsbPrescaler {
    // Can't pass u8 to the single-bit field in sv2rust; need bool.
    pub fn bit(&self) -> bool {
        match self {
            Self::Div1_5 => false,
            Self::Div1 => true,
        }
    }

    pub fn value(&self) -> f32 {
        match self {
            Self::Div1_5 => 1.5,
            Self::Div1 => 1.,
        }
    }
}

#[cfg(feature = "f4")]
#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2. Scales the input source before the PLL.
pub enum Pllq {
    Div2 = 0b0010,
    Div3 = 0b0011,
    Div4 = 0b0100,
    Div5 = 0b0101,
    Div6 = 0b0110,
    Div7 = 0b0111,
    Div8 = 0b1000,
    Div9 = 0b1001,
    Div10 = 0b1010,
    Div11 = 0b1011,
    Div12 = 0b1100,
    Div13 = 0b1101,
    Div14 = 0b1110,
    Div15 = 0b1111,
}

#[cfg(feature = "f4")]
impl Pllq {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
            Self::Div5 => 5,
            Self::Div6 => 6,
            Self::Div7 => 7,
            Self::Div8 => 8,
            Self::Div9 => 9,
            Self::Div10 => 10,
            Self::Div11 => 11,
            Self::Div12 => 12,
            Self::Div13 => 13,
            Self::Div14 => 14,
            Self::Div15 => 15,
        }
    }
}

/// Settings used to configure clocks. Create this struct by using its `Default::default()`
/// implementation, then modify as required, referencing your RM's clock tree,
/// or Stm32Cube IDE's interactive clock manager. Apply settings by running `.setup()`.
pub struct Clocks {
    /// The input source for the system and peripheral clocks. Eg HSE, HSI, PLL etc
    pub input_src: InputSrc,

    #[cfg(feature = "f3")]
    /// Input source predivision, for PLL
    pub prediv: Prediv,
    #[cfg(feature = "f3")]
    pub pll_mul: PllMul,

    #[cfg(feature = "f4")]
    pub pllm: u8,
    #[cfg(feature = "f4")]
    pub plln: u16,
    #[cfg(feature = "f4")]
    pub pllp: Pllp,

    #[cfg(feature = "f4")]
    pub pllq: Pllq, // USB prescaler, for target of 48Mhz.
    #[cfg(feature = "f3")]
    pub usb_pre: UsbPrescaler, // USB prescaler, for target of 48Mhz.
    /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
    pub hclk_prescaler: HclkPrescaler,
    /// The divider of HCLK to get the APB1 peripheral clock
    pub apb1_prescaler: ApbPrescaler,
    /// The divider of HCLK to get the APB2 peripheral clock
    pub apb2_prescaler: ApbPrescaler,
    /// Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    /// frees up the pin for use as GPIO.
    pub hse_bypass: bool,
    pub security_system: bool,
}

impl Clocks {
    /// Setup common and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    /// https://docs.rs/stm32f3xx-hal/0.5.0/stm32f3xx_hal/rcc/struct.CFGR.html
    /// Use the STM32CubeIDE Clock Configuration tab to help.
    pub fn setup(&self) -> Result<(), RccError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        let rcc = unsafe { &(*RCC::ptr()) };
        let flash = unsafe { &(*FLASH::ptr()) };

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        let sysclk = self.sysclk();

        // todo: We don't yet take into account other voltage settings for f4 wait states.
        let hclk = sysclk / self.hclk_prescaler.value() as u32;
        cfg_if! {
            if #[cfg(feature = "f3")] {  // RM section 4.5.1
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 24_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 48_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                });
            } else {  // F4
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 30_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 60_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 90_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 120_000_000 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else if hclk <= 150_000_000 {
                        w.latency().bits(WaitState::W4 as u8)
                    } else {
                        w.latency().bits(WaitState::W5 as u8)
                    }
                });
            }
        }

        // 303 RM, 9.2.3:
        // The internal PLL can be used to multiply the HSI or HSE output clock frequency. Refer to
        // Figure 13 and Clock control register (RCC_CR).
        // The PLL configuration (selection of the input clock, and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLON to 0.
        // 2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLON to 1.
        // An interrupt can be generated when the PLL is ready, if enabled in the Clock interrupt
        // register (RCC_CIR).
        // The PLL output frequency must be set in the range 16-72 MHz.
        // Set up the HSE if required.

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().is_not_ready() {}
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().bit(true));
                while rcc.cr.read().hsirdy().is_not_ready() {}
            }
            InputSrc::Pll(pll_src) => {
                match pll_src {
                    PllSrc::Hse(_) => {
                        // DRY
                        rcc.cr.modify(|_, w| w.hseon().bit(true));
                        while rcc.cr.read().hserdy().is_not_ready() {}
                    }
                    _ => {
                        // Hsi or HsiDiv2: In both cases, set up the HSI.
                        rcc.cr.modify(|_, w| w.hsion().bit(true));
                        while rcc.cr.read().hsirdy().is_not_ready() {}
                    }
                }
            }
        }
        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hsebyp().bit(self.hse_bypass)
        });

        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pllon().off());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pllrdy().is_ready() {}

            cfg_if! {
                if #[cfg(feature = "f3")] {
                   rcc.cfgr.modify(|_, w| {
                    // Some f3 varients uses a 'bit' field instead. Haven't looked up how to handle.
                    cfg_if! {
                        if #[cfg(any(feature = "f301", feature = "f373", feature = "f3x4"))] {
                            w.pllmul().bits(self.pll_mul as u8)
                        } else {
                            w.pllmul().bits(self.pll_mul as u8);
                            unsafe { w.pllsrc().bits(pll_src.bits()) } // eg: Set HSE as PREDIV1 entry.
                        }
                    }
                });
                } else if #[cfg(feature = "f4")] {
                    rcc.pllcfgr.modify(|_, w| unsafe {
                        w.pllsrc().bit(pll_src.bits() != 0);
                        w.plln().bits(self.plln);
                        w.pllm().bits(self.pllm);
                        w.pllp().bits(self.pllp as u8)
                    });
                }
            }

            #[cfg(feature = "f3")]
            rcc.cfgr2.modify(|_, w| w.prediv().bits(self.prediv as u8));

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            rcc.cr.modify(|_, w| w.pllon().on());

            while rcc.cr.read().pllrdy().is_not_ready() {}
        }

        rcc.cfgr.modify(|_, w| unsafe {
            #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f4")))]
            w.usbpre().bit(self.usb_pre.bit()); // eg: Divide by 1.5: 72/1.5 = 48Mhz, required by USB clock.

            w.sw().bits(self.input_src.bits());
            w.hpre().bits(self.hclk_prescaler as u8); // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
            w.ppre2().bits(self.apb2_prescaler as u8); // HCLK division for APB2.
            w.ppre1().bits(self.apb1_prescaler as u8) // HCLK division for APB1
        });

        rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        // If we're not using the default clock source as input source or for PLL, turn it off.
        match self.input_src {
            InputSrc::Hsi => (),
            InputSrc::Pll(pll_src) => match pll_src {
                #[cfg(feature = "f3")]
                PllSrc::HsiDiv2 => (),
                #[cfg(feature = "f4")]
                PllSrc::Hsi => (),
                _ => {
                    rcc.cr.modify(|_, w| w.hsion().clear_bit());
                }
            },
            _ => {
                rcc.cr.modify(|_, w| w.hsion().clear_bit());
            }
        }

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        rcc_en_reset!(apb2, syscfg, rcc);

        Ok(())
    }

    /// Re-select innput source; used on Stop and Standby modes, where the system reverts
    /// to HSI after wake.
    pub fn reselect_input(&self) {
        let rcc = unsafe { &(*RCC::ptr()) };
        // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

        // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
        // todo: But this saves a few reg writes.
        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().is_not_ready() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll(_) => {
                // todo: DRY with above.
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().is_not_ready() {}

                rcc.cr.modify(|_, w| w.pllon().off());
                while rcc.cr.read().pllrdy().is_ready() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pllon().on());
                while rcc.cr.read().pllrdy().is_not_ready() {}
            }
            InputSrc::Hsi => (), // Already reset to this.
        }
    }

    #[cfg(feature = "f3")]
    /// Calculate the sysclock frequency, in  Hz.
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll(pll_src) => match pll_src {
                PllSrc::HsiDiv2 => 4_000_000 * self.pll_mul.value() as u32,
                PllSrc::Hse(freq) => {
                    freq / self.prediv.value() as u32 * self.pll_mul.value() as u32
                }
            },
            InputSrc::Hsi => 8_000_000,
            InputSrc::Hse(freq) => freq,
        }
    }

    #[cfg(feature = "f4")]
    /// Calculate the sysclock frequency, in  Hz.
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Hsi => 16_000_000,
            InputSrc::Hse(freq) => freq,
            InputSrc::Pll(pll_src) => {
                let input_freq = match pll_src {
                    PllSrc::Hsi => 16_000_000,
                    PllSrc::Hse(freq) => freq,
                };
                input_freq / self.pllm as u32 * self.plln as u32 / self.pllp.value() as u32
            }
        }
    }

    /// Check if the PLL is enabled. This is useful if checking wheather to re-enable the PLL
    /// after exiting Stop or Standby modes, eg so you don't re-enable if it was already re-enabled
    /// in a different context. eg:
    /// ```
    /// if !clock_cfg.pll_is_enabled() {
    ///     clock_cfg.reselect_input();
    ///}
    ///```
    pub fn pll_is_enabled(&self) -> bool {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.cr.read().pllon().bit_is_set()
    }

    pub fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    pub fn systick(&self) -> u32 {
        self.hclk()
    }

    pub fn usb(&self) -> u32 {
        #[cfg(feature = "f3")]
        return self.sysclk() / self.usb_pre.value() as u32;
        #[cfg(feature = "f4")]
        return 0; // todo
    }

    pub fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    pub fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    pub fn apb2(&self) -> u32 {
        self.hclk() / self.apb2_prescaler.value() as u32
    }

    pub fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb2_prescaler {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    pub fn validate_speeds(&self) -> Result<(), RccError> {
        cfg_if! {
            if #[cfg(feature = "f3")] {
                let max_clock = 72_000_000;
            } else if #[cfg(feature = "f401")] {
                let max_clock = 84_000_000;
            } else if #[cfg(any(feature = "f410", feature = "f411", feature = "f412", feature = "f413"))] {
                let max_clock = 110_000_000;
            } else if #[cfg(any(feature = "f405", feature = "f407"))] {
                let max_clock = 168_000_000;
            } else {
                let max_clock = 180_000_000;
            }
        }

        #[cfg(feature = "f4")]
        if self.plln < 50 || self.plln > 432 || self.pllm < 2 || self.pllm > 63 {
            return Err(RccError::Speed);
        }

        let max_hclk = max_clock;

        // todo: min clock? eg for apxb?
        if self.sysclk() > max_hclk {
            return Err(RccError::Speed);
        }

        if self.hclk() > max_hclk {
            return Err(RccError::Speed);
        }

        if self.apb1() > max_clock {
            return Err(RccError::Speed);
        }

        if self.apb2() > max_clock {
            return Err(RccError::Speed);
        }

        Ok(())
    }
}

impl Default for Clocks {
    #[cfg(feature = "f3")]
    /// This default configures common with a HSI, a 64Mhz sysclck. All peripheral common are at
    /// 64Mhz, except for APB1, which is at and 32Mhz. Not valid for USB.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::HsiDiv2),
            prediv: Prediv::Div1,
            pll_mul: PllMul::Mul16,
            usb_pre: UsbPrescaler::Div1,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div2,
            apb2_prescaler: ApbPrescaler::Div1,
            hse_bypass: false,
            security_system: false,
        }
    }

    #[cfg(feature = "f4")]
    /// This preset configures clocks with a HSI. Configures a system clock based on variant:
    /// F401: 84Mhz
    /// F410, 411, 412, 413: 100Mhz
    /// F405, F407: 168Mhz
    /// Else: 180Mhz.
    /// Not valid for USB.
    fn default() -> Self {
        cfg_if! {
            if #[cfg(feature = "f401")] {
                let plln = 84;
            } else if #[cfg(any(feature = "f410", feature = "f411", feature = "f412", feature = "f413"))] {
                let plln = 100;
            } else if #[cfg(any(feature = "f405", feature = "f407"))] {
                let plln = 168;
            } else {
                let plln = 180;
            }
        }

        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pllm: 8,
            plln,
            pllp: Pllp::Div2,
            pllq: Pllq::Div8, // Note that this produces an invalid USB speed.
            hclk_prescaler: HclkPrescaler::Div1,
            #[cfg(any(feature = "f401", feature = "f410", feature = "f411"))]
            apb1_prescaler: ApbPrescaler::Div2,
            #[cfg(not(any(feature = "f401", feature = "f410", feature = "f411")))]
            apb1_prescaler: ApbPrescaler::Div4,
            #[cfg(any(feature = "f401", feature = "f410", feature = "f411"))]
            apb2_prescaler: ApbPrescaler::Div1,
            #[cfg(not(any(feature = "f401", feature = "f410", feature = "f411")))]
            apb2_prescaler: ApbPrescaler::Div2,
            hse_bypass: false,
            security_system: false,
        }
    }
}
