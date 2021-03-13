// todo: this is missing features. eg the other PLLs etc.
// todo: Probably not in usable state yet; need to cross-check RM and clock tree.

use crate::{
    clocks::SpeedError,
    pac::{FLASH, RCC},
    traits::{ClockCfg, ClocksValid},
};
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Clk48Src {
    Hsi48 = 0b00, // Only falivd for STM32L49x/L4Ax
    PllSai1 = 0b01,
    Pll = 0b10,
    Csi = 0b11,
}

#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Csi(CsiRange),
    Hsi,
    Hse(u8),
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::None => 0b00,
            Self::Csi(_) => 0b01,
            Self::Hsi => 0b10,
            Self::Hse(_) => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Hsi,
    Csi(CsiRange),
    Hse(u8), // freq in Mhz,
    Pll1(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi => 0b000,
            Self::Csi(_) => 0b001,
            Self::Hse(_) => 0b010,
            Self::Pll1(_) => 0b011,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum CsiRange {
    Range0 = 0b0000,
    Range1 = 0b0001,
    Range2 = 0b0010,
    Range3 = 0b0011,
    Range4 = 0b0100,
    Range5 = 0b0101,
    Range6 = 0b0110,
    Range7 = 0b0111,
    Range8 = 0b1000,
    Range9 = 0b1001,
    Range10 = 0b1010,
    Range11 = 0b1011,
}

impl CsiRange {
    // Calculate the approximate frequency, in Hz.
    fn value(&self) -> u32 {
        match self {
            Self::Range0 => 100_000,
            Self::Range1 => 200_000,
            Self::Range2 => 400_000,
            Self::Range3 => 800_000,
            Self::Range4 => 1_000_000,
            Self::Range5 => 2_000_000,
            Self::Range6 => 4_000_000,
            Self::Range7 => 8_000_000,
            Self::Range8 => 16_000_000,
            Self::Range9 => 24_000_000,
            Self::Range10 => 32_000_000,
            Self::Range11 => 48_000_000,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2
pub enum Prediv {
    Div1 = 0b0000,
    Div2 = 0b0001,
    Div3 = 0b0010,
    Div4 = 0b0011,
    Div5 = 0b0100,
    Div6 = 0b0101,
    Div7 = 0b0110,
    Div8 = 0b0111,
}

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

/// Settings used to configure clocks.
/// Note that we use integers instead of enums for some of the scalers, unlike in
/// the other clock modules. This is due to the wide range available on these fields.
pub struct Clocks {
    pub input_src: InputSrc, //
    pub divm1: u8,
    pub divn1: u16,
    pub divp1: u8,
    pub d1_core_prescaler: HclkPrescaler,
    pub d1_prescaler: ApbPrescaler,
    pub ahb_prescaler: ApbPrescaler,
    pub d2_prescaler1: ApbPrescaler,
    pub d2_prescaler2: ApbPrescaler,
    pub d3_prescaler: ApbPrescaler,
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    pub clk48_src: Clk48Src,
    pub hse_bypass: bool,
    pub security_system: bool,
}

impl Clocks {
    /// Setup common and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    /// https://docs.rs/stm32f3xx-hal/0.5.0/stm32f3xx_hal/rcc/struct.CFGR.html
    /// Use the STM32CubeIDE Clock Configuration tab to help.
    pub fn setup(&self, rcc: &mut RCC, flash: &mut FLASH) -> Result<(), SpeedError> {
        if let ClocksValid::NotValid = self.validate_speeds() {
            return Err(SpeedError {});
        }

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        let (_, sysclk) = calc_sysclock(self.input_src, self.divm1, self.divn1, self.divp1);

        let hclk = sysclk / self.ahb_prescaler.value() as u32;
        // Reference manual section 3.3.3
        flash.acr.modify(|_, w| unsafe {
            if hclk <= 16 {
                w.latency().bits(0b000)
            } else if hclk <= 32 {
                w.latency().bits(0b001)
            } else if hclk <= 48 {
                w.latency().bits(0b010)
            } else if hclk <= 64 {
                w.latency().bits(0b011)
            } else {
                w.latency().bits(0b100)
            }
        });

        // todo: Look up and document PLL config.

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::Csi(range) => {
                // rcc.cr.modify(|_, w| unsafe {
                //     // todo?
                // }
                //     w.csirange()
                //         .bits(range as u8)
                //         .Csirgsel()
                //         .set_bit()
                //         .Csion()
                //         .set_bit()
                // });
                // Wait for the Csi to be ready.
                while rcc.cr.read().csirdy().bit_is_clear() {}
                // todo: If LSE is enabled, calibrate Csi.
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().bit(true));
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
            InputSrc::Pll1(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and Csi setup above.
                match pll_src {
                    PllSrc::Csi(range) => {
                        // todo?
                        // rcc.cr.modify(|_, w| unsafe {
                        //     w.csirange()
                        //         .bits(range as u8)
                        //         .Csirgsel()
                        //         .set_bit()
                        //         .Csion()
                        //         .set_bit()
                        // });
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().bit(true));
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        rcc.cr.modify(|_, w| w.hsion().bit(true));
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                    }
                    PllSrc::None => {}
                }
            }
        }

        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hsebyp().bit(self.hse_bypass)
        });

        if let InputSrc::Pll1(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pll1on().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pll1rdy().bit_is_set() {}

            // rcc.pllcfgr.modify(|_, w| { // todo?
            //
            // })

            rcc.pllckselr.modify(|_, w| w.divm1().bits(self.divm1));

            rcc.pll1divr.modify(|_, w| unsafe {
                w.divn1().bits(self.divn1);
                w.divp1().bits(self.divp1)
                // todo: divq1? r?
            });

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            // todo: Enable sai1 and 2 with separate settings, or lump in with mail PLL
            // like this?
            rcc.cr.modify(|_, w| w.pll1on().set_bit());
            while rcc.cr.read().pll1rdy().bit_is_clear() {}
        }

        rcc.cfgr
            .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

        rcc.d1cfgr.modify(|_, w| unsafe {
            w.d1cpre().bits(self.d1_core_prescaler as u8);
            w.d1ppre().bits(self.d1_prescaler as u8);
            w.hpre().bits(self.ahb_prescaler as u8)
        });

        rcc.d2cfgr.modify(|_, w| unsafe {
            w.d2ppre1().bits(self.d2_prescaler1 as u8);
            w.d2ppre2().bits(self.d2_prescaler2 as u8)
        });

        rcc.d3cfgr
            .modify(|_, w| unsafe { w.d3ppre().bits(self.d3_prescaler as u8) });

        // todo
        // rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        #[cfg(feature = "l4")]
        rcc.ccipr
            .modify(|_, w| unsafe { w.clk48sel().bits(self.clk48_src as u8) });

        #[cfg(feature = "l5")]
        rcc.ccipr1
            .modify(|_, w| unsafe { w.clk48msel().bits(self.clk48_src as u8) });

        // Enable the HSI48 as required, which is used for USB, RNG, etc.
        // Only valid for STM32L49x/L4Ax devices.
        // if let Clk48Src::Hsi48 = self.clk48_src { // todo?
        //     rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
        //     while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        // }

        Ok(())
    }

    /// This preset configures common with a HSI, a 80Mhz sysclck. All peripheral common are at
    /// 80Mhz.
    /// HSE output is not bypassed.
    pub fn hsi_preset() -> Self {
        Self {
            input_src: InputSrc::Pll1(PllSrc::Hsi),
            divm1: 32,
            divn1: 129,
            divp1: 2,
            d1_core_prescaler: HclkPrescaler::Div1,
            d1_prescaler: ApbPrescaler::Div1,
            ahb_prescaler: ApbPrescaler::Div1,
            d2_prescaler1: ApbPrescaler::Div1,
            d2_prescaler2: ApbPrescaler::Div1,
            d3_prescaler: ApbPrescaler::Div1,
            clk48_src: Clk48Src::PllSai1,
            hse_bypass: false,
            security_system: false,
        }
    }
}

// todo: Some extra calculations here, vice doing it once and caching.
// todo: This is all wrong; haven't adapted for H7. Fix it!
impl ClockCfg for Clocks {
    fn sysclk(&self) -> u32 {
        let (_, sysclk) = calc_sysclock(self.input_src, self.divm1, self.divn1, self.divp1);
        sysclk * 1_000_000
    }

    fn hclk(&self) -> u32 {
        self.sysclk() / self.d1_core_prescaler as u32 / self.ahb_prescaler as u32
    }

    fn systick(&self) -> u32 {
        // todo: There's an optional /8 divider we're not taking into account here.
        self.hclk()
    }

    fn usb(&self) -> u32 {
        // let (input_freq, _) = calc_sysclock(self.input_src, self.divm1, self.divn1, self.divp1);
        // (input_freq * 1_000_000) as u32 / self.divm1 as u32 * self.pll_sai1_mul as u32 / 2
        0 // todo
    }

    fn apb1(&self) -> u32 {
        self.hclk() / self.d2_prescaler1.value() as u32
    }

    fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.d2_prescaler1 {
            self.apb1() as u32
        } else {
            self.apb1() as u32 * 2
        }
    }

    fn apb2(&self) -> u32 {
        self.hclk() / self.d2_prescaler2.value() as u32
    }

    fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.d2_prescaler2 {
            self.apb2() as u32
        } else {
            self.apb2() as u32 * 2
        }
    }

    fn validate_speeds(&self) -> ClocksValid {
        let mut result = ClocksValid::Valid;

        // todo: This depends on variant
        let max_clock = 550_000_000;

        // todo: L4+ (ie R, S, P, Q) can go up to 120_000.

        // todo: Are these valid for all H7 configs?
        if self.divm1 > 63 || self.divn1 > 512 || self.divp1 > 128 {
            return ClocksValid::NotValid;
        }

        // todo: QC these limits
        // todo: Note that this involves repeatedly calculating sysclk.
        // todo. We could work around thsi by calcing it once here.
        if self.sysclk() > max_clock {
            result = ClocksValid::NotValid;
        }

        if self.hclk() > max_clock {
            result = ClocksValid::NotValid;
        }

        if self.apb1() > max_clock {
            result = ClocksValid::NotValid;
        }

        if self.apb2() > max_clock {
            result = ClocksValid::NotValid;
        }

        result
    }
}

impl Default for Clocks {
    /// This default configures common with a HSE, a 32Mhz sysclck. All peripheral common are at
    /// 32 Mhz.
    /// HSE output is not bypassed.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll1(PllSrc::Hse(8)),
            divm1: 32,
            divn1: 129,
            divp1: 1,
            d1_core_prescaler: HclkPrescaler::Div1,
            d1_prescaler: ApbPrescaler::Div1,
            ahb_prescaler: ApbPrescaler::Div1,
            d2_prescaler1: ApbPrescaler::Div1,
            d2_prescaler2: ApbPrescaler::Div1,
            d3_prescaler: ApbPrescaler::Div1,
            clk48_src: Clk48Src::PllSai1,
            hse_bypass: false,
            security_system: false,
        }
    }
}

/// Calculate the systick, and input frequency.
fn calc_sysclock(input_src: InputSrc, divm1: u8, divn1: u16, divp1: u8) -> (u32, u32) {
    let input_freq;
    let sysclk = match input_src {
        InputSrc::Pll1(pll_src) => {
            input_freq = match pll_src {
                PllSrc::Csi(range) => range.value() as u32 / 1_000_000,
                PllSrc::Hsi => 64,
                PllSrc::Hse(freq) => freq as u32,
                PllSrc::None => 0, // todo?
            };
            input_freq as u32 / divm1 as u32 * divn1 as u32 / divp1 as u32
        }

        InputSrc::Csi(range) => {
            input_freq = range.value() as u32 / 1_000_000;
            input_freq
        }
        InputSrc::Hsi => {
            input_freq = 64;
            input_freq
        }
        InputSrc::Hse(freq) => {
            input_freq = freq as u32;
            input_freq
        }
    };

    (input_freq, sysclk)
}

/// Re-select innput source; used on Stop and Standby modes, where the system reverts
/// to HSI after wake.
pub(crate) fn re_select_input(input_src: InputSrc, rcc: &mut RCC) {
    // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

    // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
    // todo: But this saves a few reg writes.
    match input_src {
        InputSrc::Hse(_) => {
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().bit_is_clear() {}

            rcc.cfgr
                .modify(|_, w| unsafe { w.sw().bits(input_src.bits()) });
        }
        InputSrc::Pll1(pll_src) => {
            // todo: DRY with above.
            match pll_src {
                PllSrc::Hse(_) => {
                    rcc.cr.modify(|_, w| w.hseon().set_bit());
                    while rcc.cr.read().hserdy().bit_is_clear() {}
                }
                PllSrc::Hsi => {
                    // Generally reverts to Csi (see note below)
                    rcc.cr.modify(|_, w| w.hsion().bit(true));
                    while rcc.cr.read().hsirdy().bit_is_clear() {}
                }
                PllSrc::Csi(_) => (), // Already reverted to this.
                PllSrc::None => (),
            }

            // todo: PLL 2 and 3?
            rcc.cr.modify(|_, w| w.pll1on().clear_bit());
            while rcc.cr.read().pll1rdy().bit_is_set() {}

            rcc.cfgr
                .modify(|_, w| unsafe { w.sw().bits(input_src.bits()) });

            rcc.cr.modify(|_, w| w.pll1on().set_bit());
            while rcc.cr.read().pll1rdy().bit_is_clear() {}
        }
        InputSrc::Hsi => {
            {
                // From Reference Manual, RCC_CFGR register section:
                // "Configured by HW to force Csi oscillator selection when exiting Standby or Shutdown mode.
                // Configured by HW to force Csi or HSI16 oscillator selection when exiting Stop mode or in
                // case of failure of the HSE oscillator, depending on STOPWUCK value."
                // In tests, from stop, it tends to revert to Csi.
                rcc.cr.modify(|_, w| w.hsion().bit(true));
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
        }
        InputSrc::Csi(_) => (), // Already reset to this
    }
}
