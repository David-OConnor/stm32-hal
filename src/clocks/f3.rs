use crate::{
    clocks::SpeedError,
    pac::{FLASH, RCC},
    traits::{ClockCfg, ClocksValid}
};

#[derive(Clone, Copy)]
/// Note that this corresponds to Bits 16:15: Applicable only to some models,
///303xB/C etc use only bit 16, with bit 15 at reset value (0?) but it's equiv. 303xD/E and xE use bits 16:15.
pub enum PllSrc {
    HsiDiv2,
    Hsi,
    Hse(u8),
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::HsiDiv2 => 0b00,
            Self::Hsi => 0b01,
            Self::Hse(_) => 0b10,
        }
    }
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    Hsi,
    Hse(u8), // freq in Mhz
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi => 0b00,
            Self::Hse(_) => 0b01,
            Self::Pll(_) => 0b10,
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
    Div9 = 0b1000,
    Div10 = 0b1001,
    Div11 = 0b1010,
    Div12 = 0b1011,
    Div13 = 0b1100,
    Div14 = 0b1101,
    Div15 = 0b1110,
    Div16 = 0b1111,
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

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UsbPrescaler {
    Div1_5 = 0,
    Div1 = 1,
}

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

/// Settings used to configure common
pub struct Clocks {
    pub input_src: InputSrc,           //
    pub prediv: Prediv,                // Input source predivision, for PLL.
    pub pll_mul: PllMul,               // PLL multiplier: SYSCLK speed is input source × this value.
    pub usb_pre: UsbPrescaler,         // USB prescaler, for target of 48Mhz.
    pub hclk_prescaler: HclkPrescaler, // The AHB clock divider.
    pub apb1_prescaler: ApbPrescaler,  // APB1 divider, for the low speed peripheral bus.
    pub apb2_prescaler: ApbPrescaler,  // APB2 divider, for the high speed peripheral bus.
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
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
        let sysclk = calc_sysclock(self.input_src, self.prediv, self.pll_mul);

        let hclk = sysclk / self.hclk_prescaler.value() as f32;
        // f3 ref man section 4.5.1.
        flash.acr.modify(|_, w| {
            if hclk <= 24. {
                w.latency().ws0()
            } else if hclk <= 48. {
                w.latency().ws1()
            } else {
                w.latency().ws2()
            }
        });

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

            rcc.cfgr.modify(|_, w| {
                w.pllmul().bits(self.pll_mul as u8); // eg: 8Mhz HSE x 9 = 72Mhz
                unsafe { w.pllsrc().bits(pll_src.bits()) } // eg: Set HSE as PREDIV1 entry.
            });

            rcc.cfgr2.modify(|_, w| w.prediv().bits(self.prediv as u8));

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            rcc.cr.modify(|_, w| w.pllon().on());

            while rcc.cr.read().pllrdy().is_not_ready() {}
        }

        rcc.cfgr.modify(|_, w| {
            w.usbpre().bit(self.usb_pre.bit()); // eg: Divide by 1.5: 72/1.5 = 48Mhz, required by USB clock.

            unsafe { w.sw().bits(self.input_src.bits()) };
            unsafe { w.hpre().bits(self.hclk_prescaler as u8) }; // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
            unsafe { w.ppre2().bits(self.apb2_prescaler as u8) }; // HCLK division for APB2.
            unsafe { w.ppre1().bits(self.apb1_prescaler as u8) } // HCLK division for APB1
        });

        rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        Ok(())
    }

    /// This preset configures common with a HSE, a 48Mhz sysclck. All peripheral common are at
    /// 48Mhz, except for APB1, which is at and 24Mhz. USB is set to 48Mhz.
    /// HSE output is not bypassed.
    pub fn hsi_preset() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::HsiDiv2),
            prediv: Prediv::Div1,
            pll_mul: PllMul::Mul12,
            usb_pre: UsbPrescaler::Div1,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div2,
            apb2_prescaler: ApbPrescaler::Div1,
            hse_bypass: false,
            security_system: false,
        }
    }
}

impl ClockCfg for Clocks {
    fn sysclk(&self) -> u32 {
        (calc_sysclock(self.input_src, self.prediv, self.pll_mul) * 1_000_000.) as u32
    }

    fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    fn systick(&self) -> u32 {
        self.hclk()
    }

    fn usb(&self) -> u32 {
        self.sysclk() / self.usb_pre.value() as u32
    }

    fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1() as u32
        } else {
            self.apb1() as u32 * 2
        }
    }

    fn apb2(&self) -> u32 {
        self.hclk() / self.apb2_prescaler.value() as u32
    }

    fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb2_prescaler {
            self.apb2() as u32
        } else {
            self.apb2() as u32 * 2
        }
    }

    fn validate_speeds(&self) -> ClocksValid {
        let mut result = ClocksValid::Valid;

        // todo: QC these limits
        if self.sysclk() > 72_000_000 || self.sysclk() < 16_000_000 {
            result = ClocksValid::NotValid;
        }

        if self.hclk() > 72_000_000 {
            result = ClocksValid::NotValid;
        }

        if self.apb1() > 36_000_000 || self.apb1() < 10_000_000 {
            result = ClocksValid::NotValid;
        }

        if self.apb2() > 72_000_000 {
            result = ClocksValid::NotValid;
        }

        result
    }
}

impl Default for Clocks {
    /// This default configures common with a HSE, a 72Mhz sysclck. All peripheral common are at
    /// 72 Mhz, except for APB1, which is at and 36Mhz. USB is set to 48Mhz.
    /// HSE output is not bypassed.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hse(8)),
            prediv: Prediv::Div1,
            pll_mul: PllMul::Mul9,
            usb_pre: UsbPrescaler::Div1_5,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div2,
            apb2_prescaler: ApbPrescaler::Div1,
            hse_bypass: false,
            security_system: false,
        }
    }
}

/// Calculate the systick, and input frequency.
fn calc_sysclock(input_src: InputSrc, prediv: Prediv, pll_mul: PllMul) -> f32 {
    let sysclk = match input_src {
        InputSrc::Pll(pll_src) => {
            let input_freq = match pll_src {
                PllSrc::Hsi => 8,
                PllSrc::HsiDiv2 => 4,
                PllSrc::Hse(freq) => freq,
            };
            input_freq as f32 / prediv.value() as f32 * pll_mul.value() as f32
        }
        InputSrc::Hsi => 8.,
        InputSrc::Hse(freq) => freq as f32,
    };

    sysclk
}

// todo: make `re_select_input` a method of

/// Re-select innput source; used on Stop and Standby modes, where the system reverts
/// to HSI after wake.
pub(crate) fn re_select_input(input_src: InputSrc, rcc: &mut RCC) {
    // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

    // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
    // todo: But this saves a few reg writes.
    match input_src {
        InputSrc::Hse(_) => {
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().is_not_ready() {}

            rcc.cfgr
                .modify(|_, w| unsafe { w.sw().bits(input_src.bits()) });
        }
        InputSrc::Pll(_) => {
            // todo: DRY with above.
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().is_not_ready() {}

            rcc.cr.modify(|_, w| w.pllon().off());
            while rcc.cr.read().pllrdy().is_ready() {}

            rcc.cfgr
                .modify(|_, w| unsafe { w.sw().bits(input_src.bits()) });

            rcc.cr.modify(|_, w| w.pllon().on());
            while rcc.cr.read().pllrdy().is_not_ready() {}
        }
        InputSrc::Hsi => (), // Already reset to this.
    }
}
