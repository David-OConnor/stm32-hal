use crate::{
    clocks::SpeedError,
    pac::{FLASH, RCC},
    traits::{ClockCfg, ClocksValid},
};

use cfg_if::cfg_if;

#[cfg(not(any(feature = "g0", feature = "g4")))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Clk48Src {
    Hsi48 = 0b00, // Only falivd for STM32L49x/L4Ax
    PllSai1 = 0b01,
    Pll = 0b10,
    Msi = 0b11,
}

#[cfg(not(any(feature = "g0", feature = "g4")))]
#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Msi(MsiRange),
    Hsi,
    Hse(u8),
}

#[cfg(any(feature = "g0", feature = "g4"))]
#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Hsi,
    Hse(u8),
}

// L4 uses 0 - 4 only. Others use 1 - 15, but it's not clear when you'd
// set more than WS5 or so.
#[derive(Clone, Copy)]
#[repr(u8)]
/// Represents Flash wait states in the FLASH_ACR register.
enum WaitState {
    W0 = 0,
    W1 = 1,
    W2 = 2,
    W3 = 3,
    W4 = 4,
    W5 = 5,
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        #[cfg(not(any(feauture = "g0", feature = "g4")))]
        match self {
            Self::None => 0b00,
            Self::Msi(_) => 0b01,
            Self::Hsi => 0b10,
            Self::Hse(_) => 0b11,
        }
        #[cfg(any(feature = "g0", feature = "g4"))]
        match self {
            Self::None => 0b00,
            Self::Hsi => 0b10,
            Self::Hse(_) => 0b11,
        }
    }
}

#[cfg(feature = "g0")]
#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Hsi,
    Hse(u8), // freq in Mhz,
    Pll(PllSrc),
    Lsi,
    Lse,
}

#[cfg(feature = "g0")]
impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`. G0 RM, section 5.4.3.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi => 0b000,
            Self::Hse(_) => 0b001,
            Self::Pll(_) => 0b011,
            Self::Lsi => 0b011,
            Self::Lse => 0b100,
        }
    }
}

#[cfg(feature = "g4")]
#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Hsi,
    Hse(u8), // freq in Mhz,
    Pll(PllSrc),
}

#[cfg(feature = "g4")]
impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi => 0b01,
            Self::Hse(_) => 0b10,
            Self::Pll(_) => 0b11,
        }
    }
}

#[cfg(any(feature = "l4", feature = "l5"))]
#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Msi(MsiRange),
    Hsi,
    Hse(u8), // freq in Mhz,
    Pll(PllSrc),
}

#[cfg(any(feature = "l4", feature = "l5"))]
impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Msi(_) => 0b00,
            Self::Hsi => 0b01,
            Self::Hse(_) => 0b10,
            Self::Pll(_) => 0b11,
        }
    }
}

#[cfg(not(any(feature = "g0", feature = "g4")))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum MsiRange {
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

#[cfg(not(any(feature = "g0", feature = "g4")))]
impl MsiRange {
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

#[cfg(not(feature = "g4"))]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Pllm {
    Div1 = 0b000,
    Div2 = 0b001,
    Div3 = 0b010,
    Div4 = 0b011,
    Div5 = 0b100,
    Div6 = 0b101,
    Div7 = 0b110,
    Div8 = 0b111,
}

#[cfg(feature = "g4")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Pllm {
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

impl Pllm {
    pub fn value(&self) -> u8 {
        #[cfg(not(feature = "g4"))]
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

        #[cfg(feature = "g4")]
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

#[cfg(feature = "g0")]
#[derive(Clone, Copy)]
#[repr(u8)]
// Main PLL division factor for PLLCLK (system clock).
pub enum Pllr {
    Div2 = 0b000,
    Div3 = 0b001,
    Div4 = 0b010,
    Div5 = 0b011,
    Div6 = 0b101,
    Div7 = 0b110,
    Div8 = 0b111,
}

#[cfg(feature = "g0")]
impl Pllr {
    pub fn value(&self) -> u8 {
        match self {
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

#[cfg(not(feature = "g0"))]
#[derive(Clone, Copy)]
#[repr(u8)]
// Main PLL division factor for PLLCLK (system clock). G4 RM 7.4.4
pub enum Pllr {
    Div2 = 0b00,
    Div4 = 0b01,
    Div6 = 0b10,
    Div8 = 0b11,
}

#[cfg(not(feature = "g0"))]
impl Pllr {
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
pub struct Clocks {
    pub input_src: InputSrc, //
    pub pllm: Pllm,          // PLL divider
    pub plln: u8,            // PLL multiplier. Valid range of 7 to 86.
    #[cfg(not(any(feature = "g0", feature = "g4")))]
    pub pll_sai1_mul: u8, // PLL SAI1 multiplier. Valid range of 7 to 86.
    #[cfg(not(any(feature = "g0", feature = "g4")))]
    pub pll_sai2_mul: u8, // PLL SAI2 multiplier. Valid range of 7 to 86.
    pub pllr: Pllr,
    pub hclk_prescaler: HclkPrescaler, // The AHB clock divider.
    pub apb1_prescaler: ApbPrescaler,  // APB1 divider, for the low speed peripheral bus.
    #[cfg(not(feature = "g0"))]
    pub apb2_prescaler: ApbPrescaler, // APB2 divider, for the high speed peripheral bus.
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    #[cfg(not(any(feature = "g0", feature = "g4")))]
    pub clk48_src: Clk48Src,
    #[cfg(not(any(feature = "g0", feature = "g4")))]
    pub sai1_enabled: bool,
    #[cfg(not(any(feature = "g0", feature = "g4")))]
    pub sai2_enabled: bool,
    pub hse_bypass: bool,
    pub security_system: bool,
    #[cfg(not(feature = "g0"))]
    pub hsi48_on: bool,
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
        let (_, sysclk) = calc_sysclock(self.input_src, self.pllm, self.plln, self.pllr);

        let hclk = sysclk / self.hclk_prescaler.value() as u32;

        // TODO: these are only implemented for Vcore Rnage 1 (Normal mode as applicable)
        // todo: Other modes.

        cfg_if! {
            if #[cfg(feature = "l4")] {  // RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 16 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 32 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 48 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 64 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else {
                        w.latency().bits(WaitState::W4 as u8)
                    }
                });
            } else if #[cfg(feature = "l5")] {  // RM section 6.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 20 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 40 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 60 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 80 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else if hclk <= 100 {
                        w.latency().bits(WaitState::W4 as u8)
                    } else {
                        w.latency().bits(WaitState::W5 as u8)
                    }
                });
            } else if #[cfg(feature = "g0")] {  // G0. RM section 3.3.4
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 24 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 48 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                })
            } else {  // G4. RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 34 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 68 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 102 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 136 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else {
                        w.latency().bits(WaitState::W4 as u8)
                    }
                });
            }

        }

        // Reference Manual, 6.2.5:
        // The device embeds 3 PLLs: PLL, PLLSAI1, PLLSAI2. Each PLL provides up to three
        // independent outputs. The internal PLLs can be used to multiply the HSI16, HSE or MSI
        // output clock frequency. The PLLs input frequency must be between 4 and 16 MHz. The
        // selected clock source is divided by a programmable factor PLLM from 1 to 8 to provide a
        // clock frequency in the requested input range. Refer to Figure 15: Clock tree (for
        // STM32L47x/L48x devices) and Figure 16: Clock tree (for STM32L49x/L4Ax devices) and
        // PLL configuration register (RCC_PLLCFGR).
        // The PLLs configuration (selection of the input clock and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLON to 0 in Clock control register (RCC_CR).
        // 2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLON to 1.
        // 5. Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN in PLL
        // configuration register (RCC_PLLCFGR).

        // Enable oscillators, and wait until ready.
        match self.input_src {
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                rcc.cr.modify(|_, w| unsafe {
                    w.msirange()
                        .bits(range as u8)
                        .msirgsel()
                        .set_bit()
                        .msion()
                        .set_bit()
                });
                // Wait for the MSI to be ready.
                while rcc.cr.read().msirdy().bit_is_clear() {}
                // todo: If LSE is enabled, calibrate MSI.
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
            InputSrc::Pll(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and MSI setup above.
                match pll_src {
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => {
                        rcc.cr.modify(|_, w| unsafe {
                            w.msirange()
                                .bits(range as u8)
                                .msirgsel()
                                .set_bit()
                                .msion()
                                .set_bit()
                        });
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

        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pllrdy().bit_is_set() {}

            rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllsrc().bits(pll_src.bits());
                w.plln().bits(self.plln);
                w.pllm().bits(self.pllm as u8);
                w.pllr().bits(self.pllr as u8)
            });

            cfg_if! {
                if #[cfg(not(any(feature = "g0", feature = "g4")))] {
                     if self.sai1_enabled {
                        rcc.pllsai1cfgr
                            .modify(|_, w| unsafe { w.pllsai1n().bits(self.pll_sai1_mul) });
                    }

                    #[cfg(any(feature = "l4x5", feature = "l4x6",))]
                    if self.sai2_enabled {
                        rcc.pllsai2cfgr
                            .modify(|_, w| unsafe { w.pllsai2n().bits(self.pll_sai2_mul) });
                    }
                }
            }

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            // todo: Enable sai1 and 2 with separate settings, or lump in with mail PLL
            // like this?
            rcc.cr.modify(|_, w| w.pllon().set_bit());

            cfg_if! {
                if #[cfg(not(any(feature = "g0", feature = "g4")))] {
                    if self.sai1_enabled {
                        rcc.cr.modify(|_, w| w.pllsai1on().set_bit());
                        while rcc.cr.read().pllsai1rdy().bit_is_clear() {}
                    }
                    #[cfg(any(feature = "l4x5", feature = "l4x6",))]
                    if self.sai2_enabled {
                        rcc.cr.modify(|_, w| w.pllsai2on().set_bit());
                        while rcc.cr.read().pllsai2rdy().bit_is_clear() {}
                    }
                }
            }

            while rcc.cr.read().pllrdy().bit_is_clear() {}

            // Set Pen, Qen, and Ren after we enable the PLL.
            rcc.pllcfgr.modify(|_, w| {
                w.pllpen().set_bit();
                w.pllqen().set_bit();
                w.pllren().set_bit()
            });

            cfg_if! {
                if #[cfg(not(any(feature = "g0", feature = "g4")))] {
                    if self.sai1_enabled {
                        rcc.pllsai1cfgr.modify(|_, w| {
                            w.pllsai1pen().set_bit();
                            w.pllsai1qen().set_bit();
                            w.pllsai1ren().set_bit()
                        });
                    }

                    #[cfg(any(feature = "l4x5", feature = "l4x6"))]
                    if self.sai2_enabled {
                        rcc.pllsai2cfgr.modify(|_, w| {
                            w.pllsai2pen().set_bit();
                            w.pllsai2ren().set_bit()
                        });
                    }
                }
            }
        }

        rcc.cfgr.modify(|_, w| unsafe {
            w.sw().bits(self.input_src.bits());
            w.hpre().bits(self.hclk_prescaler as u8); // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
            w.ppre2().bits(self.apb2_prescaler as u8); // HCLK division for APB2.
            w.ppre1().bits(self.apb1_prescaler as u8) // HCLK division for APB1
        });

        rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        #[cfg(feature = "l4")]
        rcc.ccipr
            .modify(|_, w| unsafe { w.clk48sel().bits(self.clk48_src as u8) });

        #[cfg(feature = "l5")]
        rcc.ccipr1
            .modify(|_, w| unsafe { w.clk48msel().bits(self.clk48_src as u8) });

        // Enable the HSI48 as required, which is used for USB, RNG, etc.
        // Only valid for STM32L49x/L4Ax devices.
        #[cfg(not(any(feature = "g0", feature = "g4")))]
        if Clk48Src::Hsi48 == self.clk48_src && self.hsi48_on {
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        }

        #[cfg(feature = "g4")]
        if self.hsi48_on {
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        }

        Ok(())
    }

    /// This preset configures common with a 8Mhz HSE, a 80Mhz sysclck (L4). All peripheral clocks are at
    /// 80Mhz (L4). L5 speeds: 108Mhz. G4 speeds: 168Mhz. HSE output is not bypassed.
    pub fn hse_preset() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hse(8)),
            pllm: Pllm::Div1,
            #[cfg(feature = "l4")]
            plln: 20,
            #[cfg(feature = "l5")]
            plln: 27,
            #[cfg(feature = "g0")]
            plln: 16,
            #[cfg(feature = "g4")]
            plln: 42,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            pll_sai1_mul: 8,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            pll_sai2_mul: 8,
            pllr: Pllr::Div2,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            #[cfg(not(feature = "g0"))]
            apb2_prescaler: ApbPrescaler::Div1,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            clk48_src: Clk48Src::PllSai1,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            sai1_enabled: false,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            sai2_enabled: false,
            hse_bypass: false,
            security_system: false,
            #[cfg(not(feature = "g0"))]
            hsi48_on: false,
        }
    }
}

// todo: Some extra calculations here, vice doing it once and caching.
impl ClockCfg for Clocks {
    fn sysclk(&self) -> u32 {
        let (_, sysclk) = calc_sysclock(self.input_src, self.pllm, self.plln, self.pllr);
        sysclk * 1_000_000
    }

    fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    fn systick(&self) -> u32 {
        self.hclk()
    }

    cfg_if! {
        if #[cfg(feature = "g0")] {
            fn usb(&self) -> u32 {
                unimplemented!("No USB on G0");
            }
        } else if #[cfg(feature = "g4")] {
            fn usb(&self) -> u32 {
                48_000_000 // Uses hsi48.
            }
        } else { // L4 and L5
            fn usb(&self) -> u32 {
                let (input_freq, _) = calc_sysclock(self.input_src, self.pllm, self.plln, self.pllr);
                input_freq * 1_000_000 / self.pllm.value() as u32 * self.pll_sai1_mul as u32 / 2
            }
        }
    }

    fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    fn apb2(&self) -> u32 {
        self.hclk() / self.apb2_prescaler.value() as u32
    }

    fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb2_prescaler {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    fn validate_speeds(&self) -> ClocksValid {
        let mut result = ClocksValid::Valid;

        #[cfg(feature = "l4")]
        let max_clock = 80_000_000;

        #[cfg(feature = "l5")]
        let max_clock = 110_000_000;

        #[cfg(feature = "g0")]
        let max_clock = 64_000_000;

        #[cfg(feature = "g4")]
        let max_clock = 170_000_000;

        // todo: L4+ (ie R, S, P, Q) can go up to 120_000.

        #[cfg(any(feature = "l4", feature = "l5"))]
        if self.plln < 7
            || self.plln > 86
            || self.pll_sai1_mul < 7
            || self.pll_sai1_mul > 86
            || self.pll_sai2_mul < 7
            || self.pll_sai2_mul > 86
        {
            return ClocksValid::NotValid;
        }

        #[cfg(feature = "g0")]
        if self.plln < 9 || self.plln > 86 {
            return ClocksValid::NotValid;
        }

        #[cfg(feature = "g4")]
        if self.plln < 8 || self.plln > 127 {
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

        #[cfg(not(feature = "g0"))]
        if self.apb2() > max_clock {
            result = ClocksValid::NotValid;
        }

        result
    }
}

impl Default for Clocks {
    /// This default configures common with a HSI, a 80Mhz sysclck (l4). All peripheral clocks are at
    /// 80Mhz (l4). L5 speeds: 108Mhz. G4 speeds: 168Mhz.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pllm: Pllm::Div2,
            #[cfg(feature = "l4")]
            plln: 20,
            #[cfg(feature = "l5")]
            plln: 27,
            #[cfg(feature = "g0")]
            plln: 16,
            #[cfg(feature = "g4")]
            plln: 42,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            pll_sai1_mul: 8,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            pll_sai2_mul: 8,
            pllr: Pllr::Div2,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            #[cfg(not(feature = "g0"))]
            apb2_prescaler: ApbPrescaler::Div1,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            clk48_src: Clk48Src::PllSai1,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            sai1_enabled: false,
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            sai2_enabled: false,
            hse_bypass: false,
            security_system: false,
            #[cfg(not(feature = "g0"))]
            hsi48_on: false,
        }
    }
}

/// Calculate the systick, and input frequency.
fn calc_sysclock(input_src: InputSrc, pllm: Pllm, plln: u8, pllr: Pllr) -> (u32, u32) {
    let input_freq;
    let sysclk = match input_src {
        InputSrc::Pll(pll_src) => {
            input_freq = match pll_src {
                #[cfg(not(any(feature = "g0", feature = "g4")))]
                PllSrc::Msi(range) => range.value() as u32 / 1_000_000,
                PllSrc::Hsi => 16,
                PllSrc::Hse(freq) => freq as u32,
                PllSrc::None => 0, // todo?
            };
            input_freq as u32 / pllm.value() as u32 * plln as u32 / pllr.value() as u32
        }

        #[cfg(not(any(feature = "g0", feature = "g4")))]
        InputSrc::Msi(range) => {
            input_freq = range.value() as u32 / 1_000_000;
            input_freq
        }
        InputSrc::Hsi => {
            input_freq = 16;
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
        InputSrc::Pll(pll_src) => {
            // todo: DRY with above.
            match pll_src {
                PllSrc::Hse(_) => {
                    rcc.cr.modify(|_, w| w.hseon().set_bit());
                    while rcc.cr.read().hserdy().bit_is_clear() {}
                }
                PllSrc::Hsi => {
                    // Generally reverts to MSI (see note below)
                    rcc.cr.modify(|_, w| w.hsion().bit(true));
                    while rcc.cr.read().hsirdy().bit_is_clear() {}
                }
                #[cfg(not(any(feature = "g0", feature = "g4")))]
                PllSrc::Msi(_) => (), // Already reverted to this.
                PllSrc::None => (),
            }

            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while rcc.cr.read().pllrdy().bit_is_set() {}

            rcc.cfgr
                .modify(|_, w| unsafe { w.sw().bits(input_src.bits()) });

            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }
        InputSrc::Hsi => {
            {
                // From Reference Manual, RCC_CFGR register section:
                // "Configured by HW to force MSI oscillator selection when exiting Standby or Shutdown mode.
                // Configured by HW to force MSI or HSI16 oscillator selection when exiting Stop mode or in
                // case of failure of the HSE oscillator, depending on STOPWUCK value."
                // In tests, from stop, it tends to revert to MSI.
                rcc.cr.modify(|_, w| w.hsion().bit(true));
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
        }
        #[cfg(not(any(feature = "g0", feature = "g4")))]
        InputSrc::Msi(_) => (), // Already reset to this

        #[cfg(feature = "g0")]
        InputSrc::Lsi => (),
        #[cfg(feature = "g0")]
        InputSrc::Lse => (),
    }
}
