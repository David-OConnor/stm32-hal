//! Clock config for STM32L, G, and W-series MCUs

use crate::{
    clocks::{ClocksValid, SpeedError},
    pac::{self, FLASH, RCC},
    rcc_en_reset,
};

use cfg_if::cfg_if;

// todo: WB is missing second LSI2, and perhaps other things.
// todo: Mix H7 clocks into this module.

#[cfg(not(any(feature = "g0", feature = "wl")))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Clk48Src {
    // Note: On G4 which only has HSI48 and PLLQ, PLLSai1 and MSI are marked "reserved", and
    // The values it has are the same as on L4/5.
    Hsi48 = 0b00, // Not valid for some L4 variants.
    #[cfg(not(feature = "g4"))]
    PllSai1 = 0b01, // Not avail on G4
    Pllq = 0b10,
    #[cfg(not(feature = "g4"))]
    Msi = 0b11,
}

#[cfg(any(feature = "l4", feature = "l5", feature = "wb"))]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CrsSyncSrc {
    Gpio = 0b00,
    Lse = 0b01,
    Usb = 0b10,
}

#[cfg(not(any(feature = "g0", feature = "g4")))]
#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Msi(MsiRange),
    Hsi,
    Hse(u32),
}

#[cfg(any(feature = "g0", feature = "g4"))]
#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Hsi,
    Hse(u32),
}

#[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "wl"))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Select the system clock used when exiting Stop mode
pub enum StopWuck {
    Msi = 0,
    Hsi = 1,
}

#[cfg(feature = "wb")]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// RF system wakeup clock source selection
pub enum RfWakeupSrc {
    NoClock = 0b00,
    /// LSE oscillator clock used as RF system wakeup clock
    Lse = 0b01,
    /// HSE oscillator clock divided by 1024 used as RF system wakeup clock
    Hse = 0b11,
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
    #[cfg(not(feature = "wl"))]
    W3 = 3,
    #[cfg(not(any(feature = "wb", feature = "wl")))]
    W4 = 4,
    #[cfg(feature = "l5")]
    W5 = 5,
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    /// Sets PLLCFGR reg (PLLSYSCFGR on G0), PLLSRC field.
    pub fn bits(&self) -> u8 {
        // L4 RM, 6.4.4
        #[cfg(not(any(feature = "g0", feature = "g4")))]
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

cfg_if! {
    if #[cfg(feature = "g0")] {
        #[derive(Clone, Copy, PartialEq)]
        pub enum InputSrc {
            Hsi,
            Hse(u32), // freq in Mhz,
            Pll(PllSrc),
            Lsi,
            Lse,
        }

        impl InputSrc {
            /// Required due to numerical value on non-uniform discrim being experimental.
            /// (ie, can't set on `Pll(Pllsrc)`. G0 RM, section 5.4.3.
            pub fn bits(&self) -> u8 {
                match self {
                    Self::Hsi => 0b000,
                    Self::Hse(_) => 0b001,
                    Self::Pll(_) => 0b010,
                    Self::Lsi => 0b011,
                    Self::Lse => 0b100,
                }
            }
        }
    } else if #[cfg(feature = "g4")] {
        #[derive(Clone, Copy, PartialEq)]
        pub enum InputSrc {
            Hsi,
            Hse(u32), // freq in Hz,
            Pll(PllSrc),
        }

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
    } else {  // ie L4 and L5
        #[derive(Clone, Copy, PartialEq)]
        pub enum InputSrc {
            Msi(MsiRange),
            Hsi,
            Hse(u32), // freq in Hz,
            Pll(PllSrc),
        }

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
    }
}

#[cfg(not(any(feature = "g0", feature = "g4")))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Specify the range of MSI - this is effectively it's oscillation speed.
pub enum MsiRange {
    R100k = 0b0000,
    R200k = 0b0001,
    R400k = 0b0010,
    R800k = 0b0011,
    R1M = 0b0100,
    R2M = 0b0101,
    R4M = 0b0110, // default
    R8M = 0b0111,
    R16M = 0b1000,
    R24M = 0b1001,
    R32M = 0b1010,
    R48M = 0b1011,
}

#[cfg(not(any(feature = "g0", feature = "g4")))]
impl MsiRange {
    // Calculate the approximate frequency, in Hz.
    fn value(&self) -> u32 {
        match self {
            Self::R100k => 100_000,
            Self::R200k => 200_000,
            Self::R400k => 400_000,
            Self::R800k => 800_000,
            Self::R1M => 1_000_000,
            Self::R2M => 2_000_000,
            Self::R4M => 4_000_000,
            Self::R8M => 8_000_000,
            Self::R16M => 16_000_000,
            Self::R24M => 24_000_000,
            Self::R32M => 32_000_000,
            Self::R48M => 48_000_000,
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

#[cfg(not(any(feature = "l5", feature = "g4")))]
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

#[cfg(any(feature = "l5", feature = "g4"))]
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
        #[cfg(not(any(feature = "l5", feature = "g4")))]
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

        #[cfg(any(feature = "l5", feature = "g4"))]
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

#[cfg(any(feature = "g0", feature = "wb"))]
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

#[cfg(any(feature = "g0", feature = "wb"))]
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

#[cfg(not(any(feature = "g0", feature = "wb")))]
#[derive(Clone, Copy)]
#[repr(u8)]
// Main PLL division factor for PLLCLK (system clock). G4 RM 7.4.4. Also used to set PLLQ.
pub enum Pllr {
    Div2 = 0b00,
    Div4 = 0b01,
    Div6 = 0b10,
    Div8 = 0b11,
}

#[cfg(not(any(feature = "g0", feature = "wb")))]
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
/// Division factor for the AHB clock. Also known as AHB Prescaler. L4 RM, 6.4.3
/// on WB, used for all 3 HCLK prescalers.
pub enum HclkPrescaler {
    Div1 = 0b0000,
    #[cfg(feature = "wb")]
    Div3 = 0b0001,
    #[cfg(feature = "wb")]
    Div5 = 0b0010,
    #[cfg(feature = "wb")]
    Div6 = 0b0101,
    #[cfg(feature = "wb")]
    Div10 = 0b0110,
    #[cfg(feature = "wb")]
    Div32 = 0b0111,
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
            #[cfg(feature = "wb")]
            Self::Div3 => 3,
            #[cfg(feature = "wb")]
            Self::Div5 => 5,
            #[cfg(feature = "wb")]
            Self::Div6 => 6,
            #[cfg(feature = "wb")]
            Self::Div10 => 10,
            #[cfg(feature = "wb")]
            Self::Div32 => 32,
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
    /// The input source for the system and peripheral clocks. Eg HSE, HSI, PLL etc
    pub input_src: InputSrc, //
    pub pllm: Pllm, // PLL divider
    pub plln: u8,   // PLL multiplier. Valid range of 7 to 86.
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub pll_sai1_mul: u8, // PLL SAI1 multiplier. Valid range of 7 to 86.
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub pll_sai2_mul: u8, // PLL SAI2 multiplier. Valid range of 7 to 86.
    pub pllr: Pllr,
    pub pllq: Pllr,
    /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
    pub hclk_prescaler: HclkPrescaler,
    #[cfg(feature = "wb")]
    /// The value to divide SYSCLK by for HCLK2. (CPU2)
    pub hclk2_prescaler: HclkPrescaler,
    #[cfg(feature = "wl")]
    /// The value to divide SYSCLK by for HCLK3.
    pub hclk3_prescaler: HclkPrescaler,
    #[cfg(feature = "wb")]
    /// The value to divide SYSCLK by for HCLK4.
    pub hclk4_prescaler: HclkPrescaler,
    /// The divider of HCLK to get the APB1 peripheral clock
    pub apb1_prescaler: ApbPrescaler,
    #[cfg(not(feature = "g0"))]
    /// The divider of HCLK to get the APB2 peripheral clock
    pub apb2_prescaler: ApbPrescaler,
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    #[cfg(not(any(feature = "g0", feature = "wl")))]
    /// The input source for the 48Mhz clock used by USB.
    pub clk48_src: Clk48Src,
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub sai1_enabled: bool,
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub sai2_enabled: bool,
    /// Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    /// frees up the pin for use as GPIO.
    pub hse_bypass: bool,
    pub security_system: bool,
    #[cfg(not(any(feature = "g0", feature = "wl")))]
    /// Enable the HSI48. For L4, this is only applicable for some devices.
    pub hsi48_on: bool,
    #[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "wl"))]
    /// Select the input source to use after waking up from `stop` mode. Eg HSI or MSI.
    pub stop_wuck: StopWuck,
    #[cfg(feature = "wb")]
    /// Select the RF wakeup source.
    pub rf_wakeup_src: RfWakeupSrc,
}

// todo: On L4/5, add a way to enable the MSI for use as CLK48.

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
        let (_, sysclk) = self.calc_sysclock();

        cfg_if! {
            if #[cfg(feature = "wb")] {
                let hclk = sysclk / self.hclk4_prescaler.value() as u32;
            } else if #[cfg(feature = "wl")] {
                let hclk = sysclk / self.hclk3_prescaler.value() as u32;
            } else {
                let hclk = sysclk / self.hclk_prescaler.value() as u32;
            }
        }

        // TODO: these are only implemented for Vcore Rnage 1 (Normal mode as applicable)
        // todo: Other modes, like MODE 2 (For lower max system clocks) on L4.

        cfg_if! {
            if #[cfg(feature = "l4")] {  // RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 16_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 32_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 48_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 64_000_000 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else {
                        w.latency().bits(WaitState::W4 as u8)
                    }
                });
            } else if #[cfg(feature = "l5")] {  // RM section 6.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 20_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 40_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 60_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 80_000_000 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else if hclk <= 100_000_000 {
                        w.latency().bits(WaitState::W4 as u8)
                    } else {
                        w.latency().bits(WaitState::W5 as u8)
                    }
                });
            } else if #[cfg(feature = "g0")] {  // G0. RM section 3.3.4
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 24_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 48_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                });
            } else if #[cfg(feature = "wb")] {  // WB. RM section 3.3.4, Table 4.
            // Note: This applies to HCLK4 HCLK. (See HCLK4 used above for hclk var.)
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 18_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 36_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 54_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else {
                        w.latency().bits(WaitState::W3 as u8)
                    }
                });
            } else if #[cfg(any(feature = "wb", feature = "wl"))] {  // WL. RM section 3.3.4, Table 5.
            // Note: This applies to HCLK3 HCLK. (See HCLK3 used above for hclk var.)
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 18_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 36_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                });
            } else {  // G4. RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 34_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 68_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 102_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 136_000_000 {
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
                // MSI initializes to the default clock source. Turn it off before
                // Adjusting its speed etc.
                rcc.cr.modify(|_, w| w.msion().clear_bit());
                while rcc.cr.read().msirdy().bit_is_set() {}

                rcc.cr.modify(|_, w| unsafe {
                    w.msirange().bits(range as u8);
                    #[cfg(not(any(feature = "wb", feature = "wl")))]
                    w.msirgsel().set_bit();
                    w.msion().set_bit()
                });
                // Wait for the MSI to be ready.
                while rcc.cr.read().msirdy().bit_is_clear() {}
                // todo: If LSE is enabled, calibrate MSI.
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().set_bit());
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
            InputSrc::Pll(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and MSI setup above.
                match pll_src {
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => {
                        rcc.cr.modify(|_, w| unsafe {
                            w.msirange().bits(range as u8);
                            #[cfg(not(any(feature = "wb", feature = "wl")))]
                            w.msirgsel().set_bit();
                            w.msion().set_bit()
                        });
                        while rcc.cr.read().msirdy().bit_is_clear() {}
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        rcc.cr.modify(|_, w| w.hsion().set_bit());
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                    }
                    PllSrc::None => {}
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());
                while rcc.csr.read().lsirdy().bit_is_clear() {}
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());
                while rcc.bdcr.read().lserdy().bit_is_clear() {}
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

            cfg_if! {
                if #[cfg(feature = "g0")] {
                    rcc.pllsyscfgr.modify(|_, w| unsafe {
                        w.pllsrc().bits(pll_src.bits());
                        w.plln().bits(self.plln);
                        w.pllm().bits(self.pllm as u8);
                        w.pllr().bits(self.pllr as u8);
                        w.pllq().bits(self.pllq as u8)
                    });
                } else {
                    rcc.pllcfgr.modify(|_, w| unsafe {
                        w.pllsrc().bits(pll_src.bits());
                        w.plln().bits(self.plln);
                        w.pllm().bits(self.pllm as u8);
                        w.pllr().bits(self.pllr as u8);
                        w.pllq().bits(self.pllq as u8)
                    });
                }
            }

            cfg_if! {
                if #[cfg(any(feature = "l4", feature = "l5"))] {
                     if self.sai1_enabled {
                        rcc.pllsai1cfgr.modify(|_, w| unsafe { w.pllsai1n().bits(self.pll_sai1_mul) });
                    }

                    #[cfg(any(feature = "l4x5", feature = "l4x6",))]
                    if self.sai2_enabled {
                        rcc.pllsai2cfgr.modify(|_, w| unsafe { w.pllsai2n().bits(self.pll_sai2_mul) });
                    }
                } else if #[cfg(feature = "wb")] {
                    if self.sai1_enabled {
                        rcc.pllsai1cfgr.modify(|_, w| unsafe { w.plln().bits(self.pll_sai1_mul) });
                    }
                }
            }

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            // todo: Enable sai1 and 2 with separate settings, or lump in with mail PLL
            // like this?
            rcc.cr.modify(|_, w| w.pllon().set_bit());

            cfg_if! {
                if #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))] {
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

            cfg_if! {
                if #[cfg(feature = "g0")] {
                    // Set Pen, Qen, and Ren after we enable the PLL.
                    rcc.pllsyscfgr.modify(|_, w| {
                        w.pllpen().set_bit();
                        w.pllqen().set_bit();
                        w.pllren().set_bit()
                    });
                } else {
                    rcc.pllcfgr.modify(|_, w| {
                        w.pllpen().set_bit();
                        w.pllqen().set_bit();
                        w.pllren().set_bit()
                    });
                }
            }

            cfg_if! {
                if #[cfg(any(feature = "l4", feature = "l5"))] {
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
                } else if #[cfg(feature = "wb")] {
                    if self.sai1_enabled {
                        rcc.pllsai1cfgr.modify(|_, w| {
                            w.pllpen().set_bit();
                            w.pllqen().set_bit();
                            w.pllren().set_bit()
                        });
                    }
                }
            }
        }

        rcc.cfgr.modify(|_, w| unsafe {
            w.sw().bits(self.input_src.bits());
            w.hpre().bits(self.hclk_prescaler as u8);
            #[cfg(not(feature = "g0"))]
            w.ppre2().bits(self.apb2_prescaler as u8); // HCLK division for APB2.
            #[cfg(any(feature = "l4", feature = "l5"))]
            w.stopwuck().bit(self.stop_wuck as u8 != 0);
            #[cfg(not(feature = "g0"))]
            return w.ppre1().bits(self.apb1_prescaler as u8); // HCLK division for APB1
            #[cfg(feature = "g0")]
            return w.ppre().bits(self.apb1_prescaler as u8);
        });

        // todo: Adapt this logic for H7? Mix H7 into this module?
        #[cfg(feature = "wb")]
        rcc.extcfgr.modify(|_, w| unsafe {
            w.c2hpre().bits(self.hclk2_prescaler as u8);
            w.shdhpre().bits(self.hclk4_prescaler as u8)
        });

        #[cfg(feature = "wl")]
        rcc.extcfgr
            .modify(|_, w| unsafe { w.shdhpre().bits(self.hclk3_prescaler as u8) });

        rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        #[cfg(any(feature = "l4", feature = "g4"))]
        rcc.ccipr
            .modify(|_, w| unsafe { w.clk48sel().bits(self.clk48_src as u8) });

        #[cfg(feature = "l5")]
        rcc.ccipr1
            .modify(|_, w| unsafe { w.clk48msel().bits(self.clk48_src as u8) });

        // Enable the HSI48 as required, which is used for USB, RNG, etc.
        // Only valid for some devices (On at least L4, and G4.)
        #[cfg(not(any(feature = "g0", feature = "wl")))]
        if self.hsi48_on {
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        }

        // If we're not using the default clock source as input source or for PLL, turn it off.
        cfg_if! {
            if #[cfg(any(feature = "l4", feature = "l5"))] {
                match self.input_src {
                    InputSrc::Msi(_) => (),
                    InputSrc::Pll(pll_src) => {
                        match pll_src {
                        PllSrc::Msi(_) => (),
                            _ => {
                                rcc.cr.modify(|_, w| w.msion().clear_bit());
                            }
                        }
                    }
                    _ => {
                        rcc.cr.modify(|_, w| w.msion().clear_bit());
                   }
                }

            } else {
                 match self.input_src {
                    InputSrc::Hsi => (),
                    InputSrc::Pll(pll_src) => {
                        match pll_src {
                        PllSrc::Hsi => (),
                            _ => {
                                rcc.cr.modify(|_, w| w.hsion().clear_bit());
                            }
                        }
                    }
                    _ => {
                        rcc.cr.modify(|_, w| w.hsion().clear_bit());
                   }
                }
            }
        }

        #[cfg(feature = "wb")]
        rcc.csr
            .modify(|_, w| unsafe { w.rfwkpsel().bits(self.rf_wakeup_src as u8) });

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        #[cfg(not(any(feature = "wb", feature = "wl")))] // todo: Do interrupts work without enabling syscfg on wb, which
                                                         // todo doesn't have this?
        rcc_en_reset!(apb2, syscfg, rcc);

        Ok(())
    }

    /// Re-select input source; used after Stop and Standby modes, where the system reverts
    /// to MSI or HSI after wake.
    pub fn reselect_input(&self, rcc: &mut RCC) {
        // Re-select the input source; useful for changing input source, or reverting
        // from stop or standby mode. This assumes we're on a clean init,
        // or waking up from stop mode etc.

        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().bit_is_clear() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll(pll_src) => {
                // todo: DRY with above.
                match pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        #[cfg(any(feature = "l4", feature = "l5"))]
                        // Generally reverts to MSI (see note below)
                        if let StopWuck::Msi = self.stop_wuck {
                            rcc.cr.modify(|_, w| w.hsion().set_bit());
                            while rcc.cr.read().hsirdy().bit_is_clear() {}
                        }
                        // If on G, we'll already be on HSI, so need to take action.
                    }
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => {
                        // Range initializes to 4Mhz, so set that as well.
                        #[cfg(not(feature = "wb"))]
                        rcc.cr.modify(|_, w| unsafe {
                            w.msirange().bits(range as u8);
                            w.msirgsel().set_bit()
                        });
                        #[cfg(feature = "wb")]
                        rcc.cr
                            .modify(|_, w| unsafe { w.msirange().bits(range as u8) });

                        if let StopWuck::Hsi = self.stop_wuck {
                            rcc.cr.modify(|_, w| w.msion().set_bit());

                            while rcc.cr.read().msirdy().bit_is_clear() {}
                        }
                    }
                    PllSrc::None => (),
                }

                rcc.cr.modify(|_, w| w.pllon().clear_bit());
                while rcc.cr.read().pllrdy().bit_is_set() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pllon().set_bit());
                while rcc.cr.read().pllrdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                {
                    // (This note applies to L4 and L5 only)
                    // From L4 Reference Manual, RCC_CFGR register section:
                    // "Configured by HW to force MSI oscillator selection when exiting Standby or Shutdown mode.
                    // Configured by HW to force MSI or HSI16 oscillator selection when exiting Stop mode or in
                    // case of failure of the HSE oscillator, depending on STOPWUCK value."

                    // So, if stopwuck is at its default value of MSI, we need to re-enable HSI,
                    // and re-select it. Otherwise, take no action. Reverse for MSI-reselection.
                    // For G, we already are using HSI, so need to take action either.
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    if let StopWuck::Msi = self.stop_wuck {
                        rcc.cr.modify(|_, w| w.hsion().set_bit());
                        while rcc.cr.read().hsirdy().bit_is_clear() {}

                        rcc.cfgr
                            .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
                    }
                }
            }
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                // Range initializes to 4Mhz, so set that as well.
                #[cfg(not(feature = "wb"))]
                rcc.cr.modify(|_, w| unsafe {
                    w.msirange().bits(range as u8);
                    w.msirgsel().set_bit()
                });
                #[cfg(feature = "wb")]
                rcc.cr
                    .modify(|_, w| unsafe { w.msirange().bits(range as u8) });

                if let StopWuck::Hsi = self.stop_wuck {
                    rcc.cr.modify(|_, w| w.msion().set_bit());
                    while rcc.cr.read().msirdy().bit_is_clear() {}

                    rcc.cfgr
                        .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());
                while rcc.csr.read().lsirdy().bit_is_clear() {}
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());
                while rcc.bdcr.read().lserdy().bit_is_clear() {}
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
        }
    }

    #[cfg(any(feature = "l4", feature = "l5"))]
    /// Use this to change the MSI speed. Run this only if your clock source is MSI.
    /// Ends in a state with MSI on at the new speed, and HSI off.
    pub fn change_msi_speed(&mut self, range: MsiRange, rcc: &mut RCC) {
        // todo: Calibrate MSI with LSE / HSE(?) if avail?

        match self.input_src {
            InputSrc::Msi(_) => (),
            _ => panic!("Only change MSI speed using this function if MSI is the input source."),
        }

        // RM: "`"Warning: MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1).
        // MSIRANGE must NOT be modified when MSI is ON and NOT ready (MSION=1 and MSIRDY=0)"
        // So, we can change MSI range while it's running.
        while rcc.cr.read().msirdy().bit_is_clear() {}

        rcc.cr
            .modify(|_, w| unsafe { w.msirange().bits(range as u8).msirgsel().set_bit() });

        // Update our config to reflect the new speed.
        self.input_src = InputSrc::Msi(range);
    }

    #[cfg(any(feature = "l4", feature = "l5"))]
    /// Enables MSI, and configures it at 48Mhz, and trims it using the LSE. This is useful when using it as
    /// the USB clock, ie with `clk48_src: Clk48Src::Msi`. Don't use this if using MSI for the input
    /// source or PLL source. You may need to re-run this after exiting `stop` mode. Only works for USB
    /// if you have an LSE connected.
    /// Note: MSIPLLEN must be enabled after LSE is enabled. So, run this function after RCC clock setup.
    pub fn enable_msi_48(&self, rcc: &mut RCC) {
        if let InputSrc::Msi(_) = self.input_src {
            panic!(
                "Only use this function to set up MSI as 48MHz oscillator\
            if not using it as the input source."
            );
        }
        if let InputSrc::Pll(pll_src) = self.input_src {
            if let PllSrc::Msi(_) = pll_src {
                panic!(
                    "Only use this function to set up MSI as 48MHz oscillator \
                if not using it as the input source."
                );
            }
        }

        rcc.cr.modify(|_, w| w.msion().clear_bit());
        while rcc.cr.read().msirdy().bit_is_set() {}

        // L44 RM, section 6.2.3: When a 32.768 kHz external oscillator is present in the application, it is possible to configure
        // the MSI in a PLL-mode by setting the MSIPLLEN bit in the Clock control register (RCC_CR).
        // When configured in PLL-mode, the MSI automatically calibrates itself thanks to the LSE.
        // This mode is available for all MSI frequency ranges. At 48 MHz, the MSI in PLL-mode can
        // be used for the USB FS device, saving the need of an external high-speed crystal.
        // MSIPLLEN must be enabled after LSE is enabled
        rcc.cr.modify(|_, w| unsafe {
            w.msirange()
                .bits(MsiRange::R48M as u8)
                .msirgsel()
                .set_bit()
                .msipllen()
                .set_bit()
                .msion()
                .set_bit()
        });

        while rcc.cr.read().msirdy().bit_is_clear() {}
    }

    /// Calculate the systick, and input frequency, in Hz.
    fn calc_sysclock(&self) -> (u32, u32) {
        let input_freq;
        let sysclk = match self.input_src {
            InputSrc::Pll(pll_src) => {
                input_freq = match pll_src {
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => range.value() as u32,
                    PllSrc::Hsi => 16_000_000,
                    PllSrc::Hse(freq) => freq,
                    PllSrc::None => 0, // todo?
                };
                input_freq / self.pllm.value() as u32 * self.plln as u32 / self.pllr.value() as u32
            }

            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                input_freq = range.value() as u32;
                input_freq
            }
            InputSrc::Hsi => {
                input_freq = 16_000_000;
                input_freq
            }
            InputSrc::Hse(freq) => {
                input_freq = freq;
                input_freq
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                input_freq = 32_000; // todo confirm this is right.
                input_freq
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                input_freq = 32_768;
                input_freq
            }
        };

        (input_freq, sysclk)
    }

    /// Check if the PLL is enabled. This is useful if checking wheather to re-enable the PLL
    /// after exiting Stop or Standby modes, eg so you don't re-enable if it was already re-enabled
    /// in a different context. eg:
    /// ```
    /// if !clock_cfg.pll_is_enabled() {
    ///     clock_cfg.reselect_input(&mut dp.RCC);
    ///}
    ///```
    pub fn pll_is_enabled(&self, rcc: &mut RCC) -> bool {
        rcc.cr.read().pllon().bit_is_set()
    }

    pub fn sysclk(&self) -> u32 {
        let (_, sysclk) = self.calc_sysclock();
        sysclk
    }

    pub fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    pub fn systick(&self) -> u32 {
        self.hclk()
    }

    cfg_if! {
        if #[cfg(any(feature = "g0", feature = "wl"))] {
            fn usb(&self) -> u32 {
                unimplemented!("No USB on G0 or WL");
            }
        } else if #[cfg(feature = "g4")] {
            fn usb(&self) -> u32 {
                48_000_000 // Uses hsi48.
            }
        } else { // L4 and L5
            fn usb(&self) -> u32 {
                match self.clk48_src {
                    Clk48Src::Hsi48 => 48_000_000,
                    Clk48Src::PllSai1 => unimplemented!(),
                    Clk48Src::Pllq => unimplemented!(),
                    Clk48Src::Msi => unimplemented!(),
                }
            }
        }
    }

    pub fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    pub fn apb1_timer(&self) -> u32 {
        // L4 RM, 6.2.14: The timer clock frequencies are automatically defined by hardware. There are two cases:
        // 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same
        // frequency as that of the APB domain.
        // 2. Otherwise, they are set to twice (×2) the frequency of the APB domain.
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    cfg_if! {
        if #[cfg(feature = "g0")] {
            pub fn apb2(&self) -> u32 {
                unimplemented!("No apb2 on G0");
            }

            pub fn apb2_timer(&self) -> u32 {
                unimplemented!("No apb2 on G0");
            }
        } else {
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
        }
    }

    pub fn validate_speeds(&self) -> ClocksValid {
        #[cfg(feature = "l4")]
        let max_clock = 80_000_000;

        #[cfg(feature = "l5")]
        let max_clock = 110_000_000;

        #[cfg(feature = "g0")]
        let max_clock = 64_000_000;

        #[cfg(feature = "g4")]
        let max_clock = 170_000_000;

        #[cfg(feature = "wb")]
        let max_clock = 64_000_000;

        #[cfg(feature = "wl")]
        let max_clock = 48_000_000;

        // todo: Check valid PLL output range as well. You can use Cube, mousing over the PLL
        // todo speed to find these.

        // todo: L4+ (ie R, S, P, Q) can go up to 120_000.

        #[cfg(any(feature = "l4", feature = "l5", feature = "wb"))]
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

        // todo: on WB, input src / PlLM * plln Must be between 96 and 344 Mhz.
        // todo; Cube will validate this. Others probably have a similar restriction.
        // todo: Put this check here.

        // todo: QC these limits
        // todo: Note that this involves repeatedly calculating sysclk.
        // todo. We could work around thsi by calcing it once here.
        if self.sysclk() > max_clock {
            return ClocksValid::NotValid;
        }

        if self.hclk() > max_clock {
            return ClocksValid::NotValid;
        }

        if self.apb1() > max_clock {
            return ClocksValid::NotValid;
        }

        #[cfg(not(feature = "g0"))]
        if self.apb2() > max_clock {
            return ClocksValid::NotValid;
        }

        ClocksValid::Valid
    }
}

impl Default for Clocks {
    /// This default configures clocks with a HSI, with system and peripheral clocks at full rated speed.
    /// All peripheral. Speeds -> L4: 80Mhz. L5: 110Mhz. G0: 64Mhz. G4: 170Mhz. WB: 64Mhz.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pllm: Pllm::Div4,
            #[cfg(feature = "l4")]
            plln: 40,
            #[cfg(feature = "l5")]
            plln: 55,
            #[cfg(feature = "g0")]
            plln: 32,
            #[cfg(feature = "g4")]
            plln: 85,
            #[cfg(feature = "wb")]
            plln: 64,
            #[cfg(feature = "wl")]
            plln: 24,
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            pll_sai1_mul: 8,
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            pll_sai2_mul: 8,
            #[cfg(not(feature = "wb"))]
            pllr: Pllr::Div2,
            #[cfg(feature = "wb")]
            pllr: Pllr::Div4,
            pllq: Pllr::Div4,
            hclk_prescaler: HclkPrescaler::Div1,
            #[cfg(feature = "wb")]
            hclk2_prescaler: HclkPrescaler::Div2,
            #[cfg(feature = "wl")]
            hclk3_prescaler: HclkPrescaler::Div1,
            #[cfg(feature = "wb")]
            hclk4_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            #[cfg(not(feature = "g0"))]
            apb2_prescaler: ApbPrescaler::Div1,
            #[cfg(not(any(feature = "g0", feature = "wl")))]
            clk48_src: Clk48Src::Hsi48,
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            sai1_enabled: false,
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            sai2_enabled: false,
            hse_bypass: false,
            security_system: false,
            #[cfg(not(any(feature = "g0", feature = "wl")))]
            hsi48_on: false,
            #[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "wl"))]
            stop_wuck: StopWuck::Msi,
            #[cfg(feature = "wb")]
            rf_wakeup_src: RfWakeupSrc::Lse,
        }
    }
}

#[cfg(any(feature = "l4", feature = "l5", feature = "wb"))]
/// Enable the Clock Recovery System. L443 User manual:
/// "The STM32L443xx devices embed a special block which allows automatic trimming of the
/// internal 48 MHz oscillator to guarantee its optimal accuracy over the whole device
/// operational range. This automatic trimming is based on the external synchronization signal,
/// which could be either derived from USB SOF signalization, from LSE oscillator, from an
/// external signal on CRS_SYNC pin or generated by user software. For faster lock-in during
/// startup it is also possible to combine automatic trimming with manual trimming action."
/// Note: This is for HSI48 only. Note that the HSI will turn off after entering Stop or Standby.
pub fn enable_crs(sync_src: CrsSyncSrc, crs: &mut pac::CRS, rcc: &mut RCC) {
    // todo: CRSEN missing on l4x5 pac: https://github.com/stm32-rs/stm32-rs/issues/572
    cfg_if! {
        if #[cfg(feature = "l4x5")] {
            let val = rcc.apb1enr1.read().bits();
            rcc.apb1enr1.write(|w| unsafe { w.bits(val | (1 << 24)) });
        } else {
            rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());
        }
    }

    crs.cfgr
        .modify(|_, w| unsafe { w.syncsrc().bits(sync_src as u8) });

    crs.cr.modify(|_, w| {
        // Set autotrim enabled.
        w.autotrimen().set_bit();
        // Enable CRS
        w.cen().set_bit()
    });

    // "The internal 48 MHz RC oscillator is mainly dedicated to provide a high precision clock to
    // the USB peripheral by means of a special Clock Recovery System (CRS) circuitry. The CRS
    // can use the USB SOF signal, the LSE or an external signal to automatically and quickly
    // adjust the oscillator frequency on-fly. It is disabled as soon as the system enters Stop or
    // Standby mode. When the CRS is not used, the HSI48 RC oscillator runs on its default
    // frequency which is subject to manufacturing process variations
}
