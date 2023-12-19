//! Clock config for STM32L, G, and W-series MCUs. Uses a `Clocks` struct to configure
//! settings, starting with a `Default::default()` implementation. Uses the `setup` method
//! to write changes.

// Similar in from to the H7 clocks module, but includes notable differendes.

use cfg_if::cfg_if;

#[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "g4"))]
use crate::pac::CRS;
use crate::{
    clocks::RccError,
    pac::{self, FLASH, RCC},
    util::rcc_en_reset,
    MAX_ITERS,
};

// todo: WB is missing second LSI2, and perhaps other things.

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

#[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "g4"))]
#[derive(Clone, Copy)]
#[repr(u8)]
/// Select the SYNC signal source. Sets the CRS_CFGR register, SYNCSRC field.
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

#[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "wl"))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Select the system clock used when exiting Stop mode. Sets RCC_CFGR register, STOPWUCK field.
pub enum StopWuck {
    Msi = 0,
    Hsi = 1,
}

cfg_if! {
if #[cfg(feature = "g0")] {
#[derive(Clone, Copy, PartialEq)]
/// Clock input source, also known as system clock switch. Sets RCC_CFGR register, SW field.
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

/// Configures the speeds, and enable status of an individual PLL (PLL1, or SAIPLL). Note that the `enable`
/// field has no effect for PLL1.
pub struct PllCfg {
    /// Only relevant for PLLSAI1.
    pub enabled: bool,
    pub pllr_en: bool,
    pub pllq_en: bool,
    pub pllp_en: bool,
    /// Only relevant for The main PLL.
    pub divm: Pllm,
    pub divn: u8,
    pub divr: Pllr,
    pub divq: Pllr,
    pub divp: Pllp,
    /// Defaults to 0, which causes the PLLP setting to take effect (of 7 or 17), instead of this
    /// field.  Unused on "wb", "wl", "l4x5", and "l4x3" variants.
    pub pdiv: u8,
}

impl Default for PllCfg {
    fn default() -> Self {
        // todo: Different defaults for different variants.
        Self {
            enabled: true,
            pllr_en: true,
            pllq_en: false,
            pllp_en: false,
            divm: Pllm::Div4,
            #[cfg(feature = "l4")]
            divn: 40,
            #[cfg(feature = "l5")]
            divn: 55,
            #[cfg(feature = "g0")]
            divn: 32,
            #[cfg(feature = "g4")]
            divn: 85,
            #[cfg(feature = "wb")]
            divn: 64,
            #[cfg(feature = "wl")]
            divn: 24,
            #[cfg(not(feature = "wb"))]
            divr: Pllr::Div2,
            #[cfg(feature = "wb")]
            divr: Pllr::Div4,
            divq: Pllr::Div4,
            divp: Pllp::Div7,
            pdiv: 0,
        }
    }
}

impl PllCfg {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            pllp_en: false,
            pllr_en: false,
            pllq_en: false,
            ..Default::default()
        }
    }

    pub fn pvalue(&self) -> u8 {
        match self.pdiv {
            0 => self.divp.value(),
            pdiv => pdiv,
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
/// Main PLL division factor for PLLCLK (system clock). Also usd for PllQ.
/// Sets `PLLCFGR` reg.
pub enum Pllr {
    Div2 = 0b001,
    Div3 = 0b010,
    Div4 = 0b011,
    Div5 = 0b100,
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
/// Main PLL division factor for PLLCLK (system clock). G4 RM 7.4.4. Also used to set PLLQ.
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
/// Divisor for PLLP. Sets `PLLCFGR` reg, `PLLP` field.
pub enum Pllp {
    Div7 = 0,
    Div17 = 1,
}

impl Pllp {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div7 => 7,
            Self::Div17 => 17,
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

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// SAI clock input source. Sets RCC_CCIPR register, SAIxSEL fields.
pub enum SaiSrc {
    /// PLLSAI1 “P” clock (PLLSAI1PCLK) selected as SAI1 clock
    PllSai1P = 0b00,
    /// PLL “P” clock (PLLPCLK) selected as SAI1 clock
    Pllp = 0b01,
    /// HSI16 clock selected as SAI1 clock
    Hsi = 0b10,
    /// External input SAI1_EXTCLK selected as SAI1 clock
    ExtClk = 0b11,
}

#[cfg(any(feature = "g0", feature = "g4"))]
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// CAN clock input source. Sets RCC_CCIPR register, FDCANSEL field.
pub enum CanSrc {
    /// hse_ck clock is selected as FDCAN kernel clock
    Hse = 0b00,
    /// PLL1Q
    PllQ = 0b01,
    /// PCLK1 (APB1) selected as FDCAN clock
    Pclk = 0b10,
}

/// Settings used to configure clocks. Create this struct by using its `Default::default()`
/// implementation, then modify as required, referencing your RM's clock tree,
/// or Stm32Cube IDE's interactive clock manager. Apply settings by running `.setup()`.
pub struct Clocks {
    /// The input source for the system and peripheral clocks. Eg HSE, HSI, PLL etc
    pub input_src: InputSrc,
    /// Enable and speed status for the main PLL
    pub pll: PllCfg,
    /// Enable and speed status for the SAI PLL
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub pllsai1: PllCfg,
    #[cfg(any(feature = "l4x5", feature = "l4x6"))]
    pub pllsai2: PllCfg,
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
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    /// SAI1 kernel clock source selection
    pub sai1_src: SaiSrc,
    #[cfg(feature = "g4")]
    /// Range 1 boost mode: Used to increase regulator voltage to 1.28v, for when system
    /// clock frequency is up to 170Mhz. Defaults to true.
    pub boost_mode: bool,
    #[cfg(any(feature = "g0", feature = "g4"))]
    /// FDCAN kernel clock selection. Defaults to APB1.
    pub can_src: CanSrc,
}

// todo: On L4/5, add a way to enable the MSI for use as CLK48.

impl Clocks {
    /// Setup common and return Ok if the config is valid. Abort the setup if speeds
    /// are invalid.
    /// Use the STM32CubeIDE Clock Configuration tab to help identify valid configs.
    /// Use the `default()` implementation as a safe baseline.
    pub fn setup(&self) -> Result<(), RccError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        let rcc = unsafe { &(*RCC::ptr()) };
        let flash = unsafe { &(*FLASH::ptr()) };
        #[cfg(feature = "l5")]
        let icache = unsafe { &(*pac::ICACHE::ptr()) };

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        #[cfg(not(any(feature = "wb", feature = "wl")))]
        rcc_en_reset!(apb2, syscfg, rcc);

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        let sysclk = self.sysclk();

        cfg_if! {
        if #[cfg(feature = "wb")] {
        let hclk = sysclk / self.hclk4_prescaler.value() as u32;
        } else if #[cfg(feature = "wl")] {
        let hclk = sysclk / self.hclk3_prescaler.value() as u32;
        } else {
        let hclk = sysclk / self.hclk_prescaler.value() as u32;
        }
        }

        cfg_if! {
        if #[cfg(feature = "g4")] {
        if self.boost_mode {
        // The sequence to switch from Range1 normal mode to Range1 boost mode is:
        // 1. The system clock must be divided by 2 using the AHB prescaler before switching to a
        // higher system frequency.
        rcc.cfgr.modify(|_, w| unsafe { w.hpre().bits(HclkPrescaler::Div2 as u8) });
        // 2. Clear the R1MODE bit is in the PWR_CR5 register.
        let pwr = unsafe { &(*pac::PWR::ptr()) };
        pwr.cr5.modify(|_, w| w.r1mode().clear_bit());
        }

        // (Remaining steps accomplished below)
        // 3. Adjust the number of wait states according to the new frequency target in range1 boost
        // mode
        // 4. Configure and switch to new system frequency.
        // 5. Wait for at least 1us and then reconfigure the AHB prescaler to get the needed HCLK
        // clock frequency.
        }
        }

        cfg_if! {
        if #[cfg(feature = "l4")] {  // RM section 3.3.3
        let wait_state = if hclk <= 16_000_000 {
        WaitState::W0
        } else if hclk <= 32_000_000 {
        WaitState::W1
        } else if hclk <= 48_000_000 {
        WaitState::W2
        } else if hclk <= 64_000_000 {
        WaitState::W3
        } else {
        WaitState::W4
        };
        } else if #[cfg(feature = "l5")] {  // RM section 6.3.3
        let wait_state = if hclk <= 20_000_000 {
        WaitState::W0
        } else if hclk <= 40_000_000 {
        WaitState::W1
        } else if hclk <= 60_000_000 {
        WaitState::W2
        } else if hclk <= 80_000_000 {
        WaitState::W3
        } else if hclk <= 100_000_000 {
        WaitState::W4
        } else {
        WaitState::W5
        };
        } else if #[cfg(feature = "g0")] {  // G0. RM section 3.3.4
        let wait_state = if hclk <= 24_000_000 {
        WaitState::W0
        } else if hclk <= 48_000_000 {
        WaitState::W1
        } else {
        WaitState::W2
        };
        } else if #[cfg(feature = "wb")] {  // WB. RM section 3.3.4, Table 4.
        // Note: This applies to HCLK4 HCLK. (See HCLK4 used above for hclk var.)
        let wait_state = if hclk <= 18_000_000 {
        WaitState::W0
        } else if hclk <= 36_000_000 {
        WaitState::W1
        } else if hclk <= 54_000_000 {
        WaitState::W2
        } else {
        WaitState::W3
        };
        } else if #[cfg(any(feature = "wb", feature = "wl"))] {  // WL. RM section 3.3.4, Table 5.
        // Note: This applies to HCLK3 HCLK. (See HCLK3 used above for hclk var.)
        let wait_state = if hclk <= 18_000_000 {
        WaitState::W0
        } else if hclk <= 36_000_000 {
        WaitState::W1
        } else {
        WaitState::W2
        };
        } else {  // G4. RM section 3.3.3
        let wait_state = if self.boost_mode {
        // Vcore Range 1 boost mode
        if hclk <= 34_000_000 {
        WaitState::W0
        } else if hclk <= 68_000_000 {
        WaitState::W1
        } else if hclk <= 102_000_000 {
        WaitState::W2
        } else if hclk <= 136_000_000 {
        WaitState::W3
        } else {
        WaitState::W4
        }
        } else {
        // Vcore Range 1 normal mode.
        if hclk <= 30_000_000 {
        WaitState::W0
        } else if hclk <= 60_000_000 {
        WaitState::W1
        } else if hclk <= 90_000_000 {
        WaitState::W2
        } else if hclk <= 120_000_000 {
        WaitState::W3
        } else {
        WaitState::W4
        }
        };
        }
        }

        // Enable instruction and data caches, for a potential performance increase.
        // Note that this can make a significant performance impact for some CPU-bound tasks.
        // Note: This can increase power use.

        #[cfg(not(feature = "l5"))]
        flash.acr.modify(|_, w| unsafe {
            #[cfg(not(feature = "g0"))]
            w.dcrst().set_bit();
            w.icrst().set_bit()
        });

        // Note: At least on G4, Dcache and ICache are enabled by default in hardware. Although Prefetch isn't.
        #[cfg(not(feature = "l5"))]
        flash.acr.modify(|_, w| unsafe {
            // G0: Instruction cache, but no data cache.
            w.latency().bits(wait_state as u8);
            #[cfg(not(feature = "g0"))]
            w.dcen().set_bit();
            w.icen().set_bit();
            // Prefetch on the ICode bus can be used to read the next sequential instruction line from the
            // Flash memory while the current instruction line is being requested by the CPU
            // Prefetch is enabled by setting the PRFTEN bit in the Flash access control register
            // (FLASH_ACR). This feature is useful if at least one wait state is needed to access the Flash
            // memory
            w.prften().set_bit()
        });

        #[cfg(feature = "l5")]
        flash
            .acr
            .modify(|_, w| unsafe { w.latency().bits(wait_state as u8) });

        #[cfg(feature = "l5")] // todo: u5 too.
        icache.icache_cr.modify(|_, w| w.en().set_bit());

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

        let mut i = 0;
        macro_rules! wait_hang {
            ($i:expr) => {
                i += 1;
                if i >= MAX_ITERS {
                    return Err(RccError::Hardware);
                }
            };
        }

        // Enable oscillators, and wait until ready.
        match self.input_src {
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                // MSI initializes to the default clock source. Turn it off before
                // Adjusting its speed etc.
                rcc.cr.modify(|_, w| w.msion().clear_bit());

                i = 0;
                while rcc.cr.read().msirdy().bit_is_set() {
                    wait_hang!(i);
                }

                rcc.cr.modify(|_, w| unsafe {
                    w.msirange().bits(range as u8);
                    #[cfg(not(any(feature = "wb", feature = "wl")))]
                    w.msirgsel().set_bit();
                    w.msion().set_bit()
                });
                // Wait for the MSI to be ready.

                i = 0;
                while rcc.cr.read().msirdy().bit_is_clear() {
                    wait_hang!(i);
                }
                // todo: If LSE is enabled, calibrate MSI.
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                // Wait for the HSE to be ready.

                i = 0;
                while rcc.cr.read().hserdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().set_bit());

                i = 0;
                while rcc.cr.read().hsirdy().bit_is_clear() {
                    wait_hang!(i);
                }
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

                        i = 0;
                        while rcc.cr.read().msirdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());

                        i = 0;
                        while rcc.cr.read().hserdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hsi => {
                        rcc.cr.modify(|_, w| w.hsion().set_bit());

                        i = 0;
                        while rcc.cr.read().hsirdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::None => {}
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());

                i = 0;
                while rcc.csr.read().lsirdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());

                i = 0;
                while rcc.bdcr.read().lserdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
        }

        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            #[cfg(feature = "wl")]
            return w.hsebyppwr().bit(self.hse_bypass);
            #[cfg(not(feature = "wl"))]
            w.hsebyp().bit(self.hse_bypass)
        });

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

        // Note that with this code setup, PLLSAI won't work properly unless using
        // the input source is PLL.
        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.

            let mut i = 0;
            while rcc.cr.read().pllrdy().bit_is_set() {
                wait_hang!(i);
            }

            cfg_if! {
                if #[cfg(feature = "g0")] {
                    rcc.pllsyscfgr.modify(|_, w| unsafe {
                        w.pllsrc().bits(pll_src.bits());
                        w.pllren().bit(true);
                        w.pllqen().bit(self.pll.pllq_en);
                        w.pllpen().bit(self.pll.pllp_en);
                        w.plln().bits(self.pll.divn);
                        w.pllm().bits(self.pll.divm as u8);
                        w.pllr().bits(self.pll.divr as u8);
                        w.pllq().bits(self.pll.divq as u8);
                        w.pllp().bits(self.pll.divp as u8)
                    });
                } else {
                    rcc.pllcfgr.modify(|_, w| unsafe {
                        w.pllsrc().bits(pll_src.bits());
                        w.pllren().bit(true);
                        w.pllqen().bit(self.pll.pllq_en);
                        w.pllpen().bit(self.pll.pllp_en);
                        w.plln().bits(self.pll.divn);
                        w.pllm().bits(self.pll.divm as u8);
                        w.pllr().bits(self.pll.divr as u8);
                        #[cfg(not(any(feature = "wb", feature = "wl", feature = "l4x5", feature = "l4x3")))]
                        w.pllpdiv().bits(self.pll.pdiv);
                        #[cfg(not(any(feature = "wb", feature = "wl")))]
                        w.pllp().bit(self.pll.divp as u8 != 0);
                        #[cfg(any(feature = "wb", feature = "wl"))]
                        w.pllp().bits(self.pll.divp as u8);
                        w.pllq().bits(self.pll.divq as u8)


                    });
                }
            }

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
                // todo: Missing some settings I'm not sure what to make of on L.
                if #[cfg(any(feature = "l4", feature = "l5"))] {
                    rcc.pllsai1cfgr.modify(|_, w| unsafe {
                        w.pllsai1ren().bit(self.pllsai1.pllr_en);
                        w.pllsai1qen().bit(self.pllsai1.pllq_en);
                        w.pllsai1pen().bit(self.pllsai1.pllp_en);
                        w.pllsai1n().bits(self.pllsai1.divn);
                        #[cfg(not(any(feature = "l4x5", feature = "l4x3")))]
                        w.pllsai1pdiv().bits(self.pllsai1.pdiv);
                        w.pllsai1r().bits(self.pllsai1.divr as u8);
                        w.pllsai1q().bits(self.pllsai1.divq as u8);
                        w.pllsai1p().bit(self.pllsai1.divp as u8 != 0)
                    });

                    #[cfg(any(feature = "l4x5", feature = "l4x6"))]
                    rcc.pllsai2cfgr.modify(|_, w| unsafe {
                        w.pllsai2ren().bit(self.pllsai1.pllr_en);
                        // w.pllsai2qen().bit(self.pllsai1.pllq_en);
                        w.pllsai2pen().bit(self.pllsai1.pllp_en);
                        w.pllsai2n().bits(self.pllsai1.divn);
                        #[cfg(not(feature = "l4x5"))]
                        w.pllsai2pdiv().bits(self.pllsai1.pdiv);
                        w.pllsai2r().bits(self.pllsai1.divr as u8);
                        // w.pllsai2q().bits(self.pllsai1.divq as u8);
                        w.pllsai2p().bit(self.pllsai1.divp as u8 != 0)
                    });

                } else if #[cfg(feature = "wb")] {
                    rcc.pllsai1cfgr.modify(|_, w| unsafe {
                        w.pllren().bit(self.pllsai1.pllr_en);
                        w.pllqen().bit(self.pllsai1.pllq_en);
                        w.pllpen().bit(self.pllsai1.pllp_en);
                        w.plln().bits(self.pllsai1.divn);
                        w.pllr().bits(self.pllsai1.divr as u8);
                        w.pllq().bits(self.pllsai1.divq as u8);
                        w.pllp().bits(self.pllsai1.divp as u8)
                    });
                }
            }

            rcc.cr.modify(|_, w| w.pllon().set_bit());
            i = 0;
            while rcc.cr.read().pllrdy().bit_is_clear() {
                wait_hang!(i);
            }

            cfg_if! {
                if #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))] {
                    if self.pllsai1.enabled {
                        rcc.cr.modify(|_, w| w.pllsai1on().set_bit());
                        i = 0;
                        while rcc.cr.read().pllsai1rdy().bit_is_clear() {wait_hang!(i);}
                    }
                    #[cfg(any(feature = "l4x5", feature = "l4x6",))]
                    if self.pllsai2.enabled {
                        rcc.cr.modify(|_, w| w.pllsai2on().set_bit());
                        i = 0;
                        while rcc.cr.read().pllsai2rdy().bit_is_clear() {wait_hang!(i);}
                    }
                }
            }
        }

        // Enable the HSI48 as required, which is used for USB, RNG, etc.
        // Only valid for some devices (On at least L4, and G4.)
        #[cfg(not(any(feature = "g0", feature = "wl")))]
        if self.hsi48_on {
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
            i = 0;
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {
                wait_hang!(i);
            }
        }

        // This modification is separate from the other CCIPR writes due to awkward
        // feature-gate code
        #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl", feature = "l5")))]
        rcc.ccipr
            .modify(|_, w| unsafe { w.sai1sel().bits(self.sai1_src as u8) });

        #[cfg(feature = "g4")]
        rcc.ccipr
            .modify(|_, w| unsafe { w.fdcansel().bits(self.can_src as u8) });

        #[cfg(feature = "g0c1")]
        rcc.ccipr2
            .modify(|_, w| unsafe { w.fdcansel().bits(self.can_src as u8) });

        #[cfg(feature = "l5")]
        rcc.ccipr2
            .modify(|_, w| unsafe { w.sai1sel().bits(self.sai1_src as u8) });

        #[cfg(any(feature = "l4", feature = "g4"))]
        rcc.ccipr
            .modify(|_, w| unsafe { w.clk48sel().bits(self.clk48_src as u8) });

        #[cfg(feature = "l5")]
        rcc.ccipr1
            .modify(|_, w| unsafe { w.clk48msel().bits(self.clk48_src as u8) });

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

        Ok(())
    }

    /// Re-select input source; used after Stop and Standby modes, where the system reverts
    /// to MSI or HSI after wake.
    pub fn reselect_input(&self) -> Result<(), RccError> {
        let rcc = unsafe { &(*RCC::ptr()) };

        macro_rules! wait_hang {
            ($i:expr) => {
                $i += 1;
                if $i >= MAX_ITERS {
                    return Err(RccError::Hardware);
                }
            };
        }

        // Re-select the input source; useful for changing input source, or reverting
        // from stop or standby mode. This assumes we're on a clean init,
        // or waking up from stop mode etc.

        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                let mut i = 0;
                while rcc.cr.read().hserdy().bit_is_clear() {
                    wait_hang!(i);
                }

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll(pll_src) => {
                // todo: DRY with above.
                match pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        let mut i = 0;
                        while rcc.cr.read().hserdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hsi => {
                        #[cfg(any(feature = "l4", feature = "l5"))]
                        // Generally reverts to MSI (see note below)
                        if let StopWuck::Msi = self.stop_wuck {
                            rcc.cr.modify(|_, w| w.hsion().set_bit());
                            let mut i = 0;
                            while rcc.cr.read().hsirdy().bit_is_clear() {
                                wait_hang!(i);
                            }
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
                            let mut i = 0;
                            while rcc.cr.read().msirdy().bit_is_clear() {
                                wait_hang!(i);
                            }
                        }
                    }
                    PllSrc::None => (),
                }

                rcc.cr.modify(|_, w| w.pllon().clear_bit());
                let mut i = 0;
                while rcc.cr.read().pllrdy().bit_is_set() {
                    wait_hang!(i);
                }

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pllon().set_bit());
                let mut i = 0;
                while rcc.cr.read().pllrdy().bit_is_clear() {
                    wait_hang!(i);
                }
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
                        let mut i = 0;
                        while rcc.cr.read().hsirdy().bit_is_clear() {
                            wait_hang!(i);
                        }

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
                    let mut i = 0;
                    while rcc.cr.read().msirdy().bit_is_clear() {
                        wait_hang!(i);
                    }

                    rcc.cfgr
                        .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());
                let mut i = 0;
                while rcc.csr.read().lsirdy().bit_is_clear() {
                    wait_hang!(i);
                }
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());
                let mut i = 0;
                while rcc.bdcr.read().lserdy().bit_is_clear() {
                    wait_hang!(i);
                }
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
        }

        Ok(())
    }

    #[cfg(any(feature = "l4", feature = "l5"))]
    /// Use this to change the MSI speed. Run this only if your clock source is MSI.
    /// Ends in a state with MSI on at the new speed, and HSI off.
    pub fn change_msi_speed(&mut self, range: MsiRange) {
        // todo: Calibrate MSI with LSE / HSE(?) if avail?

        let rcc = unsafe { &(*RCC::ptr()) };

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
    pub fn enable_msi_48(&self) {
        let rcc = unsafe { &(*RCC::ptr()) };

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

    /// Get the sysclock frequency, in hz.
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll(pll_src) => {
                let input_freq = match pll_src {
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => range.value() as u32,
                    PllSrc::Hsi => 16_000_000,
                    PllSrc::Hse(freq) => freq,
                    PllSrc::None => unimplemented!(),
                };
                input_freq / self.pll.divm.value() as u32 * self.pll.divn as u32
                    / self.pll.divr.value() as u32
            }

            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => range.value() as u32,
            InputSrc::Hsi => 16_000_000,
            InputSrc::Hse(freq) => freq,
            #[cfg(feature = "g0")]
            InputSrc::Lsi => 32_000,
            #[cfg(feature = "g0")]
            InputSrc::Lse => 32_768,
        }
    }

    /// Check if the PLL is enabled. This is useful if checking whether to re-enable the PLL
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

    /// Get the HCLK frequency, in hz
    pub fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    /// Get the systick frequency, in  hz
    pub fn systick(&self) -> u32 {
        self.hclk()
    }

    cfg_if! {
    if #[cfg(any(feature = "g0", feature = "wl"))] {
    pub fn usb(&self) -> u32 {
    unimplemented!("No USB on G0 or WL");
    }
    } else if #[cfg(feature = "g4")] {
    pub fn usb(&self) -> u32 {
    48_000_000 // Uses hsi48.
    }
    } else { // L4 and L5
    pub fn usb(&self) -> u32 {
    match self.clk48_src {
    Clk48Src::Hsi48 => 48_000_000,
    Clk48Src::PllSai1 => unimplemented!(),
    Clk48Src::Pllq => unimplemented!(),
    Clk48Src::Msi => unimplemented!(),
    }
    }
    }
    }

    /// Get the APB1 peripheral clock frequency frequency, in hz
    pub fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    /// Get the frequency used by APB1 timers, in hz
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
    // On G0, a single APB prescaler is used for both APB1 and APB2.
    pub fn apb2(&self) -> u32 {
    self.hclk() / self.apb1_prescaler.value() as u32
    }

    pub fn apb2_timer(&self) -> u32 {
    if let ApbPrescaler::Div1 = self.apb1_prescaler {
    self.apb2()
    } else {
    self.apb2() * 2
    }
    }
    } else {
    /// Get the APB2 peipheral clock frequency, in hz.
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

    /// Get the SAI audio clock frequency, in hz
    #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
    pub fn sai1_speed(&self) -> u32 {
        let pll_src = match self.input_src {
            InputSrc::Msi(msi_rng) => PllSrc::Msi(msi_rng),
            InputSrc::Hsi => PllSrc::Hsi,
            InputSrc::Hse(freq) => PllSrc::Hse(freq),
            InputSrc::Pll(pll_src) => pll_src,
        };

        // todo: DRY with `sysclk`
        let input_freq = match pll_src {
            PllSrc::Msi(range) => range.value() as u32,
            PllSrc::Hsi => 16_000_000,
            PllSrc::Hse(freq) => freq,
            PllSrc::None => unimplemented!(),
        };

        match self.sai1_src {
            SaiSrc::PllSai1P => {
                input_freq / self.pll.divm.value() as u32 * self.pllsai1.divn as u32
                    / self.pllsai1.pvalue() as u32
            }
            SaiSrc::Pllp => {
                input_freq / self.pll.divm.value() as u32 * self.pll.divn as u32
                    / self.pll.pvalue() as u32
            }
            SaiSrc::Hsi => 16_000_000,
            SaiSrc::ExtClk => unimplemented!(),
        }
    }

    pub fn validate_speeds(&self) -> Result<(), RccError> {
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
        if self.pll.divn < 7
            || self.pll.divn > 86
            || self.pllsai1.divn < 7
            || self.pllsai1.divn > 86
        {
            return Err(RccError::Speed);
        }

        if self.pll.pdiv == 1 {
            return Err(RccError::Speed);
        }

        #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
        if self.pllsai1.pdiv == 1 {
            return Err(RccError::Speed);
        }

        #[cfg(any(feature = "l4x5", feature = "l4x6"))]
        if self.pllsai2.pdiv == 1 {
            return Err(RccError::Speed);
        }

        #[cfg(feature = "g0")]
        if self.pll.divn < 9 || self.pll.divn > 86 {
            return Err(RccError::Speed);
        }

        #[cfg(feature = "g4")]
        if self.pll.divn < 8 || self.pll.divn > 127 {
            return Err(RccError::Speed);
        }

        // todo: on WB, input src / PlLM * plln Must be between 96 and 344 Mhz.
        // todo; Cube will validate this. Others probably have a similar restriction.
        // todo: Put this check here.

        // todo: QC these limits
        // todo: Note that this involves repeatedly calculating sysclk.
        // todo. We could work around thsi by calcing it once here.
        if self.sysclk() > max_clock {
            return Err(RccError::Speed);
        }

        // todo: What are the actual hclk limits? Not always sysclk?

        if self.hclk() > max_clock {
            return Err(RccError::Speed);
        }

        if self.apb1() > max_clock {
            return Err(RccError::Speed);
        }

        #[cfg(not(feature = "g0"))]
        if self.apb2() > max_clock {
            return Err(RccError::Speed);
        }

        Ok(())
    }
}

impl Default for Clocks {
    /// This default configures clocks with a HSI, with system and peripheral clocks at full rated speed.
    /// All peripheral. Speeds -> L4: 80Mhz. L5: 110Mhz. G0: 64Mhz. G4: 170Mhz. WB: 64Mhz.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pll: PllCfg::default(),
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            pllsai1: PllCfg::disabled(),
            #[cfg(any(feature = "l4x5", feature = "l4x6"))]
            pllsai2: PllCfg::disabled(),
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
            hse_bypass: false,
            security_system: false,
            #[cfg(not(any(feature = "g0", feature = "wl")))]
            hsi48_on: false,
            #[cfg(any(feature = "l4", feature = "l5", feature = "wb", feature = "wl"))]
            stop_wuck: StopWuck::Msi,
            #[cfg(feature = "wb")]
            rf_wakeup_src: RfWakeupSrc::Lse,
            #[cfg(not(any(feature = "g0", feature = "g4", feature = "wl")))]
            sai1_src: SaiSrc::Pllp,
            #[cfg(feature = "g4")]
            boost_mode: true,
            #[cfg(any(feature = "g0", feature = "g4"))]
            can_src: CanSrc::Pclk,
        }
    }
}

#[cfg(any(feature = "l4", feature = "l5", feature = "g4", feature = "wb"))]
/// Enable the Clock Recovery System. L443 User manual:
/// "The STM32L443xx devices embed a special block which allows automatic trimming of the
/// internal 48 MHz oscillator to guarantee its optimal accuracy over the whole device
/// operational range. This automatic trimming is based on the external synchronization signal,
/// which could be either derived from USB SOF signalization, from LSE oscillator, from an
/// external signal on CRS_SYNC pin or generated by user software. For faster lock-in during
/// startup it is also possible to combine automatic trimming with manual trimming action."
/// Note: This is for HSI48 only. Note that the HSI will turn off after entering Stop or Standby.
pub fn enable_crs(sync_src: CrsSyncSrc) {
    let crs = unsafe { &(*CRS::ptr()) };
    let rcc = unsafe { &(*RCC::ptr()) };

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
