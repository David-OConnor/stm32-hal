//! Clock config for STM32L, G, and W-series MCUs. Uses a `Clocks` struct to configure
//! settings, starting with a `Default::default()` implementation. Uses the `setup` method
//! to write changes.

// Similar in from to the `baseline` clocks module, but includes notable differendes.

use cfg_if::cfg_if;

#[cfg(not(any(feature = "h5", feature = "h7b3", feature = "h735")))]
use crate::pac::SYSCFG;
use crate::{
    clocks::RccError,
    pac::{CRS, FLASH, PWR, RCC},
    MAX_ITERS,
};

#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Csi,
    Hsi(HsiDiv),
    Hse(u32),
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    /// See RCC_PLLCKSELR register, PLLSRC field.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi(_) => 0b00,
            Self::Csi => 0b01,
            Self::Hse(_) => 0b10,
            Self::None => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Select the system clock used when exiting Stop mode. Sets RCC_CFGR register, STOPWUCK field.
pub enum StopWuck {
    Hsi = 0,
    Csi = 1,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Selects USB clock source. Sets D2CCIP2R reg, USBSEL field.
pub enum UsbSrc {
    Disabled = 0b00,
    Pll1Q = 0b01,
    Pll3Q = 0b10,
    Hsi48 = 0b11,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select the SYNC signal source. Sets the CRS_CFGR register, SYNCSRC field.
pub enum CrsSyncSrc {
    #[cfg(feature = "h735")] // todo: 735 and 743 are opposites here. QC which use which.
    /// CRS_SYNC pin selected as SYNC signal source
    CrsSync = 0b00,
    #[cfg(not(feature = "h735"))]
    /// USB2 SOF selected as SYNC signal source
    Usb2 = 0b00,
    /// LSE selected as SYNC signal source
    Lse = 0b01,
    /// OTG HS1 SOF selected as SYNC signal source
    OtgHs = 0b10,
}

#[derive(Clone, Copy, PartialEq)]
/// Clock input source, also known as system clock switch. Sets RCC_CFGR register, SW field.
pub enum InputSrc {
    Hsi(HsiDiv),
    Csi,
    Hse(u32), // freq in Mhz,
    Pll1,
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    /// See RCC_CFGR register, SW field.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Hsi(_) => 0b000,
            Self::Csi => 0b001,
            Self::Hse(_) => 0b010,
            Self::Pll1 => 0b011,
        }
    }
}

/// Configures the speeds, and enable status of an individual PLL. Note that the `enable`
/// field has no effect for PLL1.
pub struct PllCfg {
    pub enabled: bool,
    // pub fractional: bool,
    pub pllp_en: bool,
    pub pllq_en: bool,
    pub pllr_en: bool,
    pub divm: u8,
    pub divn: u16,
    pub divp: u8,
    pub divq: u8,
    pub divr: u8,
}

impl Default for PllCfg {
    // Note that this assumes VOS1. (Not full speed)
    fn default() -> Self {
        cfg_if! {
            if #[cfg(feature = "h5")] {
                let divn = 250;
            } else if #[cfg(feature = "h7b3")] {
                let divn = 280;
            } else {
                let divn = 400;
            }
        }

        Self {
            enabled: true,
            // fractional: false,
            pllp_en: true,
            pllq_en: false,
            pllr_en: false,
            // todo: Getting mixed messages on if HSI on H5 is 16Mhz or 32Mhz.
            // todo: Great as 32Mhz for now, and see if speed is twice as slow
            // todo as it should be.
            #[cfg(feature = "h5")]
            divm: 32, // todo 16?
            #[cfg(feature = "h7")]
            divm: 32,
            divn,
            // We override DIVP1 to be 1, and lower DIVN1 to achieve full speed on H723 etc
            // variants.
            divp: 2,
            // Div1Q = 2 Allows <150Mhz SAI clock, if it's configured for PLL1Q (which is its default).
            // Div1Q = 8 Allows <200Mhz SPI1 clock, if it's configured for PLL1Q (which is its default).
            // At 400Mhz, Sets SAI clock to 100Mhz. At 480Mhz, sets it to 120Mhz.
            divq: 8,
            divr: 2,
        }
    }
}

impl PllCfg {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            pllq_en: false,
            pllp_en: false,
            pllr_en: false,
            ..Default::default()
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Division factor for the AHB clock. Also known as AHB Prescaler. See RCC_D1CFGR reg.
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

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// SAI clock input source. Sets RCC_D2CCIP1R register, SAIxSEL field.
pub enum SaiSrc {
    Pll1Q = 0b000,
    Pll2P = 0b001,
    Pll3P = 0b010,
    I2sCkin = 0b011,
    PerClk = 0100,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// SPI clock input source. Sets RCC_D2CCIP1R register, SPI123SEL field..
pub enum Spi123Src {
    //note: This is the same as SaiSrc.
    Pll1Q = 0b000, // This is PLL2
    Pll2P = 0b001,
    Pll3P = 0b010,
    I2sCkin = 0b011,
    PerClk = 0100,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// SPI clock input source. Sets RCC_D2CCIP1R register, SPI45SEL field.
pub enum Spi45Src {
    Apb = 0b000,
    Pll2Q = 0b001,
    Pll3Q = 0b010,
    Hsi = 0b011,
    Csi = 0100,
    HseCk = 0b101,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// SAI clock input source. Sets RCC_D2CCIP1R register, DFSDM1SEL field.
pub enum DfsdmSrc {
    /// rcc_pclk2 is selected as DFSDM1 Clk kernel clock (default after reset)
    Pclk2 = 0,
    /// sys_ck clock is selected as DFSDM1 Clk kernel clock
    Sysclk = 1,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// CAN clock input source. Sets RCC_D2CCIP1R register, FDCANSEL field.
pub enum CanSrc {
    /// hse_ck clock is selected as FDCAN kernel clock (default after reset)
    Hse = 0b00,
    /// PLL1Q
    Pll1Q = 0b01,
    /// PLL2Q
    Pll2Q = 0b10,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Clock divider for the HSI. See RCC_CR register, HSIDIV field.
pub enum HsiDiv {
    Div1 = 0b00,
    Div2 = 0b01,
    Div4 = 0b10,
    Div8 = 0b11,
}

impl HsiDiv {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Range for the VOS. See H743 RM, section 6.8.6: PWR D3 domain control register. Sets PWR_D3CR,
/// `VOS` field.
pub enum VosRange {
    /// 1.26 V - 1.40 V
    #[cfg(not(feature = "h7b3"))]
    VOS0 = 0, // This will actually be VOS1, but note that we handle the case of VOS0 activation
    // differntly than the others, using its special activation sequence.
    /// 1.15 V - 1.26 V
    VOS1 = 0b11,
    /// 1.05 V - 1.15 V
    VOS2 = 0b10,
    /// 0.95 V - 1.05 V
    VOS3 = 0b01,
}

impl VosRange {
    /// Power regulator voltage scale.
    /// Choose the wait states based on VSO range and hclk frequency.. See H743 RM, Table 17: FLASH,
    /// or RM0468, table 16.
    /// recommended number of wait states and programming delay. Returns a tuple of (number of wait states,
    /// programming delay) (FLASH ACR_LATENCY, WRHIGHFREQ) values respectively.
    pub fn wait_states(&self, hclk: u32) -> (u8, u8) {
        // todo: 280 Mhz variants.
        #[cfg(not(feature = "h735"))]
        match self {
            #[cfg(not(feature = "h7b3"))]
            Self::VOS0 => match hclk {
                0..=70_000_000 => (0, 0),
                70_000_001..=140_000_000 => (1, 1),
                140_000_001..=185_000_000 => (2, 1),
                185_000_001..=210_000_000 => (2, 2),
                210_000_001..=225_000_000 => (3, 2),
                225_000_001..=240_000_000 => (4, 2),
                _ => panic!("Can't set higher than 240Mhz HCLK with VSO0 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS1 => match hclk {
                0..=70_000_000 => (0, 0),
                70_000_001..=140_000_000 => (1, 1),
                140_000_001..=185_000_000 => (2, 1),
                185_000_001..=210_000_000 => (2, 2),
                210_000_001..=225_000_000 => (3, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VOS1 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS2 => match hclk {
                0..=55_000_000 => (0, 0),
                55_000_001..=110_000_000 => (1, 1),
                110_000_001..=165_000_000 => (2, 1),
                165_000_001..=225_000_000 => (3, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VSO2 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS3 => match hclk {
                0..=45_000_000 => (0, 0),
                45_000_001..=90_000_000 => (1, 1),
                90_000_001..=135_000_000 => (2, 1),
                135_000_001..=180_000_000 => (3, 2),
                180_000_001..=225_000_000 => (4, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VSO3 range. (Try changing the `vos_range` setting)."),
            },
        }

        #[cfg(feature = "h735")]
        match self {
            Self::VOS0 => match hclk {
                0..=70_000_000 => (0, 0b00),
                70_000_001..=140_000_000 => (1, 0b01),
                140_000_001..=210_000_000 => (2, 0b10),
                210_000_001..=275_000_000 => (3, 0b11),
                _ => panic!("Can't set higher than 275Mhz HCLK with VSO0 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS1 => match hclk {
                0..=67_000_000 => (0, 0b00),
                67_000_001..=133_000_000 => (1, 0b01),
                133_000_001..=200_000_000 => (2, 0b10),
                _ => panic!("Can't set higher than 200Mhz HCLK with VOS1 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS2 => match hclk {
                0..=50_000_000 => (0, 0b00),
                50_000_001..=100_000_000 => (1, 0b01),
                100_000_001..=150_000_000 => (2, 0b10),
                _ => panic!("Can't set higher than 150Mhz HCLK with VSO2 range. (Try changing the `vos_range` setting)."),
            },
            Self::VOS3 => match hclk {
                0..=35_000_000 => (0, 0b00),
                35_000_001..=70_000_000 => (1, 0b01),
                70_000_001..=85_000_000 => (2, 0b10),
                _ => panic!("Can't set higher than 85Mhz HCLK with VSO3 range. (Try changing the `vos_range` setting)."),
            },
        }
    }
}

/// Settings used to configure clocks. Create this struct by using its `Default::default()`
/// implementation, then modify as required, referencing your RM's clock tree,
/// or Stm32Cube IDE's interactive clock manager. Apply settings by running `.setup()`.
pub struct Clocks {
    /// The main input source
    pub input_src: InputSrc,
    /// The source driving all PLLs.
    pub pll_src: PllSrc,
    /// Enable and speed status for PLL1. Note that `input_src` controls if PLL1 is enabled, not
    /// `pll1.enabled()`.
    pub pll1: PllCfg,
    /// Enable and speed status for PLL2
    pub pll2: PllCfg,
    /// Enable and speed status for PLL3
    pub pll3: PllCfg,
    #[cfg(feature = "h7")]
    /// The prescaler between sysclk and hclk
    pub d1_core_prescaler: HclkPrescaler,
    /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
    pub hclk_prescaler: HclkPrescaler,
    /// APB3 peripheral clocks
    pub d1_prescaler: ApbPrescaler,
    /// APB1 peripheral clocks
    pub d2_prescaler1: ApbPrescaler,
    // todo: D2 prescaler 2 possibly not on H5.
    /// APB2 peripheral clocks
    pub d2_prescaler2: ApbPrescaler,
    #[cfg(feature = "h7")]
    /// APB4 peripheral clocks
    pub d3_prescaler: ApbPrescaler,
    /// Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    /// frees up the pin for use as GPIO.
    pub hse_bypass: bool,
    /// USBOTG kernel clock selection. Defaults to HSI48.
    pub usb_src: UsbSrc,
    pub security_system: bool,
    pub hsi48_on: bool,
    pub stop_wuck: StopWuck,
    pub vos_range: VosRange,
    /// SAI1 and DFSDM1 kernel Aclk clock source selection
    pub sai1_src: SaiSrc,
    #[cfg(not(feature = "h735"))]
    /// SAI2 and SAI3 kernel clock source selection
    pub sai23_src: SaiSrc,
    pub sai4a_src: SaiSrc,
    pub sai4b_src: SaiSrc,
    pub spi123_src: Spi123Src,
    pub spi45_src: Spi45Src,
    /// DFSDM1 kernel clock source selection
    pub dfsdm1_src: DfsdmSrc,
    /// FDCAN kernel clock selection. Defaults to PLL1Q.
    pub can_src: CanSrc,
}

impl Clocks {
    /// Setup common and return Ok if the config is valid. Abort the setup if speeds
    /// are invalid.
    /// Use the STM32CubeIDE Clock Configuration tab to identify valid configs.
    /// Use the `default()` implementation as a safe baseline.
    /// This method also configures the PWR VOS setting, and can be used to enable VOS boost,
    /// if `vos_range` is set to `VosRange::VOS0`.
    pub fn setup(&self) -> Result<(), RccError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        let rcc = unsafe { &(*RCC::ptr()) };
        let flash = unsafe { &(*FLASH::ptr()) };
        let pwr = unsafe { &(*PWR::ptr()) };

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        #[cfg(feature = "h7")]
        {
            rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
            rcc.apb4rstr.modify(|_, w| w.syscfgrst().set_bit());
            rcc.apb4rstr.modify(|_, w| w.syscfgrst().clear_bit());
        }

        let mut i = 0;

        macro_rules! wait_hang {
            ($i:expr) => {
                i += 1;
                if i >= MAX_ITERS {
                    return Err(RccError::Hardware);
                }
            };
        }

        // H743 RM, sefction 6.8.6, and section 6.6.2: Voltage Scaling
        //  Voltage scaling selection according to performance
        // These bits control the VCORE voltage level and allow to obtains the best trade-off between
        // power consumption and performance:
        // – When increasing the performance, the voltage scaling shall be changed before increasing
        // the system frequency.
        // – When decreasing performance, the system frequency shall first be decreased before
        // changing the voltage scaling.
        match self.vos_range {
            // todo: Do we need this on H5?
            #[cfg(not(any(feature = "h5", feature = "h7b3", feature = "h735")))]
            // Note:H735 etc have VOS0, but not oden; the RM doesn't list these steps.
            VosRange::VOS0 => {
                let syscfg = unsafe { &(*SYSCFG::ptr()) };

                // VOS0 activation/deactivation sequence: H743 HRM, section 6.6.2:
                // The system maximum frequency can be reached by boosting the voltage scaling level to
                // VOS0. This is done through the ODEN bit in the SYSCFG_PWRCR register.
                // The sequence to activate the VOS0 is the following:
                // 1. Ensure that the system voltage scaling is set to VOS1 by checking the VOS bits in
                // PWR D3 domain control register (PWR D3 domain control register (PWR_D3CR))
                cfg_if! {
                    if #[cfg(feature = "h7")] {
                        pwr.d3cr
                            .modify(|_, w| unsafe { w.vos().bits(VosRange::VOS1 as u8) });

                        i = 0;
                        while pwr.d3cr.read().vosrdy().bit_is_clear() {wait_hang!(i);}
                    } else {
                        pwr.voscr
                            .modify(|_, w| unsafe { w.vos().bits(VosRange::VOS1 as u8) });

                        i = 0;
                        while pwr.vossr.read().vosrdy().bit_is_clear() {wait_hang!(i);}
                    }
                }

                // 2. Enable the SYSCFG clock in the RCC by setting the SYSCFGEN bit in the
                // RCC_APB4ENR register.
                // (Handled above)

                // 3. Enable the ODEN bit in the SYSCFG_PWRCR register.
                syscfg.pwrcr.modify(|_, w| w.oden().set_bit());

                // 4. Wait for VOSRDY to be set.
                i = 0;
                #[cfg(feature = "h7")]
                while pwr.d3cr.read().vosrdy().bit_is_clear() {
                    wait_hang!(i);
                }
                #[cfg(feature = "h5")]
                while pwr.vossr.read().vosrdy().bit_is_clear() {
                    wait_hang!(i);
                }

                // Once the VCORE supply has reached the required level, the system frequency can be
                // increased. Figure 31 shows the recommended sequence for switching VCORE from VOS1 to
                // VOS0 sequence.
                // The sequence to deactivate the VOS0 is the following:
                // 1. Ensure that the system frequency was decreased.
                // 2. Ensure that the SYSCFG clock is enabled in the RCC by setting the SYSCFGEN bit set
                // in the RCC_APB4ENR register.
                // 3. Reset the ODEN bit in the SYSCFG_PWRCR register to disable VOS0.
            }
            _ => {
                #[cfg(feature = "h7")]
                pwr.d3cr
                    .modify(|_, w| unsafe { w.vos().bits(self.vos_range as u8) });
                #[cfg(feature = "h5")]
                pwr.voscr
                    .modify(|_, w| unsafe { w.vos().bits(self.vos_range as u8) });
            }
        }

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        // H742 RM, Table 17.
        let wait_states = self.vos_range.wait_states(self.hclk());

        flash.acr.modify(|_, w| unsafe {
            w.latency().bits(wait_states.0);
            w.wrhighfreq().bits(wait_states.1)
        });

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::Csi => {
                rcc.cr.modify(|_, w| w.csion().bit(true));
                i = 0;
                while rcc.cr.read().csirdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                i = 0;
                while rcc.cr.read().hserdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            InputSrc::Hsi(div) => {
                rcc.cr.modify(|_, w| unsafe {
                    w.hsidiv().bits(div as u8);
                    w.hsion().bit(true)
                });
                i = 0;
                while rcc.cr.read().hsirdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            InputSrc::Pll1 => {
                // todo: PLL setup here is DRY with the HSE, HSI, and Csi setup above.
                match self.pll_src {
                    PllSrc::Csi => {
                        rcc.cr.modify(|_, w| w.csion().bit(true));
                        i = 0;
                        while rcc.cr.read().csirdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().bit(true));
                        i = 0;
                        while rcc.cr.read().hserdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hsi(div) => {
                        rcc.cr.modify(|_, w| unsafe {
                            w.hsidiv().bits(div as u8);
                            w.hsion().bit(true)
                        });
                        i = 0;
                        while rcc.cr.read().hsirdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::None => {}
                }
            }
        }

        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hsebyp().bit(self.hse_bypass)
        });

        rcc.cfgr.modify(|_, w| unsafe {
            w.sw().bits(self.input_src.bits());
            w.stopwuck().bit(self.stop_wuck as u8 != 0)
        });

        #[cfg(feature = "h7")]
        rcc.d1cfgr.modify(|_, w| unsafe {
            w.d1cpre().bits(self.d1_core_prescaler as u8);
            w.d1ppre().bits(self.d1_prescaler as u8);
            w.hpre().bits(self.hclk_prescaler as u8)
        });

        #[cfg(feature = "h7")]
        rcc.d2cfgr.modify(|_, w| unsafe {
            w.d2ppre1().bits(self.d2_prescaler1 as u8);
            w.d2ppre2().bits(self.d2_prescaler2 as u8)
        });

        #[cfg(feature = "h7")]
        rcc.d3cfgr
            .modify(|_, w| unsafe { w.d3ppre().bits(self.d3_prescaler as u8) });

        #[cfg(feature = "h5")]
        rcc.cfgr2.modify(|_, w| unsafe {
            w.ppre1().bits(self.d1_prescaler as u8);
            w.ppre2().bits(self.d2_prescaler1 as u8);
            w.ppre3().bits(self.d2_prescaler2 as u8);
            w.hpre().bits(self.hclk_prescaler as u8)
        });

        #[cfg(not(any(feature = "h7b3", feature = "h5")))]
        rcc.d2ccip1r.modify(|_, w| unsafe {
            w.sai1sel().bits(self.sai1_src as u8);
            #[cfg(not(feature = "h735"))]
            w.sai23sel().bits(self.sai23_src as u8);
            w.spi123sel().bits(self.spi123_src as u8);
            w.spi45sel().bits(self.spi45_src as u8);
            w.dfsdm1sel().bit(self.dfsdm1_src as u8 != 0);
            w.fdcansel().bits(self.can_src as u8)
        });

        // todo: Add config enums for these, and add them as Clocks fields.
        #[cfg(not(any(feature = "h7b3", feature = "h5")))]
        rcc.d2ccip2r
            .modify(|_, w| unsafe { w.usbsel().bits(self.usb_src as u8) });

        #[cfg(not(any(feature = "h7b3", feature = "h5")))]
        rcc.d3ccipr.modify(|_, w| unsafe {
            w.sai4asel().bits(self.sai4a_src as u8);
            w.sai4bsel().bits(self.sai4b_src as u8)
        });

        // #[cfg(feature = "h5")]
        // rcc.ccipr1.modify(|_, w| unsafe {
        // });

        // #[cfg(feature = "h5")]
        // rcc.ccipr2.modify(|_, w| unsafe {
        // });

        #[cfg(feature = "h5")]
        rcc.ccipr3.modify(|_, w| unsafe {
            // todo: This is broken down into each spi individually on H5.
            w.spi1sel().bits(self.spi123_src as u8);
            w.spi2sel().bits(self.spi123_src as u8);
            w.spi3sel().bits(self.spi45_src as u8);
            w.spi4sel().bits(self.spi45_src as u8);
            w.spi5sel().bits(self.spi123_src as u8)
            // w.spi6sel().bits(self.spi123_src as u8);
        });

        #[cfg(feature = "h5")]
        rcc.ccipr4.modify(|_, w| unsafe {
            w.usbfssel().bits(self.usb_src as u8)
            // Also: OctoSPI and I2C.
        });

        #[cfg(feature = "h5")]
        rcc.ccipr5.modify(|_, w| unsafe {
            w.sai1sel().bits(self.sai1_src as u8);
            w.sai2sel().bits(self.sai23_src as u8);
            w.fdcan12sel().bits(self.can_src as u8)
            // also: ADC and DAC.
        });

        rcc.cr.modify(|_, w| w.hsecsson().bit(self.security_system));

        // todo: Allow configuring the PLL in fractional mode.

        #[cfg(feature = "h7")]
        rcc.pllckselr
            .modify(|_, w| w.pllsrc().bits(self.pll_src.bits()));

        // Note that with this code setup, PLL2 and PLL3 won't work properly unless using
        // the input source is PLL1.
        if let InputSrc::Pll1 = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pll1on().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pll1rdy().bit_is_set() {}

            // Set and reset by software to select the proper reference frequency range used for PLL1.
            // This bit must be written before enabling the PLL1.
            let pll1_rng_val = match self.pll_input_speed(self.pll_src, 1) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL1 input source must be between 1Mhz and 16Mhz."),
            };

            // todo: Don't enable all these pqr etc by default!
            // todo don't enable pll2 and 3 by default!!
            // todo: This has an impact on power consumption.

            // todo: MOre DRY
            // H743 RM:
            // 0: Wide VCO range: 192 to 836 MHz (default after reset)
            // 1: Medium VCO range: 150 to 420 MHz
            let pll1_vco = match self.pll_input_speed(self.pll_src, 1) {
                0..=2_000_000 => 0,
                _ => 1,
            };

            // The user application can then configure the proper VCO: if the frequency of the reference
            // clock is lower or equal to 2 MHz, then VCOL must be selected, otherwise VCOH must be
            // chosen. To reduce the power consumption, it is recommended to configure the VCO output
            // to the lowest frequency.

            // The frequency of the reference clock provided to the PLLs (refx_ck) must range from 1 to
            // 16 MHz. The user application has to program properly the DIVMx dividers of the RCC PLLs
            // Clock Source Selection Register (RCC_PLLCKSELR) in order to match this condition. In
            // addition, the PLLxRGE of the RCC PLLs Configuration Register (RCC_PLLCFGR) field
            // must be set according to the reference input frequency to guarantee an optimal
            // performance of the PLL.

            // If using multiple PLLs, we do a write to `PLLCLKSELR` and `PLLCFGR` for each
            // enabled PLL. This is unecessary, but makes the code clearer. This is worth it, given
            // we expect the user to run `.setup()` only once.
            #[cfg(feature = "h7")]
            rcc.pllckselr.modify(|_, w| w.divm1().bits(self.pll1.divm));

            #[cfg(feature = "h7")]
            rcc.pllcfgr.modify(|_, w| {
                w.pll1rge().bits(pll1_rng_val);
                w.pll1vcosel().bit(pll1_vco != 0);
                w.divp1en().bit(true);
                w.divq1en().bit(self.pll1.pllq_en);
                w.divr1en().bit(self.pll1.pllr_en)
            });

            #[cfg(feature = "h5")]
            rcc.pll1cfgr.modify(|_, w| unsafe {
                w.pll1src().bits(self.pll_src.bits());
                w.divm1().bits(self.pll1.divm);
                w.pll1rge().bits(pll1_rng_val);
                w.pll1vcosel().bit(pll1_vco != 0);
                w.pll1pen().bit(true);
                w.pll1qen().bit(self.pll1.pllq_en);
                w.pll1ren().bit(self.pll1.pllr_en)
            });

            #[cfg(feature = "h7")]
            rcc.pll1divr.modify(|_, w| unsafe {
                w.divn1().bits(self.pll1.divn - 1);
                w.divp1().bits(self.pll1.divp - 1);
                w.divq1().bits(self.pll1.divq - 1);
                w.divr1().bits(self.pll1.divr - 1)
            });

            #[cfg(feature = "h5")]
            rcc.pll1divr.modify(|_, w| unsafe {
                w.pll1n().bits(self.pll1.divn - 1);
                w.pll1p().bits(self.pll1.divp - 1);
                w.pll1q().bits(self.pll1.divq - 1);
                w.pll1r().bits(self.pll1.divr - 1)
            });

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            rcc.cr.modify(|_, w| w.pll1on().set_bit());
            i = 0;
            while rcc.cr.read().pll1rdy().bit_is_clear() {
                wait_hang!(i);
            }
        }

        // todo DRY
        if self.pll2.enabled {
            rcc.cr.modify(|_, w| w.pll2on().clear_bit());
            i = 0;
            while rcc.cr.read().pll2rdy().bit_is_set() {
                wait_hang!(i);
            }

            let pll2_rng_val = match self.pll_input_speed(self.pll_src, 2) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL2 input source must be between 1Mhz and 16Mhz."),
            };

            let pll2_vco = match self.pll_input_speed(self.pll_src, 2) {
                0..=2_000_000 => 0,
                _ => 1,
            };

            #[cfg(feature = "h7")]
            rcc.pllckselr.modify(|_, w| w.divm2().bits(self.pll2.divm));

            #[cfg(feature = "h7")]
            rcc.pllcfgr.modify(|_, w| {
                w.pll2rge().bits(pll2_rng_val);
                w.pll2vcosel().bit(pll2_vco != 0);
                w.divp2en().bit(self.pll2.pllp_en);
                w.divq2en().bit(self.pll2.pllq_en);
                w.divr2en().bit(self.pll2.pllr_en)
            });

            #[cfg(feature = "h5")]
            rcc.pll2cfgr.modify(|_, w| unsafe {
                w.pll2src().bits(self.pll_src.bits());
                w.pll2rge().bits(pll2_rng_val);
                w.pll2vcosel().bit(pll2_vco != 0);
                w.pll2pen().bit(self.pll2.pllp_en);
                w.pll2qen().bit(self.pll2.pllq_en);
                w.pll2ren().bit(self.pll2.pllr_en)
            });

            #[cfg(feature = "h7")]
            rcc.pll2divr.modify(|_, w| unsafe {
                w.divn2().bits(self.pll2.divn - 1);
                w.divp2().bits(self.pll2.divp - 1);
                w.divq2().bits(self.pll2.divq - 1);
                w.divr2().bits(self.pll2.divr - 1)
            });

            #[cfg(feature = "h5")]
            rcc.pll2divr.modify(|_, w| unsafe {
                w.pll2n().bits(self.pll2.divn - 1);
                w.pll2p().bits(self.pll2.divp - 1);
                w.pll2q().bits(self.pll2.divq - 1);
                w.pll2r().bits(self.pll2.divr - 1)
            });

            rcc.cr.modify(|_, w| w.pll2on().set_bit());
            i = 0;
            while rcc.cr.read().pll2rdy().bit_is_clear() {
                wait_hang!(i);
            }
        }

        if self.pll3.enabled {
            rcc.cr.modify(|_, w| w.pll3on().clear_bit());
            i = 0;
            while rcc.cr.read().pll3rdy().bit_is_set() {
                wait_hang!(i);
            }

            let pll3_rng_val = match self.pll_input_speed(self.pll_src, 3) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL3 input source must be between 1Mhz and 16Mhz."),
            };

            let pll3_vco = match self.pll_input_speed(self.pll_src, 3) {
                0..=2_000_000 => 0,
                _ => 1,
            };

            #[cfg(feature = "h7")]
            rcc.pllckselr.modify(|_, w| w.divm3().bits(self.pll3.divm));

            #[cfg(feature = "h7")]
            rcc.pllcfgr.modify(|_, w| {
                w.pll3rge().bits(pll3_rng_val);
                w.pll3vcosel().bit(pll3_vco != 0);
                w.divp3en().bit(self.pll3.pllp_en);
                w.divq3en().bit(self.pll3.pllq_en);
                w.divr3en().bit(self.pll3.pllr_en)
            });

            #[cfg(feature = "h5")]
            rcc.pll3cfgr.modify(|_, w| unsafe {
                w.pll3src().bits(self.pll_src.bits());
                w.pll3rge().bits(pll3_rng_val);
                w.pll3vcosel().bit(pll3_vco != 0);
                w.pll3pen().bit(self.pll3.pllp_en);
                w.pll3qen().bit(self.pll3.pllq_en);
                w.pll3ren().bit(self.pll3.pllr_en)
            });

            #[cfg(feature = "h7")]
            rcc.pll3divr.modify(|_, w| unsafe {
                w.divn3().bits(self.pll3.divn - 1);
                w.divp3().bits(self.pll3.divp - 1);
                w.divq3().bits(self.pll3.divq - 1);
                w.divr3().bits(self.pll3.divr - 1)
            });

            #[cfg(feature = "h5")]
            rcc.pll3divr.modify(|_, w| unsafe {
                w.pll3n().bits(self.pll3.divn - 1);
                w.pll3p().bits(self.pll3.divp - 1);
                w.pll3q().bits(self.pll3.divq - 1);
                w.pll3r().bits(self.pll3.divr - 1)
            });

            rcc.cr.modify(|_, w| w.pll3on().set_bit());
            i = 0;
            while rcc.cr.read().pll3rdy().bit_is_clear() {
                wait_hang!(i);
            }
        }

        if self.hsi48_on {
            rcc.cr.modify(|_, w| w.hsi48on().set_bit());
            i = 0;
            while rcc.cr.read().hsi48rdy().bit_is_clear() {
                wait_hang!(i);
            }
        }

        Ok(())
    }

    /// Re-select input source; used on Stop and Standby modes, where the system reverts
    /// to HSI after wake.
    pub fn reselect_input(&self) -> Result<(), RccError> {
        // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

        let rcc = unsafe { &(*RCC::ptr()) };

        let mut i = 0;
        macro_rules! wait_hang {
            ($i:expr) => {
                i += 1;
                if i >= MAX_ITERS {
                    return Err(RccError::Hardware);
                }
            };
        }

        // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
        // todo: But this saves a few reg writes.
        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                i = 0;
                while rcc.cr.read().hserdy().bit_is_clear() {
                    wait_hang!(i);
                }

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll1 => {
                // todo: DRY with above.
                match self.pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        i = 0;
                        while rcc.cr.read().hserdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Hsi(div) => {
                        // Generally reverts to Csi (see note below)
                        rcc.cr.modify(|_, w| unsafe {
                            w.hsidiv().bits(div as u8); // todo: Do we need to reset the HSI div after low power?
                            w.hsion().bit(true)
                        });
                        i = 0;
                        while rcc.cr.read().hsirdy().bit_is_clear() {
                            wait_hang!(i);
                        }
                    }
                    PllSrc::Csi => (), // todo
                    PllSrc::None => (),
                }

                // todo: PLL 2 and 3?
                rcc.cr.modify(|_, w| w.pll1on().clear_bit());
                i = 0;
                while rcc.cr.read().pll1rdy().bit_is_set() {
                    wait_hang!(i);
                }

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pll1on().set_bit());
                i = 0;
                while rcc.cr.read().pll1rdy().bit_is_clear() {
                    wait_hang!(i);
                }
            }
            InputSrc::Hsi(div) => {
                {
                    // From Reference Manual, RCC_CFGR register section:
                    // "Configured by HW to force Csi oscillator selection when exiting Standby or Shutdown mode.
                    // Configured by HW to force Csi or HSI16 oscillator selection when exiting Stop mode or in
                    // case of failure of the HSE oscillator, depending on STOPWUCK value."
                    // In tests, from stop, it tends to revert to Csi.
                    rcc.cr.modify(|_, w| unsafe {
                        w.hsidiv().bits(div as u8); // todo: Do we need to reset the HSI div after low power?
                        w.hsion().bit(true)
                    });
                    i = 0;
                    while rcc.cr.read().hsirdy().bit_is_clear() {
                        wait_hang!(i);
                    }
                }
            }
            InputSrc::Csi => (), // ?
        }

        Ok(())
    }

    /// Calculate the input speed to the PLL. This must be between 1 and 16 Mhz. Called `refx_ck`
    /// in the RM.
    pub fn pll_input_speed(&self, pll_src: PllSrc, pll_num: u8) -> u32 {
        let input_freq = match pll_src {
            PllSrc::Csi => 4_000_000,
            // todo: QC if H5 HSI is 64 or 32M.
            PllSrc::Hsi(div) => 64_000_000 / (div.value() as u32),
            PllSrc::Hse(freq) => freq,
            PllSrc::None => 0,
        };

        match pll_num {
            1 => input_freq / (self.pll1.divm as u32),
            2 => input_freq / (self.pll2.divm as u32),
            3 => input_freq / (self.pll3.divm as u32),
            _ => panic!("Pll num must be between 1 and 3."),
        }
    }

    /// Calculate VCO output frequency: = Fref1_ck x DIVN1
    pub fn vco_output_freq(&self, pll_src: PllSrc, pll_num: u8) -> u32 {
        let input_speed = self.pll_input_speed(pll_src, pll_num);
        match pll_num {
            1 => input_speed * self.pll1.divn as u32,
            2 => input_speed * self.pll2.divn as u32,
            3 => input_speed * self.pll3.divn as u32,
            _ => panic!("Pll num must be between 1 and 3."),
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
        rcc.cr.read().pll1on().bit_is_set()
    }

    /// Calculate the sysclock frequency, in hz. Note that for dual core variants, this is for CPU1.
    /// CPU2 syclock is equal to the HCLK, so use the `hclk()` method.
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll1 => {
                // divm1 is included in `pll_input_speed`.
                self.pll_input_speed(self.pll_src, 1) * self.pll1.divn as u32
                    / self.pll1.divp as u32
            }
            InputSrc::Csi => 4_000_000,
            // todo: QC if H5 is 32 or 64M.
            InputSrc::Hsi(div) => 64_000_000 / (div.value() as u32),
            InputSrc::Hse(freq) => freq,
        }
    }

    #[cfg(feature = "h7")]
    /// Get the Domain 1 core prescaler frequency, in hz
    pub fn d1cpreclk(&self) -> u32 {
        self.sysclk() / self.d1_core_prescaler.value() as u32
    }

    /// Get the HCLK frequency, in hz
    pub fn hclk(&self) -> u32 {
        #[cfg(feature = "h7")]
        return self.sysclk()
            / self.d1_core_prescaler.value() as u32
            / self.hclk_prescaler.value() as u32;
        #[cfg(feature = "h5")]
        return self.sysclk() / self.hclk_prescaler.value() as u32;
    }

    /// Get the systick speed. Note that for dual core variants, this is for CPU1.
    /// CPU2 systick is equal to the HCLK (possibly divided by 8), so use the `hclk()` method.
    pub fn systick(&self) -> u32 {
        // todo: There's an optional /8 divider we're not taking into account here.
        #[cfg(feature = "h7")]
        return self.d1cpreclk();
        #[cfg(feature = "h5")]
        return self.sysclk();
    }

    /// Get the USB clock frequency, in hz
    pub fn usb(&self) -> u32 {
        // let (input_freq, _) = sysclock(self.input_src, self.divm1, self.divn1, self.divp1);
        // (input_freq * 1_000_000) as u32 / self.divm1 as u32 * self.pll_sai1_mul as u32 / 2
        0 // todo
    }

    pub fn apb1(&self) -> u32 {
        self.hclk() / self.d2_prescaler1.value() as u32
    }

    /// Get the frequency used by APB1 timers, in hz
    pub fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.d2_prescaler1 {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    pub fn apb2(&self) -> u32 {
        self.hclk() / self.d2_prescaler2.value() as u32
    }

    /// Get the frequency used by APB2 timers, in hz
    pub fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.d2_prescaler2 {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    /// Get the SAI1 audio clock frequency, in hz
    pub fn sai1_speed(&self) -> u32 {
        let pll_src = match self.input_src {
            InputSrc::Pll1 => self.pll_src,
            InputSrc::Csi => PllSrc::Csi,
            InputSrc::Hsi(div) => PllSrc::Hsi(div),
            InputSrc::Hse(freq) => PllSrc::Hse(freq),
        };

        match self.sai1_src {
            SaiSrc::Pll1Q => {
                self.pll_input_speed(pll_src, 1) * self.pll1.divn as u32 / self.pll1.divq as u32
            }
            SaiSrc::Pll2P => {
                self.pll_input_speed(pll_src, 1) * self.pll2.divn as u32 / self.pll2.divp as u32
            }
            SaiSrc::Pll3P => {
                self.pll_input_speed(pll_src, 1) * self.pll3.divn as u32 / self.pll3.divp as u32
            }
            SaiSrc::I2sCkin => unimplemented!(),
            SaiSrc::PerClk => unimplemented!(),
        }
    }

    pub fn validate_speeds(&self) -> Result<(), RccError> {
        cfg_if! {
            if #[cfg(feature = "h735")] {
                let max_sysclk = 480_000_000;
            } else {
                let max_sysclk = 550_000_000;
            }
        }
        // #[cfg(feature = "h743")]
        let max_hclk = 240_000_000;
        // #[cfg(feature = "h743")]
        let max_apb = 120_000_000; // todo: Different depending on apb

        // todo: PLL input (Post DIVM1, pre DIVN1) must be between 1Mhz and 16 Mhz

        // todo after pool today: FIgure out what other bit needs to be set to reflect this.

        // todo: Are these valid for all H7 configs?
        if self.pll1.divm > 63
            || self.pll1.divm > 63
            || self.pll3.divm > 63
            || self.pll1.divn > 512
            || self.pll2.divn > 512
            || self.pll3.divn > 512
            || self.pll1.divp > 128
            || self.pll2.divp > 128
            || self.pll3.divp > 128
            || self.pll1.divp < 2
            || self.pll2.divp < 2
            || self.pll3.divp < 2
        // todo: R and Q
        {
            return Err(RccError::Speed);
        }

        if let InputSrc::Pll1 = self.input_src {
            let pll_input_speed = self.pll_input_speed(self.pll_src, 1);
            if pll_input_speed < 1_000_000 || pll_input_speed > 16_000_000 {
                return Err(RccError::Speed);
            }
            // VCO0: Wide VCO range: 192 to 836 MHz (default after reset) (VCOH)
            // Note: The RM appears out of date: Revision "V" supports 960_000_000Mhz
            // VCO speed, to allow a max core speed of 480Mhz.
            let vco_speed = self.vco_output_freq(self.pll_src, 1);
            if pll_input_speed >= 2_000_000 && (vco_speed < 192_000_000 || vco_speed > 960_000_000)
            {
                return Err(RccError::Speed);
            }
            // 1: Medium VCO range: 150 to 420 MHz. (VCOL)
            // Note: You may get power savings
            if pll_input_speed < 2_000_000 && (vco_speed < 150_000_000 || vco_speed > 420_000_000) {
                return Err(RccError::Speed);
            }
        }

        // todo: Validate PLL2 and PLL3 speeds.

        // todo: More work on this, including feature gates

        // todo: QC these limits
        // todo: Note that this involves repeatedly calculating sysclk.
        // todo. We could work around thsi by calcing it once here.
        if self.sysclk() > max_sysclk {
            return Err(RccError::Speed);
        }

        if self.hclk() > max_hclk {
            return Err(RccError::Speed);
        }

        if self.apb1() > max_apb {
            return Err(RccError::Speed);
        }

        if self.apb2() > max_apb {
            return Err(RccError::Speed);
        }

        // todo: Apb3/4

        Ok(())
    }
}

// todo: support default for 280Mhz variants.

impl Default for Clocks {
    /// This default configures clocks with the HSI, and a 400Mhz sysclock speed. (280Mhz sysclock
    /// on variants that only go that high). Note that H723-745 still use this default speed
    /// due to needing VOS0 for higher.
    /// Peripheral and timer clocks are set to 100Mhz or 200Mhz, depending on their limits.
    /// HSE output is not bypassed.
    fn default() -> Self {
        Self {
            /// The input source for the system and peripheral clocks. Eg HSE, HSI, PLL etc
            input_src: InputSrc::Pll1,
            pll_src: PllSrc::Hsi(HsiDiv::Div1),
            pll1: PllCfg::default(),
            pll2: PllCfg::disabled(),
            pll3: PllCfg::disabled(),
            #[cfg(feature = "h7")]
            d1_core_prescaler: HclkPrescaler::Div1,
            d1_prescaler: ApbPrescaler::Div2,
            /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
            #[cfg(feature = "h5")]
            hclk_prescaler: HclkPrescaler::Div1,
            #[cfg(feature = "h7")]
            hclk_prescaler: HclkPrescaler::Div2,
            #[cfg(feature = "h5")]
            d2_prescaler1: ApbPrescaler::Div1,
            #[cfg(feature = "h7")]
            d2_prescaler1: ApbPrescaler::Div2,
            #[cfg(feature = "h5")]
            d2_prescaler2: ApbPrescaler::Div1,
            #[cfg(feature = "h7")]
            d2_prescaler2: ApbPrescaler::Div2,
            #[cfg(feature = "h7")]
            d3_prescaler: ApbPrescaler::Div2,
            /// Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
            /// frees up the pin for use as GPIO.
            hse_bypass: false,
            usb_src: UsbSrc::Hsi48,
            security_system: false,
            /// Enable the HSI48.
            hsi48_on: false,
            /// Select the input source to use after waking up from `stop` mode. Eg HSI or MSI.
            stop_wuck: StopWuck::Hsi,
            /// Note that to get full speed, we need to use VOS0. These are configured
            /// in the `full_speed` method.
            vos_range: VosRange::VOS1,
            sai1_src: SaiSrc::Pll1Q,
            #[cfg(not(feature = "h735"))]
            sai23_src: SaiSrc::Pll1Q,
            sai4a_src: SaiSrc::Pll1Q,
            sai4b_src: SaiSrc::Pll1Q,
            spi123_src: Spi123Src::Pll1Q,
            spi45_src: Spi45Src::Apb,
            dfsdm1_src: DfsdmSrc::Pclk2,
            can_src: CanSrc::Pll1Q,
        }
    }
}

#[cfg(not(feature = "h7b3"))] // todo
impl Clocks {
    /// Full speed of 480Mhz, with VC0 range 0. Correspondingly higher periph clock speeds as well.
    /// (520Mhz core speed on H723-35) Note that special consideration needs to be taken
    /// when using low power modes (ie anything with wfe or wfi) in this mode; may need to manually
    /// disable and re-enable it.
    ///
    /// Note that this overwrites some fields of the pll1 config.
    pub fn full_speed() -> Self {
        // todo: 550Mhz on on H723 etc instead of 520Mhz. Need to set CPU_FREQ_BOOST as well for that.
        // todo: "The CPU frequency boost can be enabled through the CPUFREQ_BOOST option byte in
        // todo FLASH_OPTSR2_PRG register."
        // todo: The change is
        //
        cfg_if! {
            if #[cfg(feature = "h735")] {
                // let flash_regs = unsafe { &(*pac::FLASH::ptr()) };

                // To modify user option bytes, follow the sequence below:
                // 1.Unlock FLASH_OPTCR register as described in Section 4.5.1: FLASH configuration
                // protection, unless the register is already unlocked.
                // 2. Write the desired new option byte values in the corresponding option registers
                // (FLASH_XXX_PRG).
                // flash_regs.optsr2_prg.modify(|_, w| w.cpufreq_boost().set_bit());
                // 3. Set the option byte start change OPTSTART bit to 1 in the FLASH_OPTCR register.
                // flash_regs.optcr.modify(|_, w| w.optstart().set_bit());
                // 4. Wait until OPT_BUSY bit is cleared.
                // while flash_regs.optcr.read().opt_busy().bit_is_set() { }

                // let divn 275; // for 550Mhz
                let divn = 260;
                let divp = 1;
            } else {
                let divn = 480;
                let divp = 2;
            }
        }
        Self {
            pll1: PllCfg {
                divn,
                divp,
                ..Default::default()
            },
            vos_range: VosRange::VOS0,
            ..Default::default()
        }
    }
}

#[cfg(not(feature = "h5"))] // todo: Come back to
// todo
/// Enable the Clock Recovery System.
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

    rcc.apb1henr.modify(|_, w| w.crsen().set_bit());

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
