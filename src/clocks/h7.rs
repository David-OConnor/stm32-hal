// todo: this is missing features. eg the other PLLs etc.
// todo: Probably not in usable state yet; need to cross-check RM and clock tree.

use crate::{
    clocks::SpeedError,
    pac::{FLASH, PWR, RCC, SYSCFG},
};

use cfg_if::cfg_if;

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
pub enum InputSrc {
    Hsi(HsiDiv),
    Csi,
    Hse(u32), // freq in Mhz,
    Pll1(PllSrc),
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
            Self::Pll1(_) => 0b011,
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
/// SAI clock input source. Sets RCC_D2CCIP1R register, SAIxSEL fields.
pub enum SaiSrc {
    Pll1Q = 0b000,
    Pll2P = 0b001,
    Pll3P = 0b010,
    I2sCkin = 0b011,
    PerClk = 0100,
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
/// Range for the VSO. See H743 RM, section 6.8.6: PWR D3 domain control register. Sets PWR_D3CR,
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
    /// Choose the wait states based on VSO range and hclk frequency.. See H743 RM, Table 17: FLASH
    /// recommended number of wait states and programming delay. Returns a tuple of (number of wait states,
    /// programming delay) (FLASH ACR_LATENCY, WRHIGHFREQ) values respectively.
    pub fn wait_states(&self, hclk: u32) -> (u8, u8) {
        // todo: Feature gate for other H7 variants. This is likely only valid on H743 and/or
        // todo 480MHz single-core variants.
        match self {
            #[cfg(not(feature = "h7b3"))]
            Self::VOS0 => match hclk {
                0..=70_000_000 => (0, 0),
                70_000_001..=140_000_000 => (1, 1),
                140_000_001..=185_000_000 => (2, 1),
                185_000_001..=210_000_000 => (2, 2),
                210_000_001..=225_000_000 => (3, 2),
                225_000_001..=240_000_000 => (4, 2),
                _ => panic!("Can't set higher than 240Mhz HCLK with VSO0 range."),
            },
            Self::VOS1 => match hclk {
                0..=70_000_000 => (0, 0),
                70_000_001..=140_000_000 => (1, 1),
                140_000_001..=185_000_000 => (2, 1),
                185_000_001..=210_000_000 => (2, 2),
                210_000_001..=225_000_000 => (3, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VOS1 range."),
            },
            Self::VOS2 => match hclk {
                0..=55_000_000 => (0, 0),
                55_000_001..=110_000_000 => (1, 1),
                110_000_001..=165_000_000 => (2, 1),
                165_000_001..=225_000_000 => (3, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VSO2 range."),
            },
            Self::VOS3 => match hclk {
                0..=45_000_000 => (0, 0),
                45_000_001..=90_000_000 => (1, 1),
                90_000_001..=135_000_000 => (2, 1),
                135_000_001..=180_000_000 => (3, 2),
                180_000_001..=225_000_000 => (4, 2),
                _ => panic!("Can't set higher than 225Mhz HCLK with VSO3 range."),
            },
        }
    }
}

/// Settings used to configure clocks.
/// Note that we use integers instead of enums for some of the scalers, unlike in
/// the other clock modules. This is due to the wide range available on these fields.
pub struct Clocks {
    pub input_src: InputSrc,
    /// Prescaler for PLL1
    pub divm1: u8,
    /// PLL1 DIVN division factor
    pub divn1: u16,
    /// PLL1 DIVP division factor
    pub divp1: u8,
    /// PLL1 DIVQ division factor
    pub divq1: u8,
    /// PLL1 DIVR division factor
    pub divr1: u8,
    /// Prescaler for PLL2
    pub divm2: u8,
    /// PLL2 DIVN division factor
    pub divn2: u16,
    /// PLL2 DIVP division factor
    pub divp2: u8,
    /// PLL2 DIVQ division factor
    pub divq2: u8,
    /// PLL2 DIVR division factor
    pub divr2: u8,
    /// Prescaler for PLL3
    pub divm3: u8,
    /// PLL3 DIVN division factor
    pub divn3: u16,
    /// PLL3 DIVP division factor
    pub divp3: u8,
    /// PLL3 DIVQ division factor
    pub divq3: u8,
    /// PLL3 DIVR division factor
    pub divr3: u8,
    /// Enable PLL2
    pub pll2_enable: bool,
    /// Enable PLL3
    pub pll3_enable: bool,
    pub d1_core_prescaler: HclkPrescaler,
    pub d1_prescaler: ApbPrescaler,
    pub hclk_prescaler: HclkPrescaler,
    pub d2_prescaler1: ApbPrescaler,
    pub d2_prescaler2: ApbPrescaler,
    pub d3_prescaler: ApbPrescaler,
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    pub hse_bypass: bool,
    pub security_system: bool,
    pub hsi48_on: bool,
    pub vos_range: VosRange,
    /// SAI1 and DFSDM1 kernel Aclk clock source selection
    pub sai1_src: SaiSrc,
    /// SAI2 and SAI3 kernel clock source selection
    pub sai23_src: SaiSrc,
}

impl Clocks {
    /// Setup common and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    /// https://docs.rs/stm32f3xx-hal/0.5.0/stm32f3xx_hal/rcc/struct.CFGR.html
    /// Use the STM32CubeIDE Clock Configuration tab to identify valid configs.
    /// Use the `default()` implementation as a safe baseline.
    /// This method also configures the PWR VOS setting, and can be used to enable VOS boost,
    /// if `vos_range` is set to `VosRange::VOS0`.
    pub fn setup(
        &self,
        rcc: &mut RCC,
        flash: &mut FLASH,
        pwr: &mut PWR,
        syscfg: &mut SYSCFG,
    ) -> Result<(), SpeedError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
        rcc.apb4rstr.modify(|_, w| w.syscfgrst().set_bit());
        rcc.apb4rstr.modify(|_, w| w.syscfgrst().clear_bit());

        // H743 RM, sefction 6.8.6, and section 6.6.2: Voltage Scaling
        //  Voltage scaling selection according to performance
        // These bits control the VCORE voltage level and allow to obtains the best trade-off between
        // power consumption and performance:
        // – When increasing the performance, the voltage scaling shall be changed before increasing
        // the system frequency.
        // – When decreasing performance, the system frequency shall first be decreased before
        // changing the voltage scaling.

        match self.vos_range {
            #[cfg(not(feature = "h7b3"))]
            VosRange::VOS0 => {
                // VOS0 activation/deactivation sequence
                // The system maximum frequency can be reached by boosting the voltage scaling level to
                // VOS0. This is done through the ODEN bit in the SYSCFG_PWRCR register.
                // The sequence to activate the VOS0 is the following:
                // 1. Ensure that the system voltage scaling is set to VOS1 by checking the VOS bits in
                // PWR D3 domain control register (PWR D3 domain control register (PWR_D3CR))
                pwr.d3cr
                    .modify(|_, w| unsafe { w.vos().bits(VosRange::VOS1 as u8) });

                // 2. Enable the SYSCFG clock in the RCC by setting the SYSCFGEN bit in the
                // RCC_APB4ENR register.
                rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());

                // 3. Enable the ODEN bit in the SYSCFG_PWRCR register.
                // PAC inconsistency between variants on if there's a modify field, and if
                // `write` has a `bits()` or `bit()` method.
                cfg_if! {
                    if #[cfg(any(feature = "h747cm4", feature = "h747cm7"))] {
                        syscfg.pwrcr.modify(|_, w| w.oden().set_bit());
                    } else {
                        syscfg.pwrcr.write(|w| unsafe { w.oden().bits(1) });
                    }
                }

                // 4. Wait for VOSRDY to be set.
                while pwr.d3cr.read().vosrdy().bit_is_clear() {}

                // Once the VCORE supply has reached the required level, the system frequency can be
                // increased. Figure 31 shows the recommended sequence for switching VCORE from VOS1 to
                // VOS0 sequence.
                // The sequence to deactivate the VOS0 is the following:
                // 1. Ensure that the system frequency was decreased.
                // 2. Ensure that the SYSCFG clock is enabled in the RCC by setting the SYSCFGEN bit set
                // in the RCC_APB4ENR register.
                // 3. Reset the ODEN bit in the SYSCFG_PWRCR register to disable VOS0.
            }
            _ => pwr
                .d3cr
                .modify(|_, w| unsafe { w.vos().bits(self.vos_range as u8) }),
        }

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        // H742 RM, Table 17.
        // todo: What should this be?
        let wait_states = self.vos_range.wait_states(self.hclk());
        flash.acr.modify(|_, w| unsafe {
            w.latency().bits(wait_states.0);
            w.wrhighfreq().bits(wait_states.1)
        });

        // todo: Look up and document PLL config.

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::Csi => {
                rcc.cr.modify(|_, w| w.csion().bit(true));
                while rcc.cr.read().csirdy().bit_is_clear() {}
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().bit_is_clear() {}
            }
            InputSrc::Hsi(div) => {
                rcc.cr.modify(|_, w| {
                    w.hsidiv().bits(div as u8);
                    w.hsion().bit(true)
                });
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
            InputSrc::Pll1(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and Csi setup above.
                match pll_src {
                    PllSrc::Csi => {
                        rcc.cr.modify(|_, w| w.csion().bit(true));
                        while rcc.cr.read().csirdy().bit_is_clear() {}
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().bit(true));
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi(div) => {
                        rcc.cr.modify(|_, w| {
                            w.hsidiv().bits(div as u8);
                            w.hsion().bit(true)
                        });
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

        rcc.cfgr
            .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

        rcc.d1cfgr.modify(|_, w| unsafe {
            w.d1cpre().bits(self.d1_core_prescaler as u8);
            w.d1ppre().bits(self.d1_prescaler as u8);
            w.hpre().bits(self.hclk_prescaler as u8)
        });

        rcc.d2cfgr.modify(|_, w| unsafe {
            w.d2ppre1().bits(self.d2_prescaler1 as u8);
            w.d2ppre2().bits(self.d2_prescaler2 as u8)
        });

        rcc.d3cfgr
            .modify(|_, w| unsafe { w.d3ppre().bits(self.d3_prescaler as u8) });

        // Set USART2 to HSI, and USB to HSI48. Temp hardcoded.
        #[cfg(not(feature = "h7b3"))]
        rcc.d2ccip2r.modify(|_, w| unsafe {
            w.usart234578sel().bits(0b111);
            w.usbsel().bits(0b11)
        });

        // todo: Allow configuring the PLL in fractinoal mode.

        if let InputSrc::Pll1(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pll1on().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pll1rdy().bit_is_set() {}

            rcc.pllckselr.modify(|_, w| {
                w.pllsrc().bits(pll_src.bits());
                w.divm1().bits(self.divm1);
                w.divm2().bits(self.divm2);
                w.divm3().bits(self.divm3)
            });

            // Set and reset by software to select the proper reference frequency range used for PLL1.
            // This bit must be written before enabling the PLL1.
            let pll1_rng_val = match self.pll_input_speed(pll_src, 1) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL1 input source must be between 1Mhz and 16Mhz."),
            };

            // todo DRY!
            let pll2_rng_val = match self.pll_input_speed(pll_src, 2) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL2 input source must be between 1Mhz and 16Mhz."),
            };

            // todo DRY!
            let pll3_rng_val = match self.pll_input_speed(pll_src, 3) {
                1_000_000..=2_000_000 => 0b00,
                2_000_001..=4_000_000 => 0b01,
                4_000_001..=8_000_000 => 0b10,
                8_000_001..=16_000_000 => 0b11,
                _ => panic!("PLL3 input source must be between 1Mhz and 16Mhz."),
            };

            // todo: Don't enable all these pqr etc by default!
            // todo don't enable pll2 and 3 by default!!
            // todo: This has an impact on power consumption.

            // todo: MOre DRY
            // H743 RM:
            // 0: Wide VCO range: 192 to 836 MHz (default after reset)
            // 1: Medium VCO range: 150 to 420 MHz
            let pll1_vco = match self.pll_input_speed(pll_src, 1) {
                0..=2_000_000 => 0,
                _ => 1,
            };
            let pll2_vco = match self.pll_input_speed(pll_src, 2) {
                0..=2_000_000 => 0,
                _ => 1,
            };
            let pll3_vco = match self.pll_input_speed(pll_src, 3) {
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

            // struct PllEnCfg {
            //     p_en: bool,
            //     q_en: bool,
            //     r_en: bool
            //

            rcc.pllcfgr.modify(|_, w| {
                w.pll1rge().bits(pll1_rng_val);
                w.pll1vcosel().bit(pll1_vco != 0);
                w.divp1en().set_bit()
                // w.divr1en().set_bit();
                // w.divq1en().set_bit();
            });

            rcc.pll1divr.modify(|_, w| unsafe {
                w.divn1().bits(self.divn1 - 1);
                w.divp1().bits(self.divp1 - 1);
                w.divq1().bits(self.divq1 - 1);
                w.divr1().bits(self.divr1 - 1)
            });

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            rcc.cr.modify(|_, w| w.pll1on().set_bit());
            while rcc.cr.read().pll1rdy().bit_is_clear() {}

            if self.pll2_enable {
                rcc.pllcfgr.modify(|_, w| {
                    w.pll2rge().bits(pll2_rng_val);
                    w.pll2vcosel().bit(pll2_vco != 0);
                    w.divp2en().set_bit();
                    w.divr2en().set_bit();
                    w.divq2en().set_bit()
                });

                rcc.pll2divr.modify(|_, w| unsafe {
                    w.divn2().bits(self.divn2 - 1);
                    w.divp2().bits(self.divp2 - 1);
                    w.divq2().bits(self.divq2 - 1);
                    w.divr2().bits(self.divr2 - 1)
                });

                rcc.cr.modify(|_, w| w.pll2on().set_bit());
                while rcc.cr.read().pll2rdy().bit_is_clear() {}
            }

            if self.pll3_enable {
                rcc.pllcfgr.modify(|_, w| {
                    w.pll3rge().bits(pll3_rng_val);
                    w.pll3vcosel().bit(pll3_vco != 0);
                    w.divp3en().set_bit();
                    w.divr3en().set_bit();
                    w.divq3en().set_bit()
                });

                rcc.pll3divr.modify(|_, w| unsafe {
                    w.divn3().bits(self.divn3 - 1);
                    w.divp3().bits(self.divp3 - 1);
                    w.divq3().bits(self.divq3 - 1);
                    w.divr3().bits(self.divr3 - 1)
                });

                rcc.cr.modify(|_, w| w.pll3on().set_bit());
                while rcc.cr.read().pll3rdy().bit_is_clear() {}
            }
        }

        rcc.d2ccip1r.modify(|_, w| unsafe {
            w.sai1sel().bits(self.sai1_src as u8);
            w.sai23sel().bits(self.sai23_src as u8)
        });

        rcc.cr.modify(|_, w| w.hsecsson().bit(self.security_system));

        if self.hsi48_on {
            rcc.cr.modify(|_, w| w.hsi48on().set_bit());
            while rcc.cr.read().hsi48rdy().bit_is_clear() {}
        }

        Ok(())
    }

    /// Re-select input source; used on Stop and Standby modes, where the system reverts
    /// to HSI after wake.
    pub fn reselect_input(&self, rcc: &mut RCC) {
        // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

        // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
        // todo: But this saves a few reg writes.
        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().bit_is_clear() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll1(pll_src) => {
                // todo: DRY with above.
                match pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi(div) => {
                        // Generally reverts to Csi (see note below)
                        rcc.cr.modify(|_, w| {
                            w.hsidiv().bits(div as u8); // todo: Do we need to reset the HSI div after low power?
                            w.hsion().bit(true)
                        });
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                    }
                    PllSrc::Csi => (), // todo
                    PllSrc::None => (),
                }

                // todo: PLL 2 and 3?
                rcc.cr.modify(|_, w| w.pll1on().clear_bit());
                while rcc.cr.read().pll1rdy().bit_is_set() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pll1on().set_bit());
                while rcc.cr.read().pll1rdy().bit_is_clear() {}
            }
            InputSrc::Hsi(div) => {
                {
                    // From Reference Manual, RCC_CFGR register section:
                    // "Configured by HW to force Csi oscillator selection when exiting Standby or Shutdown mode.
                    // Configured by HW to force Csi or HSI16 oscillator selection when exiting Stop mode or in
                    // case of failure of the HSE oscillator, depending on STOPWUCK value."
                    // In tests, from stop, it tends to revert to Csi.
                    rcc.cr.modify(|_, w| {
                        w.hsidiv().bits(div as u8); // todo: Do we need to reset the HSI div after low power?
                        w.hsion().bit(true)
                    });
                    while rcc.cr.read().hsirdy().bit_is_clear() {}
                }
            }
            InputSrc::Csi => (), // ?
        }
    }

    /// Calculate the input speed to the PLL. This must be between 1 and 16 Mhz. Called `refx_ck`
    /// in the RM.
    pub fn pll_input_speed(&self, pll_src: PllSrc, pll_num: u8) -> u32 {
        let input_freq = match pll_src {
            PllSrc::Csi => 4_000_000,
            PllSrc::Hsi(div) => 64_000_000 / (div.value() as u32),
            PllSrc::Hse(freq) => freq,
            PllSrc::None => 0,
        };

        match pll_num {
            1 => input_freq / (self.divm1 as u32),
            2 => input_freq / (self.divm2 as u32),
            3 => input_freq / (self.divm3 as u32),
            _ => panic!("Pll num must be between 1 and 3."),
        }
    }

    /// Calculate VCO output frequency: = Fref1_ck x DIVN1
    pub fn vco_output_freq(&self, pll_src: PllSrc, pll_num: u8) -> u32 {
        let input_speed = self.pll_input_speed(pll_src, pll_num);
        match pll_num {
            1 => input_speed * self.divn1 as u32,
            2 => input_speed * self.divn2 as u32,
            3 => input_speed * self.divn3 as u32,
            _ => panic!("Pll num must be between 1 and 3."),
        }
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
        rcc.cr.read().pll1on().bit_is_set()
    }

    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll1(pll_src) => {
                // divm1 is included in `pll_input_speed`.
                self.pll_input_speed(pll_src, 1) * self.divn1 as u32 / self.divp1 as u32
            }
            InputSrc::Csi => 4_000_000,
            InputSrc::Hsi(div) => 64_000_000 / (div.value() as u32),
            InputSrc::Hse(freq) => freq,
        }
    }

    pub fn d1cpreclk(&self) -> u32 {
        self.sysclk() / self.d1_core_prescaler.value() as u32
    }

    pub fn hclk(&self) -> u32 {
        self.sysclk() / self.d1_core_prescaler.value() as u32 / self.hclk_prescaler.value() as u32
    }

    pub fn systick(&self) -> u32 {
        // todo: There's an optional /8 divider we're not taking into account here.
        self.d1cpreclk()
    }

    pub fn usb(&self) -> u32 {
        // let (input_freq, _) = calc_sysclock(self.input_src, self.divm1, self.divn1, self.divp1);
        // (input_freq * 1_000_000) as u32 / self.divm1 as u32 * self.pll_sai1_mul as u32 / 2
        0 // todo
    }

    pub fn apb1(&self) -> u32 {
        self.hclk() / self.d2_prescaler1.value() as u32
    }

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

    pub fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.d2_prescaler2 {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    pub fn validate_speeds(&self) -> Result<(), SpeedError> {
        // todo: This depends on variant
        // let max_clock = 550_000_000;
        // #[cfg(feature = "h743")]
        let max_sysclk = 480_000_000;
        // #[cfg(feature = "h743")]
        let max_hclk = 240_000_000;
        // #[cfg(feature = "h743")]
        let max_apb = 120_000_000; // todo: Different depending on apb

        // todo: PLL input (Post DIVM1, pre DIVN1) must be between 1Mhz and 16 Mhz

        // todo after pool today: FIgure out what other bit needs to be set to reflect this.

        // todo: Are these valid for all H7 configs?
        if self.divm1 > 63
            || self.divm2 > 63
            || self.divm3 > 63
            || self.divn1 > 512
            || self.divn2 > 512
            || self.divn3 > 512
            || self.divp1 > 128
            || self.divp2 > 128
            || self.divp3 > 128
            || self.divp1 < 2
            || self.divp2 < 2
            || self.divp2 < 2
        {
            return Err(SpeedError::new("A PLL divider is out of limits"));
        }

        if let InputSrc::Pll1(pll_src) = self.input_src {
            let pll_input_speed = self.pll_input_speed(pll_src, 1);
            if pll_input_speed < 1_000_000 || pll_input_speed > 16_000_000 {
                return Err(SpeedError::new("Invalid PLL input speed"));
            }
            // VCO0: Wide VCO range: 192 to 836 MHz (default after reset) (VCOH)
            let vco_speed = self.vco_output_freq(pll_src, 1);
            if pll_input_speed <= 2_000_000 && (vco_speed < 192_000_000 || vco_speed > 836_000_000)
            {
                return Err(SpeedError::new("Invalid wide VCO speed"));
            }
            // 1: Medium VCO range: 150 to 420 MHz. (VCOL)
            // Note: You may get power savings
            if pll_input_speed > 2_000_000 && (vco_speed < 150_000_000 || vco_speed > 420_000_000) {
                return Err(SpeedError::new("Invalid medium VCO speed"));
            }
        }

        // todo: Validate PLL2 and PLL3 speeds.

        // todo: More work on this, including feature gates

        // todo: QC these limits
        // todo: Note that this involves repeatedly calculating sysclk.
        // todo. We could work around thsi by calcing it once here.
        if self.sysclk() > max_sysclk {
            return Err(SpeedError::new("Sysclock out of limits"));
        }

        if self.hclk() > max_hclk {
            return Err(SpeedError::new("Hclk out of limits"));
        }

        if self.apb1() > max_apb {
            return Err(SpeedError::new("APB1 out of limits"));
        }

        if self.apb2() > max_apb {
            return Err(SpeedError::new("APB2 out of limits"));
        }

        // todo: Apb3/4?

        Ok(())
    }
}

impl Default for Clocks {
    /// This default configures clocks with the HSI, and a 240Mhz sysclock speed.
    /// Peripheral clocks are all set to 112.5Mhz, with timer clocks at 225Mhz.
    /// HSE output is not bypassed.
    /// todo: Not 248Mhz due to an issue with wait states on Table 17 not showing higher than 225Mhz
    /// // todo HCLK unless in VOS0.
    fn default() -> Self {
        // todo: Feature-gate based on variant. Ie the 5xxMhz ones, the 240Mhz ones, and dual cores.
        Self {
            input_src: InputSrc::Pll1(PllSrc::Hsi(HsiDiv::Div1)),
            divm1: 32,
            // #[cfg(feature = "h743")]
            // divn1: 240, // todo: For 480Mhz with right vco set.
            // #[cfg(feature = "h743")]
            divn1: 225,
            divp1: 2,
            divq1: 3, // Allows <150Mhz SAI clock, if it's configureud for PLL1Q.
            divr1: 2,
            divm2: 32,
            divn2: 225,
            divp2: 2,
            divq2: 2,
            divr2: 2,
            divm3: 32,
            divn3: 225,
            divp3: 2,
            divq3: 2,
            divr3: 2,
            pll2_enable: false,
            pll3_enable: false,
            d1_core_prescaler: HclkPrescaler::Div1,
            d1_prescaler: ApbPrescaler::Div1,
            hclk_prescaler: HclkPrescaler::Div2,
            d2_prescaler1: ApbPrescaler::Div2,
            d2_prescaler2: ApbPrescaler::Div2,
            d3_prescaler: ApbPrescaler::Div2,
            hse_bypass: false,
            security_system: false,
            hsi48_on: false,
            vos_range: VosRange::VOS1,
            sai1_src: SaiSrc::Pll1Q,
            sai23_src: SaiSrc::Pll1Q,
        }
    }
}
