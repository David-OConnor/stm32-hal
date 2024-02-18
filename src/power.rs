//! Manage STM32H7 supply configuration. This is required on some H7 variants, to specify
//! which regulator to use. This must match the way the MCU power pins are wired on the hardware design.

use crate::pac::PWR;

#[derive(Clone, Copy)]
#[repr(u8)]
/// SMPS step-down converter voltage output level selection.
/// This bit is used when both the LDO and SMPS step-down converter are enabled with SDEN and
/// LDOEN enabled or when SDEXTHP is enabled. In this case SDLEVEL has to be written with a
/// value different than 00 at system startup.
pub enum VoltageLevel {
    /// 1.8V
    V1_8 = 0b01,
    /// 2.5V
    V2_5 = 0b10,
}

#[derive(Clone, Copy)]
/// See RM0399, Table 32. Supply configuration control, for available configurations.
/// Sets the PWR_CR3 register, LDOEN, SDEN, SDEXTHP, SDLEVEL, and BYPASS fields.
pub enum SupplyConfig {
    /// Default configuration
    Default,
    /// LDO supply
    Ldo,
    /// Direct SMPS step-down converter supply
    DirectSmps,
    /// SMPS step-down converter supplies LDO
    SmpsStepdownLdo,
    /// SMPS step-down converter supplies External and LDO
    SmpsStepdownExtLdo,
    /// SMPS step-down converter supplies external and LDO Bypass
    SmpsStpdownExtBypass,
    /// SMPS step-down converter disabled and LDO Bypass
    SmpsStepdownDisabledBypass,
}

impl SupplyConfig {
    /// Apply a given supply config. `voltage_level` only affects certain variants.
    pub fn setup(&self, pwr: &mut PWR, voltage_level: VoltageLevel) {
        match self {
            Self::Default => pwr.cr3.modify(|_, w| unsafe {
                w.sdlevel().bits(voltage_level as u8);
                w.sdexthp().clear_bit();
                w.sden().set_bit();
                w.ldoen().set_bit();
                w.bypass().clear_bit()
            }),
            Self::Ldo => pwr.cr3.modify(|_, w| unsafe {
                w.sden().clear_bit();
                w.ldoen().set_bit();
                w.bypass().clear_bit()
            }),
            Self::DirectSmps => pwr.cr3.modify(|_, w| unsafe {
                w.sdexthp().clear_bit();
                w.sden().set_bit();
                w.ldoen().clear_bit();
                w.bypass().clear_bit()
            }),
            Self::SmpsStepdownLdo => pwr.cr3.modify(|_, w| unsafe {
                w.sdlevel().bits(voltage_level as u8);
                w.sdexthp().clear_bit();
                w.sden().set_bit();
                w.ldoen().set_bit();
                w.bypass().clear_bit()
            }),
            Self::SmpsStepdownExtLdo => pwr.cr3.modify(|_, w| unsafe {
                w.sdlevel().bits(voltage_level as u8);
                w.sdexthp().set_bit();
                w.sden().set_bit();
                w.ldoen().set_bit();
                w.bypass().clear_bit()
            }),
            Self::SmpsStpdownExtBypass => pwr.cr3.modify(|_, w| unsafe {
                w.sdlevel().bits(voltage_level as u8);
                w.sdexthp().set_bit();
                w.sden().set_bit();
                w.ldoen().clear_bit();
                w.bypass().set_bit()
            }),
            Self::SmpsStepdownDisabledBypass => pwr.cr3.modify(|_, w| unsafe {
                w.sden().clear_bit();
                w.ldoen().clear_bit();
                w.bypass().set_bit()
            }),
        }
    }
}
