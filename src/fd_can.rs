//! This module supports the FD-CAN peripheral.

use core::ops::Deref;

use crate::{clocks::Clocks, pac::{RCC, FDCAN}, util::RccPeriph};


#[derive(Clone, Copy)]
#[repr(u8)]
/// Select Stereo or Mono mode. Sets xCR1 register, MONO field.
pub enum Mono {
    Stereo = 0,
    Mono = 1,
}


#[derive(Clone, Copy)]
/// The type of SAI interrupt to configure. Reference Section 41.5 of the L4 RM.
/// Enabled in xIM register, yIE fields. See H743 RM, section 51.5: SAI interrupts.
pub enum CanInterrupt {
    /// FIFO request interrupt enable. When this bit is set, an interrupt is generated if the FREQ bit in the SAI_xSR register is set.
    /// Since the audio block defaults to operate as a transmitter after reset, the MODE bit must be
    /// configured before setting FREQIE to avoid a parasitic interrupt in receiver mode
    Freq,
}


/// Configuration for the SAI peripheral. Mainly affects the ACR and BCR registers.
/// Used for either channel. For details, see documentation of individual structs and fields.
/// You may be forced into certain settings based on the device used.
#[derive(Clone)]
pub struct CanConfig {
    pub mode: SaiMode,
    /// Select protocols between Free, Ac'97, and SPDIF. Defaults to Free.
    pub protocol: Protocol,
    /// Select mono or stereo modes. Default to mono.
    pub mono: Mono,
}


impl Default for CanConfig {
    fn default() -> Self {
        Self {
            mode: SaiMode::MasterTransmitter,
            protocol: Protocol::Free,
            mono: Mono::Stereo,
        }
    }
}

/// Represents the Serial Audio Interface (SAI) peripheral, used for digital audio
/// input and output.
pub struct Can<R> {
    pub regs: R,
    config_a: CanConfig,
    config_b: CanConfig,
}

impl<R> Can<R>
where
    R: Deref<Target = sai::RegisterBlock> + RccPeriph,
{
    /// Initialize a SAI peripheral, including  enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, config: CanConfig, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        R::en_reset(rcc);
    }
}