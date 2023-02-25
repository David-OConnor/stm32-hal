//! This module supports the FD-CAN peripheral.

use core::ops::Deref;

use crate::{clocks::Clocks, pac::{self, RCC, FDCAN}, util::RccPeriph};



#[derive(Clone, Copy)]
#[repr(u8)]
/// Select ISO, or Bosche format. Sets CCCR register, NISO field.
pub enum IsoOrBosch {
    /// CAN FD frame format according to ISO11898-1
    Iso = 0,
    /// CAN FD frame format according to Bosch CAN FD Specification V1.0
    Bosch = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Can pause for two CAN bit times before starting next transmission. Sets CCCR register, TXP field.
pub enum Txp {
    /// Does not pause before starting next transmission
    Disabled = 0,
    /// Pauses two frames before starting next transmission.
    Enabled = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Edge filtering during bus integration. Sets CCCR register, EFBI field.
pub enum EdgeFiltering {
    /// Edge filtering disabled
    Disabled = 0,
    /// Two consecutive dominant tq required to detect an edge for hard synchronization
    Enabled = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Protocol exception handling disable. Sets CCCR register, Pxhd field.
pub enum ProtocolExceptionHandling {
    Disabled = 0,
    Enabled = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Bit rate switching. Sets CCCR register, BRSE field.
pub enum BitRateSwitching {
    Disabled = 0,
    Enabled = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// FD operation enable. Sets CCCR register, FDOE field.
pub enum FdOperation {
    Disabled = 0,
    Enabled = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Configuration change enable. Sets CCCR register, CCE field.
pub enum ConfigChange {
    /// The CPU has no write access to the protected configuration registers.
    WriteProtected = 0,
    /// The CPU has write access to the protected configuration registers (while CCCR.INIT = 1).
    NotWriteProtected = 1,
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
    /// Select ISO, or Bosch format. Defaults to ISO.
    pub iso_or_bosch: IsoOrBosch,
    /// Determines if CAN pauses 2 frames between frames. Defaults to no pause.
    pub txp: Txp,
    /// Edge filtering during bus integration. Defaults to disabled.
    pub edge_filtering: EdgeFiltering,
    /// Protocol exception hanlding disabled. Defaults to enabled.
    pub protocol_exception_handlilng: ProtocolExceptionHandling,
    /// Bit rate switching. Defaults to disabled.
    pub bit_rate_switching: BitRateSwitching,
    /// Operating in FD mode. Defaults to enabled.
    pub fd_operation: FdOperation,
    pub config_change: ConfigChange,
    // todo: Fill out the rest of CFG fields.

}


impl Default for CanConfig {
    fn default() -> Self {
        Self {
            iso_or_bosch: IsoOrBosch::Iso,
            txp: Txp::Disabled,
            edge_filtering: EdgeFiltering::Disabled,
            protocol_exception_handlilng: ProtocolExceptionHandling::Enabled,
            bit_rate_switching: BitRateSwitching::Disabled,
            fd_operation: FdOperation::Enabled,
            config_change: ConfigChange::WriteProtected,
        }
    }
}

/// Represents the Serial Audio Interface (SAI) peripheral, used for digital audio
/// input and output.
pub struct Can<R> {
    pub regs: R,
    config: CanConfig,
}

impl<R> Can<R>
where
    R: Deref<Target = pac::fdcan::RegisterBlock> + RccPeriph,
{
    /// Initialize a FD-CAN, including  enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, config: CanConfig, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        R::en_reset(rcc);

        // See G4 RM, section 44.3.2: Operating modes, Software initialization section.
        //
        // Software initialization is started by setting INIT bit in FDCAN_CCCR register, either by
        // software or by a hardware reset, or by going Bus_Off. While INIT bit in FDCAN_CCCR
        // register is set, message transfer from and to the CAN bus is stopped, the status of the CAN
        // bus output FDCAN_TX is recessive (high). The EML (error management logic) counters are
        // unchanged. Setting INIT bit in FDCAN_CCCR does not change any configuration register.
        // Clearing INIT bit in FDCAN_CCCR finishes the software initialization. Afterwards the bit
        // stream processor (BSP) synchronizes itself to the data transfer on the CAN bus by waiting
        // for the occurrence of a sequence of 11 consecutive recessive bits (Bus_Idle) before it can
        // take part in bus activities and start the message transfer.
        // Access to the FDCAN configuration registers is only enabled when both INIT bit in
        // FDCAN_CCCR register and CCE bit in FDCAN_CCCR register are set.
        // CCE bit in FDCAN_CCCR register can only be set/cleared while INIT bit in FDCAN_CCCR
        // is set. CCE bit in FDCAN_CCCR register is automatically cleared when INIT bit in
        // FDCAN_CCCR is cleared.

        regs.cccr.modify(|_, w| {
           w.niso().bit(config.iso_or_bosch as u8 != 0);
           w.txp().bit(config.txp as u8 != 0);
           w.efbi().bit(config.edge_filtering as u8 != 0);
           w.pxhd().bit(config.protocol_exception_handlilng as u8 != 0);
           w.brse().bit(config.bit_rate_switching as u8 != 0);
           w.fdoe().bit(config.fd_operation as u8 != 0);
            // todo: More settings as you add fields for them.
           w.cce().bit(config.config_change as u8 != 0);
           w.init().set_bit() // Starts initialization.
        });

        // Set normal operation.
        // todo: Where? Here after a delay? How long?
        regs.cccr.modify(|_, w| w.init().clear_bit());
        // todo: Do we need to clear the init bit?


        Self {
            regs,
            config
        }
    }
}