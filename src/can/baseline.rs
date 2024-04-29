use cfg_if::cfg_if;

use crate::pac::FDCAN1 as CAN;
use crate::{pac::RCC, util::rcc_en_reset};
use fdcan;

/// Interface to the CAN peripheral.
pub struct Can {
    pub regs: CAN,
}

impl Can {
    /// Initialize a CAN peripheral, including  enabling and resetting
    /// its RCC peripheral clock. This is not handled by the `bxcan` or `canfd` crates.
    pub fn new(regs: CAN) -> Self {
        let rcc = unsafe { &*RCC::ptr() };

        rcc_en_reset!(apb1, fdcan, rcc);

        Self { regs }
    }

    /// Print the (raw) contents of the status register.
    pub fn read_status(&self) -> u32 {
        unsafe { self.regs.msr.read().bits() }
    }
}

#[cfg(feature = "h7")]
// todo: Troubleshooting. COpied from H7xx-hal
/// Set the message RAM layout. This is flexible on H7. This function hard-sets it to the setting
/// that is hard-set by hardware on G4.
/// todo: Allow flexibility.
///
/// Note: Perhaps due to a reset of message ram called by the FDCAN crate's `.into_config_mode()`,
/// we run this in application firmware once in config mode. Although a better API would be in the constructor
/// This must be done after initial setup (Enabling RCC clocks most-likely).
pub fn set_message_ram_layout() {
    let regs = unsafe { &(*CAN::ptr()) };

    // RM, section 56.4.1: Operation modes: "Access to the FDCAN configuration registers is only
    // enabled when both INIT bit in FDCAN_CCCR register and CCE bit in FDCAN_CCCR register are set.
    // Note: we do this as 2 separate writes. RM: "CCE bit in FDCAN_CCCR register can only be set/cleared while INIT bit in FDCAN_CCCR
    // is set. CCE bit in FDCAN_CCCR register is automatically cleared when INIT bit in
    // FDCAN_CCCR is cleared."
    regs.cccr.modify(|_, w| w.init().set_bit());
    while regs.cccr.read().init().bit_is_clear() {}
    regs.cccr.modify(|_, w| w.cce().set_bit());
    while regs.cccr.read().cce().bit_is_clear() {}

    let mut word_addr = 0x000; // todo: 0x400 for FDCAN2?

    use fdcan::message_ram::*;

    // 11-bit filter
    regs.sidfc
        .modify(|_, w| unsafe { w.flssa().bits(word_addr) });
    word_addr += STANDARD_FILTER_MAX as u16;

    // 29-bit filter
    regs.xidfc
        .modify(|_, w| unsafe { w.flesa().bits(word_addr) });
    word_addr += 2 * EXTENDED_FILTER_MAX as u16;

    // Rx FIFO 0
    regs.rxf0c.modify(|_, w| unsafe {
        w.f0sa()
            .bits(word_addr)
            .f0s()
            .bits(RX_FIFO_MAX)
            .f0wm()
            .bits(RX_FIFO_MAX)
    });
    word_addr += 18 * RX_FIFO_MAX as u16;

    // Rx FIFO 1
    regs.rxf1c.modify(|_, w| unsafe {
        w.f1sa()
            .bits(word_addr)
            .f1s()
            .bits(RX_FIFO_MAX)
            .f1wm()
            .bits(RX_FIFO_MAX)
    });
    word_addr += 18 * RX_FIFO_MAX as u16;

    // Rx buffer - see below
    // Tx event FIFO
    regs.txefc.modify(|_, w| unsafe {
        w.efsa()
            .bits(word_addr)
            .efs()
            .bits(TX_EVENT_MAX)
            .efwm()
            .bits(TX_EVENT_MAX)
    });
    word_addr += 2 * TX_EVENT_MAX as u16;

    // Tx buffers
    regs.txbc
        .modify(|_, w| unsafe { w.tbsa().bits(word_addr).tfqs().bits(TX_FIFO_MAX) });
    word_addr += 18 * TX_FIFO_MAX as u16;

    // Rx Buffer - not used
    regs.rxbc.modify(|_, w| unsafe { w.rbsa().bits(word_addr) });

    // TX event FIFO?
    // Trigger memory?

    // Set the element sizes to 16 bytes
    regs.rxesc
        .modify(|_, w| unsafe { w.rbds().bits(0b111).f1ds().bits(0b111).f0ds().bits(0b111) });
    regs.txesc.modify(|_, w| unsafe { w.tbds().bits(0b111) });
}

// Implement the traits required for the `bxcan` or `fdcan` library.
unsafe impl fdcan::Instance for Can {
    const REGISTERS: *mut fdcan::RegisterBlock = FDCAN::ptr() as *mut _;
}

unsafe impl fdcan::message_ram::Instance for Can {
    #[cfg(feature = "h7")]
    // H743 RM, table 8. "Register boundary addresses". 0x4000_AC00 - 0x4000_D3FF". CAN message RAM.
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (0x4000_ac00 as *mut _);
    // todo: (0x4000_ac00 + 0x1000) for H7, CAN2.
    // todo: (0x4000_a750 as *mut _) for G4, CAN2
    // todo: (0x4000_aaa0 as *mut _) fir G4 CAN3.
}
