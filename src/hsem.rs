//! Hardware semaphore (HSEM)
//! Used on STM32WB to synchronize processes running on different cores.

use crate::pac::{self, HSEM, RCC};

#[derive(Clone, Copy)]
/// The core that's performing the requested operation. Core 1 is the M4 core, and Core 2 is the M0+ core.
pub enum Core {
    // todo: This is the same as ipcc::Core; DRY; keep in one place and import in the other/both?
    C1,
    C2,
}

pub struct Hsem {
    regs: HSEM
}

/// Represents an Hardware Semiphore (HSEM) peripheral.
impl Hsem {
    pub fn new(regs: HSEM) -> Self {
        rcc.ahb3enr.modify(|_, w| w.hsemen().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.hsemrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.hsemrst().clear_bit());

        rcc.ahb4enr.modify(|_, w| w.hsemen().set_bit());
        rcc.ahb4rstr.modify(|_, w| w.hsemrst().set_bit());
        rcc.ahb4rstr.modify(|_, w| w.hsemrst().clear_bit());

        Self { regs }
    }

    /// RM: The 2-step lock procedure consists in a write to lock the semaphore, followed by a read to
    /// check if the lock has been successful, carried out from the HSEM_Rx register
    pub fn lock_2_step(&mut self, semaphore_num: u8) {
        if semaphore_num > 31 {
            panic!("Semaphore number must be 0 - 31.")
        }

        // todo: You need a macro to do this!

        // * Write semaphore with PROCID and COREID, and LOCK = 1. The COREID data
        // written by software must match the AHB bus master information. i.e. a AHB bus master
        self.regs.rN.modify(|_, w| {
            w.procid().bits(procid);
            w.coreid().bits(coreid);
            w.lock().set_bit()
        })
        // ID = 1writes data COREID = 1.
        // Lock is put in place when the semaphore is free at write time.
        // * Read-back the semaphore
        // The software checks the lock status, if PROCID and COREID match the written data,
        // then the lock is confirmed.
        // * Else retry (the semaphore has been locked by another process, AHB bus master ID).
    }

    /// RM: The 1-step procedure consists in a read to lock and check the semaphore in a single step,
    /// carried out from the HSEM_RLRx register.
    pub fn lock_1_step(&mut self, semaphore_num: u8) {
        if semaphore_num > 31 {
            panic!("Semaphore number must be 0 - 31.")
        }
        // * Read lock semaphore with the AHB bus master COREID.
        // * If read COREID matches and PROCID = 0, then lock is put in place. If COREID
        // matches and PROCID is not 0, this means that another process from the same
        // COREID has locked the semaphore with a 2-step (write) procedure.
        // * Else retry (the semaphore has been locked by another process, AHB bus master ID).
        // A semaphore can only be locked when it is free. When read locking a free semaphore,
        // PROCID is 0. Read locking a locked semaphore returns the COREID and PROCID that
        // locked it. All read locks, including the first one that locks the semaphore, return the COREID
        // that locks or locked the semaphore.
    }

    pub fn unlock(&self, semaphore_num: u8) {
        if semaphore_num > 31 {
            panic!("Semaphore number must be 0 - 31.")
        }
    }

    /// Enable an interrupt.
    pub fn enable_interrupt(&mut self, core: Core, semaphore_num: u8) {
        if semaphore_num > 31 {
            panic!("Semaphore number must be 0 - 31.")
        }
        // Cnier doesn't have individual fields
        match core {
            Core::C1 => {
                let orig_value = self.regs.c1ier.read().bits();
                self.regs.c1ier.write(|w| w.bits(orig_value | (1 << semaphore_num)));
            }
            Core::C2 => {
                let orig_value = self.regs.c2ier.read().bits();
                self.regs.c2ier.write(|w| w.bits(orig_value | (1 << semaphore_num)));
            }
        }
    }

    /// Clear an interrupt flag - run this in the interrupt's handler to prevent
    /// repeat firings.
    pub fn clear_interrupt(&mut self, semaphore_num: u8) {
        if semaphore_num > 31 {
            panic!("Semaphore number must be 0 - 31.")
        }
        match core {
            Core::C1 => {
                let orig_value = self.regs.c1icer.read().bits();
                self.regs.c1icer.write(|w| w.bits(orig_value | (1 << semaphore_num)));
            }
            Core::C2 => {
                let orig_value = self.regs.c2icer.read().bits();
                self.regs.c2icer.write(|w| w.bits(orig_value | (1 << semaphore_num)));
            }
        }
    }
}