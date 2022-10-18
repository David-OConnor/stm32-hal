// todo: Between L5 and the rest, and L5 secure and non, there's a lot of DRY!
// todo: The answer may be macros.

use crate::pac::FLASH;

#[cfg(feature = "h7")]
use crate::pac::flash::BANK;

use core;

use cfg_if::cfg_if;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;
const FLASH_OPT_KEY1: u32 = 0x0819_2A3B;
const FLASH_OPT_KEY2: u32 = 0x4C5D_6E7F;

#[derive(Clone, Copy, PartialEq)]
/// Set dual bank mode (DBANK option bit). Eg G4
pub enum DualBank {
    Dual,
    Single,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// For dual-bank variants. u8 value is used for page erase: sets `CR` reg, `BKER` bit.
pub enum Bank {
    B1 = 0,
    // todo: PAC bank 2 error
    #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
    B2 = 1,
}

#[derive(Copy, Clone, Debug)]
/// Possible error states for flash operations.
pub enum Error {
    /// Flash controller is not done yet
    Busy,
    /// Error detected (by command execution, or because no command could be executed)
    Illegal,
    /// Set during read if ECC decoding logic detects correctable or uncorrectable error
    EccError,
    /// Page number is out of range
    PageOutOfRange,
    /// (Legal) command failed
    Failure,
}

// todo: Bank 2 support on H7 and others.

#[cfg(not(feature = "h7"))]
/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, PGSERR is set.
fn clear_error_flags(regs: &FLASH) {
    let sr = regs.sr.read();

    cfg_if! {
        if #[cfg(any(feature = "f3"))] {
            if sr.wrprterr().bit_is_set() {
                regs.sr.modify(|_, w| w.wrprterr().set_bit());
            }
            if sr.pgerr().bit_is_set() {
                regs.sr.modify(|_, w| w.pgerr().set_bit());
            }
        } else if #[cfg(any(feature = "f4"))] {
            // if sr.rderr().bit_is_set() {
            //     regs.sr.modify(|_, w| w.rderr().set_bit());
            // }
            if sr.pgserr().bit_is_set() {
                regs.sr.modify(|_, w| w.pgserr().set_bit());
            }
            if sr.pgperr().bit_is_set() {
                regs.sr.modify(|_, w| w.pgperr().set_bit());
            }
            if sr.pgaerr().bit_is_set() {
                regs.sr.modify(|_, w| w.pgaerr().set_bit());
            }
            if sr.wrperr().bit_is_set() {
                regs.sr.modify(|_, w| w.wrperr().set_bit());
            }
            if sr.operr().bit_is_set() {
                regs.sr.modify(|_, w| w.operr().set_bit());
            }
        } else {
            if sr.optverr().bit_is_set() {
                regs.sr.write(|w| w.optverr().set_bit());
            }
            if sr.rderr().bit_is_set() {
                regs.sr.write(|w| w.rderr().set_bit());
            }
            if sr.fasterr().bit_is_set() {
                regs.sr.write(|w| w.fasterr().set_bit());
            }
            #[cfg(not(feature = "wl"))]
            if sr.miserr().bit_is_set() {
                regs.sr.write(|w| w.miserr().set_bit());
            }
            if sr.pgserr().bit_is_set() {
                regs.sr.write(|w| w.pgserr().set_bit());
            }
            if sr.sizerr().bit_is_set() {
                regs.sr.write(|w| w.sizerr().set_bit());
            }
            if sr.pgaerr().bit_is_set() {
                regs.sr.write(|w| w.pgaerr().set_bit());
            }
            if sr.wrperr().bit_is_set() {
                regs.sr.write(|w| w.wrperr().set_bit());
            }
            if sr.progerr().bit_is_set() {
                regs.sr.write(|w| w.progerr().set_bit());
            }
            if sr.operr().bit_is_set() {
                regs.sr.write(|w| w.operr().set_bit());
            }

        }
    }
}

#[cfg(feature = "h7")]
/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, PGSERR is set.
fn clear_error_flags(regs: &BANK) {
    let sr = regs.sr.read();

    if sr.dbeccerr().bit_is_set() {
        regs.ccr.write(|w| w.clr_dbeccerr().set_bit());
    }
    #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
    if sr.sneccerr1().bit_is_set() {
        regs.ccr.write(|w| w.clr_sneccerr().set_bit());
    }
    #[cfg(any(feature = "h747cm4", feature = "h747cm7"))]
    if sr.sneccerr().bit_is_set() {
        regs.ccr.write(|w| w.clr_sneccerr().set_bit());
    }
    if sr.rdserr().bit_is_set() {
        regs.ccr.write(|w| w.clr_rdserr().set_bit());
    }
    if sr.rdperr().bit_is_set() {
        regs.ccr.write(|w| w.clr_rdperr().set_bit());
    }
    if sr.operr().bit_is_set() {
        regs.ccr.write(|w| w.clr_operr().set_bit());
    }
    if sr.incerr().bit_is_set() {
        regs.ccr.write(|w| w.clr_incerr().set_bit());
    }
    if sr.strberr().bit_is_set() {
        regs.ccr.write(|w| w.clr_strberr().set_bit());
    }
    if sr.pgserr().bit_is_set() {
        regs.ccr.write(|w| w.clr_pgserr().set_bit());
    }
    if sr.wrperr().bit_is_set() {
        regs.ccr.write(|w| w.clr_wrperr().set_bit());
    }
}

pub struct Flash {
    pub regs: FLASH,
    #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))]
    pub dual_bank: DualBank,
}

impl Flash {
    /// Create a struct used to perform operations on Flash.
    pub fn new(regs: FLASH) -> Self {
        cfg_if! {
            if #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))] {
                // Some G4 variants let you select dual or single-bank mode.
                Self { regs, dual_bank: DualBank::Single }
            } else {
                Self { regs}
            }
        }
    }

    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 3.3.5.
    /// (G4 RM section 5.3.5)
    /// "After reset, write is not allowed in the Flash control register (FLASH_CR) to protect the
    /// Flash memory against possible unwanted operations due, for example, to electric
    /// disturbances."
    pub fn unlock(&mut self) -> Result<(), Error> {
        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = self.regs.bank1();

        if regs.cr.read().lock().bit_is_clear() {
            return Ok(());
        }

        // The following sequence is used to unlock this register:
        // 1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
        // 2. Write KEY2 = 0xCDEF89AB in the FLASH_KEYR register.
        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

        if regs.cr.read().lock().bit_is_clear() {
            Ok(())
        } else {
            Err(Error::Failure)
        }
    }

    // /// Unlock the FLASH_OPTCR register, for writing option bits.
    // pub fn unlock_options(&mut self) -> Result<(), Error> {
    //     #[cfg(not(feature = "h7"))]
    //     let regs = &self.regs;
    //     #[cfg(feature = "h7")]
    //     let regs = self.regs.bank1();
    //
    //     if regs.cr.read().lock().bit_is_clear() {
    //         return Ok(());
    //     }
    //
    //     regs.optkeyr.write(|w| unsafe { w.bits(FLASH_OPT_KEY1) });
    //     regs.optkeyr.write(|w| unsafe { w.bits(FLASH_OPT_KEY2) });
    //
    //     if regs.cr.read().lock().bit_is_clear() {
    //         Ok(())
    //     } else {
    //         Err(Error::Failure)
    //     }
    // }

    pub fn lock(&mut self) {
        // The FLASH_CR register cannot be written when the BSY bit in the Flash status register
        // (FLASH_SR) is set. Any attempt to write to it with the BSY bit set causes the AHB bus to
        // stall until the BSY bit is cleared.
        // todo: bank

        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = &self.regs.bank1();

        while regs.sr.read().bsy().bit_is_set() {}
        regs.cr.modify(|_, w| w.lock().set_bit());
    }

    #[cfg(not(feature = "h7"))]
    #[allow(unused_variables)] // bank arg on single-bank MCUs.
    /// Erase an entire page. See L4 Reference manual, section 3.3.5.
    /// For why this is required, reference L4 RM, section 3.3.7:
    /// "Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR)."
    pub fn erase_page(&mut self, bank: Bank, page: usize) -> Result<(), Error> {
        self.unlock()?;
        let regs = &self.regs;

        // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
        // status register (FLASH_SR).
        if regs.sr.read().bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        clear_error_flags(regs);

        // 3. Set the PER bit and select the page you wish to erase (PNB). For dual bank variants:
        //  - with the associated bank(BKER) in the Flash control register (FLASH_CR).
        cfg_if! {
            if #[cfg(feature = "f3")] {
                // F3 RM: "Erase procedure"
                // Set the PER bit in the FLASH_CR register
                regs.cr.modify(|_, w| w.per().set_bit());

                // Program the FLASH_CR register
                // regs.ar.modify(|_, w| w.far().bits(page as u8));
                regs.ar.write(|w| unsafe { w.bits(page as u32) }); // todo: Is this right?
            } else if #[cfg(feature = "f4")] {
                // Set the SER bit and select the sector out of the 12 sectors (for STM32F405xx/07xx and
                // STM32F415xx/17xx) and out of 24 (for STM32F42xxx and STM32F43xxx) in the main
                // memory block you wish to erase (SNB) in the FLASH_CR register
                regs.cr.modify(|_, w| unsafe {
                    w.ser().set_bit();
                    w.snb().bits(page as u8) // todo: Probably not right?
                });
            } else if #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))] {
                // 3. (G4 dual-bank devices: In dual bank mode (DBANK option bit is set), set the PER bit and
                // select the page to
                // erase (PNB) with the associated bank (BKER) in the Flash control register
                // (FLASH_CR). In single bank mode (DBANK option bit is reset), set the PER bit and
                // select the page to erase (PNB). The BKER bit in the Flash control register
                // (FLASH_CR) must be kept cleared)
                if self.dual_bank == DualBank::Dual {
                     regs.cr.modify(|_, w| unsafe {
                        // w.bker().bits(bank as u8); // todo: PAC error
                        w.pnb().bits(page as u8);
                        w.per().set_bit()
                    });
                } else {
                     regs.cr.modify(|_, w| unsafe {
                        w.pnb().bits(page as u8);
                        w.per().set_bit()
                    });
                }
            } else {
                 regs.cr.modify(|_, w| unsafe {
                    w.pnb().bits(page as u8);
                    w.per().set_bit()
                });
            }
        }

        // 4. Set the STRT bit in the FLASH_CR register.
        cfg_if! {
            if #[cfg(not(any(feature = "l4", feature = "l4p", feature = "h7")))] {
                regs.cr.modify(|_, w| w.strt().set_bit());
            } else {
                regs.cr.modify(|_, w| w.start().set_bit());
            }
        }

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while regs.sr.read().bsy().bit_is_set() {}

        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4"))] {
                // Check the EOP flag in the FLASH_SR register (it is set when the erase operation has
                // succeeded), and then clear it by software.
                while regs.sr.read().eop().bit_is_clear() {}
                regs.sr.modify(|_, w| w.eop().set_bit());
            }
        }
        #[cfg(not(feature = "f4"))]
        regs.cr.modify(|_, w| w.per().clear_bit());
        #[cfg(feature = "f4")]
        regs.cr.modify(|_, w| w.ser().clear_bit());

        self.lock();

        Ok(())
    }

    #[cfg(feature = "h7")]
    /// Erase a 128kb sector. See H743 RM, section 4.3.10: FLASH erase operations; subsection
    /// Flash sector erase sequence. Note that this is similar to the procedure for other
    /// families, but has a different name "sector" vice "page", and the RM instructions
    /// are phrased differently.
    pub fn erase_sector(&mut self, bank: Bank, sector: usize) -> Result<(), Error> {
        self.unlock()?;

        let regs = &match bank {
            Bank::B1 => self.regs.bank1(),
            // todo: PAC bank 2 error
            #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
            Bank::B2 => self.regs.bank2(),
        };

        // To erase a 128-Kbyte user sector, proceed as follows:
        // 1. Check and clear (optional) all the error flags due to previous programming/erase
        // operation. Refer to Section 4.7: FLASH error management for details.
        clear_error_flags(&regs);

        // 2.Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration
        // protection (only if register is not already unlocked).
        // self.unlock()?; // (Above; out of order due to borrow-checker issue)

        // 3. Set the SER1/2 bit and SNB1/2 bitfield in the corresponding FLASH_CR1/2 register.
        // SER1/2 indicates a sector erase operation, while SNB1/2 contains the target sector
        // number.
        regs.cr.modify(|_, w| unsafe {
            w.ser().set_bit();
            w.snb().bits(sector as u8) // todo: Probably not right?
        });

        // 4. Set the START1/2 bit in the FLASH_CR1/2 register.
        regs.cr.modify(|_, w| w.start().set_bit());

        // 5. Wait for the QW1/2 bit to be cleared in the corresponding FLASH_SR1/2 register.
        while regs.sr.read().qw().bit_is_set() {}

        self.lock();

        Ok(())
    }

    /// Erase one or both banks. Called "Mass erase" on single-bank variants like G4.
    pub fn erase_bank(&mut self, bank: Bank) -> Result<(), Error> {
        // todo: DRY
        // (H7): 2. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration
        // protection (only if register is not already unlocked).
        self.unlock()?;

        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = &match bank {
            Bank::B1 => self.regs.bank1(),
            // todo: PAC bank 2 error
            #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
            Bank::B2 => self.regs.bank2(),
        };

        // To perform a bank Mass Erase, follow the procedure below:
        // RM0351 Rev 7 105/1903
        // RM0351 Embedded Flash memory (FLASH)
        // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
        // FLASH_SR register.
        // (H7): 1. Check and clear (optional) all the error flags due to previous programming/erase
        // operation. Refer to Section 4.7: FLASH error management for details.
        let sr = regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        clear_error_flags(regs);

        // 3. Set the MER1 bit or/and MER2 (depending on the bank) in the Flash control register
        // (FLASH_CR). Both banks can be selected in the same operation.
        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4", feature = "g0", feature = "wb", feature = "wl"))] {
                regs.cr.modify(|_, w| w.mer().set_bit());
            } else if #[cfg(feature = "h7")] {
                // 3. Set the BER1/2 bit in the FLASH_CR1/2 register corresponding to the targeted bank.
                regs.cr.modify(|_, w| w.ber().set_bit());
            } else { // L4, G4
                match bank {
                    Bank::B1 => {
                        regs.cr.modify(|_, w| w.mer1().set_bit());
                    }
                    Bank::B2 => {
                        // todo: Other families that have dual bank too!
                        // #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))]
                        // regs.cr.modify(|_, w| w.mer2().set_bit()); // todo PAC error
                    }
                }
            }
        }

        // 4. Set the STRT bit in the FLASH_CR register.
        cfg_if! {
             if #[cfg(feature = "h7")] {
                // 4. Set the START bit in the FLASH_CR1/2 register to start the bank erase operation. Then
                // wait until the QW1/2 bit is cleared in the corresponding FLASH_SR1/2 register.
                regs.cr.modify(|_, w| w.start().set_bit());
                while regs.sr.read().qw().bit_is_set() {}
            } else if #[cfg(any(feature = "l4", feature = "l4p"))] {
                regs.cr.modify( | _, w | w.start().set_bit());
            } else {
                regs.cr.modify(|_, w| w.strt().set_bit());
            }
        }

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while regs.sr.read().bsy().bit_is_set() {}

        // (Some RMs describe this procedure, to clear mer, with ambiguity of if it's required)
        cfg_if! {
            if #[cfg(feature = "h7")] {
                regs.cr.modify(|_, w| w.ber().clear_bit());
            } else if #[cfg(any(feature = "l4", feature = "l4p", feature = "g4"))] {
                regs.cr.modify(|_, w| w.mer1().clear_bit());
            } else {
                regs.cr.modify(|_, w| w.mer().clear_bit());
            }
        }

        self.lock();

        Ok(())
    }

    // todo: For multibank variants, accept a bank argument.
    /// Write the contents of a page. Must be erased first. See L4 RM, section 3.3.7.
    /// Make sure the page is one your MCU has, and isn't being used for the program itself.
    #[cfg(not(feature = "h7"))]
    #[allow(unused_variables)] // bank arg on single-bank MCUs.
    pub fn write_page(&mut self, bank: Bank, page: usize, data: &[u8]) -> Result<(), Error> {
        // todo: Consider a u8-based approach.
        // todo: DRY from `erase_page`.

        self.unlock()?;

        let regs = &self.regs;

        // The Flash memory programming sequence in standard mode is as follows:
        // 1. Check that no Flash main memory operation is ongoing by checking the BSY bit in the
        // Flash status register (FLASH_SR).
        let sr = regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        clear_error_flags(regs);

        // 3. Set the PG bit in the Flash control register (FLASH_CR).
        regs.cr.modify(|_, w| w.pg().set_bit());

        // 4. Perform the data write operation at the desired memory address, inside main memory
        // block or OTP area. Only double word can be programmed.

        cfg_if! {
             if #[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))] {
                let mut address = page_to_address(self.dual_bank, bank, page) as *mut u32;
            } else {
                let mut address = page_to_address(page) as *mut u32;
            }
        }

        // Map our 8-bit data input API to the 64-bit write API. (Used by all variants, even though
        // they have different read sizes.)
        // "The Flash memory is programmed 72 bits at a time (64 bits + 8 bits ECC)."
        for chunk in data.chunks_exact(8) {
            let word1 = u32::from_le_bytes(chunk[0..4].try_into().unwrap());
            let word2 = u32::from_le_bytes(chunk[4..8].try_into().unwrap());

            unsafe {
                // Write a first word in an address aligned with double wor
                core::ptr::write_volatile(address, word1);
                address = address.add(1);
                // Write the second word
                core::ptr::write_volatile(address, word2);
                address = address.add(1);
            }
            // 5. Wait until the BSY bit is cleared in the FLASH_SR register.
            while regs.sr.read().bsy().bit_is_set() {}

            // 6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
            // operation has succeed), and clear it by software.
            if regs.sr.read().eop().bit_is_set() {
                regs.sr.modify(|_, w| w.eop().set_bit()); // Clear
            }
        }

        // 7. Clear the PG bit in the FLASH_CR register if there no more programming request
        // anymore.
        regs.cr.modify(|_, w| w.pg().clear_bit());

        self.lock();

        Ok(())
    }

    /// Write the contents of a sector. Must be erased first. See H742 or H723-35 RM, section 4.3.9.
    /// Make sure the sector is one your MCU has, and isn't being used for the program itself. Writes
    /// a byte array, 256 bits at a time.
    #[cfg(feature = "h7")]
    pub fn write_sector(&mut self, bank: Bank, sector: usize, data: &[u8]) -> Result<(), Error> {
        // 1. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration
        // protection (only if register is not already unlocked).
        self.unlock()?;

        let regs = &self.regs.bank1(); // todo: Bank 2 support!

        // 2. Enable write operations by setting the PG1/2 bit in the FLASH_CR1/2 register.
        regs.cr.modify(|_, w| w.pg().set_bit());

        // 3. Check the protection of the targeted memory area. (todo?)

        // 4. Write one Flash-word corresponding to 32-byte data starting at a 32-byte aligned
        // address.
        let mut address = sector_to_address(bank, sector) as *mut u32;

        // Note that the key element separating each 256-bit writes is wating until the `qw` bit
        // is cleared.
        for chunk in data.chunks_exact(32) {
            // We use 8 pointer-sized (32-bit) words to meet our full 32-byte (256-bit) write.
            for i in 0..8 {
                let word = u32::from_le_bytes(chunk[i * 4..i * 4 + 4].try_into().unwrap());
                unsafe {
                    core::ptr::write_volatile(address, word);
                    address = address.add(1);
                }
            }

            // 5. Check that QW has been raised and wait until it is reset to 0.
            while regs.sr.read().qw().bit_is_set() {}
        }

        self.lock();

        Ok(())
    }

    /// Erase a page, then write to it.
    #[cfg(not(feature = "h7"))]
    pub fn erase_write_page(&mut self, bank: Bank, page: usize, data: &[u8]) -> Result<(), Error> {
        self.erase_page(bank, page)?;
        self.write_page(bank, page, data)?;

        Ok(())
    }

    /// Erase a page, then write to it.
    /// Make sure the sector is one your MCU has, and isn't being used for the program itself.
    #[cfg(feature = "h7")]
    pub fn erase_write_sector(
        &mut self,
        bank: Bank,
        page: usize,
        data: &[u8],
    ) -> Result<(), Error> {
        self.erase_sector(bank, page)?;
        self.write_sector(bank, page, data)?;

        Ok(())
    }

    /// Read flash memory at a given page and offset into an 8-bit-dword buffer.
    #[allow(unused_variables)] // bank arg on single-bank MCUs.
    pub fn read(&self, bank: Bank, page: usize, offset: usize, buf: &mut [u8]) {
        // H742 RM, section 4.3.8:
        // Single read sequence
        // The recommended simple read sequence is the following:
        // 1. Freely perform read accesses to any AXI-mapped area.
        // 2. The embedded Flash memory effectively executes the read operation from the read
        // command queue buffer as soon as the non-volatile memory is ready and the previously
        // requested operations on this specific bank have been served.
        cfg_if! {
            if #[cfg(any(
                feature = "g473",
                feature = "g474",
                feature = "g483",
                feature = "g484",
            ))] {
                let mut addr = page_to_address(self.dual_bank, bank, page) as *mut u32;
            } else if #[cfg(feature = "h7")]{
                let mut addr = sector_to_address(bank, page) as *mut u32;
            } else {
                let mut addr = page_to_address(page) as *mut u32;
            }
        }

        unsafe {
            // Offset it by the start position
            addr = unsafe { addr.add(offset) };
            // Iterate on chunks of 32bits
            for chunk in buf.chunks_exact_mut(4) {
                let word = unsafe { core::ptr::read_volatile(addr) };
                chunk[0..4].copy_from_slice(&word.to_le_bytes());
                unsafe { addr = addr.add(1) };
            }
        }
    }
}

/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
#[cfg(not(any(
    feature = "g473",
    feature = "g474",
    feature = "g483",
    feature = "g484",
    feature = "h7"
)))]
fn page_to_address(page: usize) -> usize {
    super::BANK1_START_ADDR + page * super::PAGE_SIZE
}

#[cfg(any(feature = "g473", feature = "g474", feature = "g483", feature = "g484"))]
fn page_to_address(dual_bank: DualBank, bank: Bank, page: usize) -> usize {
    if dual_bank == DualBank::Single {
        super::BANK1_START_ADDR + page * super::PAGE_SIZE_SINGLE_BANK
    } else {
        match bank {
            Bank::B1 => super::BANK1_START_ADDR + page * super::PAGE_SIZE_DUAL_BANK,
            Bank::B2 => super::BANK2_START_ADDR + page * super::PAGE_SIZE_DUAL_BANK,
        }
    }
}

#[cfg(feature = "h7")]
/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
fn sector_to_address(bank: Bank, sector: usize) -> usize {
    // Note; Named sector on H7.
    let starting_pt = match bank {
        Bank::B1 => super::BANK1_START_ADDR,
        // todo: This isn't the same bank2 starting point for all H7 variants!
        #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
        Bank::B2 => super::BANK2_START_ADDR,
    };

    starting_pt + sector * super::SECTOR_SIZE
}
