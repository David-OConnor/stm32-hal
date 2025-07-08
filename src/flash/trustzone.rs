// todo: Dual bank support.

use core;

use cfg_if::cfg_if;

use super::{Flash, page_to_address};
use crate::pac::FLASH;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

#[derive(Clone, Copy)]
/// Cortex-M33 secure programming, or nonsecure.
pub enum Security {
    NonSecure,
    Secure,
}

// todo
#[derive(Clone, Copy, PartialEq)]
/// Set dual bank mode (DBANK option bit)
pub enum DualBank {
    Dual,
    Single,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Bank {
    B1 = 0,
    B2 = 1,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
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

/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, NSPGSERR is set.
fn clear_error_flags(regs: &FLASH, security: Security) {
    match security {
        Security::NonSecure => {
            let sr = regs.nssr.read();

            // todo: Can't find this, although it's in the RM.
            // if sr.nsoptwerr().bit_is_set() {
            //     regs.nssr.write(|w| w.nsoptwerr().bit(true));
            // }
            if sr.nspgserr().bit_is_set() {
                regs.nssr.write(|w| w.nspgserr().bit(true));
            }
            if sr.nssizerr().bit_is_set() {
                regs.nssr.write(|w| w.nssizerr().bit(true));
            }
            if sr.nspgaerr().bit_is_set() {
                regs.nssr.write(|w| w.nspgaerr().bit(true));
            }
            if sr.nswrperr().bit_is_set() {
                regs.nssr.write(|w| w.nswrperr().bit(true));
            }
            if sr.nsprogerr().bit_is_set() {
                regs.nssr.write(|w| w.nsprogerr().bit(true));
            }
            if sr.nsoperr().bit_is_set() {
                regs.nssr.write(|w| w.nsoperr().bit(true));
            }
        }
        Security::Secure => {
            let sr = regs.secsr().read();

            if sr.secpgserr().bit_is_set() {
                regs.secsr().write(|w| w.secpgserr().bit(true));
            }
            if sr.secsizerr().bit_is_set() {
                regs.secsr().write(|w| w.secsizerr().bit(true));
            }
            if sr.secpgaerr().bit_is_set() {
                regs.secsr().write(|w| w.secpgaerr().bit(true));
            }
            if sr.secwrperr().bit_is_set() {
                regs.secsr().write(|w| w.secwrperr().bit(true));
            }
            if sr.secprogerr().bit_is_set() {
                regs.secsr().write(|w| w.secprogerr().bit(true));
            }
            if sr.secoperr().bit_is_set() {
                regs.secsr().write(|w| w.secoperr().bit(true));
            }
        }
    }
}

impl Flash {
    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 6.3.5.
    pub fn unlock(&mut self, security: Security) -> Result<(), Error> {
        match security {
            Security::NonSecure => {
                if self.regs.nscr().read().nslock().bit_is_clear() {
                    return Ok(());
                }

                self.regs.nskeyr().write(|w| unsafe { w.bits(FLASH_KEY1) });
                self.regs.nskeyr().write(|w| unsafe { w.bits(FLASH_KEY2) });

                if self.regs.nscr().read().nslock().bit_is_clear() {
                    Ok(())
                } else {
                    Err(Error::Failure)
                }
            }
            Security::Secure => {
                if self.regs.seccr().read().seclock().bit_is_clear() {
                    return Ok(());
                }

                self.regs.seckeyr().write(|w| unsafe { w.bits(FLASH_KEY1) });
                self.regs.seckeyr().write(|w| unsafe { w.bits(FLASH_KEY2) });

                if self.regs.seccr().read().seclock().bit_is_clear() {
                    Ok(())
                } else {
                    Err(Error::Failure)
                }
            }
        }
    }

    /// Lock the flash memory, allowing writes.
    pub fn lock(&mut self, security: Security) {
        match security {
            Security::NonSecure => {
                while self.regs.nssr.read().nsbsy().bit_is_set() {}
                self.regs.nscr().modify(|_, w| w.nslock().bit(true));
            }
            Security::Secure => {
                while self.regs.secsr().read().secbsy().bit_is_set() {}
                self.regs.seccr().modify(|_, w| w.seclock().bit(true));
            }
        };
    }

    /// Erase an entire page. See L5 Reference manual, section 6.3.6.
    /// For why this is required, reference L4 RM, section 3.3.7:
    /// "Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR)."
    pub fn erase_page(&mut self, bank: Bank, page: usize, security: Security) -> Result<(), Error> {
        self.unlock(security)?;

        match security {
            Security::NonSecure => {
                // 1. Check that no Flash memory operation is ongoing by checking the NSBSY bit in the Flash
                // status register (FLASH_NSSR).
                let sr = self.regs.nssr.read();
                if sr.nsbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                // 2. Check and clear all error programming flags due to a previous programming. If not,
                // NSPGSERR is set.
                clear_error_flags(&self.regs, security);

                // 3. In dual-bank mode (DBANK option bit is set), set the NSPER bit and select the
                // non-secure page to erase (NSPNB) with the associated bank (NSBKER) in the
                // FLASH_NScr(). In single-bank mode (DBANK option bit is reset), set the NSPER bit
                // and select the page to erase (NSPNB). The NSBKER bit in the FLASH_NSCR must be
                // kept cleared.
                if self.dual_bank == DualBank::Dual {
                    self.regs.nscr().modify(|_, w| unsafe {
                        w.nsbker().bit(bank as u8 != 0);
                        w.nspnb().bits(page as u8);
                        w.nsper().bit(true)
                    });
                } else {
                    self.regs.nscr().modify(|_, w| unsafe {
                        w.nspnb().bits(page as u8);
                        w.nsper().bit(true)
                    });
                }

                // 4. Set the NSSTRT bit in the FLASH_NSCR register.
                self.regs.nscr().modify(|_, w| w.nsstrt().bit(true));

                // 5. Wait for the NSBSY bit to be cleared in the FLASH_SR register.
                while self.regs.nssr.read().nsbsy().bit_is_set() {}
                self.regs.nscr().modify(|_, w| w.nsper().clear_bit());
            }
            Security::Secure => {
                let sr = self.regs.secsr().read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                if self.dual_bank == DualBank::Dual {
                    self.regs.seccr().modify(|_, w| unsafe {
                        w.secbker().bit(bank as u8 != 0);
                        w.secpnb().bits(page as u8);
                        w.secper().bit(true)
                    });
                } else {
                    self.regs.seccr().modify(|_, w| unsafe {
                        w.secpnb().bits(page as u8);
                        w.secper().bit(true)
                    });
                }

                self.regs.seccr().modify(|_, w| w.secstrt().bit(true));

                while self.regs.secsr().read().secbsy().bit_is_set() {}
                self.regs.nscr().modify(|_, w| w.nsper().clear_bit());
            }
        }

        self.lock(security);

        Ok(())
    }

    /// Mass erase: L5 RM section 6.3.6
    pub fn erase_bank(&mut self, bank: Bank, security: Security) -> Result<(), Error> {
        self.unlock(security)?;

        match security {
            Security::NonSecure => {
                // To perform a bank Mass Erase, follow the procedure below:

                // 1. Check that no Flash memory operation is ongoing by checking the NSBSY bit in the
                // FLASH_NSSR register.
                let sr = self.regs.nssr.read();
                if sr.nsbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                // 2. Check and clear all error programming flags due to a previous programming. If not,
                // NSPGSERR is set.
                clear_error_flags(&self.regs, security);

                // 3. Set the NSMER1 bit or NSMER2 (depending on the bank) in the FLASH_NSCR
                // register. Both banks can be selected in the same operation, in that case it corresponds
                // to a mass erase.
                match bank {
                    Bank::B1 => self.regs.nscr().modify(|_, w| w.nsmer1().bit(true)),
                    Bank::B2 => self.regs.nscr().modify(|_, w| w.nsmer2().clear_bit()),
                }

                // 4. Set the NSSTRT bit in the FLASH_NSCR register.
                self.regs.nscr().modify(|_, w| w.nsstrt().bit(true));

                // 5. Wait for the NSBSY bit to be cleared in the FLASH_NSSR register.
                while self.regs.nssr.read().nsbsy().bit_is_set() {}

                // 6. The NSMER1 or NSMER2 bits can be cleared if no more non-secure bank erase is
                // requested.
                match bank {
                    Bank::B1 => self.regs.nscr().modify(|_, w| w.nsmer1().clear_bit()),
                    Bank::B2 => self.regs.nscr().modify(|_, w| w.nsmer2().clear_bit()),
                }
            }
            Security::Secure => {
                let sr = self.regs.secsr().read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                match bank {
                    Bank::B1 => self.regs.seccr().modify(|_, w| w.secmer1().bit(true)),
                    Bank::B2 => self.regs.seccr().modify(|_, w| w.secmer2().clear_bit()),
                }

                self.regs.seccr().modify(|_, w| w.secstrt().bit(true));

                while self.regs.secsr().read().secbsy().bit_is_set() {}

                match bank {
                    Bank::B1 => self.regs.seccr().modify(|_, w| w.secmer1().clear_bit()),
                    Bank::B2 => self.regs.seccr().modify(|_, w| w.secmer2().clear_bit()),
                }
            }
        }

        self.lock(security);

        Ok(())
    }

    /// Write the contents of a page. Must be erased first. See L5 RM, section 6.3.7.
    pub fn write_page(
        &mut self,
        bank: Bank,
        page: usize,
        data: &[u8],
        security: Security,
    ) -> Result<(), Error> {
        // todo: Consider a u8-based approach.
        // todo: DRY from `erase_page`.
        // The Flash memory programming sequence in standard mode is as follows:
        // 1. Check that no Flash main memory operation is ongoing by checking the NBBSY bit in the
        // Flash status register (FLASH_SR).
        self.unlock(security)?;

        match security {
            Security::NonSecure => {
                let sr = self.regs.nssr.read();
                if sr.nsbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                // 2. Check and clear all error programming flags due to a previous programming. If not,
                // NSPGSERR is set.
                clear_error_flags(&self.regs, security);

                // 3. Set the NSPG bit in tFLASH_NSCR register
                self.regs.nscr().modify(|_, w| w.nspg().bit(true));

                // todo: You have 3x DRY here re teh writing. Put that in  a fn?
                // 4. Perform the data write operation at the desired memory address, inside main memory
                // block or OTP area. Only double word can be programmed.
                let mut address = page_to_address(self.dual_bank, bank, page) as *mut u32;

                // Map our 8-bit data input API to the 64-bit write API.
                // "The Flash memory is programmed 72 bits at a time (64 bits + 8 bits ECC)."
                for chunk in data.chunks(8) {
                    // 8 bytes is 64 bits. (Double word)
                    // Pad to 64 bits if required, ie on the last word.
                    let mut padded = [0xff; 8];

                    let (word1, word2) = if chunk.len() < 8 {
                        // 0xff due to the default value of erased pages.
                        padded[0..chunk.len()].clone_from_slice(&chunk);

                        (
                            u32::from_le_bytes(padded[0..4].try_into().unwrap()),
                            u32::from_le_bytes(padded[4..8].try_into().unwrap()),
                        )
                    } else {
                        (
                            u32::from_le_bytes(chunk[0..4].try_into().unwrap()),
                            u32::from_le_bytes(chunk[4..8].try_into().unwrap()),
                        )
                    };

                    // 5. Wait until the BSY bit is cleared in the FLASH_NSSR register.
                    while self.regs.nssr.read().nsbsy().bit_is_set() {}

                    // 6. Check that NSEOP flag is set in the FLASH_NSSR register (meaning that the programming
                    // operation has succeed), and clear it by software.
                    if self.regs.nssr.read().nseop().bit_is_set() {
                        self.regs.nssr.modify(|_, w| w.nseop().bit(true));
                    }
                }

                // 7. Clear the NSPG bit in the FLASH_CR register if there no more programming request
                // anymore.
                self.regs.nscr().modify(|_, w| w.nspg().clear_bit());
            }
            Security::Secure => {
                // Process here is monstly the same, but sub in sec registers and fields.
                let sr = self.regs.secsr().read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                self.regs.seccr().modify(|_, w| w.secpg().bit(true));

                let mut address = page_to_address(self.dual_bank, bank, page) as *mut u32;

                // todo: No need to repeat this section.
                for chunk in data.chunks(8) {
                    // 8 bytes is 64 bits. (Double word)
                    // Pad to 64 bits if required, ie on the last word.
                    let mut padded = [0xff; 8];

                    let (word1, word2) = if chunk.len() < 8 {
                        // 0xff due to the default value of erased pages.
                        padded[0..chunk.len()].clone_from_slice(&chunk);

                        (
                            u32::from_le_bytes(padded[0..4].try_into().unwrap()),
                            u32::from_le_bytes(padded[4..8].try_into().unwrap()),
                        )
                    } else {
                        (
                            u32::from_le_bytes(chunk[0..4].try_into().unwrap()),
                            u32::from_le_bytes(chunk[4..8].try_into().unwrap()),
                        )
                    };

                    unsafe {
                        // Write a first word in an address aligned with double wor
                        core::ptr::write_volatile(address, word1);
                        address = address.add(1);
                        // Write the second word
                        core::ptr::write_volatile(address, word2);
                        address = address.add(1);
                    }

                    while self.regs.secsr().read().secbsy().bit_is_set() {}

                    if self.regs.secsr().read().seceop().bit_is_set() {
                        self.regs.secsr().modify(|_, w| w.seceop().bit(true)); // clear
                    }
                }

                self.regs.seccr().modify(|_, w| w.secpg().clear_bit());
            }
        }

        self.lock(security);

        Ok(())
    }

    /// Erase a page, then write to it.
    pub fn erase_write_page(
        &mut self,
        bank: Bank,
        page: usize,
        data: &[u8],
        security: Security,
    ) -> Result<(), Error> {
        self.erase_page(bank, page, security)?;
        self.write_page(bank, page, data, security)?;

        Ok(())
    }
}
