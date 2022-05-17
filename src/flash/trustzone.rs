// todo: Dual bank support.

use crate::pac::FLASH;

// use crate::pac::flash::BANK;

use core;

use cfg_if::cfg_if;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

#[derive(Clone, Copy)]
/// Cortex-M33 secure programming, or nonsecure.
pub enum Security {
    NonSecure,
    Secure,
}

// todo
#[derive(Clone, Copy)]
/// Set dual bank mode (DBANK option bit)
enum _DualBank {
    Dual,
    Single,
}

#[derive(Clone, Copy)]
pub enum Bank {
    B1,
    B2,
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

/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, NSPGSERR is set.
fn clear_error_flags(regs: &FLASH, security: Security) {
    match security {
        Security::NonSecure => {
            let sr = regs.nssr.read();

            // todo: Can't find this, although it's in the RM.
            // if sr.nsoptwerr().bit_is_set() {
            //     regs.nssr.write(|w| w.nsoptwerr().set_bit());
            // }
            if sr.nspgserr().bit_is_set() {
                regs.nssr.write(|w| w.nspgserr().set_bit());
            }
            if sr.nssizerr().bit_is_set() {
                regs.nssr.write(|w| w.nssizerr().set_bit());
            }
            if sr.nspgaerr().bit_is_set() {
                regs.nssr.write(|w| w.nspgaerr().set_bit());
            }
            if sr.nswrperr().bit_is_set() {
                regs.nssr.write(|w| w.nswrperr().set_bit());
            }
            if sr.nsprogerr().bit_is_set() {
                regs.nssr.write(|w| w.nsprogerr().set_bit());
            }
            if sr.nsoperr().bit_is_set() {
                regs.nssr.write(|w| w.nsoperr().set_bit());
            }
        }
        Security::Secure => {
            let sr = regs.secsr.read();

            if sr.secpgserr().bit_is_set() {
                regs.secsr.write(|w| w.secpgserr().set_bit());
            }
            if sr.secsizerr().bit_is_set() {
                regs.secsr.write(|w| w.secsizerr().set_bit());
            }
            if sr.secpgaerr().bit_is_set() {
                regs.secsr.write(|w| w.secpgaerr().set_bit());
            }
            if sr.secwrperr().bit_is_set() {
                regs.secsr.write(|w| w.secwrperr().set_bit());
            }
            if sr.secprogerr().bit_is_set() {
                regs.secsr.write(|w| w.secprogerr().set_bit());
            }
            if sr.secoperr().bit_is_set() {
                regs.secsr.write(|w| w.secoperr().set_bit());
            }
        }
    }
}

pub struct Flash {
    pub regs: FLASH,
}

impl Flash {
    /// Create a struct used to perform operations on Flash.
    pub fn new(regs: FLASH) -> Self {
        // todo: Implement and configure dual bank mode.
        Self { regs }
    }

    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 6.3.5.
    pub fn unlock(&mut self, security: Security) -> Result<(), Error> {
        match security {
            Security::NonSecure => {
                if self.regs.nscr.read().nslock().bit_is_clear() {
                    return Ok(());
                }

                self.regs.nskeyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
                self.regs.nskeyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

                if self.regs.nscr.read().nslock().bit_is_clear() {
                    Ok(())
                } else {
                    Err(Error::Failure)
                }
            }
            Security::Secure => {
                if self.regs.seccr.read().seclock().bit_is_clear() {
                    return Ok(());
                }

                self.regs.seckeyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
                self.regs.seckeyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

                if self.regs.seccr.read().seclock().bit_is_clear() {
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
                self.regs.nscr.modify(|_, w| w.nslock().set_bit());
            }
            Security::Secure => {
                while self.regs.secsr.read().secbsy().bit_is_set() {}
                self.regs.seccr.modify(|_, w| w.seclock().set_bit());
            }
        };
    }

    /// Erase an entire page. See L5 Reference manual, section 6.3.6.
    /// For why this is required, reference L4 RM, section 3.3.7:
    /// "Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR)."
    pub fn erase_page(&mut self, page: usize, security: Security) -> Result<(), Error> {
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
                // FLASH_NSCR. In single-bank mode (DBANK option bit is reset), set the NSPER bit
                // and select the page to erase (NSPNB). The NSBKER bit in the FLASH_NSCR must be
                // kept cleared.
                // todo: Follow that procedure; this may not be right.

                match page {
                    0..=255 => {
                        self.regs.nscr.modify(|_, w| unsafe {
                            w.nsbker()
                                .clear_bit()
                                .nspnb()
                                .bits(page as u8)
                                .nsper()
                                .set_bit()
                        });
                    }
                    256..=511 => {
                        self.regs.nscr.modify(|_, w| unsafe {
                            w.nsbker()
                                .set_bit()
                                .nspnb()
                                .bits((page - 256) as u8)
                                .nsper()
                                .set_bit()
                        });
                    }
                    _ => {
                        return Err(Error::PageOutOfRange);
                    }
                }

                // 4. Set the NSSTRT bit in the FLASH_NSCR register.
                self.regs.nscr.modify(|_, w| w.nsstrt().set_bit());

                // 5. Wait for the NSBSY bit to be cleared in the FLASH_SR register.
                while self.regs.nssr.read().nsbsy().bit_is_set() {}
                self.regs.nscr.modify(|_, w| w.nsper().clear_bit());
            }
            Security::Secure => {
                let sr = self.regs.secsr.read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                match page {
                    0..=255 => {
                        self.regs.seccr.modify(|_, w| unsafe {
                            w.secbker().clear_bit();
                            w.secpnb().bits(page as u8);
                            w.secper().set_bit()
                        });
                    }
                    256..=511 => {
                        self.regs.seccr.modify(|_, w| unsafe {
                            w.secbker().set_bit();
                            w.secpnb().bits((page - 256) as u8);
                            w.secper().set_bit()
                        });
                    }
                    _ => {
                        return Err(Error::PageOutOfRange);
                    }
                }

                self.regs.seccr.modify(|_, w| w.secstrt().set_bit());

                while self.regs.secsr.read().secbsy().bit_is_set() {}
                self.regs.nscr.modify(|_, w| w.nsper().clear_bit());
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

                // 3. Set the MER1 bit or/and MER2 (depending on the bank) in the Flash control register
                // (FLASH_CR). Both banks can be selected in the same operation.
                match bank {
                    Bank::B1 => self.regs.nscr.modify(|_, w| w.nsmer1().clear_bit()),
                    #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
                    Bank::B2 => self.regs.nscr.modify(|_, w| w.nsmer2().clear_bit()),
                }

                // 4. Set the NSSTRT bit in the FLASH_NSCR register.
                self.regs.nscr.modify(|_, w| w.nsstrt().set_bit());

                // 5. Wait for the NSBSY bit to be cleared in the FLASH_NSSR register.
                while self.regs.nssr.read().nsbsy().bit_is_set() {}
            }
            Security::Secure => {
                let sr = self.regs.secsr.read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                match bank {
                    Bank::B1 => self.regs.seccr.modify(|_, w| w.secmer1().clear_bit()),
                    #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
                    Bank::B2 => self.regs.seccr.modify(|_, w| w.secmer2().clear_bit()),
                }

                self.regs.seccr.modify(|_, w| w.secstrt().set_bit());

                while self.regs.secsr.read().secbsy().bit_is_set() {}
            }
        }

        self.lock(security);

        Ok(())
    }

    /// Write the contents of a page. Must be erased first. See L5 RM, section 6.3.7.
    pub fn write_page(
        &mut self,
        page: usize,
        data: &[u64],
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
                self.regs.nscr.modify(|_, w| w.nspg().set_bit());

                // todo: You have 3x DRY here re teh writing. Put that in  a fn?
                // 4. Perform the data write operation at the desired memory address, inside main memory
                // block or OTP area. Only double word can be programmed.
                let mut address = page_to_address(page) as *mut u32;

                for dword in data {
                    unsafe {
                        // – Write a first word in an address aligned with double word
                        core::ptr::write_volatile(address, *dword as u32);
                        // – Write the second word
                        core::ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                        address = address.add(2);
                    }

                    // 5. Wait until the BSY bit is cleared in the FLASH_NSSR register.
                    while self.regs.nssr.read().nsbsy().bit_is_set() {}

                    // 6. Check that NSEOP flag is set in the FLASH_NSSR register (meaning that the programming
                    // operation has succeed), and clear it by software.
                    if self.regs.nssr.read().nseop().bit_is_set() {
                        self.regs.nssr.modify(|_, w| w.nseop().set_bit());
                    } // todo: Else return error?
                }

                // 7. Clear the NSPG bit in the FLASH_CR register if there no more programming request
                // anymore.
                self.regs.nscr.modify(|_, w| w.nspg().clear_bit());
            }
            Security::Secure => {
                // Process here is monstly the same, but sub in sec registers and fields.
                let sr = self.regs.secsr.read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                clear_error_flags(&self.regs, security);

                self.regs.seccr.modify(|_, w| w.secpg().set_bit());

                let mut address = page_to_address(page) as *mut u32;

                for dword in data {
                    unsafe {
                        // – Write a first word in an address aligned with double word
                        core::ptr::write_volatile(address, *dword as u32);
                        // – Write the second word
                        core::ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                        address = address.add(2);
                    }

                    while self.regs.secsr.read().secbsy().bit_is_set() {}

                    if self.regs.secsr.read().seceop().bit_is_set() {
                        self.regs.secsr.modify(|_, w| w.seceop().set_bit()); // clear
                    } // todo: Else return error?
                }

                self.regs.seccr.modify(|_, w| w.secpg().clear_bit());
            }
        }

        self.lock(security);

        Ok(())
    }

    /// Read a single 64-bit memory cell, indexed by its page, and an offset from the page.
    pub fn read(&self, page: usize, offset: isize) -> u64 {
        let addr = page_to_address(page) as *const u64;
        unsafe { core::ptr::read(addr.offset(offset)) }
    }

    /// Read flash memory at a given page and offset into a buffer.
    pub fn read_to_buffer(&self, page: usize, offset: isize, buff: &mut [u8]) {
        // H742 RM, section 4.3.8:
        // Single read sequence
        // The recommended simple read sequence is the following:
        // 1. Freely perform read accesses to any AXI-mapped area.
        // 2. The embedded Flash memory effectively executes the read operation from the read
        // command queue buffer as soon as the non-volatile memory is ready and the previously
        // requested operations on this specific bank have been served.

        // todo: This is untested.
        #[cfg(not(feature = "h7"))]
        let addr = page_to_address(page) as *const u8; // todo is this right?
                                                       // let addr = page_to_address(page).as_ptr(); // todo is this right?
        #[cfg(feature = "h7")]
        // todo: Don't hard-code bank1.
        let addr = sector_to_address(page, Bank::B1) as *const u8; // todo is this right?
                                                                   // let addr = sector_to_address(page, Bank::B1).as_ptr(); // todo is this right?

        for val in buff {
            *val = unsafe { core::ptr::read(addr.offset(offset)) }
        }
    }
}

/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
fn page_to_address(page: usize) -> usize {
    0x0800_0000 + page * 2048
}
