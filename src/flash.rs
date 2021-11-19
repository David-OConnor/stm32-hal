//! Read and write onboard flash memory.
//! Note that on dual bank variants, only bank 1 is supported for now.

// Note that most of the code for L5 and U5 is feature-gated due to different
// register names, differentiating secure and non-secure. We keep them in the same file
// as the other families, to demonstrate that the code is still mostly the same; this
// makes it easier to maintain than separate files.

// todo: Between L5 and the rest, and L5 secure and non, there's a lot of DRY!
// todo: The answer may be macros.

// todo: Review the instructions regarding error checking, eg the first few steps in write.
// todo: Is our code (unlock, check_illegal especially) doing that?

// Based on `stm32l4xx-hal`.

use crate::pac::FLASH;

#[cfg(feature = "h7")]
use crate::pac::flash::BANK;

use core;

use cfg_if::cfg_if;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

#[cfg(feature = "l5")]
#[derive(Clone, Copy)]
/// Cortex-M33 secure programming, or nonsecure.
pub enum Security {
    NonSecure,
    Secure,
}

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

// todo: Bank 2 support on H7 and others.

#[cfg(not(any(feature = "l5", feature = "h7")))]
/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, PGSERR is set.
fn check_illegal(regs: &FLASH) -> Result<(), Error> {
    // todo: QC this fn and its l5 variant.

    let sr = regs.sr.read();

    cfg_if! {
        if #[cfg(any(feature = "f3"))] {
            if sr.pgerr().bit_is_set() || sr.wrprterr().bit_is_set() {
                return Err(Error::Illegal);
            }
        } else if #[cfg(any(feature = "f4"))] {
            if sr.pgaerr().bit_is_set() {  // todo: Others for f4?
                return Err(Error::Illegal);
            }
        } else {
            if sr.pgaerr().bit_is_set() || sr.progerr().bit_is_set() || sr.wrperr().bit_is_set() {
                return Err(Error::Illegal);
            }
        }
    }
    Ok(())
}

#[cfg(feature = "h7")]
/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, PGSERR is set.
fn check_illegal(regs: &BANK) -> Result<(), Error> {
    // todo: QC this fn and its l5 variant.
    let sr = regs.sr.read();
    if sr.pgserr().bit_is_set() || sr.wrperr().bit_is_set() {
        return Err(Error::Illegal);
    }

    Ok(())
}

#[cfg(feature = "l5")]
/// Check and clear all non-secure error programming flags due to a previous
/// programming. If not, NSPGSERR is set.
fn check_illegal(regs: &FLASH, security: Security) -> Result<(), Error> {
    match security {
        Security::NonSecure => {
            let sr = regs.nssr.read();
            if sr.nspgaerr().bit_is_set()
                || sr.nspgserr().bit_is_set()
                || sr.nsprogerr().bit_is_set()
                || sr.nswrperr().bit_is_set()
            {
                return Err(Error::Illegal);
            }
        }
        Security::Secure => {
            let sr = regs.secsr.read();
            if sr.secpgaerr().bit_is_set()
                || sr.secpgaerr().bit_is_set()
                || sr.secprogerr().bit_is_set()
                || sr.secwrperr().bit_is_set()
            {
                return Err(Error::Illegal);
            }
        }
    }

    Ok(())
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

    #[cfg(feature = "l5")]
    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 6.3.5.
    pub fn unlock(&mut self, security: Security) -> Result<(), Error> {
        match security {
            Security::NonSecure => {
                self.regs.nskeyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
                self.regs.nskeyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

                if self.regs.nscr.read().nslock().bit_is_clear() {
                    Ok(())
                } else {
                    Err(Error::Failure)
                }
            }
            Security::Secure => {
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

    #[cfg(not(feature = "l5"))]
    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 3.3.5.
    pub fn unlock(&mut self) -> Result<(), Error> {
        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = self.regs.bank1();

        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

        if regs.cr.read().lock().bit_is_clear() {
            Ok(())
        } else {
            Err(Error::Failure)
        }
    }

    #[cfg(not(feature = "l5"))]
    pub fn lock(&mut self) {
        #[cfg(not(feature = "h7"))]
        self.regs.cr.modify(|_, w| w.lock().set_bit());
        #[cfg(feature = "h7")]
        self.regs.bank1().cr.modify(|_, w| w.lock().set_bit());
    }

    #[cfg(feature = "l5")]
    /// Lock the flash memory, allowing writes.
    pub fn lock(&mut self, security: Security) {
        match security {
            Security::NonSecure => self.regs.nscr.modify(|_, w| w.nslock().set_bit()),
            Security::Secure => self.regs.seccr.modify(|_, w| w.seclock().set_bit()),
        };
    }

    #[cfg(not(feature = "l5"))]
    /// Erase an entire page. See L4 Reference manual, section 3.3.5.
    /// For why this is required, reference L4 RM, section 3.3.7:
    /// "Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR)."
    pub fn erase_page(&mut self, page: usize) -> Result<(), Error> {
        self.unlock()?;

        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = &self.regs.bank1(); // todo: Don't hard code bank1

        // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
        // status register (FLASH_SR).
        let sr = regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        if check_illegal(regs).is_err() {
            self.lock();
            return Err(Error::Illegal);
        };

        // 3. Set the PER bit and select the page you wish to erase (PNB) with the associated bank
        // (BKER) in the Flash control register (FLASH_CR).

        // Note that `STM32L4` includes the `.bker()` bit to select banks for all variants, but
        // some variants only have 1 memory bank; eg ones with a smaller amount of memory.

        cfg_if! {
            if #[cfg(feature = "f3")] {
                // F3 RM: "Erase procedure"
                // Set the PER bit in the FLASH_CR register
                regs.cr.modify(|_, w| w.per().set_bit());

                // Program the FLASH_CR register
                // regs.ar.modify(|_, w| w.far().bits(page as u8));
                regs.ar.write(|w| unsafe { w.bits(page as u32) }); // todo: Is this right?
            } else if #[cfg(any(feature = "f4", feature = "h7"))] {
                // Set the SER bit and select the sector out of the 12 sectors (for STM32F405xx/07xx and
                // STM32F415xx/17xx) and out of 24 (for STM32F42xxx and STM32F43xxx) in the main
                // memory block you wish to erase (SNB) in the FLASH_CR register
                regs.cr.modify(|_, w| unsafe {
                    w.ser().set_bit();
                    w.snb().bits(page as u8) // todo: Probably not right?
                });

            } else if #[cfg(any(feature = "g0", feature = "g4", feature = "wb", feature = "wl"))] {
                 regs.cr.modify(|_, w| unsafe {
                    w.pnb()
                    .bits(page as u8)
                    .per()
                    .set_bit()
                });
            } else {
                match page {
                    0..=255 => {
                        regs.cr.modify(|_, w| unsafe {
                            w.bker().clear_bit().pnb().bits(page as u8).per().set_bit()
                        });
                    }
                    256..=511 => {
                        regs.cr.modify(|_, w| unsafe {
                            w.bker()
                                .set_bit()
                                .pnb()
                                .bits((page - 256) as u8)
                                .per()
                                .set_bit()
                        });
                    }
                    _ => {
                        return Err(Error::PageOutOfRange);
                    }
                }
            }
        }

        // 4. Set the STRT bit in the FLASH_CR register.
        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4", feature = "wb", feature = "wl"))] {
                regs.cr.modify(|_, w| w.strt().set_bit());
            } else {
                #[cfg(any(feature = "g0", feature = "g4"))]
                regs.cr.modify(|_, w| w.strt().set_bit());
                #[cfg(not(any(feature = "g0", feature = "g4")))]
                regs.cr.modify(|_, w| w.start().set_bit());
            }
        }

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while regs.sr.read().bsy().bit_is_set() {}

        // todo on F3: "Read the erased option bytes and verify" as final step
        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4", feature = "h7"))] {
                // Check the EOP flag in the FLASH_SR register (it is set when the erase operation has
                // succeeded), and then clear it by software. (todo)

                // todo: Check H7 procedure here.

                // Clear the EOP flag
                regs.sr.modify(|_, w| w.eop().set_bit());
            } else {
                regs.cr.modify(|_, w| w.per().clear_bit());
            }
        }

        self.lock();

        Ok(())
    }

    #[cfg(feature = "l5")]
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
                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

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
            }
            Security::Secure => {
                let sr = self.regs.secsr.read();
                if sr.secbsy().bit_is_set() {
                    self.lock(security);
                    return Err(Error::Busy);
                }

                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

                match page {
                    0..=255 => {
                        self.regs.seccr.modify(|_, w| unsafe {
                            w.secbker()
                                .clear_bit()
                                .secpnb()
                                .bits(page as u8)
                                .secper()
                                .set_bit()
                        });
                    }
                    256..=511 => {
                        self.regs.seccr.modify(|_, w| unsafe {
                            w.secbker()
                                .set_bit()
                                .secpnb()
                                .bits((page - 256) as u8)
                                .secper()
                                .set_bit()
                        });
                    }
                    _ => {
                        return Err(Error::PageOutOfRange);
                    }
                }

                self.regs.seccr.modify(|_, w| w.secstrt().set_bit());

                while self.regs.secsr.read().secbsy().bit_is_set() {}
            }
        }

        self.lock(security);

        Ok(())
    }

    #[cfg(not(feature = "l5"))]
    /// Erase one or both banks.
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
            Bank::B2 => self.regs.bank2(),
        };

        // To perform a bank Mass Erase, follow the procedure below:
        // RM0351 Rev 7 105/1903
        // RM0351 Embedded Flash memory (FLASH)
        // 139
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
        if check_illegal(regs).is_err() {
            self.lock();
            return Err(Error::Illegal);
        };

        // 3. Set the MER1 bit or/and MER2 (depending on the bank) in the Flash control register
        // (FLASH_CR). Both banks can be selected in the same operation.

        cfg_if! {
            if #[cfg(any(feature = "f3", feature = "f4", feature = "wb", feature = "wl", feature = "h7"))] {
                #[cfg(not(feature = "h7"))]
                regs.cr.modify(|_, w| w.mer().clear_bit());
                #[cfg(feature = "h7")]
                // 3. Set the BER1/2 bit in the FLASH_CR1/2 register corresponding to the targeted bank.
                regs.cr.modify(|_, w| w.ber().clear_bit());

                // 4. Set the STRT bit in the FLASH_CR register.
                #[cfg(not(feature = "h7"))]
                regs.cr.modify(|_, w| w.strt().set_bit());
                #[cfg(feature = "h7")]
                // Set the START1/2 bit in the FLASH_CR1/2 register to start the bank erase operation.
                // Then wait until the QW1/2 bit is cleared in the corresponding FLASH_SR1/2 register.
                regs.cr.modify(|_, w| w.start().set_bit());
            } else if #[cfg(any(feature = "g0"))] {
                regs.cr.modify(|_, w| w.mer().clear_bit());
            } else if #[cfg(any(feature = "g4"))] {
                regs.cr.modify(|_, w| w.mer1().clear_bit());
            } else {
                match bank {
                    Bank::B1 => regs.cr.modify(|_, w| w.mer1().clear_bit()),
                    Bank::B2 => regs.cr.modify(|_, w| w.mer2().clear_bit()),
                }

                // 4. Set the STRT bit in the FLASH_CR register.
                #[cfg(any(feature = "g4", feature = "wl"))]
                regs.cr.modify(|_, w| w.strt().set_bit());
                #[cfg(not(feature = "g4"))]
                regs.cr.modify(|_, w| w.start().set_bit());
            }
        }

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while regs.sr.read().bsy().bit_is_set() {}

        self.lock();

        Ok(())
    }

    #[cfg(feature = "l5")]
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
                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

                // 3. Set the MER1 bit or/and MER2 (depending on the bank) in the Flash control register
                // (FLASH_CR). Both banks can be selected in the same operation.
                match bank {
                    Bank::B1 => self.regs.nscr.modify(|_, w| w.nsmer1().clear_bit()),
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

                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

                match bank {
                    Bank::B1 => self.regs.seccr.modify(|_, w| w.secmer1().clear_bit()),
                    Bank::B2 => self.regs.seccr.modify(|_, w| w.secmer2().clear_bit()),
                }

                self.regs.seccr.modify(|_, w| w.secstrt().set_bit());

                while self.regs.secsr.read().secbsy().bit_is_set() {}
            }
        }

        self.lock(security);

        Ok(())
    }

    // todo: For H7 etc, accept a bank argument.

    #[cfg(not(feature = "l5"))]
    /// Write the contents of a page. Must be erased first. See L4 RM, section 3.3.7.
    pub fn write_page(&mut self, page: usize, data: &[u64]) -> Result<(), Error> {
        // todo: Consider a u8-based approach.
        // todo: DRY from `erase_page`.
        // The Flash memory programming sequence in standard mode is as follows:
        // 1. Check that no Flash main memory operation is ongoing by checking the BSY bit in the
        // Flash status register (FLASH_SR).
        self.unlock()?;

        #[cfg(not(feature = "h7"))]
        let regs = &self.regs;
        #[cfg(feature = "h7")]
        let regs = &self.regs.bank1();

        let sr = regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        if check_illegal(regs).is_err() {
            self.lock();
            return Err(Error::Illegal);
        };

        // 3. Set the PG bit in the Flash control register (FLASH_CR).
        regs.cr.modify(|_, w| w.pg().set_bit());

        // 4. Perform the data write operation at the desired memory address, inside main memory
        // block or OTP area. Only double word can be programmed.

        #[cfg(not(feature = "h7"))]
        let mut address = page_to_address(page) as *mut u32;
        #[cfg(feature = "h7")]
        let mut address = sector_to_address(page, Bank::B1) as *mut u32;

        for dword in data {
            unsafe {
                // – Write a first word in an address aligned with double word
                core::ptr::write_volatile(address, *dword as u32);
                // – Write the second word
                core::ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                address = address.add(2);
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

    #[cfg(feature = "l5")]
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
                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

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

                if check_illegal(&self.regs, security).is_err() {
                    self.lock(security);
                    return Err(Error::Illegal);
                };

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

    #[cfg(not(feature = "h7"))]
    /// Read a single 64-bit memory cell, indexed by its page, and an offset from the page.
    pub fn read(&self, page: usize, offset: isize) -> u64 {
        let addr = page_to_address(page) as *const u64;
        unsafe { core::ptr::read(addr.offset(offset)) }
    }

    // #[cfg(not(feature = "h7"))]
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

    // #[cfg(feature = "h7")]
    // /// Read flash memory at a given page and offset into a buffer.
    // ///    H742 RM, section 4.3.8:
    // /// Single read sequence
    // /// The recommended simple read sequence is the following:
    // /// 1. Freely perform read accesses to any AXI-mapped area.
    // /// 2. The embedded Flash memory effectively executes the read operation from the read
    // /// command queue buffer as soon as the non-volatile memory is ready and the previously
    // /// requested operations on this specific bank have been served.
    // pub fn read_to_buffer(&self, sector: usize, offset: isize, buff: &mut [u8]) {
    //     //  todo: H7 can read in 64 bit, 32 bit, 16 bit, or one byte granularities.
    //     //     // todo: H7 uses 256 bit words for writing and erasing.
    //     // for val in buff {
    //     for i in buff.len() {
    //         let val = buff[i]
    //         *val = unsafe { core::ptr::read(addr.offset(i)) };
    //     }
    // }
}

#[cfg(not(feature = "h7"))]
/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
fn page_to_address(page: usize) -> usize {
    0x0800_0000 + page * 2048
}

#[cfg(feature = "h7")]
/// Calculate the address of the start of a given page. Each page is 2,048 Kb for non-H7.
/// For H7, sectors are 128Kb, with 8 sectors per bank.
fn sector_to_address(sector: usize, bank: Bank) -> usize {
    let starting_pt = match bank {
        Bank::B1 => 0x0800_0000,
        // todo: This isn't the same bank2 starting point for all H7 variants!
        Bank::B2 => 0x0810_0000,
    };

    starting_pt + sector * 0x2_0000
}
