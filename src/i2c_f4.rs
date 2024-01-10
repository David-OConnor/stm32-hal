//! I2C support for F4, which uses an older peripheral than the other families supported
//! by this library.

// todo: Merge this with the other i2c module?

// Based on `stm32f4xx-hal`.

use core::ops::Deref;

#[cfg(feature = "embedded_hal")]
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use paste::paste;

use crate::{
    clocks::Clocks,
    pac::{self, i2c1, RCC},
    util::rcc_en_reset,
};

#[derive(Clone, Copy)]
pub enum I2cDevice {
    One,
    Two,
    #[cfg(not(feature = "f410"))]
    Three,
}

#[derive(Debug)]
pub enum Error {
    OVERRUN,
    NACK,
    TIMEOUT,
    // Note: The BUS error type is not currently returned, but is maintained for backwards
    // compatibility.
    BUS,
    CRC,
    ARBITRATION,
}

/// Represents an Inter-Integrated Circuit (I2C) peripheral.
pub struct I2c<R> {
    pub regs: R,
}

impl<R> I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    pub fn new(regs: R, device: I2cDevice, speed: u32, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };

        match device {
            I2cDevice::One => {
                rcc_en_reset!(apb1, i2c1, rcc);
            }
            I2cDevice::Two => {
                rcc_en_reset!(apb1, i2c2, rcc);
            }
            #[cfg(not(feature = "f410"))]
            I2cDevice::Three => {
                rcc_en_reset!(apb1, i2c3, rcc);
            }
        }

        let result = Self { regs };
        result.i2c_init(speed, clocks.apb1());
        result
    }

    fn i2c_init(&self, speed: u32, pclk: u32) {
        // Make sure the I2C unit is disabled so we can configure it
        self.regs.cr1.modify(|_, w| w.pe().clear_bit());

        // Calculate settings for I2C speed modes
        let clock = pclk;
        let freq = clock / 1_000_000;
        assert!(freq >= 2 && freq <= 50);

        // Configure bus frequency into I2C peripheral
        self.regs
            .cr2
            .write(|w| unsafe { w.freq().bits(freq as u8) });

        let trise = if speed <= 100_000 {
            freq + 1
        } else {
            (freq * 300) / 1000 + 1
        };

        // Configure correct rise times
        self.regs.trise.write(|w| w.trise().bits(trise as u8));

        // I2C clock control calculation
        if speed <= 100_000 {
            let ccr = {
                let ccr = clock / (speed * 2);
                if ccr < 4 {
                    4
                } else {
                    ccr
                }
            };

            // Set clock to standard mode with appropriate parameters for selected speed
            self.regs.ccr.write(|w| unsafe {
                w.f_s()
                    .clear_bit()
                    .duty()
                    .clear_bit()
                    .ccr()
                    .bits(ccr as u16)
            });
        } else {
            const DUTYCYCLE: u8 = 0;
            if DUTYCYCLE == 0 {
                let ccr = clock / (speed * 3);
                let ccr = if ccr < 1 { 1 } else { ccr };

                // Set clock to fast mode with appropriate parameters for selected speed (2:1 duty cycle)
                self.regs.ccr.write(|w| unsafe {
                    w.f_s().set_bit().duty().clear_bit().ccr().bits(ccr as u16)
                });
            } else {
                let ccr = clock / (speed * 25);
                let ccr = if ccr < 1 { 1 } else { ccr };

                // Set clock to fast mode with appropriate parameters for selected speed (16:9 duty cycle)
                self.regs.ccr.write(|w| unsafe {
                    w.f_s().set_bit().duty().set_bit().ccr().bits(ccr as u16)
                });
            }
        }

        // Enable the I2C processing
        self.regs.cr1.modify(|_, w| w.pe().set_bit());
    }

    pub fn check_and_clear_error_flags(&self) -> Result<i2c1::sr1::R, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let sr1 = self.regs.sr1.read();

        if sr1.timeout().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.timeout().clear_bit());
            return Err(Error::TIMEOUT);
        }

        if sr1.pecerr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.pecerr().clear_bit());
            return Err(Error::CRC);
        }

        if sr1.ovr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.ovr().clear_bit());
            return Err(Error::OVERRUN);
        }

        if sr1.af().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.af().clear_bit());
            return Err(Error::NACK);
        }

        if sr1.arlo().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.arlo().clear_bit());
            return Err(Error::ARBITRATION);
        }

        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if sr1.berr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.berr().clear_bit());
        }

        Ok(sr1)
    }

    pub fn write_bytes(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Send a START condition
        self.regs.cr1.modify(|_, w| w.start().set_bit());

        // Wait until START condition was generated
        while self.check_and_clear_error_flags()?.sb().bit_is_clear() {}

        // Also wait until signalled we're master and everything is waiting for us
        while {
            self.check_and_clear_error_flags()?;

            let sr2 = self.regs.sr2.read();
            sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
        } {}

        // Set up current address, we're trying to talk to
        self.regs
            .dr
            .write(|w| unsafe { w.bits(u32::from(addr) << 1) });

        // Wait until address was sent
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            let sr1 = self.check_and_clear_error_flags()?;

            // Wait for the address to be acknowledged
            sr1.addr().bit_is_clear()
        } {}

        // Clear condition by reading SR2
        self.regs.sr2.read();

        // Send bytes
        for c in bytes {
            self.send_byte(*c)?;
        }

        // Fallthrough is success
        Ok(())
    }

    pub fn send_byte(&self, byte: u8) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            self.check_and_clear_error_flags()?.tx_e().bit_is_clear()
        } {}

        // Push out a byte of data
        self.regs.dr.write(|w| unsafe { w.bits(u32::from(byte)) });

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            self.check_and_clear_error_flags()?.btf().bit_is_clear()
        } {}

        Ok(())
    }

    pub fn recv_byte(&self) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            self.check_and_clear_error_flags()?;

            self.regs.sr1.read().rx_ne().bit_is_clear()
        } {}

        let value = self.regs.dr.read().bits() as u8;
        Ok(value)
    }
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl<R> WriteRead for I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.write_bytes(addr, bytes)?;
        self.read(addr, buffer)?;

        Ok(())
    }
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl<R> Write for I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(addr, bytes)?;

        // Send a STOP condition
        self.regs.cr1.modify(|_, w| w.stop().set_bit());

        // Wait for STOP condition to transmit.
        while self.regs.cr1.read().stop().bit_is_set() {}

        // Fallthrough is success
        Ok(())
    }
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl<R> Read for I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Send a START condition and set ACK bit
            self.regs
                .cr1
                .modify(|_, w| w.start().set_bit().ack().set_bit());

            // Wait until START condition was generated
            while self.regs.sr1.read().sb().bit_is_clear() {}

            // Also wait until signalled we're master and everything is waiting for us
            while {
                let sr2 = self.regs.sr2.read();
                sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
            } {}

            // Set up current address, we're trying to talk to
            self.regs
                .dr
                .write(|w| unsafe { w.bits((u32::from(addr) << 1) + 1) });

            // Wait until address was sent
            while {
                self.check_and_clear_error_flags()?;
                self.regs.sr1.read().addr().bit_is_clear()
            } {}

            // Clear condition by reading SR2
            self.regs.sr2.read();

            // Receive bytes into buffer
            for c in buffer {
                *c = self.recv_byte()?;
            }

            // Prepare to send NACK then STOP after next byte
            self.regs
                .cr1
                .modify(|_, w| w.ack().clear_bit().stop().set_bit());

            // Receive last byte
            *last = self.recv_byte()?;

            // Wait for the STOP to be sent.
            while self.regs.cr1.read().stop().bit_is_set() {}

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::OVERRUN)
        }
    }
}
