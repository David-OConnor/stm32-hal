//! I2C support for F4, which uses an older peripheral than the other families supported
//! by this library.

// todo: Merge this with the other i2c module?

// Based on `stm32f4xx-hal`.

use core::ops::Deref;

#[cfg(feature = "embedded_hal")]
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use paste::paste;

use crate::{
    clocks::Clocks,
    pac::{self, RCC, i2c1},
    util::rcc_en_reset,
};

#[derive(Clone, Copy)]
pub enum I2cDevice {
    One,
    Two,
    #[cfg(not(feature = "f410"))]
    Three,
}

#[derive(Debug, defmt::Format)]
pub enum I2cError {
    Overrun,
    Nack,
    Timeout,
    // Note: The  error type is not currently returned, but is maintained for backwards
    // compatibility.
    Bus,
    Crc,
    Arbitration,
    RegisterUnchanged,
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            I2cError::Overrun => ErrorKind::Overrun,
            I2cError::Nack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            I2cError::Timeout | I2cError::Crc => ErrorKind::Other,
            I2cError::Bus => ErrorKind::Bus,
            I2cError::Arbitration => ErrorKind::ArbitrationLoss,
        }
    }
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
                if ccr < 4 { 4 } else { ccr }
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

    pub fn check_and_clear_error_flags(&self) -> Result<i2c1::sr1::R, I2cError> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let sr1 = self.regs.sr1.read();

        if sr1.timeout().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.timeout().clear_bit());
            return Err(I2cError::Timeout);
        }

        if sr1.pecerr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.pecerr().clear_bit());
            return Err(I2cError::Crc);
        }

        if sr1.ovr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.ovr().clear_bit());
            return Err(I2cError::Overrun);
        }

        if sr1.af().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.af().clear_bit());
            return Err(I2cError::Nack);
        }

        if sr1.arlo().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.arlo().clear_bit());
            return Err(I2cError::Arbitration);
        }

        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if sr1.berr().bit_is_set() {
            self.regs.sr1.modify(|_, w| w.berr().clear_bit());
        }

        Ok(sr1)
    }

    pub fn write_bytes(&mut self, addr: u8, bytes: &[u8]) -> Result<(), I2cError> {
        // Send a START condition
        self.regs.cr1.modify(|_, w| w.start().set_bit());

        // Wait until START condition was generated
        let sr1 = self.check_and_clear_error_flags()?;
        bounded_loop!(sr1.sb().bit_is_clear(), I2cError::RegisterUnchanged);

        // Also wait until signalled we're master and everything is waiting for us
        let sr2 = self.regs.sr2.read();
        bounded_loop!(
            sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear(),
            I2cError::RegisterUnchanged
        );

        // Set up current address, we're trying to talk to
        self.regs
            .dr
            .write(|w| unsafe { w.bits(u32::from(addr) << 1) });

        let sr1 = self.check_and_clear_error_flags()?;
        // Wait until address was sent
        //
        // Check for any I2C errors. If a Nack occurs, the ADDR bit will never be set.
        // Wait for the address to be acknowledged
        bounded_loop!(sr1.addr().bit_is_clear(), I2cError::RegisterUnchanged);

        // Clear condition by reading SR2
        self.regs.sr2.read();

        // Send bytes
        for c in bytes {
            self.send_byte(*c)?;
        }

        // Fallthrough is success
        Ok(())
    }

    pub fn send_byte(&self, byte: u8) -> Result<(), I2cError> {
        // Wait until we're ready for sending
        // Check for any I2C errors. If a Nack occurs, the ADDR bit will never be set.
        let sr1 = self.check_and_clear_error_flags()?;
        bounded_loop!(sr1.tx_e().bit_is_clear(), I2cError::RegisterUnchanged);

        // Push out a byte of data
        self.regs.dr.write(|w| unsafe { w.bits(u32::from(byte)) });

        // Check for any potential error conditions.
        let sr1 = self.check_and_clear_error_flags()?;

        // Wait until byte is transferred
        bounded_loop!(sr1.btf().bit_is_clear(), I2cError::RegisterUnchanged);

        Ok(())
    }

    pub fn recv_byte(&self) -> Result<u8, I2cError> {
        let sr1 = self.check_and_clear_error_flags()?;
        // Check for any potential error conditions.
        bounded_loop!(sr1.rx_ne().bit_is_clear(), I2cError::RegisterUnchanged);

        let value = self.regs.dr.read().bits() as u8;
        Ok(value)
    }
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl<R> embedded_hal::i2c::ErrorType for I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = I2cError;
}

#[cfg(feature = "embedded_hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded_hal")))]
impl<R> embedded_hal::i2c::I2c<SevenBitAddress> for I2c<R>
where
    R: Deref<Target = i2c1::RegisterBlock>,
{
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::I2cError> {
        /*

        embedded_hal Operation Contract:

        1 - Before executing the first operation an ST is sent automatically. This is followed by SAD+R/W as appropriate.
        2 - Data from adjacent operations of the same type are sent after each other without an SP or SR.
        3 - Between adjacent operations of a different type an SR and SAD+R/W is sent.
        4 - After executing the last operation an SP is sent automatically.
        5 - If the last operation is a Read the master does not send an acknowledge for the last byte.
          - ST = start condition
          - SAD+R/W = slave address followed by bit 1 to indicate reading or 0 to indicate writing
          - SR = repeated start condition
          - SP = stop condition

        Note: This hasn't been implemented! I assume 1 and 3 will be, but for 2 we start/stop after each operation
        */

        for op in operations {
            match op {
                Operation::Write(buffer) => {
                    self.write_bytes(address, buffer)?;

                    // Send a STOP condition
                    self.regs.cr1.modify(|_, w| w.stop().set_bit());

                    // Wait for STOP condition to transmit.
                    bounded_loop!(
                        self.regs.cr1.read().stop().bit_is_set(),
                        I2cError::RegisterUnchanged
                    );
                }

                Operation::Read(buffer) => {
                    if let Some((last, buffer)) = buffer.split_last_mut() {
                        // Send a START condition and set ACK bit
                        self.regs
                            .cr1
                            .modify(|_, w| w.start().set_bit().ack().set_bit());

                        // Wait until START condition was generated
                        bounded_loop!(
                            self.regs.sr1.read().sb().bit_is_clear(),
                            I2cError::RegisterUnchanged
                        );

                        // Also wait until signalled we're master and everything is waiting for us
                        let sr2 = self.regs.sr2.read();
                        bounded_loop!(
                            sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear(),
                            I2cError::RegisterUnchanged
                        );

                        // Set up current address, we're trying to talk to
                        self.regs
                            .dr
                            .write(|w| unsafe { w.bits((u32::from(address) << 1) + 1) });

                        let sr1 = self.check_and_clear_error_flags()?;
                        // Wait until address was sent
                        bounded_loop!(
                            sr1.read().addr().bit_is_clear(),
                            I2cError::RegisterUnchanged
                        );

                        // Clear condition by reading SR2
                        self.regs.sr2.read();

                        // Receive bytes into buffer
                        for c in buffer {
                            *c = self.recv_byte()?;
                        }

                        // Prepare to send Nack then STOP after next byte
                        self.regs
                            .cr1
                            .modify(|_, w| w.ack().clear_bit().stop().set_bit());

                        // Receive last byte
                        *last = self.recv_byte()?;

                        // Wait for the STOP to be sent.
                        bounded_loop!(
                            self.regs.cr1.read().stop().bit_is_set(),
                            I2cError::RegisterUnchanged
                        );
                    } else {
                        return Err(I2cError::Overrun);
                    }
                }
            }
        }

        Ok(())
    }
}
