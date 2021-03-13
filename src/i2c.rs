//! Inter-Integrated Circuit (I2C) bus. Based on `stm32h7xx-hal`.

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::{
    gpio::{AltFn, GpioPin, OutputType, PinMode},
    pac::{i2c1, RCC},
    traits::ClockCfg,
};

use cast::{u16, u8};
use core::ops::Deref;

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[derive(Clone, Copy)]
pub enum I2cDevice {
    One,
    Two,
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C> {
    i2c: I2C,
}

impl<I2C> I2c<I2C>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    /// Configures the I2C peripheral. `freq` is in Hz. Checks pin cfg.
    pub fn new<C: ClockCfg, SDA: GpioPin, SCL: GpioPin>(
        i2c: I2C,
        device: I2cDevice,
        sda: SDA,
        scl: SCL,
        freq: u32,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        if let OutputType::PushPull = sda.get_output_type() {
            panic!("SDA pin must be configured as open drain.")
        }
        if let OutputType::PushPull = scl.get_output_type() {
            panic!("SCL pin must be configured as open drain.")
        }
        // todo: Check we're using the right pins, and the alt fns.

        Self::new_unchecked(i2c, device, freq, clocks, rcc)
    }

    /// Configures the I2C peripheral. `freq` is in Hz. Doesn't check pin config.
    pub fn new_unchecked<C: ClockCfg>(
        i2c: I2C,
        device: I2cDevice,
        freq: u32,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        match device {
            I2cDevice::One => {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "f3")] {
                        rcc.apb1enr.modify(|_, w| w.i2c1en().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
                        rcc.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());
                    } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                        rcc.apb1enr1.modify(|_, w| w.i2c1en().set_bit());
                        rcc.apb1rstr1.modify(|_, w| w.i2c1rst().set_bit());
                        rcc.apb1rstr1.modify(|_, w| w.i2c1rst().clear_bit());
                    }
                }
            }

            I2cDevice::Two => {
                cfg_if::cfg_if! {
                    if #[cfg(any(feature = "f301", feature = "f302", feature = "f303", feature = "f373"))] {
                        rcc.apb1enr.modify( | _, w| w.i2c2en().set_bit());
                        rcc.apb1rstr.modify( | _, w | w.i2c2rst().set_bit());
                        rcc.apb1rstr.modify(| _, w | w.i2c2rst().clear_bit());
                    } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                        rcc.apb1enr1.modify( | _, w| w.i2c2en().set_bit());
                        rcc.apb1rstr1.modify( | _, w | w.i2c2rst().set_bit());
                        rcc.apb1rstr1.modify(| _, w | w.i2c2rst().clear_bit());
                    }
                }
            }
        }

        assert!(freq <= 1_000_000);
        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // TODO review compliance with the timing requirements of I2C
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let i2cclk = clocks.apb1();
        let ratio = i2cclk / freq - 4;
        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
            // fast-mode or fast-mode plus
            // here we pick SCLL + 1 = 2 * (SCLH + 1)
            let presc = ratio / 387;

            let sclh = ((ratio / (presc + 1)) - 3) / 3;
            let scll = 2 * (sclh + 1) - 1;

            let (sdadel, scldel) = if freq > 400_000 {
                // fast-mode plus
                let sdadel = 0;
                let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            } else {
                // fast-mode
                let sdadel = i2cclk / 8_000_000 / (presc + 1);
                let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            };

            (presc, scll, sclh, sdadel, scldel)
        } else {
            // standard-mode
            // here we pick SCLL = SCLH
            let presc = ratio / 514;

            let sclh = ((ratio / (presc + 1)) - 2) / 2;
            let scll = sclh;

            let sdadel = i2cclk / 2_000_000 / (presc + 1);
            let scldel = i2cclk / 800_000 / (presc + 1) - 1;

            (presc, scll, sclh, sdadel, scldel)
        };

        let presc = u8(presc).unwrap();
        assert!(presc < 16);
        let scldel = u8(scldel).unwrap();
        assert!(scldel < 16);
        let sdadel = u8(sdadel).unwrap();
        assert!(sdadel < 16);
        let sclh = u8(sclh).unwrap();
        let scll = u8(scll).unwrap();

        // Configure for "fast mode" (400 KHz)
        i2c.timingr.write(|w| unsafe {
            w.presc()
                .bits(presc)
                .scll()
                .bits(scll)
                .sclh()
                .bits(sclh)
                .sdadel()
                .bits(sdadel)
                .scldel()
                .bits(scldel)
        });

        // Enable the peripheral
        i2c.cr1.write(|w| w.pe().set_bit());

        I2c { i2c }
    }

    /// Frees the I2C peripheral
    pub fn free(self) -> I2C {
        self.i2c
    }
}

/// Copy+pasted from H7. Called from c+p `busy_wait`
/// Sequence to flush the TXDR register. This resets the TXIS and TXE
// flags
macro_rules! flush_txdr {
    ($i2c:expr) => {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if $i2c.isr.read().txis().bit_is_set() {
            $i2c.txdr.write(|w| unsafe { w.txdata().bits(0) });
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if $i2c.isr.read().txe().bit_is_clear() {
            $i2c.isr.write(|w| w.txe().set_bit());
        }
    };
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().bit_is_set() {
                $i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                $i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

impl<I2C> Write for I2c<I2C>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // C+P from H7 HAL.
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to send `bytes`. The
        // START bit can be set even if the bus is BUSY or
        // I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            unsafe {
                w.start()
                    .set_bit()
                    .sadd()
                    .bits(u16(addr << 1 | 0))
                    .add10()
                    .clear_bit()
                    .rd_wrn()
                    .clear_bit() // write
                    .nbytes()
                    .bits(bytes.len() as u8)
                    .autoend()
                    .clear_bit() // software end mode
            }
        });

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte when
            // through)
            busy_wait!(self.i2c, txis, bit_is_set); // TXDR register is empty

            // Put byte on the wire
            self.i2c.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        // Wait until the write finishes
        busy_wait!(self.i2c, tc, bit_is_set); // transfer is complete

        // Stop
        self.i2c.cr2.write(|w| w.stop().set_bit());

        Ok(())
    }
}

impl<I2C> Read for I2c<I2C>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to receive bytes into
        // `buffer`. The START bit can be set even if the bus
        // is BUSY or I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            unsafe {
                w.sadd()
                    .bits((addr << 1 | 0) as u16)
                    .rd_wrn()
                    .set_bit() // read
                    .nbytes()
                    .bits(buffer.len() as u8)
                    .start()
                    .set_bit()
                    .autoend()
                    .set_bit() // automatic end mode
            }
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne, bit_is_set);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        // automatic STOP

        Ok(())
    }
}

impl<I2C> WriteRead for I2c<I2C>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to send `bytes`. The
        // START bit can be set even if the bus is BUSY or
        // I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            unsafe {
                w.start()
                    .set_bit()
                    .sadd()
                    .bits(u16(addr << 1 | 0))
                    .add10()
                    .clear_bit()
                    .rd_wrn()
                    .clear_bit() // write
                    .nbytes()
                    .bits(bytes.len() as u8)
                    .autoend()
                    .clear_bit() // software end mode
            }
        });

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte went through)

            busy_wait!(self.i2c, txis, bit_is_set); // TXDR register is empty

            // Put byte on the wire
            self.i2c.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        // Wait until the write finishes before beginning to read.
        busy_wait!(self.i2c, tc, bit_is_set); // transfer is complete

        // reSTART and prepare to receive bytes into `buffer`
        self.i2c.cr2.write(|w| {
            unsafe {
                w.sadd()
                    .bits(u16(addr << 1 | 1))
                    .add10()
                    .clear_bit()
                    .rd_wrn()
                    .set_bit() // read mode
                    .nbytes()
                    .bits(buffer.len() as u8)
                    .start()
                    .set_bit()
                    .autoend()
                    .set_bit() // automatic end mode
            }
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne, bit_is_set);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        Ok(())
    }
}
