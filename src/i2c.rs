//! Inter-Integrated Circuit (I2C) bus. Also supports SMBUS. Implements traits from `embedded-hal`.

// Based on `stm32h7xx-hal`.

use cast::{u16, u8};
use core::ops::Deref;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(feature = "f3", feature = "l4", feature = "g0", feature = "g4"))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

use embedded_dma::{ReadBuffer, WriteBuffer};

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
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

// #[derive(Clone, Copy)]
// #[repr(u8)]
// /// Set master or slave mode.
// pub enum I2cMode {
//     Master = 0,
//     Slave = 1,
// }

#[derive(Clone, Copy)]
pub enum I2cDevice {
    One,
    Two,
    #[cfg(feature = "h7")]
    Three,
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C> {
    regs: I2C,
    device: I2cDevice,
    // mode: I2cMode,
}

impl<I> I2c<I>
where
    I: Deref<Target = pac::i2c1::RegisterBlock>,
{
    /// Configures the I2C peripheral. `freq` is in Hz. Doesn't check pin config.
    pub fn new<C: ClockCfg>(
        regs: I,
        device: I2cDevice,
        freq: u32,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self {
        match device {
            I2cDevice::One => {
                rcc_en_reset!(apb1, i2c1, rcc);
            }
            I2cDevice::Two => {
                rcc_en_reset!(apb1, i2c2, rcc);
            }
            #[cfg(feature = "h7")]
            I2cDevice::Three => {
                rcc_en_reset!(apb1, i2c3, rcc);
            }
        }

        assert!(freq <= 1_000_000);
        // Make sure the I2C unit is disabled so we can configure it
        regs.cr1.modify(|_, w| w.pe().clear_bit());

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
        // RM: Before enabling the peripheral, the I2C master clock must be configured by setting the
        // SCLH and SCLL bits in the I2C_TIMINGR register.
        regs.timingr.write(|w| unsafe {
            w.presc().bits(presc);
            w.scll().bits(scll);
            w.sclh().bits(sclh);
            w.sdadel().bits(sdadel);
            w.scldel().bits(scldel)
        });

        // Enable the peripheral
        regs.cr1.write(|w| w.pe().set_bit());

        I2c { regs, device }
    }

    /// Helper function to prevent repetition between `write`, and `write_read`.
    fn set_cr2_write(&mut self, addr: u8, bytes: &[u8]) {
        // L44 RM: "Master communication initialization (address phase)
        // In order to initiate the communication, the user must program the following parameters for
        // the addressed slave in the I2C_CR2 register:
        self.regs.cr2.write(|w| {
            unsafe {
                // Addressing mode (7-bit or 10-bit): ADD10
                w.add10().clear_bit();
                // Slave address to be sent: SADD[9:0]
                w.sadd().bits(u16(addr << 1 | 0));
                // Transfer direction: RD_WRN
                w.rd_wrn().clear_bit(); // write
                                        // The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
                                        // or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
                w.nbytes().bits(bytes.len() as u8);
                w.autoend().clear_bit(); // software end mode
                                         // The user must then set the START bit in I2C_CR2 register. Changing all the above bits is
                                         // not allowed when START bit is set.
                w.start().set_bit()
            }
        });
    }

    /// Read data, using DMA. See L44 RM, 37.4.16: "Transmissino using DMA"
    /// Note that the `channel` argument is only used on F3 and L4.
    pub fn write_dma<D, B>(&mut self, buf: &B, channel: DmaChannel, dma: &mut Dma<D>)
    where
        B: ReadBuffer,
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = unsafe { buf.read_buffer() };

        // todo: DMA2 support.
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            I2cDevice::One => DmaChannel::C6,
            I2cDevice::Two => DmaChannel::C4,
            #[cfg(feature = "h7")]
            I2cDevice::Three => DmaChannel::C2,
        };

        #[cfg(feature = "l4")]
        match self.device {
            I2cDevice::One => dma.channel_select(DmaChannel::C6, 0b011),
            I2cDevice::Two => dma.channel_select(DmaChannel::C4, 0b011),
            #[cfg(feature = "h7")]
            I2cDevice::Three => dma.channel_select(DmaChannel::C2, 0b011),
        }

        // DMA (Direct Memory Access) can be enabled for transmission by setting the TXDMAEN bit
        // in the I2C_CR1 register. Data is loaded from an SRAM area configured using the DMA
        // peripheral (see Section 11: Direct memory access controller (DMA) on page 295) to the
        // I2C_TXDR register whenever the TXIS bit is set.
        self.regs.cr1.modify(|_, w| w.txdmaen().set_bit());

        // Only the data are transferred with DMA.
        // • In master mode: the initialization, the slave address, direction, number of bytes and
        // START bit are programmed by software (the transmitted slave address cannot be
        // transferred with DMA). When all data are transferred using DMA, the DMA must be
        // initialized before setting the START bit. The end of transfer is managed with the
        // NBYTES counter. Refer to Master transmitter on page 1151.
        // • In slave mode:
        // – With NOSTRETCH=0, when all data are transferred using DMA, the DMA must be
        // initialized before the address match event, or in ADDR interrupt subroutine, before
        // clearing ADDR.
        // – With NOSTRETCH=1, the DMA must be initialized before the address match
        // event.
        // • For instances supporting SMBus: the PEC transfer is managed with NBYTES counter.
        // Refer to SMBus Slave transmitter on page 1165 and SMBus Master transmitter on
        // page 1169.
        // Note: If DMA is used for transmission, the TXIE bit does not need to be enabled

        dma.cfg_channel(
            channel,
            &self.regs.txdr as *const _ as u32,
            ptr as u32,
            len as u16,
            dma::Direction::ReadFromMem,
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );
    }

    /// Read data, using DMA. See L44 RM, 37.4.16: "Reception using DMA"
    /// Note that the `channel` argument is only used on F3 and L4.
    pub fn read_dma<D, B>(&mut self, buf: &mut B, channel: DmaChannel, dma: &mut Dma<D>)
    where
        B: WriteBuffer,
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = unsafe { buf.write_buffer() };

        // todo: DMA2 support.
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            I2cDevice::One => DmaChannel::C7,
            I2cDevice::Two => DmaChannel::C5,
            #[cfg(feature = "h7")]
            I2cDevice::Three => DmaChannel::C3,
        };

        #[cfg(feature = "l4")]
        match self.device {
            I2cDevice::One => dma.channel_select(DmaChannel::C7, 0b011),
            I2cDevice::Two => dma.channel_select(DmaChannel::C5, 0b011),
            #[cfg(feature = "h7")]
            I2cDevice::Three => dma.channel_select(DmaChannel::C3, 0b011),
        }

        // DMA (Direct Memory Access) can be enabled for reception by setting the RXDMAEN bit in
        // the I2C_CR1 register. Data is loaded from the I2C_RXDR register to an SRAM area
        // configured using the DMA peripheral (refer to Section 11: Direct memory access controller
        // (DMA) on page 295) whenever the RXNE bit is set. Only the data (including PEC) are
        // transferred with DMA.
        self.regs.cr1.modify(|_, w| w.rxdmaen().set_bit());

        // • In master mode, the initialization, the slave address, direction, number of bytes and
        // START bit are programmed by software. When all data are transferred using DMA, the
        // DMA must be initialized before setting the START bit. The end of transfer is managed
        // with the NBYTES counter.
        // • In slave mode with NOSTRETCH=0, when all data are transferred using DMA, the
        // DMA must be initialized before the address match event, or in the ADDR interrupt
        // subroutine, before clearing the ADDR flag.
        // • If SMBus is supported (see Section 37.3: I2C implementation): the PEC transfer is
        // managed with the NBYTES counter. Refer to SMBus Slave receiver on page 1167 and
        // SMBus Master receiver on page 1171.
        // Note: If DMA is used for reception, the RXIE bit does not need to be enabled

        dma.cfg_channel(
            channel,
            &self.regs.rxdr as *const _ as u32,
            ptr as u32,
            len as u16,
            dma::Direction::ReadFromPeriph,
            dma::DataSize::S8,
            dma::DataSize::S8,
            Default::default(),
        );
    }
}

macro_rules! busy_wait {
    ($regs:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $regs.isr.read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().bit_is_set() {
                $regs.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                $regs.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $regs.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());

                // If a pending TXIS flag is set, write dummy data to TXDR
                if $regs.isr.read().txis().bit_is_set() {
                    $regs.txdr.write(|w| unsafe { w.txdata().bits(0) });
                }

                // If TXDR is not flagged as empty, write 1 to flush it
                if $regs.isr.read().txe().bit_is_clear() {
                    $regs.isr.write(|w| w.txe().set_bit());
                }

                return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

impl<I2C> Write for I2c<I2C>
where
    I2C: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // C+P from H7 HAL.
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        self.set_cr2_write(addr, bytes);

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte when
            // through)
            busy_wait!(self.regs, txis, bit_is_set); // TXDR register is empty

            // Put byte on the wire
            self.regs.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        // Wait until the write finishes
        busy_wait!(self.regs, tc, bit_is_set); // transfer is complete

        // Stop
        self.regs.cr2.write(|w| w.stop().set_bit());

        Ok(())
    }
}

impl<I2C> Read for I2c<I2C>
where
    I2C: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        // Set START and prepare to receive bytes into
        // `buffer`. The START bit can be set even if the bus
        // is BUSY or I2C is in slave mode.
        self.regs.cr2.write(|w| {
            unsafe {
                w.sadd().bits((addr << 1 | 0) as u16);
                w.add10().clear_bit();
                w.rd_wrn().set_bit(); // read
                w.nbytes().bits(buffer.len() as u8);
                w.start().set_bit();
                w.autoend().set_bit() // automatic end mode
            }
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.regs, rxne, bit_is_set);

            *byte = self.regs.rxdr.read().rxdata().bits();
        }

        // automatic STOP

        Ok(())
    }
}

impl<I2C> WriteRead for I2c<I2C>
where
    I2C: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        self.set_cr2_write(addr, bytes);

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte went through)

            busy_wait!(self.regs, txis, bit_is_set); // TXDR register is empty

            // Put byte on the wire
            self.regs.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        // Wait until the write finishes before beginning to read.
        busy_wait!(self.regs, tc, bit_is_set); // transfer is complete

        // reSTART and prepare to receive bytes into `buffer`

        // The addressing is also different, from in `read`, which may be important.
        self.regs.cr2.write(|w| {
            unsafe {
                w.sadd().bits(u16(addr << 1 | 1));
                w.add10().clear_bit();
                w.rd_wrn().set_bit(); // read mode
                w.nbytes().bits(buffer.len() as u8);
                w.autoend().set_bit(); // automatic end mode
                w.start().set_bit()
            }
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.regs, rxne, bit_is_set);

            *byte = self.regs.rxdr.read().rxdata().bits();
        }

        Ok(())
    }
}
