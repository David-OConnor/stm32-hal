//! Support for the Inter-Integrated Circuit (I2C) bus peripheral. Also supports SMBUS.
//! Provides APIs to configure, read, and write from
//! I2C, with blocking, nonblocking, and DMA functionality.

use cast::{u16, u8};
use core::ops::Deref;

use cortex_m::interrupt::free;

#[cfg(feature = "embedded-hal")]
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::{
    clocks::Clocks,
    pac::{self, RCC},
    rcc_en_reset,
};

#[cfg(any(feature = "g0"))]
use crate::pac::dma as dma_p;
#[cfg(any(
    feature = "f3",
    feature = "l4",
    feature = "g4",
    feature = "h7",
    feature = "wb",
    feature = "wl"
))]
use crate::pac::dma1 as dma_p;

#[cfg(not(feature = "l5"))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

// todo: Get rid of this macro.
macro_rules! busy_wait {
    ($regs:expr, $flag:ident) => {
        loop {
            let isr = $regs.isr.read();

            if isr.$flag().bit_is_set() {
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
    #[cfg(not(any(feature = "wb", feature = "f3x4")))]
    Two,
    #[cfg(any(feature = "h7", feature = "wb"))]
    Three,
    // Four ommitted for now; just need to add some non-standard
    // periph clock setup/enable
    // #[cfg(any(feature = "l5", feature = "h7"))]
    // Four,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Set master or slave mode. Sets the __ register, _ field.
pub enum I2cMode {
    /// In Master mode, the I2C interface initiates a data transfer and generates the clock signal. A
    /// serial data transfer always begins with a START condition and ends with a STOP condition.
    /// Both START and STOP conditions are generated in master mode by software.
    Master = 0,
    /// In Slave mode, the interface is capable of recognizing its own addresses (7 or 10-bit), and
    /// the general call address. The general call address detection can be enabled or disabled by
    /// software. The reserved SMBus addresses can also be enabled by software.
    Slave = 1,
}

#[derive(Clone, Copy)]
/// Set a preset I2C speed, based on RM tables: Examples of timings settings.
/// Sets 5 fields of the TIMINGR register.
pub enum I2cSpeed {
    /// Standard-mode: 10kHz.
    Standard10K,
    /// Standard-mode: 100kHz.
    Standard100K,
    /// Fast-mode: 400kHz.
    Fast400K,
    /// Fast-mode +: 1Mhz.
    FastPlus1M,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Set the number of address bits to 7 or 10. Sets the CR2 register, ADD10 field.
pub enum AddressBits {
    B7 = 0,
    B10 = 1,
}

#[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
/// Set the number of address bits to 7 or 10. Sets the CR1 register, ANFOFF and DNF fields.
pub enum NoiseFilter {
    /// Analog noise filter enabled.
    Analog,
    /// Digital filter enabled and filtering capability ( filters spikes with a length of)
    /// up to (value) t_I2CCLK
    Digital(u8),
    /// Analog and digital filters disabled.
    Disabled,
}

/// Initial configuration data for the I2C peripheral.
pub struct I2cConfig {
    /// Select master or slave mode. Defaults to Master.
    pub mode: I2cMode,
    // /// Select between standard, fast, and fast-plus speeds. Default to standard.
    /// pub speed_mode: SpeedMode,
    /// Select between one of 4 preset speeds. If you'd like to use custom
    /// speed settings, use the PAC directly, with I2C disabled, after the
    /// peripheral clocks are enabled by `new()`. Default to Standard mode, 100kHz.
    pub speed: I2cSpeed,
    /// Allows setting 7 or 10-bit addresses. Defaults to 7.
    pub address_bits: AddressBits,
    /// Select the analog noise filter, a digital filter, or no filter.
    pub noise_filter: NoiseFilter,
    /// Support for SMBUS, including hardware PEC, and alert pin. Defaults to false.
    pub smbus: bool,
    /// Optionally disable clock stretching. Defaults to false (stretching allowed)
    /// Only relevant in slave mode.
    pub nostretch: bool,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            mode: I2cMode::Master,
            speed: I2cSpeed::Standard100K,
            address_bits: AddressBits::B7,
            noise_filter: NoiseFilter::Analog,
            smbus: false,
            nostretch: false,
        }
    }
}

/// Represents an Inter-Integrated Circuit (I2C) peripheral.
pub struct I2c<R> {
    pub regs: R,
    device: I2cDevice,
    pub cfg: I2cConfig,
}

impl<R> I2c<R>
where
    R: Deref<Target = pac::i2c1::RegisterBlock>,
{
    /// Initialize a I2C peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock. `freq` is in Hz.
    pub fn new(regs: R, device: I2cDevice, cfg: I2cConfig, clocks: &Clocks) -> Self {
        free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };

            match device {
                I2cDevice::One => {
                    rcc_en_reset!(apb1, i2c1, rcc);
                }
                #[cfg(not(any(feature = "wb", feature = "f3x4")))]
                I2cDevice::Two => {
                    rcc_en_reset!(apb1, i2c2, rcc);
                }
                #[cfg(any(feature = "h7", feature = "wb"))]
                I2cDevice::Three => {
                    rcc_en_reset!(apb1, i2c3, rcc);
                } // todo: APB1LR2 on L5, and AHB4 on H7. Fix it.
                  // #[cfg(any(feature = "l5", feature = "h7"))]
                  // I2cDevice::Four => {
                  //     rcc_en_reset!(apb1, i2c4, rcc);
                  // }
            }
        });

        // Make sure the I2C unit is disabled so we can configure it
        regs.cr1.modify(|_, w| w.pe().clear_bit());

        // todo: Slave currently nonfunctional!
        // todo: Check out the RM recipes for slave transmitter and receiver.

        // RM: I2C timings:
        // The timings must be configured in order to guarantee a correct data hold and setup time,
        // used in master and slave modes. This is done by programming the PRESC[3:0],
        // SCLDEL[3:0] and SDADEL[3:0] bits in the I2C_TIMINGR register.
        // ... Additionally, in master mode, the SCL clock high and low levels must be configured by
        // programming the PRESC[3:0], SCLH[7:0] and SCLL[7:0] bits in the I2C_TIMINGR register

        // For these speed and frequency variables, we use the RM's conventions.
        let t_i2cclk = clocks.apb1();

        // assert!(t_i2cclk < (t_low - f_f) / 4);
        // assert!(t_i2cclk < t_high);

        // Set the prescaler using RM tables as a guide;
        // L552 RM, Tables 324 - 326: Examples of timings settings.
        // Note that the table only includes I2C clock multiples of 4Mhz (well, multiples of 8Mhz).
        // In this case, we'll use the integer floor rounding to handle in-between
        // values.

        // We use this constant in several calculations.
        let presc_const = match cfg.speed {
            I2cSpeed::Standard10K => 4_000_000,
            I2cSpeed::Standard100K => 4_000_000,
            I2cSpeed::Fast400K => 8_000_000,
            // Note: The 16Mhz example uses F+ / 16. The other 2 examples
            // use 8e6.
            I2cSpeed::FastPlus1M => 8_000_000,
        };

        // This is (offset by 1) which we set as the prescaler.
        let presc_val = t_i2cclk / presc_const;

        // Hit the target freq by setting up t_scll (Period of SCL low)
        // to be half the whole period. These constants
        // are from the tables.
        let freq = match cfg.speed {
            I2cSpeed::Standard10K => 10_000,
            I2cSpeed::Standard100K => 100_000,
            I2cSpeed::Fast400K => 400_000,
            I2cSpeed::FastPlus1M => 1_000_000,
        };

        // Set SCLL (SCL low time) to be half the duty period
        // associated with the target frequency.
        let scll_val = presc_const / (2 * freq);

        // SCLH is smaller than SCLH. For standard mode it's close, although
        // in the example tables, 20% different for 100Khz, and 2% different for
        // 10K. THis may be due to delays
        // involved. The ratio is different for Fast-mode and Fast-mode+.
        // todo: Come back to this. How should we set this?
        let sclh_val = match cfg.speed {
            I2cSpeed::Standard10K => scll_val - 4,
            I2cSpeed::Standard100K => scll_val - 4,
            I2cSpeed::Fast400K => scll_val * 4 / 10,
            I2cSpeed::FastPlus1M => scll_val / 2,
        };

        // Timing prescaler. This field is used to prescale I2CCLK in order to generate the clock period tPRESC used for
        // data setup and hold counters (refer to I2C timings on page 1495) and for SCL high and low
        // level counters (refer to I2C master initialization on page 1510).
        // Sets TIMINGR reg, PRESC field.

        let presc = presc_val - 1;

        // SCL low period (master mode)
        // This field is used to generate the SCL low period in master mode.
        // tSCLL = (SCLL+1) x tPRESC
        // Note: SCLL is also used to generate tBUF and tSU:STA timings.
        // Sets TIMINGR reg, SCLL field.
        let scll = scll_val - 1;

        // SCL high period (master mode)
        // This field is used to generate the SCL high period in master mode.
        // tSCLH = (SCLH+1) x tPRESC
        // Note: SCLH is also used to generate tSU:STO and tHD:STA timing
        // Set the clock prescaler value. Sets TIMINGR reg, SCLH field.
        let sclh = sclh_val - 1;

        // todo: Can't find the sdadel and scldel pattern
        // Data hold time
        // This field is used to generate the delay tSDADEL between SCL falling edge and SDA edge. In
        // master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during
        // tSDADEL.
        // tSDADEL= SDADEL x tPRESC
        // Note: SDADEL is used to generate tHD:DAT timing
        // Sets TIMINGR reg, SDADEL field.
        let sdadel = match cfg.speed {
            I2cSpeed::Standard10K => 0x2,
            I2cSpeed::Standard100K => 0x2,
            I2cSpeed::Fast400K => 0x3,
            I2cSpeed::FastPlus1M => 0x0,
        };

        // Data setup time
        // This field is used to generate a delay tSCLDEL between SDA edge and SCL rising edge. In
        // master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during
        // tSCLDEL.
        // tSCLDEL = (SCLDEL+1) x tPRESC
        // Note: tSCLDEL is used to generate tSU:DAT timing
        // Sets TIMINGR reg, SCLDEL field.
        let scldel = match cfg.speed {
            I2cSpeed::Standard10K => 0x4,
            I2cSpeed::Standard100K => 0x4,
            I2cSpeed::Fast400K => 0x3,
            I2cSpeed::FastPlus1M => 0x1,
        };

        // The fields for PRESC, SCLDEL, and SDADEL are 4-bits; don't overflow.
        // The other TIMINGR fields we set are 8-bits, so won't overflow with u8.
        assert!(presc <= 15);
        assert!(scldel <= 15);
        assert!(sdadel <= 15);

        assert!(scll <= 255);
        assert!(sclh <= 255);

        regs.timingr.write(|w| unsafe {
            w.presc().bits(presc as u8);
            w.scldel().bits(scldel as u8);
            w.sdadel().bits(sdadel as u8);
            w.sclh().bits(sclh as u8);
            w.scll().bits(scll as u8)
        });

        // Before enabling the I2C peripheral by setting the PE bit in I2C_CR1 register, the user must
        // configure the noise filters, if needed. By default, an analog noise filter is present on the SDA
        // and SCL inputs. This analog filter is compliant with the I2C specification which requires the
        // suppression of spikes with a pulse width up to 50 ns in Fast-mode and Fast-mode Plus. The
        // user can disable this analog filter by setting the ANFOFF bit, and/or select a digital filter by
        // configuring the DNF[3:0] bit in the I2C_CR1 register.
        // When the digital filter is enabled, the level of the SCL or the SDA line is internally changed
        // only if it remains stable for more than DNF x I2CCLK periods. This allows spikes with a
        // programmable length of 1 to 15 I2CCLK periods to be suppressed.
        let (anf_bit, dnf_bits) = match cfg.noise_filter {
            NoiseFilter::Analog => (false, 0),
            NoiseFilter::Digital(filtering_len) => {
                assert!(filtering_len <= 0b1111);
                (true, filtering_len)
            }
            NoiseFilter::Disabled => (true, 0),
        };

        regs.cr1.modify(|_, w| unsafe {
            w.anfoff().bit(anf_bit);
            w.dnf().bits(dnf_bits)
        });

        if let I2cMode::Slave = cfg.mode {
            regs.cr1.modify(|_, w| w.nostretch().bit(cfg.nostretch));
        }

        let mut result = Self { regs, device, cfg };

        if result.cfg.smbus {
            result.enable_smbus();
        }

        // Enable the peripheral
        result.regs.cr1.write(|w| w.pe().set_bit());

        result
    }

    /// Enable SMBus support. See L44 RM, section 37.4.11: SMBus initialization
    pub fn enable_smbus(&mut self) {
        // todo: Roll this into an init setting or I2cConfig struct etc.
        // PEC calculation is enabled by setting the PECEN bit in the I2C_CR1 register. Then the PEC
        // transfer is managed with the help of a hardware byte counter: NBYTES[7:0] in the I2C_CR2
        // register. The PECEN bit must be configured before enabling the I2C.

        // The PEC transfer is managed with the hardware byte counter, so the SBC bit must be set
        // when interfacing the SMBus in slave mode. The PEC is transferred after NBYTES-1 data
        // have been transferred when the PECBYTE bit is set and the RELOAD bit is cleared. If
        // RELOAD is set, PECBYTE has no effect.
        // Caution: Changing the PECEN configuration is not allowed when the I2C is enabled.

        let originally_enabled = self.regs.cr1.read().pe().bit_is_set();
        if originally_enabled {
            self.regs.cr1.modify(|_, w| w.pe().clear_bit());
            while self.regs.cr1.read().pe().bit_is_set() {}
        }

        self.regs.cr1.modify(|_, w| w.pecen().set_bit());

        // todo: Timeout detection?

        // todo: HWCFGR Missing from PAC
        // self.regs.hwcfgr.modify(|_, w| w.smbus().set_bit());

        if originally_enabled {
            self.regs.cr1.modify(|_, w| w.pe().set_bit());
        }
    }

    /// Read multiple words to a buffer. Can return an error state rela
    pub fn read(&mut self, addr: u8, bytes: &mut [u8]) -> Result<(), Error> {
        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        // Set START and prepare to receive bytes into
        // `buffer`. The START bit can be set even if the bus
        // is BUSY or I2C is in slave mode.
        self.set_cr2_read(addr, bytes.len() as u8);

        for byte in bytes {
            // Wait until we have received something
            busy_wait!(self.regs, rxne);

            *byte = self.regs.rxdr.read().rxdata().bits();
        }

        Ok(())
    }

    /// Write an array of words. Can return an error due to Bus, Arbitration, or NACK.
    pub fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        self.set_cr2_write(addr, bytes.len() as u8, true);

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte when
            // through)
            busy_wait!(self.regs, txis); // TXDR register is empty

            // Put byte on the wire
            self.regs.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        Ok(())
    }

    /// Write and read an array of words. Can return an error due to Bus, Arbitration, or NACK.
    pub fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.regs.cr2.read().start().bit_is_set() {}

        self.set_cr2_write(addr, bytes.len() as u8, false);

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte went through)

            busy_wait!(self.regs, txis); // TXDR register is empty

            // Put byte on the wire
            self.regs.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
        }

        // Wait until the write finishes before beginning to read.
        busy_wait!(self.regs, tc); // transfer is complete

        // reSTART and prepare to receive bytes into `buffer`

        self.set_cr2_read(addr, buffer.len() as u8);

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.regs, rxne);

            *byte = self.regs.rxdr.read().rxdata().bits();
        }

        Ok(())
    }

    /// Helper function to prevent repetition between `write`, `write_read`, and `write_dma`.
    fn set_cr2_write(&mut self, addr: u8, len: u8, autoend: bool) {
        // L44 RM: "Master communication initialization (address phase)
        // In order to initiate the communication, the user must program the following parameters for
        // the addressed slave in the I2C_CR2 register:
        self.regs.cr2.write(|w| {
            unsafe {
                // Addressing mode (7-bit or 10-bit): ADD10
                w.add10().bit(self.cfg.address_bits as u8 != 0);
                // Slave address to be sent: SADD[9:0]
                // SADD0: "This bit is don’t care"
                // SADD[7:1]: "These bits should be written with the 7-bit slave address to be sent"
                w.sadd().bits(u16(addr << 1));
                // Transfer direction: RD_WRN
                w.rd_wrn().clear_bit(); // write
                                        // The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
                                        // or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
                w.nbytes().bits(len);
                w.autoend().bit(autoend); // software end mode
                                          // The user must then set the START bit in I2C_CR2 register. Changing all the above bits is
                                          // not allowed when START bit is set.
                                          // When the SMBus master wants to transmit the PEC, the PECBYTE bit must be set and the
                                          // number of bytes must be programmed in the NBYTES[7:0] field, before setting the START
                                          // bit. In this case the total number of TXIS interrupts is NBYTES-1. So if the PECBYTE bit is
                                          // set when NBYTES=0x1, the content of the I2C_PECR register is automatically transmitted.
                                          // If the SMBus master wants to send a STOP condition after the PEC, automatic end mode
                                          // must be selected (AUTOEND=1). In this case, the STOP condition automatically follows the
                                          // PEC transmission.
                w.pecbyte().bit(self.cfg.smbus);
                w.start().set_bit()
            }
        });
        // Note on start bit (RM):
        // If the I2C is already in master mode with AUTOEND = 0, setting this bit generates a
        // Repeated Start condition when RELOAD=0, after the end of the NBYTES transfer.
        // Otherwise setting this bit generates a START condition once the bus is free.
        // (This is why we don't set autoend on the write portion of a write_read.)
    }

    /// Helper function to prevent repetition between `read`, `write_read`, and `read_dma`.
    fn set_cr2_read(&mut self, addr: u8, len: u8) {
        self.regs.cr2.write(|w| {
            unsafe {
                w.add10().bit(self.cfg.address_bits as u8 != 0);
                w.sadd().bits(u16(addr << 1));
                w.rd_wrn().set_bit(); // read
                w.nbytes().bits(len);
                w.autoend().set_bit(); // automatic end mode
                                       // When the SMBus master wants to receive the PEC followed by a STOP at the end of the
                                       // transfer, automatic end mode can be selected (AUTOEND=1). The PECBYTE bit must be
                                       // set and the slave address must be programmed, before setting the START bit. In this case,
                                       // after NBYTES-1 data have been received, the next received byte is automatically checked
                                       // versus the I2C_PECR register content. A NACK response is given to the PEC byte, followed
                                       // by a STOP condition.
                w.pecbyte().bit(self.cfg.smbus);
                w.start().set_bit()
            }
        });
    }

    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    /// Read data, using DMA. See L44 RM, 37.4.16: "Transmission using DMA"
    /// Note that the `channel` argument is only used on F3 and L4.
    /// For a single write, set `autoend` to `true`. For a write_read and other use cases,
    /// set it to `false`.
    pub unsafe fn write_dma<D>(
        &mut self,
        addr: u8,
        buf: &[u8],
        autoend: bool,
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        while self.regs.cr2.read().start().bit_is_set() {}

        let (ptr, len) = (buf.as_ptr(), buf.len());

        // todo: DMA2 support.
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            I2cDevice::One => DmaInput::I2c1Tx.dma1_channel(),
            #[cfg(not(any(feature = "wb", feature = "f3x4")))]
            I2cDevice::Two => DmaInput::I2c2Tx.dma1_channel(),
            #[cfg(any(feature = "h7", feature = "wb"))]
            I2cDevice::Three => DmaInput::I2c3Tx.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match self.device {
            I2cDevice::One => dma.channel_select(DmaInput::I2c1Tx),
            I2cDevice::Two => dma.channel_select(DmaInput::I2c2Tx),
        }

        // DMA (Direct Memory Access) can be enabled for transmission by setting the TXDMAEN bit
        // in the I2C_CR1 register. Data is loaded from an SRAM area configured using the DMA
        // peripheral (see Section 11: Direct memory access controller (DMA) on page 295) to the
        // I2C_TXDR register whenever the TXIS bit is set.
        self.regs.cr1.modify(|_, w| w.txdmaen().set_bit());
        while self.regs.cr1.read().txdmaen().bit_is_clear() {}

        // Only the data are transferred with DMA.
        // • In master mode: the initialization, the slave address, direction, number of bytes and
        // START bit are programmed by software (the transmitted slave address cannot be
        // transferred with DMA). When all data are transferred using DMA, the DMA must be
        // initialized before setting the START bit. The end of transfer is managed with the
        // NBYTES counter. Refer to Master transmitter on page 1151.
        // (The steps above are handled in the write this function performs.)
        self.set_cr2_write(addr, len as u8, autoend);

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

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        dma.cfg_channel(
            channel,
            &self.regs.txdr as *const _ as u32,
            ptr as u32,
            len,
            dma::Direction::ReadFromMem,
            dma::DataSize::S8,
            dma::DataSize::S8,
            channel_cfg,
        );
    }

    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    /// Read data, using DMA. See L44 RM, 37.4.16: "Reception using DMA"
    /// Note that the `channel` argument is only used on F3 and L4.
    pub unsafe fn read_dma<D>(
        &mut self,
        addr: u8,
        buf: &mut [u8],
        channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        // while self.regs.cr2.read().start().bit_is_set() {}
        // todo: Think about how you want to do write reads. Ie there's no stopping there.

        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        // todo: DMA2 support.
        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match self.device {
            I2cDevice::One => DmaInput::I2c1Rx.dma1_channel(),
            #[cfg(not(any(feature = "wb", feature = "f3x4")))]
            I2cDevice::Two => DmaInput::I2c2Rx.dma1_channel(),
            #[cfg(any(feature = "h7", feature = "wb"))]
            I2cDevice::Three => DmaInput::I231Rx.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match self.device {
            I2cDevice::One => dma.channel_select(DmaInput::I2c1Rx),
            I2cDevice::Two => dma.channel_select(DmaInput::I2c2Rx),
        }

        // DMA (Direct Memory Access) can be enabled for reception by setting the RXDMAEN bit in
        // the I2C_CR1 register. Data is loaded from the I2C_RXDR register to an SRAM area
        // configured using the DMA peripheral (refer to Section 11: Direct memory access controller
        // (DMA) on page 295) whenever the RXNE bit is set. Only the data (including PEC) are
        // transferred with DMA.
        self.regs.cr1.modify(|_, w| w.rxdmaen().set_bit());
        while self.regs.cr1.read().rxdmaen().bit_is_clear() {}

        // • In master mode, the initialization, the slave address, direction, number of bytes and
        // START bit are programmed by software. When all data are transferred using DMA, the
        // DMA must be initialized before setting the START bit. The end of transfer is managed
        // with the NBYTES counter.
        self.set_cr2_read(addr, len as u8);

        // • In slave mode with NOSTRETCH=0, when all data are transferred using DMA, the
        // DMA must be initialized before the address match event, or in the ADDR interrupt
        // subroutine, before clearing the ADDR flag.
        // • If SMBus is supported (see Section 37.3: I2C implementation): the PEC transfer is
        // managed with the NBYTES counter. Refer to SMBus Slave receiver on page 1167 and
        // SMBus Master receiver on page 1171.
        // Note: If DMA is used for reception, the RXIE bit does not need to be enabled

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        dma.cfg_channel(
            channel,
            &self.regs.rxdr as *const _ as u32,
            ptr as u32,
            len,
            dma::Direction::ReadFromPeriph,
            dma::DataSize::S8,
            dma::DataSize::S8,
            channel_cfg,
        );
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> Write for I2c<R>
where
    R: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        I2c::write(self, addr, bytes)
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> Read for I2c<R>
where
    R: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, bytes: &mut [u8]) -> Result<(), Error> {
        I2c::read(self, addr, bytes)
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> WriteRead for I2c<R>
where
    R: Deref<Target = pac::i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        I2c::write_read(self, addr, bytes, buffer)
    }
}
