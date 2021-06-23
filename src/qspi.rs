//! Quad Serial Peripheral Interface (SPI) bus: A specialized interface used for
//! high-speed communications with external flash memory.


// todo: WIP



use crate::{
    pac::{QUADSPI, RCC},
    traits::ClockCfg,
};

use core::ptr;

use cfg_if::cfg_if;

#[derive(Copy, Clone)]
#[repr(u8)]
/// Sets the Qspi mode to single, dual, or quad. Affects the IMODE/ADMODE/ABMODE/DMODE fields of
/// the CCR reg.
pub enum ProtocolMode {
    /// Only a single IO line (IO0) is used for transmit and a separate line (IO1) is used for receive.
    Single = 0b01,
    /// Two IO lines (IO0 and IO1) are used for transmit/receive.
    Dual = 0b10,
    /// All four IO lines are used for transmit/receive.
    Quad = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Sets the Qspi data mode. Affects the DDRM field of the CCR reg.
pub enum DataMode {
    /// In SDR mode, when the QUADSPI is driving the IO0/SO, IO1, IO2, IO3 signals, these
    /// signals transition only with the falling edge of CLK.
    Sdr = 0,
    /// In DDR mode, when the QUADSPI is driving the IO0/SO, IO1, IO2, IO3 signals in the
    /// address/alternate-byte/data phases, a bit is sent on each of the falling and rising edges of
    /// CLK.
    Ddr = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Sets the Qspi mode to single, dual, or quad. Affects the FMODE field of the CCR reg.
pub enum QspiMode {
    Indirect,
    StatusPolling,
    MemoryMapped,
}

/// Address sizes used by the QSPI interface
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AddressSize {
    EightBit,
    SixteenBit,
    TwentyFourBit,
    ThirtyTwoBit,
}

/// Sampling mode for the QSPI interface
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SamplingEdge {
    Falling,
    Rising,
}

/// Indicates an error with the QSPI peripheral.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum QspiError {
    Busy,
    Underflow,
}

/// Indicates a specific QSPI bank to use.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Bank {
    One,
    Two,
    Dual,
}

/// A structure for specifying the QSPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// ```
/// let config = Config::new().dummy_cycles(1);
/// ```
#[derive(Copy, Clone)]
pub struct QspiConfig {
    mode: QspiMode,
    protocol_mode: ProtocolMode,
    data_mode: DataMode,
    frequency: u32,
    address_size: AddressSize,
    dummy_cycles: u8,
    sampling_edge: SamplingEdge,
    fifo_threshold: u8,
}

impl Default for QspiConfig {
    fn default() -> Self {
        Self { // todo: QC what you want here.
            mode: QspiMode::Indirect,
            protocol_mode: ProtocolMode::Quad,
            data_mode: DataMode::Sdr,
            frequency: freq.into(),
            address_size: AddressSize::EightBit,
            dummy_cycles: 0,
            sampling_edge: SamplingEdge::Falling,
            fifo_threshold: 1,
        }
    }
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum QspiInterrupt {
    /// FIFO Threashold
    FIFOThreashold,
    /// Transfer complete
    Complete,
    /// Tranfer error
    Error,
}

/// Represents a Quad Serial Peripheral Interface (QSPI) peripheral.
pub struct Qspi {
    regs: QUADSPI,
    cfg: QspiConfig,
}

impl Qspi {
    pub fn new(regs: QUADSPI, cfg: QspiConfig) -> Self {
        // todo: QC this init code.
        // cfg_if! {
        //     if #[cfg(any(feature = "l4", feature = "l5", feature = "")] {
        //         rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
        //         rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
        //         rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());
        //     } else { // G and H7
        //         rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
        //         rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
        //         rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());
        //     }
        // }
        rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        while regs.sr.read().busy().bit_is_set() {}

        let config: QspiConfig = config.into();

        // Configure the FSIZE to maximum. It appears that even when addressing is not used, the
        // flash size violation may still trigger.
        regs.dcr.write(|w| unsafe { w.fsize().bits(0x1F) });

        // Clear all pending flags.
        regs.fcr.write(|w| {
            w.ctof()
                .set_bit()
                .csmf()
                .set_bit()
                .ctcf()
                .set_bit()
                .ctef()
                .set_bit()
        });

        // Configure the communication method for QSPI.
        regs.ccr.write(|w| unsafe {
            w.fmode()
                .bits(0) // indirect mode
                .dmode()
                .bits(config.mode.reg_value())
                .admode()
                .bits(config.mode.reg_value())
                .adsize()
                .bits(config.address_size as u8)
                .imode()
                .bits(0) // No instruction phase
                .dcyc()
                .bits(config.dummy_cycles)
        });

        let spi_frequency = config.frequency;
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency {
            divisor @ 1..=256 => divisor - 1,
            _ => panic!("Invalid QSPI frequency requested"),
        };

        // Write the prescaler and the SSHIFT bit.
        //
        // Note that we default to setting SSHIFT (sampling on the falling
        // edge). This is because it appears that the QSPI may have signal
        // contention issues when reading with zero dummy cycles. Setting SSHIFT
        // forces the read to occur on the falling edge instead of the rising
        // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
        // more information
        //
        // SSHIFT must not be set in DDR mode.
        regs.cr.write(|w| unsafe {
            w.prescaler()
                .bits(divisor as u8)
                .sshift()
                .bit(config.sampling_edge == SamplingEdge::Falling)
                .fthres()
                .bits(config.fifo_threshold - 1)
        });

        match bank {
            Bank::One => regs.cr.modify(|_, w| w.fsel().clear_bit()),
            Bank::Two => regs.cr.modify(|_, w| w.fsel().set_bit()),
            Bank::Dual => regs.cr.modify(|_, w| w.dfm().set_bit()),
        }

        // Enable ther peripheral
        regs.cr.modify(|_, w| w.en().set_bit());

        Self { regs, cfg }
    }


    /// Check if the QSPI peripheral is currently busy with a transaction
    pub fn is_busy(&self) -> bool {
        self.rb.sr.read().busy().bit_is_set()
    }

    /// Enable an interrupt
    pub fn enable_interrupt(&mut self, interrupt: QspiInterrupt) {
        self.rb.cr.modify(|_, w| match interrupt {
            QspiInterrupt::FIFOThreashold => w.ftie().set_bit(),
            QspiInterrupt::Complete => w.tcie().set_bit(),
            QspiInterrupt::Error => w.teie().set_bit(),
        });
    }

    /// Clear an interrupt flag
    pub fn clear_interrupt(&mut self, interrupt: QspiInterrupt) {
        self.rb.cr.modify(|_, w| match event {
            QspiInterrupt::FIFOThreashold => w.ftie().set_bit(),
            QspiInterrupt::Complete => w.tcie().set_bit(),
            QspiInterrupt::Error => w.teie().set_bit(),
        });
    }

    /// Disable interrupts for the given `event`
    pub fn unlisten(&mut self, event: Event) {
        self.rb.cr.modify(|_, w| match event {
            Event::FIFOThreashold => w.ftie().clear_bit(),
            Event::Complete => w.tcie().clear_bit(),
            Event::Error => w.teie().clear_bit(),
        });
        let _ = self.rb.cr.read();
        let _ = self.rb.cr.read(); // Delay 2 peripheral clocks
    }

    fn get_clock<C: ClockCfg>(clocks: &C) -> Option<u32> {
        // todo temp. Look this up.
        Some(clocks.apb1())

        // let d1ccipr = unsafe { (*pac::RCC::ptr()).d1ccipr.read() };

        // match d1ccipr.qspisel().variant() {
        //     pac::rcc::d1ccipr::QSPISEL_A::RCC_HCLK3 => Some(clocks.hclk()),
        //     pac::rcc::d1ccipr::QSPISEL_A::PLL1_Q => clocks.pll1_q_ck(),
        //     pac::rcc::d1ccipr::QSPISEL_A::PLL2_R => clocks.pll2_r_ck(),
        //     pac::rcc::d1ccipr::QSPISEL_A::PER => clocks.per_ck(),
        // }
    }

    /// Configure the operational mode of the QSPI interface.
    ///
    /// # Args
    /// * `mode` - The newly desired mode of the interface.
    ///
    /// # Errors
    /// Returns QspiError::Busy if an operation is ongoing
    pub fn configure_mode(&mut self, mode: QspiMode) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        self.rb.ccr.modify(|_, w| unsafe {
            w.admode()
                .bits(mode.reg_value())
                .dmode()
                .bits(mode.reg_value())
        });

        Ok(())
    }

    /// Begin a write over the QSPI interface. This is mostly useful for use with
    /// DMA or if you are managing the read yourself. If you want to complete a
    /// whole transaction, see the [`write`](#method.write) method.
    ///
    /// # Args
    /// * `addr` - The address to write data to. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `length` - The length of the write operation in bytes
    pub fn begin_write(&mut self, addr: u32, length: usize) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

        // Configure the mode to indirect write.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b00) });

        self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

        Ok(())
    }

    /// Write data over the QSPI interface.
    ///
    /// # Args
    /// * `addr` - The address to write data to. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `data` - An array of data to transfer over the QSPI interface.
    ///
    /// # Panics
    ///
    /// Panics if the length of `data` is greater than the size of the QSPI
    /// hardware FIFO (32 bytes).
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), QspiError> {
        assert!(
            data.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        self.begin_write(addr, data.len())?;

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}

        Ok(())
    }

    /// Begin a read over the QSPI interface. This is mostly useful for use with
    /// DMA or if you are managing the read yourself. If you want to complete a
    /// whole transaction, see the [`read`](#method.read) method.
    ///
    /// # Args
    /// * `addr` - The address to read data from. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `length` - The length of the read operation in bytes
    pub fn begin_read(&mut self, addr: u32, length: usize) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

        // Configure the mode to indirect read.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b01) });

        // Write the address to force the read to start.
        self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

        Ok(())
    }

    /// Read data over the QSPI interface.
    ///
    /// # Args
    /// * `addr` - The address to read data from. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `dest` - An array to store the result of the read into.
    ///
    /// # Panics
    ///
    /// Panics if the length of `data` is greater than the size of the QSPI
    /// hardware FIFO (32 bytes).
    pub fn read(&mut self, addr: u32, dest: &mut [u8]) -> Result<(), QspiError> {
        assert!(
            dest.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        // Begin the read operation
        self.begin_read(addr, dest.len())?;

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check for underflow on the FIFO.
        if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(&self.rb.dr as *const _ as *const u8);
            }
        }

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}

        Ok(())
    }
}

impl QspiExt for QUADSPI {
    fn bank1<CONFIG, C>(self, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Qspi
    where
        CONFIG: Into<QspiConfig>,
        C: ClockCfg,
    {
        Qspi::qspi_unchecked(self, config, Bank::One, clocks, rcc)
    }

    fn bank2<CONFIG, C>(self, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Qspi
    where
        CONFIG: Into<QspiConfig>,
        C: ClockCfg,
    {
        Qspi::qspi_unchecked(self, config, Bank::Two, clocks, rcc)
    }

    fn qspi_unchecked<CONFIG, C>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Qspi
    where
        CONFIG: Into<QspiConfig>,
        C: ClockCfg,
    {
        Qspi::qspi_unchecked(self, config, bank, clocks, rcc)
    }
}
