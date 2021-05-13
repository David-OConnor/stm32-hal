//! Quad Serial Peripheral Interface (SPI) bus.

// Based on `stm32f7xx-hal`.

//! Quad SPI (QSPI) bus
//!
//! The QSPI peripheral supports a SPI interface operating over 1, 2, or 4 IO lines.
//!
//! # Usage
//!
//! This driver supports using the QSPI peripheral in indirect mode. This allows the peripheral to
//! be used to read and write from an address over a quad-SPI interface.
//!
//! The SPI can be configured to operate on either of the two available banks on the board. In the
//! simplest case, this can be accomplished with just the peripheral and the GPIO pins.

//!
//! # Limitations
//!
//! This driver currently only supports indirect operation mode of the QSPI
//! interface. Automatic polling or memory-mapped modes are not supported.  This
//! driver support either bank 1 or bank 2 as well as a dual flash bank (in
//! which all 8 IOs are used for the interface).

use crate::{
    pac::{self, QUADSPI, RCC},
    traits::ClockCfg,
};

use core::ptr;

use cfg_if::cfg_if;

/// Represents operation modes of the QSPI interface.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum QspiMode {
    /// Only a single IO line (IO0) is used for transmit and a separate line (IO1) is used for receive.
    OneBit,

    /// Two IO lines (IO0 and IO1) are used for transmit/receive.
    TwoBit,

    /// All four IO lines are used for transmit/receive.
    FourBit,
}
impl QspiMode {
    pub(self) fn reg_value(&self) -> u8 {
        match self {
            QspiMode::OneBit => 1,
            QspiMode::TwoBit => 2,
            QspiMode::FourBit => 3,
        }
    }
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
pub struct Config {
    mode: QspiMode,
    frequency: u32,
    address_size: AddressSize,
    dummy_cycles: u8,
    sampling_edge: SamplingEdge,
    fifo_threshold: u8,
}

impl Config {
    /// Create a default configuration for the QSPI interface.
    ///
    /// * Bus in 1-bit Mode
    /// * 8-bit Address
    /// * No dummy cycle
    /// * Sample on falling edge
    pub fn new(freq: u32) -> Self {
        Config {
            mode: QspiMode::OneBit,
            frequency: freq.into(),
            address_size: AddressSize::EightBit,
            dummy_cycles: 0,
            sampling_edge: SamplingEdge::Falling,
            fifo_threshold: 1,
        }
    }

    /// Specify the operating mode of the QSPI bus. Can be a 1-bit, 2-bit or
    /// 4-bit width.
    ///
    /// The operating mode can also be changed using the
    /// [`configure_mode`](Qspi#method.configure_mode) method
    pub fn mode(mut self, mode: QspiMode) -> Self {
        self.mode = mode;
        self
    }

    /// Specify the size of the address phase
    pub fn address_size(mut self, address_size: AddressSize) -> Self {
        self.address_size = address_size;
        self
    }

    /// Specify the number of dummy cycles in between the address and data
    /// phases.
    ///
    /// Hardware supports 0-31 dummy cycles.
    ///
    /// # Note
    ///
    /// With zero dummy cycles, the QSPI peripheral will erroneously drive the
    /// output pins for an extra half clock cycle before IO is swapped from
    /// output to input. Refer to
    /// https://github.com/quartiq/stabilizer/issues/101 for more information.
    pub fn dummy_cycles(mut self, cycles: u8) -> Self {
        debug_assert!(cycles < 32, "Hardware only supports 0-31 dummy cycles");

        self.dummy_cycles = cycles;
        self
    }

    /// Specify the sampling edge for the QSPI receiver.
    ///
    /// # Note
    ///
    /// If zero dummy cycles are used, during read operations the QSPI
    /// peripheral will erroneously drive the output pins for an extra half
    /// clock cycle before IO is swapped from output to input. Refer to
    /// https://github.com/quartiq/stabilizer/issues/101 for more information.
    ///
    /// In this case it is recommended to sample on the falling edge. Although
    /// this doesn't stop the possible bus contention, delaying the sampling
    /// point by an extra half cycle results in a sampling point after the bus
    /// contention.
    pub fn sampling_edge(mut self, sampling_edge: SamplingEdge) -> Self {
        self.sampling_edge = sampling_edge;
        self
    }

    /// Specify the number of bytes in the FIFO that will set the FIFO threshold
    /// flag. Must be in the range 1-32 inclusive.
    ///
    /// In indirect write mode, this is the number of free bytes that will raise
    /// the FIFO threshold flag.
    ///
    /// In indirect read mode, this is the number of valid pending bytes that
    /// will raise the FIFO threshold flag.
    pub fn fifo_threshold(mut self, threshold: u8) -> Self {
        debug_assert!(threshold > 0 && threshold <= 32);

        self.fifo_threshold = threshold;
        self
    }
}

// todo: Do we need this trait? What is it?
pub trait QspiExt {
    fn bank1<CONFIG, C>(self, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Qspi
    where
        CONFIG: Into<Config>,
        C: ClockCfg;

    fn bank2<CONFIG, C>(self, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Qspi
    where
        CONFIG: Into<Config>,
        C: ClockCfg;

    fn qspi_unchecked<CONFIG, C>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
        C: ClockCfg;
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// FIFO Threashold
    FIFOThreashold,
    /// Transfer complete
    Complete,
    /// Tranfer error
    Error,
}

pub struct Qspi {
    rb: QUADSPI,
}

impl Qspi {
    pub fn bank1<CONFIG, C>(regs: QUADSPI, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Self
    where
        CONFIG: Into<Config>,
        C: ClockCfg,
    {
        Self::qspi_unchecked(regs, config, Bank::One, clocks, rcc)
    }

    pub fn bank2<CONFIG, C>(regs: QUADSPI, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Self
    where
        CONFIG: Into<Config>,
        C: ClockCfg,
    {
        Self::qspi_unchecked(regs, config, Bank::Two, clocks, rcc)
    }

    pub fn qspi_unchecked<CONFIG, C>(
        regs: QUADSPI,
        config: CONFIG,
        bank: Bank,
        clocks: &C,
        rcc: &mut RCC,
    ) -> Self
    where
        CONFIG: Into<Config>,
        C: ClockCfg,
    {
        cfg_if! {
            if #[cfg(feature = "l4")] {
                rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());
            } else { // G4 and H7
                rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());
            }
        }

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = match Self::get_clock(clocks) {
            Some(freq_hz) => freq_hz,
            _ => panic!("QSPI kernel clock not running!"),
        };

        while regs.sr.read().busy().bit_is_set() {}

        let config: Config = config.into();

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

        Qspi { rb: regs }
    }

    /// Deconstructs the QSPI HAL and returns the register block.
    pub fn free(self) -> QUADSPI {
        self.rb
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &QUADSPI {
        &self.rb
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut QUADSPI {
        &mut self.rb
    }

    /// Check if the QSPI peripheral is currently busy with a transaction
    pub fn is_busy(&self) -> bool {
        self.rb.sr.read().busy().bit_is_set()
    }

    /// Enable interrupts for the given `event`
    pub fn listen(&mut self, event: Event) {
        self.rb.cr.modify(|_, w| match event {
            Event::FIFOThreashold => w.ftie().set_bit(),
            Event::Complete => w.tcie().set_bit(),
            Event::Error => w.teie().set_bit(),
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
        CONFIG: Into<Config>,
        C: ClockCfg,
    {
        Qspi::qspi_unchecked(self, config, Bank::One, clocks, rcc)
    }

    fn bank2<CONFIG, C>(self, config: CONFIG, clocks: &C, rcc: &mut RCC) -> Qspi
    where
        CONFIG: Into<Config>,
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
        CONFIG: Into<Config>,
        C: ClockCfg,
    {
        Qspi::qspi_unchecked(self, config, bank, clocks, rcc)
    }
}
