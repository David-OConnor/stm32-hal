//! Quad Serial Peripheral Interface (SPI) bus: A specialized interface used for
//! high-speed communications with external flash memory. Also supports OctoSPI
//! on variants that support it.

use core::ptr;

use cfg_if::cfg_if;

use crate::{clocks::Clocks, pac::RCC};

cfg_if! {
    if #[cfg(any(feature = "l5", feature = "h735", feature = "h7b3"))] {
        use crate::pac::OCTOSPI1 as QUADSPI;
    } else if #[cfg(feature = "h5")] {
        use crate::pac::OCTOSPI as QUADSPI;
    } else {
        use crate::pac::QUADSPI;
    }
}

// todo: Status-polling mode.

// todo: Is this avail in PAC? Feature-gate if diff on diff platforms?
const MEM_MAPPED_BASE_ADDR: usize = 0x9000_0000;

#[derive(Copy, Clone)]
#[repr(u8)]
/// Sets the Qspi mode to single, dual, or quad. Affects the IMODE, ADMODE, ABMODE,
/// and DMODE fields of the CCR reg. Each of these fields affects a different mode of operation.
pub enum ProtocolMode {
    /// Only a single IO line (IO0) is used for transmit and a separate line (IO1) is used for receive.
    Single = 0b01,
    /// Two IO lines (IO0 and IO1) are used for transmit/receive.
    Dual = 0b10,
    /// All four IO lines are used for transmit/receive.
    Quad = 0b11,
}

// #[derive(Copy, Clone)]
// #[repr(u8)]
// pub enum AlternateBytesMode {
//     None = 0b00,
//     SingleLine = 0b01,
//     TwoLines = 0b010,
//     FourLines = 0b11,
// }

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
/// Sets the Qspi Functional Mode. Affects the FMODE field of the CCR reg.
pub enum FunctionalMode {
    IndirectWrite = 0b00,
    IndirectRead = 0b01,
    StatusPolling = 0b10,
    MemoryMapped = 0b11,
}

/// Address sizes used by the QSPI interface
#[derive(Copy, Clone, PartialEq)]
pub enum AddressSize {
    /// 8 byte address size.
    A8 = 0b00,
    /// 16 byte address size.
    A16 = 0b01,
    /// 24 byte address size.
    A24 = 0b10,
    /// 32 byte address size.
    A32 = 0b11,
}

/// Sampling mode for the QSPI interface
#[derive(Copy, Clone, PartialEq)]
pub enum SamplingEdge {
    Rising = 0,
    Falling = 1,
}

/// Indicates an error with the QSPI peripheral.
#[derive(Copy, Clone, PartialEq)]
pub enum QspiError {
    Busy,
    Underflow,
}

// todo: Use bank on suitable MCUs? Which? F7 / H7?

/// A structure for specifying QSPI configuration.
#[derive(Copy, Clone)]
pub struct QspiConfig {
    /// Note: For configuration purposes, use IndirectRead or Indirect Write
    // pub functional_mode: FunctionalMode,
    pub protocol_mode: ProtocolMode,
    pub data_mode: DataMode,
    // pub address_mode: ProtocolMode,
    // pub alternate_bytes_mode: AlternateBytesMode,
    /// Dide the QSPI kernel clock by this to get the QSPI speed. Defaults to 4.
    pub clock_division: u8,
    pub address_size: AddressSize,
    pub dummy_cycles: u8,
    pub sampling_edge: SamplingEdge,
    pub fifo_threshold: u8,
    /// Size of memory, in megabytes. (not megabits). Defaults to 64M-bits (8 M-bytes)
    pub mem_size: u32,
}

impl Default for QspiConfig {
    fn default() -> Self {
        Self {
            // functional_mode: FunctionalMode::IndirectRead,
            // todo: QC what you want here.
            protocol_mode: ProtocolMode::Quad,
            data_mode: DataMode::Sdr,
            // alternate_bytes_mode: AlternateBytesMode::None,
            // For example: On an H743, this might be 240Mhz/4 = 60Mhz.
            clock_division: 4,
            address_size: AddressSize::A8,
            dummy_cycles: 0,
            sampling_edge: SamplingEdge::Falling,
            fifo_threshold: 1, // todo: What is this?
            // This is
            mem_size: 8,
        }
    }
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum QspiInterrupt {
    FifoThreshold,
    StatusMatch,
    Timeout,
    TransferComplete,
    TransferError,
}

/// Represents a Quad Serial Peripheral Interface (QSPI) peripheral.
pub struct Qspi {
    pub regs: QUADSPI,
    pub cfg: QspiConfig,
}

// todo: Use the deref pattern for OCTOSPI2 support.
impl Qspi {
    pub fn new(regs: QUADSPI, cfg: QspiConfig, clocks: &Clocks) -> Self {
        assert!(
            cfg.dummy_cycles < 32,
            "Dumy cycles must be between 0 and 31."
        );

        let rcc = unsafe { &(*RCC::ptr()) };

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

        // todo: You need to get rcc en reset working for this to make it work on octospi2.

        cfg_if! {
            if #[cfg(any(feature = "h735", feature = "h7b3"))] {
                rcc.ahb3enr.modify(|_, w| w.octospi1en().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.octospi1rst().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.octospi1rst().clear_bit());
            } else if #[cfg(feature = "h5")] {
                rcc.ahb4enr.modify(|_, w| w.octospi1en().set_bit());
                rcc.ahb4rstr.modify(|_, w| w.octospi1rst().set_bit());
                rcc.ahb4rstr.modify(|_, w| w.octospi1rst().clear_bit());
            } else if #[cfg(feature = "l5")] {
                rcc.ahb3enr.modify(|_, w| w.ospi1en().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.ospi1rst().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.ospi1rst().clear_bit());
            } else {
                rcc.ahb3enr.modify(|_, w| w.qspien().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().set_bit());
                rcc.ahb3rstr.modify(|_, w| w.qspirst().clear_bit());
            }
        }

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        // Many fields, including all CCR fields, can only be set when `BUSY` is clear.
        while regs.sr.read().busy().bit_is_set() {}

        regs.ccr.modify(|_, w| unsafe {
            w.abmode().bits(cfg.protocol_mode as u8);
            w.admode().bits(cfg.protocol_mode as u8);
            w.imode().bits(cfg.protocol_mode as u8);
            w.dmode().bits(cfg.protocol_mode as u8);
            #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
            // todo: Equiv for octo?
            w.ddrm().bit(cfg.data_mode as u8 != 0);
            #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
            // todo: Equiv for octo?
            w.dcyc().bits(cfg.dummy_cycles);
            w.adsize().bits(cfg.address_size as u8)
        });

        // RM: The FSIZE[4:0] field defines the size of external memory using the following formula:
        // Number of bytes in Flash memory = 2^[FSIZE+1]
        // The addressable space in memory-mapped mode is limited to 256MB.
        //
        let fsize = (cfg.mem_size * 1_000_000).ilog2() - 1;
        #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
        regs.dcr
            .modify(|_, w| unsafe { w.fsize().bits(fsize as u8) });

        // RM: This field [prescaler] defines the scaler factor for generating CLK based on the
        // clock (value+1).
        // 0: FCLK = F, clock used directly as QUADSPI CLK (prescaler bypassed)
        // 1: FCLK = F/2
        // 2: FCLK = F/3
        // ...
        // 255: FCLK = F/256

        let sampling_edge = match cfg.data_mode {
            // When receiving data in SDR mode, the QUADSPI assumes that the Flash memories also
            // send the data using CLK’s falling edge. By default (when SSHIFT = 0), the signals are
            // sampled using the following (rising) edge of CLK.
            DataMode::Sdr => cfg.sampling_edge,
            // When receiving data in DDR mode, the QUADSPI assumes that the Flash memories also
            // send the data using both rising and falling CLK edges. When DDRM = 1, firmware must
            // clear SSHIFT bit (bit 4 of QUADSPI_CR). Thus, the signals are sampled one half of a CLK
            // cycle later (on the following, opposite edge).
            DataMode::Ddr => SamplingEdge::Rising,
        };

        #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
        // todo: Equiv for octo?
        regs.cr.write(|w| unsafe {
            w.prescaler().bits(cfg.clock_division as u8 - 1);
            w.sshift().bit(sampling_edge as u8 != 0);
            w.fthres().bits(cfg.fifo_threshold - 1)
        });

        // Enable ther peripheral
        regs.cr.modify(|_, w| w.en().set_bit());

        Self { regs, cfg }
    }

    /// Check if the QSPI peripheral is currently busy with a transaction
    pub fn is_busy(&self) -> bool {
        self.regs.sr.read().busy().bit_is_set()
    }

    /// Enable an interrupt
    pub fn enable_interrupt(&mut self, interrupt: QspiInterrupt) {
        self.regs.cr.modify(|_, w| match interrupt {
            QspiInterrupt::FifoThreshold => w.ftie().set_bit(),
            QspiInterrupt::StatusMatch => w.smie().set_bit(),
            QspiInterrupt::TransferComplete => w.tcie().set_bit(),
            QspiInterrupt::Timeout => w.toie().set_bit(),
            QspiInterrupt::TransferError => w.teie().set_bit(),
        });
    }

    /// Clear an interrupt flag
    pub fn clear_interrupt(&mut self, interrupt: QspiInterrupt) {
        self.regs.fcr.write(|w| match interrupt {
            QspiInterrupt::FifoThreshold => panic!("Can't clear that interrupt manually."),
            QspiInterrupt::StatusMatch => w.csmf().set_bit(),
            QspiInterrupt::TransferComplete => w.ctcf().set_bit(),
            QspiInterrupt::Timeout => w.ctof().set_bit(),
            QspiInterrupt::TransferError => w.ctef().set_bit(),
        });
    }

    /// Perform a memory write in indirect mode.
    pub fn write_indirect(&mut self, addr: u32, data: &[u8]) {
        // FMODE, and perhaps othe rfields can only be set when BUSY = 0.
        while self.is_busy() {}

        // todo: Fix this
        assert!(
            data.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        // RM: Indirect Mode procedure:
        // When FMODE is programmed to 00, indirect write mode is selected and data can be sent to
        // the Flash memory. With FMODE = 01, indirect read mode is selected where data can be
        // read from the Flash memory.
        // When the QUADSPI is used in indirect mode, the frames are constructed in the following
        // way:

        // 1. Specify a number of data bytes to read or write in the QUADSPI_DLR.
        // (From DLR field description: Number of data to be retrieved (value+1) in indirect
        // and status-polling modes... 0x0000_0000: 1 byte is to be transferred etc)
        self.regs
            .dlr
            .write(|w| unsafe { w.dl().bits(data.len() as u32 - 1) });

        // 2. Specify the frame format, mode and instruction code in the QUADSPI_CCR.
        // 3. Specify optional alternate byte to be sent right after the address phase in the
        // QUADSPI_ABR.
        // (Handled in init)
        // 4. Specify the operating mode in the QUADSPI_CR. If FMODE = 00 (indirect write mode)
        // and DMAEN = 1, then QUADSPI_AR should be specified before QUADSPI_CR,
        // because otherwise QUADSPI_DR might be written by the DMA before QUADSPI_AR
        // is updated (if the DMA controller has already been enabled)
        #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
        // todo: Equiv for octo?
        self.regs
            .ccr
            .modify(|_, w| unsafe { w.fmode().bits(FunctionalMode::IndirectWrite as u8) });
        // 5. Specify the targeted address in the QUADSPI_AR.
        self.regs
            .ar
            .modify(|_, w| unsafe { w.address().bits(addr) });

        // 6. Read/Write the data from/to the FIFO through the QUADSPI_DR.
        // When writing the control register (QUADSPI_CR) the user specifies the following settings:
        // * The enable bit (EN) set to ‘1’
        // * The DMA enable bit (DMAEN) for transferring data to/from RAM
        // * Timeout counter enable bit (TCEN)
        // * Sample shift setting (SSHIFT)
        // * FIFO threshold level (FTRHES) to indicate when the FTF flag should be set
        // * Interrupt enables
        // * Automatic polling mode parameters: match mode and stop mode (valid when
        // FMODE = 11)
        // * Clock prescalerQuad-SPI interface (QUADSPI) RM0434
        // 400/1529 RM0434 Rev 7
        // When writing the communication configuration register (QUADSPI_CCR) the user specifies
        // the following parameters:
        // * The instruction byte through the INSTRUCTION bits
        // * The way the instruction has to be sent through the IMODE bits (1/2/4 lines)
        // * The way the address has to be sent through the ADMODE bits (None/1/2/4 lines)
        // * The address size (8/16/24/32-bit) through the ADSIZE bits
        // * The way the alternate bytes have to be sent through the ABMODE (None/1/2/4 lines)
        // * The alternate bytes number (1/2/3/4) through the ABSIZE bits
        // * The presence or not of dummy bytes through the DBMODE bit
        // * The number of dummy bytes through the DCYC bits
        // * The way the data have to be sent/received (None/1/2/4 lines) through the DMODE bits
        // (Above items handled in init)

        unsafe {
            for word in data {
                #[allow(invalid_reference_casting)]
                ptr::write_volatile(&self.regs.dr as *const _ as *mut u8, *word);
            }
        }

        // Wait for the transaction to complete
        while self.regs.sr.read().tcf().bit_is_clear() {}

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}
    }

    /// Perform a memory read in indirect mode.
    pub fn read_indirect(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), QspiError> {
        while self.is_busy() {}

        // todo: Fix this
        assert!(
            buf.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        // Steps are equivalent to those listed in `write_indirect`.
        self.regs
            .dlr
            .write(|w| unsafe { w.dl().bits(buf.len() as u32 - 1) });
        #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
        // todo: Equiv for octo?
        self.regs
            .ccr
            .modify(|_, w| unsafe { w.fmode().bits(FunctionalMode::IndirectRead as u8) });
        self.regs
            .ar
            .modify(|_, w| unsafe { w.address().bits(addr) });

        // Check for underflow on the FIFO.
        if (self.regs.sr.read().flevel().bits() as usize) < buf.len() {
            return Err(QspiError::Underflow);
        }

        unsafe {
            for word in buf {
                *word = ptr::read_volatile(&self.regs.dr as *const _ as *const u8);
            }
        }

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}

        Ok(())
    }

    // todo: write_indirect_dma fn.

    /// Read one word from memory in memory-mapped mode
    pub fn read_1_mem_mapped(&mut self, offset: isize) -> u32 {
        // todo: unsafe fn? word size?
        while self.is_busy() {}

        #[cfg(not(any(feature = "l5", feature = "h735", feature = "h7b3")))]
        // todo: Equiv for octo?
        if self.regs.ccr.read().fmode().bits() != FunctionalMode::MemoryMapped as u8 {
            self.regs
                .ccr
                .modify(|_, w| unsafe { w.fmode().bits(FunctionalMode::MemoryMapped as u8) });
        }

        let addr = MEM_MAPPED_BASE_ADDR as *const u32; // as const what?
        unsafe { core::ptr::read(addr.offset(offset)) }
    }
}
