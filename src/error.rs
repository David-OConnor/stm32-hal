//! Common error definitions.

#[cfg(not(any(feature = "f", feature = "wb", feature = "wl", feature = "h5")))]
use crate::crc::PolynomialError;
#[cfg(not(any(feature = "l552", feature = "h5", feature = "f4")))]
use crate::dma::DmaError;
#[cfg(not(feature = "h735"))]
use crate::flash::FlashError;
#[cfg(not(any(
    feature = "f",
    feature = "l4x3", // todo: PAC bug?
    feature = "g0",
    feature = "g431",
    feature = "g441",
    feature = "g471",
    feature = "g491",
    feature = "g4a1",
    feature = "wl",
    feature = "l5", // todo: PAC errors on some regs.
    feature = "h5",
    feature = "c0",
)))]
use crate::qspi::QspiError;
use crate::rtc::RtcError;
#[cfg(not(feature = "f301"))]
use crate::spi::SpiError;
use crate::{clocks::RccError, i2c::I2cError, timer::TimerError, usart::UsartError};

macro_rules! impl_from_error {
    ($error:ident) => {
        impl From<$error> for Error {
            fn from(error: $error) -> Self {
                Self::$error(error)
            }
        }
    };
}

/// Alias for Result<T, Error>.
pub type Result<T> = core::result::Result<T, Error>;

/// Collection of all errors that can occur.
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum Error {
    /// Occurs when an expected change of a register does happen in time.
    ///
    /// This is returned when a bounded loop exceeds its alotted iteration count.
    RegisterUnchanged,
    #[cfg(not(any(feature = "l552", feature = "h5", feature = "f4")))]
    /// Direct Memory Access (DMA) error
    DmaError(DmaError),
    I2cError(I2cError),
    UsartError(UsartError),
    TimerError(TimerError),
    ///
    #[cfg(not(feature = "h735"))]
    FlashError(FlashError),
    #[cfg(not(any(feature = "f", feature = "wb", feature = "wl", feature = "h5")))]
    /// CRC
    PolynomialError(PolynomialError),
    #[cfg(not(feature = "f301"))]
    /// SPI errors.
    SpiError(SpiError),
    /// Clock errors.
    RtcError(RtcError),
    RccError(RccError),
    #[cfg(not(any(
        feature = "f",
        feature = "l4x3", // todo: PAC bug?
        feature = "g0",
        feature = "g431",
        feature = "g441",
        feature = "g471",
        feature = "g491",
        feature = "g4a1",
        feature = "wl",
        feature = "l5", // todo: PAC errors on some regs.
        feature = "h5",
        feature = "c0",
    )))]
    QspiError(QspiError),
}

#[cfg(not(any(feature = "l552", feature = "h5", feature = "f4")))]
impl_from_error!(DmaError);
impl_from_error!(I2cError);
impl_from_error!(UsartError);
impl_from_error!(TimerError);
#[cfg(not(feature = "h735"))]
impl_from_error!(FlashError);
#[cfg(not(any(feature = "f", feature = "wb", feature = "wl", feature = "h5")))]
impl_from_error!(PolynomialError);
#[cfg(not(feature = "f301"))]
impl_from_error!(SpiError);
impl_from_error!(RtcError);
impl_from_error!(RccError);
#[cfg(not(any(
    feature = "f",
    feature = "l4x3", // todo: PAC bug?
    feature = "g0",
    feature = "g431",
    feature = "g441",
    feature = "g471",
    feature = "g491",
    feature = "g4a1",
    feature = "wl",
    feature = "l5", // todo: PAC errors on some regs.
    feature = "h5",
    feature = "c0",
)))]
impl_from_error!(QspiError);

#[cfg(feature = "embedded_hal")]
mod embedded_io_impl {
    use embedded_hal::i2c::{Error as I2cEhError, ErrorKind as I2cErrorKind, NoAcknowledgeSource};
    use embedded_io::{Error as IoError, ErrorKind as IoErrorKind};

    use super::{Error, I2cError, UsartError};

    // Other,
    // NotFound,
    // PermissionDenied,
    // ConnectionRefused,
    // ConnectionReset,
    // ConnectionAborted,
    // NotConnected,
    // AddrInUse,
    // AddrNotAvailable,
    // BrokenPipe,
    // AlreadyExists,
    // InvalidInput,
    // InvalidData,
    // TimedOut,
    // Interrupted,
    // Unsupported,
    // OutOfMemory,
    // WriteZero,

    impl I2cEhError for Error {
        fn kind(&self) -> I2cErrorKind {
            match self {
                Error::I2cError(i) => match i {
                    I2cError::Bus => I2cErrorKind::Bus,
                    I2cError::Arbitration => I2cErrorKind::ArbitrationLoss,
                    I2cError::Nack => I2cErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
                    I2cError::Overrun => I2cErrorKind::Overrun,
                    _ => I2cErrorKind::Other,
                    // I2cError::Other => I2cErrorKind::Other,
                },
                _ => I2cErrorKind::Other,
            }
        }
    }

    impl IoError for Error {
        fn kind(&self) -> IoErrorKind {
            match self {
                Error::RegisterUnchanged => IoErrorKind::TimedOut,
                Error::UsartError(u) => match u {
                    UsartError::Framing => IoErrorKind::Other,
                    UsartError::Noise => IoErrorKind::Other,
                    UsartError::Overrun => IoErrorKind::OutOfMemory,
                    UsartError::Parity => IoErrorKind::InvalidData,
                },
                _ => IoErrorKind::Other,
            }
        }
    }
}
