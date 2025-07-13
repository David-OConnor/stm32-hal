//! This module contains utility macros that are not STM32-specific, or specific
//! to this library.

/// Syntax helper for getting global variables of the form `Mutex<RefCell<Option>>>` from an interrupt-free
/// context - eg in interrupt handlers.
///
/// Example: `access_global!(DELAY, delay, cs)`
#[macro_export]
macro_rules! access_global {
    ($NAME_GLOBAL:ident, $name_local:ident, $cs:expr) => {
        let mut part1 = $NAME_GLOBAL.borrow($cs).borrow_mut();
        let $name_local = part1.as_mut().unwrap();
    };
}

/// Similar to `access_global`, but combining multiple calls.
///
/// Example: `access_globals!([
///     (USB_DEV, usb_dev),
///     (USB_SERIAL, usb_serial),
/// ], cs);`
#[macro_export]
macro_rules! access_globals {
    ([$(($NAME_GLOBAL:ident, $name_local:ident)),* $(,)?], $cs:expr) => {
        $(
            let mut part = $NAME_GLOBAL.borrow($cs).borrow_mut();
            let $name_local = part.as_mut().unwrap();
        )*
    };
}

/// Syntax helper for setting global variables of the form `Mutex<RefCell<Option>>>`.
/// eg in interrupt handlers. Ideal for non-copy-type variables that can't be initialized
/// immediatiately.
///
/// Example: `make_globals!(
///     (USB_SERIAL, SerialPort<UsbBusType>),
///     (DELAY, Delay),
/// )`
#[macro_export]
macro_rules! make_globals {
    ($(($NAME:ident, $type:ty)),+ $(,)?) => {
        $(
            static $NAME: ::critical_section::Mutex<core::cell::RefCell<Option<$type>>> = ::critical_section::Mutex::new(core::cell::RefCell::new(None));
        )+
    };
}

/// Syntax helper for setting global variables of the form `Mutex<Cell<>>>`.
/// eg in interrupt handlers. Ideal for copy-type variables.
///
/// Example: `make_simple_globals!(
///     (VALUE, f32, 2.),
///     (SETTING, Setting, Setting::A),
/// )`
#[macro_export]
macro_rules! make_simple_globals {
    ($(($NAME:ident, $type:ty, $val:expr)),+ $(,)?) => {
        $(
            static $NAME: ::critical_section::Mutex<core::cell::Cell<$type>> = ::critical_section::Mutex::new(core::cell::Cell::new($val));
        )+
    };
}

/// Initialize one or more globals inside a critical section.
///
/// Usage:
/// ```rust
/// init_globals!(
///     (FLASH, flash),
///     (SPI_IMU, spi1),
///     (I2C_MAG, i2c1),
///     // …
/// );
/// ```
#[macro_export]
macro_rules! init_globals {
    ($(($NAME:ident, $val:expr)),* $(,)?) => {
        ::critical_section::with(|cs| {
            $(
                $NAME.borrow(cs).replace(Some($val));
            )*
        });
    };
}

/// Automates Cortex-M NVIC setup. The second value is NVIC priority; lower
/// is higher priority. Example use:
/// setup_nvic!([
///     (TIM2, 6),
///     (TIM3, 7),
///     (EXTI0, 4),
/// ], cp);
#[macro_export]
macro_rules! setup_nvic {
    (
        [ $( ($int:ident, $prio:expr) ),* $(,)? ],
        $cp:ident
    ) => {
        unsafe {
            $(
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::$int);
            )*
            $(
                $cp.NVIC.set_priority(pac::Interrupt::$int, $prio);
            )*
        }
    };
}

/// Syntax helper for parsing multi-byte fields into primitives.
///
/// Example: `parse_le!(bytes, i32, 5..9);`
#[macro_export]
macro_rules! parse_le {
    ($bytes:expr, $t:ty, $range:expr) => {{ <$t>::from_le_bytes($bytes[$range].try_into().unwrap()) }};
}

/// Syntax helper for parsing multi-byte fields into primitives.
///
/// Example: `parse_be!(bytes, i32, 5..9);`
#[macro_export]
macro_rules! parse_be {
    ($bytes:expr, $t:ty, $range:expr) => {{ <$t>::from_be_bytes($bytes[$range].try_into().unwrap()) }};
}

/// Syntax helper for converting primitives to multi-byte fields.
///
/// Example: `copy_le!(bytes, self.position, 5..9);`
#[macro_export]
macro_rules! copy_le {
    ($dest:expr, $src:expr, $range:expr) => {{ $dest[$range].copy_from_slice(&$src.to_le_bytes()) }};
}

/// Syntax helper for converting primitives to multi-byte fields.
///
/// Example: `copy_be!(bytes, self.position, 5..9);`
#[macro_export]
macro_rules! copy_be {
    ($dest:expr, $src:expr, $range:expr) => {{ $dest[$range].copy_from_slice(&$src.to_be_bytes()) }};
}
