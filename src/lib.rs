// Some overall notes:
// We generally don't use the named field methods provided by PACs, as these are inconsistently
// implemented among PACs. Ie f3's may have a `'`.enabled()` method, but `l4` does not;
// in these cases, writing `set_bit()` works for both.

// We use a combination of macros and feature-gating to handle differences in families, as appropriate.
// We leverage the `paste` and `cfg-if` crates to improve syntax.

// The main way we divide MCUs is by PAC modules. Note that there are sub-variants that may have differences
// that this doesn't take into account. (eg different USB memory sizes among f303 variants)

// We use `unsafe` blocks for most multi-fit field writes. This is required by some PACs, but not others.
// The rust embedded team removes requirement for `unsafe` on fields that are deemed sufficiently
// constrained as to not need these blocks.
// Using `unsafe` for all is cleaner than feature-gating, due to how many fields this affects. We've allowed
// these warnings; ie hidden during build.

// todo: Modify `rust.yml` to test with `usb` and `can` features.

#![no_std]
// Some reg modifications are marked `unsafe` in some PAC crates, but not others.
// Disable these warnings.
#![allow(unused_unsafe)]

#[cfg(not(any(
    feature = "f301",
    feature = "f302",
    feature = "f303",
    feature = "f373",
    feature = "f3x4",
    feature = "f401",
    feature = "f405",
    feature = "f407",
    feature = "f410",
    feature = "f411",
    feature = "f412",
    feature = "f413",
    feature = "f427",
    feature = "f429",
    feature = "f446",
    feature = "f469",
    feature = "l4x1",
    feature = "l4x2",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l552",
    feature = "l562",
    feature = "g030",
    feature = "g031",
    feature = "g041",
    feature = "g070",
    feature = "g071",
    feature = "g081",
    feature = "g431",
    feature = "g441",
    feature = "g471",
    feature = "g473",
    feature = "g474",
    feature = "g483",
    feature = "g484",
    feature = "g491",
    feature = "g4a1",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm4",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
)))]
compile_error!("This crate requires an MCU-specifying feature to be enabled. eg `l552`.");

// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for
// stm32 peripherals.

#[cfg(feature = "f301")]
pub use stm32f3::stm32f301 as pac;

#[cfg(feature = "f302")]
pub use stm32f3::stm32f302 as pac;

#[cfg(feature = "f303")]
pub use stm32f3::stm32f303 as pac;

#[cfg(feature = "f373")]
pub use stm32f3::stm32f373 as pac;

#[cfg(feature = "f3x4")]
pub use stm32f3::stm32f3x4 as pac;

// F4 PAC
#[cfg(feature = "f401")]
pub use stm32f4::stm32f401 as pac;

#[cfg(feature = "f405")]
pub use stm32f4::stm32f405 as pac;

#[cfg(feature = "f407")]
pub use stm32f4::stm32f407 as pac;

#[cfg(feature = "f410")]
pub use stm32f4::stm32f410 as pac;

#[cfg(feature = "f411")]
pub use stm32f4::stm32f411 as pac;

#[cfg(feature = "f412")]
pub use stm32f4::stm32f412 as pac;

#[cfg(feature = "f413")]
pub use stm32f4::stm32f413 as pac;

#[cfg(feature = "f427")]
pub use stm32f4::stm32f427 as pac;

#[cfg(feature = "f429")]
pub use stm32f4::stm32f429 as pac;

#[cfg(feature = "f446")]
pub use stm32f4::stm32f446 as pac;

#[cfg(feature = "f469")]
pub use stm32f4::stm32f469 as pac;

// L4 PAC
#[cfg(feature = "l4x1")]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(feature = "l4x2")]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(feature = "l4x3")]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(feature = "l4x5")]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(feature = "l4x6")]
pub use stm32l4::stm32l4x6 as pac;

// L5 PAC
#[cfg(feature = "l552")]
pub use stm32l5::stm32l552 as pac;

#[cfg(feature = "l562")]
pub use stm32l5::stm32l562 as pac;

// G0 PAC
#[cfg(feature = "g030")]
pub use stm32g0::stm32g030 as pac;

#[cfg(feature = "g031")]
pub use stm32g0::stm32g031 as pac;

#[cfg(feature = "g041")]
pub use stm32g0::stm32g041 as pac;

#[cfg(feature = "g070")]
pub use stm32g0::stm32g070 as pac;

#[cfg(feature = "g071")]
pub use stm32g0::stm32g071 as pac;

#[cfg(feature = "g081")]
pub use stm32g0::stm32g081 as pac;

// G4 PAC
#[cfg(feature = "g431")]
pub use stm32g4::stm32g431 as pac;

#[cfg(feature = "g441")]
pub use stm32g4::stm32g441 as pac;

#[cfg(feature = "g471")]
pub use stm32g4::stm32g471 as pac;

#[cfg(feature = "g473")]
pub use stm32g4::stm32g473 as pac;

#[cfg(feature = "g474")]
pub use stm32g4::stm32g474 as pac;

#[cfg(feature = "g483")]
pub use stm32g4::stm32g483 as pac;

#[cfg(feature = "g484")]
pub use stm32g4::stm32g484 as pac;

#[cfg(feature = "g491")]
pub use stm32g4::stm32g491 as pac;

#[cfg(feature = "g4a1")]
pub use stm32g4::stm32g4a1 as pac;

// H7 PAC
#[cfg(feature = "h743")]
pub use stm32h7::stm32h743 as pac;

#[cfg(feature = "h743v")]
pub use stm32h7::stm32h743v as pac;

#[cfg(feature = "h747cm4")]
pub use stm32h7::stm32h747cm4 as pac;

#[cfg(feature = "h747cm7")]
pub use stm32h7::stm32h747cm7 as pac;

#[cfg(feature = "h753")]
pub use stm32h7::stm32h753 as pac;

#[cfg(feature = "h753v")]
pub use stm32h7::stm32h753v as pac;

#[cfg(feature = "h7b3")]
pub use stm32h7::stm32h7b3 as pac;

// todo: U5 once SVD is out.

pub mod traits;

#[cfg(not(any(feature = "f301", feature = "f302")))]
pub mod adc;

// bxCAN families: F3, F4, L4,
// fdCAN families: L5, U5, G4, H7
// H7 suppords fd and can_ccu. (What's that?)
#[cfg(all(
    feature = "can",
    any(
        feature = "f3",
        all(feature = "f4", not(any(feature = "f401", feature = "f410"))),
        feature = "l4"
    )
))]
pub mod can;

pub mod clocks;
#[cfg(not(any(feature = "f4", feature = "g0", feature = "g4", feature = "l5")))] // todo
pub mod crc;
pub mod dac;

// todo: F3, G0 missing many DMA registers like CCR? H7 DMA layout is different.
// todo: F4 needs some mods. So, only working on L4 and G4.
// todo: L5 has a PAC bug on CCR registers past 1.
#[cfg(not(any(feature = "h7", feature = "f4", feature = "l5")))]
pub mod dma;

pub mod delay;

#[cfg(not(feature = "h7"))] // todo
pub mod flash;
pub mod gpio;

#[cfg(not(any(feature = "f4", feature = "f3x4")))]
pub mod i2c;
#[cfg(feature = "f4")]
pub mod i2c_f4;
#[cfg(feature = "f4")]
pub use i2c_f4 as i2c;

pub mod low_power;

#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "l4x3",
    feature = "l5",
    feature = "g0",
    feature = "g431",
    feature = "g441",
    feature = "g471",
    feature = "g491",
    feature = "g4a1",
    feature = "h7b3",
)))]
pub mod qspi;
pub mod rtc;

pub mod usart;

// #[cfg(not(any(feature = "f3")))]
// pub mod sai;

#[cfg(not(feature = "h7"))] // todo
pub mod spi;

pub mod timer;

// In the l4 series, only l4x2 and l4x3 have USB.
// Non-OTG: F3, L4, [L5, G4?] (Are these newer ones really non-OTG? Use the usbd crate?)
// OTG HS: F4, H7
// OTG FS: F4
// todo: This needs work. For example, where does G4 fit in? has USB_FS_DEVICE reg block in PAC.
cfg_if::cfg_if! {
    if #[cfg(all(
        feature = "usb",
        not(any(feature = "f301", feature = "f3x4", feature = "l4x1", feature = "l4x5", feature = "l4x6", feature = "g4", feature = "h7"))
    ))] {
        pub mod usb;
    } else if #[cfg(all(feature = "h7", feature = "usbotg"))] {
        pub mod usb_otg as usb;
    }
}

mod util;

// todo: should these helper macros be removed from this library? It has nothing to do with STM32.

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
    ($(($NAME:ident, $type:ty)),+) => {
        $(
            static $NAME: Mutex<RefCell<Option<$type>>> = Mutex::new(RefCell::new(None));
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
    ($(($NAME:ident, $type:ty, $val:expr)),+) => {
        $(
            static $NAME: Mutex<Cell<$type>> = Mutex::new(Cell::new($val));
        )+
    };
}

// todo: Remove this function on MCUs that don't require it. Ie, is this required on G4? G0?

#[cfg(not(any(feature = "g0", feature = "h7")))]
/// Workaround due to debugger disconnecting in WFI (and low-power) modes.
/// This affects most (all?) STM32 devices. In production on battery-powered
/// devices that don't use DMA, consider removing this, to prevent power
/// use by the DMA clock.
/// For why we enable the DMA clock, see STM32F446 errata, section 2.1.1.
pub fn debug_workaround(dbgmcu: &mut pac::DBGMCU, rcc: &mut pac::RCC) {
    #[cfg(not(feature = "l5"))]
    dbgmcu.cr.modify(|_, w| w.dbg_sleep().set_bit());
    dbgmcu.cr.modify(|_, w| w.dbg_stop().set_bit());
    dbgmcu.cr.modify(|_, w| w.dbg_standby().set_bit());

    // todo Some MCUs may need the dbgmcu lines, but not DMA enabled.
    // todo: Remove this part on MCUs not affected. F4 and L4 are confirmed affected.

    #[cfg(feature = "f3")]
    rcc.ahbenr.modify(|_, w| w.dma1en().set_bit());

    #[cfg(not(feature = "f3"))]
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
}

/// In the prelude, we export helper macros.
pub mod prelude {
    pub use access_global;
    pub use make_globals;
    pub use make_simple_globals;
}
