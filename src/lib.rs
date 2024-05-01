//! This library provides high-level access to STM32 peripherals.
//!
//! **Current family support**: F3, F4, L4, L5, G0, G4, H7, and WB. U5 is planned once its SVD files and PAC
//! become available.
//!
//! Please see the [Readme](https://github.com/David-OConnor/stm32-hal/blob/main/README.md) for a detailed overview,
//! and the [examples folder on Github](https://github.com/David-OConnor/stm32-hal/tree/main/examples)
//! for example code and project structure.
//!
//! ## Getting started
//! Review the [syntax overview example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/syntax_overview)
//! for example uses of many of this library's features. Copy and paste its whole folder (It's set up
//! using [Knurling's app template](https://github.com/knurling-rs/app-template)), or copy parts of `Cargo.toml`
//! and `main.rs` as required.
//!
//! The [blinky example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/blinky) provides a detailed example and instructions for how to set up a blinking
//! light (ie hello world) using an STM32F411 "blackpill" board. Its readme provides instructions for how to get
//! started from scratch, and its code contains detailed comments explaining each part. The
//! [blinky with timer interrupt example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/blinky_timer_interrupt)
//! demonstrates how to accomplish the same in a non-blocking way, using a hardware timer. It uses RTIC.
//!
//! The [conductivity module example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/conductivity_module)
//! is a complete example of simple production firmware. It uses the DAC, I2C, Timer, and UART peripherals,
//! with a simple interupt-based control flow.
//!
//! The [PDM mic, DAC output passthrough example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/pdm_mic_dac_output.rs)
//! demonstrates how to read audio from a digital microphone, output it to headphones or speakers using the DAC, and use DMA
//! to do this efficiently. It conducts minimal processing, but can be modified to process using DSP between input and output.
//! This example uses RTIC.
//!
//! Additional examples in the [examples folder](https://github.com/David-OConnor/stm32-hal/tree/main/examples) demonstrate
//! how to use various STM32 peripherals; most of these examples focus on a single peripheral.
//!
//! When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
//! representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
//!  include a run-time feature, following the template `l4rt`. For example:
//! ```toml
//! cortex-m = { version = "^0.7.7", features = ["critical-section-single-core"] }
//! cortex-m-rt = "0.7.2"
//! hal = { package = "stm32-hal2", version = "^1.5.5", features = ["l4x3", "l4rt"]}
//! ```
//!
//! If you need `embedded-hal` traits, include the `embedded-hal` feature.
//!
//! You can review [this section of Cargo.toml](https://github.com/David-OConnor/stm32-hal/blob/main/Cargo.toml#L61)
//! to see which MCU and runtime features are available.
//!
//! ### Example highlights:
//! ```rust
//! use cortex_m;
//! use cortex_m_rt::entry;
//! use hal::{
//!     clocks::Clocks,
//!     gpio::{Pin, Port, PinMode, OutputType},
//!     i2c::I2c,
//!     low_power,
//!     pac,
//!     timer::{Timer, TimerInterrupt},
//! };
//!
//! #[entry]
//! fn main() -> ! {
//!    let mut dp = pac::Peripherals::take().unwrap();
//!
//!    let clock_cfg = Clocks::default();
//!    clock_cfg.setup().unwrap();
//!
//!    let mut pb15 = Pin::new(Port::A, 15, PinMode::Output);
//!    pb15.set_high();
//!
//!    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, Default::default(), &clock_cfg);
//!    timer.enable_interrupt(TimerInterrupt::Update);
//!
//!    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
//!    scl.output_type(OutputType::OpenDrain);
//!
//!    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
//!    sda.output_type(OutputType::OpenDrain);
//!
//!    let mut dma = Dma::new(dp.DMA1);
//!    dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::I2c1Tx);
//!
//!    let i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);
//!
//!    loop {
//!        i2c.write(0x50, &[1, 2, 3]);
//!        // Or:
//!        i2c.write_dma(0x50, &BUF, DmaChannel::C1, Default::default(), DmaPeriph::Dma1);
//!
//!        low_power::sleep_now();
//!    }
//!}
//! ```
//!
//! Supports the RTIC `Monotonic` trait. To enable, use the `monotonic` feature.
//!
//! [This article](https://www.anyleaf.org/blog/writing-embedded-firmware-using-rust) provides some information
//! on using this library, as well as background information on Rust embedded in general.
//!
//! ## Docs caveat
//! This Rust docs page is built for `STM32H735`, and some aspects are not accurate for other
//! variants. Clock (RCC) config in particular varies significantly between variants. We currently
//! don't have a good solution to this problem, and may self-host docs in the future.

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

#![no_std]
// Some reg modifications are marked `unsafe` in some PAC crates, but not others.
// Disable these warnings.
#![allow(unused_unsafe)]
// The `doc_cfg` feature allows us to show functionality that is feature-gated on `docs.rs`.
// todo: Re-implement the doc_cfg feature and the relevant tags (From all modules that impl EH traits)
// todo oncoe this is in stable.
// #![feature(doc_cfg)]

// todo: H7B3 has too many changes in v14 PAC; not supporting at this time. (2021-10-07)

// Used for while loops, to allow returning an error instead of hanging.
pub(crate) const MAX_ITERS: u32 = 300_000; // todo: What should this be?

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
    feature = "l412",
    feature = "l4x3",
    feature = "l4x5",
    feature = "l4x6",
    feature = "l552",
    feature = "l562",
    feature = "g030",
    feature = "g031",
    feature = "g041",
    feature = "g050",
    feature = "g051",
    feature = "g061",
    feature = "g070",
    feature = "g071",
    feature = "g081",
    feature = "g0b0",
    feature = "g0b1",
    feature = "g0c1",
    feature = "g431",
    feature = "g441",
    feature = "g471",
    feature = "g473",
    feature = "g474",
    feature = "g483",
    feature = "g484",
    feature = "g491",
    feature = "g4a1",
    feature = "h503",
    feature = "h562",
    feature = "h563",
    feature = "h573",
    feature = "h735",
    feature = "h743",
    feature = "h743v",
    feature = "h747cm4",
    feature = "h747cm7",
    feature = "h753",
    feature = "h753v",
    feature = "h7b3",
    feature = "wb55",
    feature = "wle5",
)))]
compile_error!("This crate requires an MCU-specifying feature to be enabled. eg `l552`.");

// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for
// stm32 peripherals.

// todo: U5 once SVD is out.
use cfg_if::cfg_if;
use cortex_m::{self, delay::Delay};
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
// todo: Test and make accomodations for recently added G0 variants 50, 51, 61, B0, B1 and C1, in
// todo the individual modules.

// G0 PAC
#[cfg(feature = "g030")]
pub use stm32g0::stm32g030 as pac;
#[cfg(feature = "g031")]
pub use stm32g0::stm32g031 as pac;
#[cfg(feature = "g041")]
pub use stm32g0::stm32g041 as pac;
#[cfg(feature = "g050")]
pub use stm32g0::stm32g050 as pac;
#[cfg(feature = "g051")]
pub use stm32g0::stm32g051 as pac;
#[cfg(feature = "g061")]
pub use stm32g0::stm32g061 as pac;
#[cfg(feature = "g070")]
pub use stm32g0::stm32g070 as pac;
#[cfg(feature = "g071")]
pub use stm32g0::stm32g071 as pac;
#[cfg(feature = "g081")]
pub use stm32g0::stm32g081 as pac;
#[cfg(feature = "g0b0")]
pub use stm32g0::stm32g0b0 as pac;
#[cfg(feature = "g0b1")]
pub use stm32g0::stm32g0b1 as pac;
#[cfg(feature = "g0c1")]
pub use stm32g0::stm32g0c1 as pac;
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
// H5 PAC
#[cfg(feature = "h503")]
pub use stm32h5::stm32h503 as pac;
#[cfg(feature = "h562")]
pub use stm32h5::stm32h562 as pac;
#[cfg(feature = "h563")]
pub use stm32h5::stm32h563 as pac;
#[cfg(feature = "h573")]
pub use stm32h5::stm32h573 as pac;
// H7 PAC
#[cfg(feature = "h735")]
pub use stm32h7::stm32h735 as pac;
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
#[cfg(feature = "l412")]
pub use stm32l4::stm32l412 as pac;
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
#[cfg(feature = "wb55")]
pub use stm32wb::stm32wb55 as pac;
#[cfg(feature = "wle5")]
pub use stm32wl::stm32wle5 as pac;

#[cfg(not(any(feature = "f301", feature = "f302")))]
pub mod adc;

// bxCAN families: F3, F4, L4,
// fdCAN families: L5, U5, G4, H7
// H7 suppords fd and can_ccu. (What's that?)
// WB and WL?
#[cfg(all(
    any(feature = "can_bx", feature = "can_fd_g", feature = "can_fd_h"),
    // not(any(feature = "f301", feature = "f401", feature = "f410", feature = "f411"))
))]
pub mod can;

// For now, we're using the `fdcan` crate
// #[cfg(any(feature = "g0c1", feature = "g4", feature = "h7"))]
// pub mod fd_can;

pub mod clocks;
// todo: You could get CRC working on these.
#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "wb",
    feature = "wl",
    feature = "h5", // todo: COme back to
)))]
pub mod crc;

#[cfg(not(any(
    feature = "f401",
    feature = "f411",
    feature = "f412",
    feature = "wb",
    feature = "g0",
    feature = "h5", // todo: H5 DAC pending PAC fix.
)))]
// WB doesn't have a DAC. Some G0 variants do - add it! Most F4 variants have it, some don't
pub mod dac;

#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "l4x1",
    feature = "l4x2",
    feature = "l412",
    feature = "l4x3",
    feature = "l4x5",
    feature = "g0",
    feature = "g4",
    feature = "wb",
    feature = "wl",
    feature = "h5", // todo: Check PAC.
// todo: DFSDM support for other platforms that don't support clustering
)))]
pub mod dfsdm;

#[cfg(not(any(feature = "f4", feature = "l552", feature = "h5")))]
pub mod dma;

#[cfg(all(feature = "h7", feature = "net"))]
pub mod ethernet;

#[cfg(not(feature = "h5"))] // todo: Come back to
pub mod flash;

// todo: PAC doesn't yet support these newer H7 MCUs that use FMAC.
// #[cfg(any(feature = "h723", feature = "h725", feature = "h733", feature = "h735"))]
// todo: Also G4.
// pub mod fmac;

pub mod gpio;

#[cfg(feature = "wb")]
pub mod hsem;

#[cfg(not(any(feature = "f4")))]
pub mod i2c;
#[cfg(feature = "f4")]
pub mod i2c_f4;
#[cfg(feature = "f4")]
pub use i2c_f4 as i2c;

#[cfg(feature = "wb")]
pub mod ipcc;

pub mod iwdg;

pub mod low_power;

#[cfg(any(feature = "h747cm4", feature = "h747cm7"))]
pub mod power;

// F3, F4, G0, and WL don't have Quad SPI. L5 and newer H variants (eg H735) use OctoSPI,
// also supported by this module.
#[cfg(not(any(
feature = "f3",
feature = "f4",
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
)))]
pub mod qspi;

// Note: Some F4 variants support RNG, but we haven't figured out the details yet. Send a PR if interested.
#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "g030",
    feature = "g031",
    feature = "g070",
    feature = "g071",
    feature = "g0b1",
    feature = "g0c1",
)))]
pub mod rng;

pub mod rtc;

#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "g0",
    // feature = "g4", // todo: G4 PAC issue re getting channel-specific reg blocks.
    feature = "h7b3",
    feature = "wl",
    feature = "h5", // todo
)))]
pub mod sai;

#[cfg(not(feature = "h5"))] // todo: Add H5 SPI!
pub mod spi;

#[cfg(not(feature = "h5"))] // todo temp
pub mod timer;

// #[cfg(not(feature = "h5"))] // todo temp. Needs CR1 and ISR added, among other things.
pub mod usart;

#[cfg(any(
    feature = "l4",
    // feature = "g4",
    feature = "g473", // todo: Not compiling on G431
    feature = "h7"
))]
pub mod comp;

// See note at top of `usb` module for info on G0; not avail on modules the PAC has avail.
cfg_if! {
    if #[cfg(all(
        feature = "usb",
        all(
            any(
                feature = "f303",
                feature = "l4x2",
                feature = "l412",
                feature = "l4x3",
                feature = "l4x5",
                feature = "l5",
                feature = "g4",
                feature = "wb",
            ),
        not(feature = "g4a1"))
    ))] {
        pub mod usb;
    } else if #[cfg(all(
        // H7 has HS (high-speed), while F4 and L4 have FS. The names get confusing, starting
        // on ST's side.
        any(feature = "usbotg_fs", feature = "usbotg_hs"),
        any(feature = "f4", feature = "l4x6", feature = "h7"),
        not(feature = "f410")
    ))] {
        pub mod usb_otg;
        pub use usb_otg as usb;
    }
}

// For use with timers; converting ticks to real time.
pub mod instant;
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
    ($(($NAME:ident, $type:ty, $val:expr)),+ $(,)?) => {
        $(
            static $NAME: Mutex<Cell<$type>> = Mutex::new(Cell::new($val));
        )+
    };
}

/// Syntax helper for parsing multi-byte fields into primitives.
///
/// Example: `parse_le!(bytes, i32, 5..9);`
#[macro_export]
macro_rules! parse_le {
    ($bytes:expr, $t:ty, $range:expr) => {{
        <$t>::from_le_bytes($bytes[$range].try_into().unwrap())
    }};
}

/// Syntax helper for parsing multi-byte fields into primitives.
///
/// Example: `parse_be!(bytes, i32, 5..9);`
#[macro_export]
macro_rules! parse_be {
    ($bytes:expr, $t:ty, $range:expr) => {{
        <$t>::from_be_bytes($bytes[$range].try_into().unwrap())
    }};
}

/// Syntax helper for converting primitives to multi-byte fields.
///
/// Example: `copy_le!(bytes, self.position, 5..9);`
#[macro_export]
macro_rules! copy_le {
    ($dest:expr, $src:expr, $range:expr) => {{
        $dest[$range].copy_from_slice(&$src.to_le_bytes())
    }};
}

/// Syntax helper for converting primitives to multi-byte fields.
///
/// Example: `copy_be!(bytes, self.position, i32, 5..9);`
#[macro_export]
macro_rules! copy_be {
    ($dest:expr, $src:expr, $range:expr) => {{
        $dest[$range].copy_from_slice(&$src.to_be_bytes())
    }};
}

// todo: Remove this debug_workaroudn function on MCUs that don't require it. Ie, is this required on G4? G0?
#[cfg(not(any(feature = "g0")))]
/// Workaround due to debugger disconnecting in WFI (and low-power) modes.
/// This affects most (all?) STM32 devices. In production on battery-powered
/// devices that don't use DMA, consider removing this, to prevent power
/// use by the DMA clock.
/// For why we enable the DMA clock, see STM32F446 errata, section 2.1.1.
pub fn debug_workaround() {
    let dbgmcu = unsafe { &(*pac::DBGMCU::ptr()) };

    cfg_if! {
        if #[cfg(all(feature = "h7", not(any(feature = "h747cm4", feature = "h747cm7"))))] {
            dbgmcu.cr.modify(|_, w| w.dbgsleep_d1().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbgstop_d1().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbgstby_d1().set_bit());
        } else if #[cfg(feature = "h7")] {
            dbgmcu.cr.modify(|_, w| w.dbgslpd1().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbgstpd1().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbgstbd1().set_bit());
        } else {
            #[cfg(not(any(feature = "l5", feature = "h5")))]
            dbgmcu.cr.modify(|_, w| w.dbg_sleep().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbg_stop().set_bit());
            dbgmcu.cr.modify(|_, w| w.dbg_standby().set_bit());
        }
    }

    let rcc = unsafe { &(*pac::RCC::ptr()) };

    // todo Some MCUs may need the dbgmcu lines, but not DMA enabled.
    // todo: Remove this part on MCUs not affected. F4 and L4 are confirmed affected.

    cfg_if! {
        if #[cfg(feature = "f3")] {
            rcc.ahbenr.modify(|_, w| w.dma1en().set_bit());
        } else if #[cfg(feature = "h5")] {
            rcc.ahb1enr.modify(|_, w| w.gpdma1en().set_bit());
        } else {
            rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
        }
    }
}

/// A blocking delay, for a specified time in ms.
pub fn delay_ms(num_ms: u32, ahb_freq: u32) {
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, ahb_freq);
    delay.delay_ms(num_ms);
}

/// A blocking delay, for a specified time in μs.
pub fn delay_us(num_us: u32, ahb_freq: u32) {
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, ahb_freq);
    delay.delay_us(num_us);
}

/// In the prelude, we export helper macros.
pub mod prelude {
    pub use access_global;
    pub use access_globals;
    pub use make_globals;
    pub use make_simple_globals;
}
