# STM32-HAL

[![Crate](https://img.shields.io/crates/v/stm32-hal2.svg)](https://crates.io/crates/stm32-hal2)
[![Docs](https://docs.rs/stm32-hal2/badge.svg)](https://docs.rs/stm32-hal2)

This library provides high-level access to STM32 peripherals.

## Requirements
1. Provide high-level access to most STM32 peripherals
2. Support these STM32 families: `F3`, `F4`, `L4`, `L5`, `G`, `H`, `U`, and `W`
3. Allow switching MCUs with minimal code change
4. Provide a consistent API across peripheral modules
5. Support both DMA and non-DMA interfaces
6. Be suitable for commercial projects
7. Provide a clear, concise API
8. Provide source code readable by anyone cross-checking a reference manual (RM)


## Specifications
- Base code on instructions described in reference manuals (RM); document inline with
  the relevant excerpts [4, 8]
- Use [STM32 Peripheral Access Crates](https://github.com/stm32-rs/stm32-rs) to allow high-level
  register access [2]
- Wrap PAC register blocks in structs that represent the applicable peripheral, and access features
  of these peripherals using public methods [1]
- Use `#[cfg]` blocks, and the `cfg_if!` macro to handle differences between MCUs; use separate modules
  where large differences exist [2, 3]
- Favor functionality, ergonomics, and explicit interfaces [6, 7]
- Document configuration code with what registers and fields it sets, and desriptions from RMs [4, 8]
- Provide examples and documentation that demonstrate peripheral use with interrupts and DMA [6]


## Supported MCUs
F3, F4, L4, L5, G0, G4, H5, H7, WB, and WL. U5 is planned once its SVD files and PAC
become available.

Tested on the following devices:
- STM32F303
- STM32F401, F411
- STM32L476, L433, L443, L412, L432
- STM32L552
- STM32WB5MMG
- STM32G431, G491, G473
- STM32H743(V), H745 (both cores)


## Getting started

### Quickstart
- [Install Rust](https://www.rust-lang.org/tools/install).
- Install the compilation target for your MCU. Eg run `rustup target add thumbv7em-none-eabihf`. You'll need to change the last part if using a Cortex-M0, Cortex-M33, (Eg Stm32G0 or L5 respectively) or if you don't want to use hardware floats.
- Install flash and debug tools: `cargo install flip-link`, `cargo install probe-rs --features cli`.
- Clone the [quickstart repo](https://github.com/David-OConnor/stm32-hal-quickstart1): `git clone https://github.com/David-OConnor/stm32-hal-quickstart1`.
- Change the following lines to match your MCU. Post an issue if you need help with this:
    - `Cargo.toml`: `hal = { package = "stm32-hal2", version = "^1.5.0", features = ["l4x3", "l4rt"]}`
    - `memory.x`: `FLASH` and `RAM` lines
    - `.cargo/config.toml`: `runner` and `target` lines.
- Connect your device. Run `cargo run --release` to compile and flash.


### Details
Review the [syntax overview example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/syntax_overview)
for example uses of many of this library's features. Copy and paste its whole folder (It's set up
using [Knurling's app template](https://github.com/knurling-rs/app-template)), or copy parts of `Cargo.toml`
and `main.rs` as required.

The [blinky example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/blinky) provides a detailed example and instructions for how to set up a blinking
light (ie hello world) using an STM32F411 "blackpill" board. Its readme provides instructions for how to get
started from scratch, and its code contains detailed comments explaining each part. The 
[blinky with timer interrupt example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/blinky_timer_interrupt)
demonstrates how to accomplish the same in a non-blocking way, using a hardware timer. It uses RTIC.

The [conductivity module example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/conductivity_module)
is a complete example of simple production firmware. It uses the DAC, I2C, Timer, and UART peripherals,
with a simple interupt-based control flow.

The [PDM mic, DAC output passthrough example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/pdm_mic_dac_output.rs)
demonstrates how to read audio from a digital microphone, output it to headphones or speakers using the DAC, and use DMA
to do this efficiently. It conducts minimal processing, but can be modified to process using DSP between input and output.
This example uses RTIC.

The [SPI IMU filtered example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/spi_imu_filtered.rs)
demonstrates how to configure an external IMU (3-axis accelerometer and gyroscope) over SPI, read from multiple
registers using a single SPI transfer using DMA as soon as data is ready, and apply digital filtering using the
CMSIS-DSP library. This example uses RTIC.

Additional examples in the [examples folder](https://github.com/David-OConnor/stm32-hal/tree/main/examples) demonstrate
how to use various STM32 peripherals; most of these examples focus on a single peripheral.

When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
include a run-time feature, following the template `l4rt`. For example:
```toml
cortex-m = { version = "^0.7.7", features = ["critical-section-single-core"] }
cortex-m-rt = "0.7.2"
hal = { package = "stm32-hal2", version = "^1.5.5", features = ["l4x3", "l4rt"]}
```

If you need `embedded-hal` traits, include the `embedded_hal` feature.

You can review [this section of Cargo.toml](https://github.com/David-OConnor/stm32-hal/blob/main/Cargo.toml#L61)
to see which MCU and runtime features are available.

### Example highlights:
```rust
use cortex_m;
use cortex_m_rt::entry;
use hal::{
    clocks::Clocks,
    gpio::{Pin, Port, PinMode, OutputType},
    i2c::I2c,
    low_power,
    pac,
    timer::{Timer, TimerInterrupt},
};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    let mut pb15 = Pin::new(Port::A, 15, PinMode::Output);
    pb15.set_high();

    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, Default::default(), &clock_cfg);
    timer.enable_interrupt(TimerInterrupt::Update);

    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    let mut dma = Dma::new(dp.DMA1);
    dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::I2c1Tx);
  
    let i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);

    loop {
        i2c.write(0x50, &[1, 2, 3]);
        // Or:       
        i2c.write_dma(0x50, &BUF, DmaChannel::C1, Default::default(), DmaPeriph::Dma1);
      
        low_power::sleep_now();
    }
}
```

## API 

The API for most peripherals has these methods:

- `new()` This accepts a PAC register struct, and usually a Config struct.
- `enable_interrupt()` Accepts an enum of interrupt types.
- `clear_interrupt()` Accepts an enum of interrupt types.
- `read_status()` Returns the peripheral's status register as an integer. Compare to the Reference manual, eg after converting to binary.
- `read()`, `write()` etc as required: Blocking
- `read_dma()`, `write_dma()`, etc as required: Starts a DMA transfer that should be cleaned up in an ISR

Specific peripherals have different functionality, as required. Reference the docs for details.


## Compatible with RTIC
[Real-Time Interrupt-driven Concurrency](https://rtic.rs/0.5/book/en/) is
a light-weight framework that manages safely sharing state between contexts. Eg between ISRs and the main loop.
Some examples use global `Mutex`es, `RefCell`s, and `Cell`s; others use macros to simplify syntax; 
some use RTIC.

Supports the RTIC `Monotonic` trait. To enable, use the `monotonic` feature.

## Why this is different from `stm32yxx-hal` libraries
- Works with multiple STM32 families, with identical syntax when able
- Simpler syntax
- Doesn't use typestates, and less reliance on ownership of peripheral abstractions
- Doesn't rely on `embedded-hal` traits; treats them as an optional add-on
- Different approach to DMA
- Explicit clock config
- Detailed, consistent code documentation, with reference manual excerpts and references

If you'd like to learn more about the other HALs, check them out on the [stm32-rs Github](https://github.com/stm32-rs).
You may prefer them if you prioritize strict type checks on GPIO pins and other hardware, for example.


## Docs caveat
The Rust docs page is built for `STM32H735`, and some aspects are not accurate for other
variants. We currently don't have a good solution to this problem, and may
self-host docs in the future, with a separate page for each STM32 family.


## Contributing

PRs are encouraged. Documenting each step using reference manuals is encouraged where applicable.

Most peripheral modules use the following format:

- Enums for various config settings, that implement `#[repr(u8)]` for their associated register values
- A peripheral struct that has public fields for config. This struct also includes
  a private `regs` field that is the appropriate reg block. Where possible, this is defined generically
  in the implementation, eg:
  `U: Deref<Target = pac::usart1::RegisterBlock>`. Reference the [stm32-rs-nightlies Github](https://github.com/stm32-rs/stm32-rs-nightlies)
  to identify when we can take advantage of this.
- If config fields are complicated, we use a separate `PeriphConfig` struct owned by the peripheral struct.
  This struct impls `Default`.
- A constructor named `new` that performs setup code
- `enable_interrupt` and `clear_interrupt` functions, which accept an enum of interrupt type.
- Add `embedded-hal` implementations as required, that call native methods. Note that
  we design APIs based on STM32 capabilities, and apply EH traits as applicable. We only
  expose these implementations if the `embedded_hal` feature is selected.
- When available, base setup and usage steps on instructions provided in Reference Manuals.
  These steps are copy+pasted in comments before the code that performs each one.
- Don't use PAC convenience field settings; they're implemented inconsistently across PACs.
  (eg don't use something like `en.enabled()`; use `en.set_bit()`.)
- If using a commonly-named configuration enum like `Mode`, prefix it with the peripheral type,
  eg use `RadarMode` instead. This prevents namespace conflicts when importing the enums directly.


### Example module structure:
```rust
#[derive(clone, copy)]
#[repr(u8)]
/// Select pulse repetition frequency. Sets `FCRDR_CR` register, `PRF` field.
enum Prf {
    /// Medium PRF (less than 10Ghz)
    Medium = 0,
    /// High PRF (10Ghz or greater)
    High = 1,
}

#[derive(clone, copy)]
/// Available interrupts. Enabled in `FCRDR_CR`, `...IE` fields. Cleared in `FCRDR_ICR`.
enum FcRadarInterrupt {
    /// Target acquired, and the system is now in tracking mode.
    TgtAcq,
    /// Lost the track, for any reason.
    LostTrack,
}

/// Represents a Fire Control Radar (FCR) peripheral.
pub struct FcRadar<R> {
    // (`regs` is public, so users can use the PAC API directly, eg for unsupported features.)
    pub regs: R, 
    pub prf: Prf,
}

impl<F> FcRadar<R>
where
    R: Deref<Target = pac::fcrdr1::RegisterBlock>,
{
    /// Initialize a FCR peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, prf: Prf) -> Self {
        // (A critical section here prevents race conditions, while preventing
        // the user from needing to pass RCC in explicitly.)
        let mut rcc = unsafe { &(*RCC::ptr()) };
        rcc_en_reset!(apb1, fcradar1, rcc);

        regs.cr.modify(|_, w| w.prf().bit(prf as u8 != 0));        

        Self { regs, prf }
    }

    /// Track a target. See H8 RM, section 3.3.5: Tracking procedures.
    pub fn track(&mut self, hit_num: u8) -> Self {
        // RM: "To begin tracking a target, perform the following steps:"

        // 1. Select the hit to track by setting the HIT bits in the FCRDR_TR register. 
        #[cfg(feature = "h8")]
        self.regs.tr.modify(|_, w| unsafe { w.hit().bits(hit_num) });
        #[cfg(feature = "g5")]
        self.regs.tr.modify(|_, w| unsafe { w.hitn().bits(hit_num) });

        // 2. Begin tracking by setting the TRKEN bit in the FCRDR_TR register.
        self.regs.tr.modify(|_, w| w.trken().set_bit());

        // In tracking mode, the TA flag can be monitored to make sure that the radar
        // is still tracking the target.
    }
    
    /// Enable an interrupt.
    pub fn enable_interrupt(&mut self, interrupt: FcRadarInterrupt) {
        self.regs.cr.modify(|_, w| match interrupt {
            FcRadarInterrupt::TgtAcq => w.taie().set_bit(),
            FcRadarInterrupt::LostTrack => w.ltie().set_bit(),
        });
    }

    /// Clear an interrupt flag - run this in the interrupt's handler to prevent
    /// repeat firings.
    pub fn clear_interrupt(&mut self, interrupt: FcRadarInterrupt) {
        self.regs.icr.write(|w| match interrupt {
            FcRadarInterrupt::TgtAcq =>  w.tacf().set_bit(),
            FcRadarInterrupt::LostTrack => w.ltcf().set_bit(),
        });
    }
}
```

[This article](https://www.anyleaf.org/blog/writing-embedded-firmware-using-rust) provides some information
on using this library, as well as background information on Rust embedded in general.


## STM32WB and WL radio
This library doesn't include any radio functionality for the STM32WB. If you'd like to use it
with bluetooth, use this HAL in conjuction with with [@eupn](https://github.com/eupn)'s [stm32wb55](https://github.com/eupn/stm32wb55)
bluetooth library.

STM32WL radio support is WIP, and will be provided through interaction with newAM's
[stm32wl-hal](https://github.com/newAM/stm32wl-hal) library.


## Errata
- SDIO and ethernet unimplemented
- DMA unimplemented on F4, and L552
- H7 BDMA and MDMA unimplemented
- H5 GPDMA unimplemented
- USART interrupts unimplemented on F4
- CRC unimplemented for F4
- High-resolution timers (HRTIM), Low power timers (LPTIM), and low power usart (LPUSART) unimplemented
- ADC unimplemented on F4
- Low power modes beyond csleep and cstop aren't implemented for H7
- WB and WL are missing features relating to second core operations and RF
- L4+ MCUs not supported
- WL is missing GPIO port C, and GPIO interrupt support
- If using PWM (or output compare in general) on an Advanced control timer (eg TIM1 or 8),
you must manually set the `TIMx_BDTR` register, `MOE` bit.
- Octospi implementation is broken
- DFSDM on L4x6 is missing Filter 1.
- G0 and H7: Only FDCAN1 is implemented.
- H5 is missing a lot of functionality, including DMA.