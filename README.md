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
7. Implement [embedded-hal](https://github.com/rust-embedded/embedded-hal) traits for all applicable peripherals
8. Provide a clear, concise API


## Specifications
- Base code on instructions described in reference manuals (RM); document inline with
the relevant excerpts [4]
- Use [STM32 Peripheral Access Crates](https://github.com/stm32-rs/stm32-rs) to allow high-level
register access [2]
- Wrap PAC register blocks in structs that represent the applicable peripheral, and access features
of these peripherals using public methods [1]
- Use `#[cfg]` blocks, and the `cfg_if!` macro to handle differences between MCUs; use separate modules
 where large differences exist [2, 3]
- Use both peripheral struct methods, and `embedded-hal` trait implementations for non-DMA interfaces; use additional
 struct methods for DMA interfaces [4, 5, 7]
- Favor functionality, ergonomics, and explicit interfaces [6, 8]
- Provide examples and documentation that demonstrate peripheral use with interrupts and DMA [6]


**Current family support**: F3, F4, L4, L5, G0, G4, H7, and WB. U5 is planned once its SVD files and PAC
become available. WL support is a WIP, with many features not implemented.


## Getting started
Review the [syntax overview example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/syntax_overview)
for example uses of many of this library's features. Copy and paste its whole folder (It's set up
using [Knurling's app template](https://github.com/knurling-rs/app-template)), or copy parts of `Cargo.toml` 
and `main.rs` as required.

When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
 include a run-time feature, following the template `l4rt`. For example: 
```toml
cortex-m = "0.7.3"
cortex-m-rt = "0.6.13"
stm32-hal2 = { version = "^0.2.10", features = ["l4x3", "l4rt"]}
```

If you need `embedded-hal` traits, include the `embedded-hal` feature.

You can review [this section of Cargo.toml](https://github.com/David-OConnor/stm32-hal/blob/main/Cargo.toml#L61)
to see which MCU and runtime features are available.

### Example highlights:
```rust
use cortex_m;
use cortex_m_rt::entry;
use stm32_hal2::{
    clocks::Clocks,
    gpio::{Pin, Port, PinMode, OutputType},
    i2c::{I2c, I2cDevice},
    low_power,
    timer::{Timer, TimerInterrupt},
};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).unwrap();

    let mut pb15 = Pin::new(Port::A, 15, PinMode::Output);
    pb15.set_high();

    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, &clock_cfg);
    timer.enable_interrupt(TimerInterrupt::Update);

    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clock_cfg);

    loop {
        low_power::sleep_now(&mut cp.SCB);
    }
}
```

## Why this module is different from `stm32yxx-hal` libraries
There are some areas where design philosophy is different. For example: GPIO 
type-checking, level-of-abstraction from registers/PAC, role of DMA, role of `embedded-hal` traits in the API, 
feature parity among STM32 families, code documentation, and clock config.

   
## Docs caveat
The Rust docs page is built for `STM32L4x3`, and some aspects are not accurate for other
variants. We currently don't have a good solution to this problem, and may
self-host docs in the future.


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
- `enable_interrupt` and `clear_interrupt` functions, which accept an enum of interrupt type
- Add `embedded-hal` implementations as required, that call native methods. Note that
we design APIs based on STM32 capabilities, and apply EH traits as applicable. We only
expose these implementations if the `embedded-hal` feature is selected.
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
/// Select pulse repetition frequency. Modifies `FCRDR_CR` register, `PRF` field.
enum Prf {
    Medium = 0,
    High = 1,
}

#[derive(clone, copy)]
/// Available interrupts. Enabled in `FCRDR_CR`, `...IE` fields. Cleared in `FCRDR_ICR`.
enum FcRadarInterrupt {
    TgtAcq,
    LostTrack,
}

/// Represents a Fire Control Radar (FCR) peripheral.
pub struct FcRadar<R> {
    regs: R,
    pub prf: Prf,
}

impl<F> FcRadar<R>
where
    R: Deref<Target = pac::fcrdr1::RegisterBlock>,
{
    pub fn new(regs: R, prf: Prf) -> Self {
        // A critical section here prevents race conditions, while preventing
        // the user from needing to pass RCC in explicitly.
        free(|cs| {
            let mut rcc = unsafe { &(*RCC::ptr()) };
            rcc_en_reset!(apb1, fcradar1, rcc);
        });

        regs.cr.modify(|_, w| w.prf().bit(prf as u8 != 0));        

        Self { regs, prf }
    }

    /// Track a target. See H8 RM, section 3.3.5.
    pub fn track(&mut self, hit_num: u8) -> Self {
        // RM: "To begin tracking a target, perform the following steps:"

        // 1. Select the hit to track by setting the HIT bits in the FCRDR_TR register. 
        #[cfg(feature = "h8")]
        self.regs.tr.modify(|_, w| unsafe { w.HIT().bits(hit_num) });
        #[cfg(feature = "g5")]
        self.regs.tr.modify(|_, w| unsafe { w.HITN().bits(hit_num) });

        // 2. Begin tracking by setting the TRKEN bit in the FCRDR_TR register.
        self.regs.tr.modify(|_, w| w.TRKEN().set_bit());

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

#[cfg(feature = "embedded-hal")]
/// Wrap our native methods with `embedded-hal` traits.
impl<F> embedded_hal::TargetTrack for FcRadar<F>
where
    F: Deref<Target = pac::fcrdr1::RegisterBlock>,
{
    type Error = Error;

    fn track(&mut self, track: u8) -> Result<(), Error> {
        FcRadar::track(self, track);
        Ok(())
    }
}
```


## STM32WB and WL radio
This library doesn't include any radio functionality for the STM32WB. If you'd like to use it
with bluetooth, use this HAL in conjuction with with @eupn's [stm32wb55](https://github.com/eupn/stm32wb55)
bluetooth library.

STM32WL radio support is WIP, and will be provided through interaction withnewAM's 
[stm32wl-hal](https://github.com/newAM/stm32wl-hal) library.


## Errata
- SAI, SDIO, ethernet unimplemented
- DMA unimplemented on F4 and H7
- The DMA2 peripheral is unimplemented
- DMA circular buffers unimplemented
- Only bxCAN is implemented - the fdCAN used on newer families is unimplemented
- USB unimplemented for H7
- USART synchronous mode, and auto-baud-rate detection unimplemented
- USART interrupts unimplemented on F4
- H7 clocks are missing advanced features
- PWM input unimplemented
- CRC unimplemented for L5, F4, G0, and G4
- Flash read/write unimplemented on H7
- Low power timers (LPTIM) and low power usart (LPUSART) unimplemented
- ADC unimplemented on F4
- ADC3 unimplemented on H7
- Low power modes beyond sleep and cstop aren't implemented for H7
- WB and WL are missing features relating to second core operations and RF
- WL is missing support for many peripherals
- L4+ MCUs not supported
