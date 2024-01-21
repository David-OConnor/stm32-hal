//! For project structure and debugging boilerplate, see the `synax_overview` example.
//! Clock config varies significantly by family
//! Reference the Cube Mx interactive clock tree tool, or the clock tree in the reference
//! manual for a visual overview. The above code lines are not valid for all STM32 families
//! supported by this crate.
//!
//! Note that the Rust docs are built for L4, and may not be accurate for other MCUs.

#![no_main]
#![no_std]

use cortex_m_rt::entry;
use hal::{
    clocks::{self, ApbPrescaler, Clocks, InputSrc, MsiRng, PllCfg, PllSrc, Pllm, Pllr},
    low_power, pac,
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // Set up a default setting. See documentation on Rust docs for details about
    // what this does (it depends on the MCU), but it usually runs the core and most
    // peripheral clocks at the max rated speed, using HSI as the input source.
    // Unfortunately, the docs hosted on `docs.rs` can only show a single variant, so
    // will likely be innacurate. Inspect the [clocks modules here](https://github.com/David-OConnor/stm32-hal/tree/main/src/clocks)
    // for details.

    // The default clock setting will (for most variants) enable the (or a) PLL, and configure
    // scalers so system clock is at its maximum speed, using the internal oscillator (HSE).
    // Most H7 variants default to their non-boosted
    // speed, ie either 400Mhz or 520Mhz (not 480 or 550Mhz).
    let mut clock_cfg = Clocks::default();

    // An example modifying some settings, while leaving the rest default. This is the intended
    // pattern.
    let clock_cfg = Clocks {
        input_src: InputSrc::Hsi,
        stop_wuck: StopWuck::Hsi,
        ..Default::default()
    };

    // Here's an example clock config, where you're using a 16Mhz SMD oscillator (meaning we can enable
    // HSE bypass), and are using the internal HSI48 for USB. This also demonstrates how to configure
    // an external oscillator. (HSE)
    let clock_cfg = Clocks {
        input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
        hse_bypass: true,
        hsi48_on: true,
        clk48_src: Clk48Src::Hsi48,
        ..Default::default()
    };

    // If you'd prefer, you can modify fields individually from a mutable clock config, instead
    // of using the code pattern above.

    // Set it up to use the HSI directly, with no PLL. This will result in a reduced speed:
    clock_cfg.input_src = InputSrc::Hsi;

    // Set up to use a 8MHze HSE, with no PLL.
    clock_cfg.input_src = InputSrc::Hse(8_000_000);

    // If you set a 8Mhz HSE as above, you may need to reduce the default PLLM value to compensate
    // compared to a 16Mhz HSI:
    clock_cfg.pll.divm = Pllm::Div2;

    // Enable PLLQ etc
    clocks_cfg.pll.pllp_en = true;
    clocks_cfg.pll.divp = Pllr::Div8;

    // Note that on H7, we have 3 separate PLLs we can configure, at the `pll1`, `pll2`, and `pll3` fields.
    // L4 and WB have a second PLL, called `PLLSAI`, primarily intended for the SAI audio peripheral.
    // Its config field is `pllsai`. (and `pllsai2` for L4x5 and L4x6).
    // For example, here's a PLL config on H7 that sets PLL2P as the SAI2 source, and configures
    // its speed. (By default, only PLL1(R) is enabled.
    let clock_cfg = Clocks {
        pll2: PllCfg {
            enabled: true,
            pllp_en: true,
            divn: 99,
            divp: 16,
            ..PllCfg::disabled()
        },
        sai1_src: SaiSrc::Pll2P,
        ..Default::default()
    };

    // Or on L4 or WB, using the PLLSAI:
    let clock_cfg = Clocks {
        pllsai1: PllCfg {
            enabled: true,
            pllp_en: true,
            divn: 32,
            ..PllCfg::disabled()
        },
        sai1_src: SaiSrc::PllSai1P, // Note that thsi is the default, but you can change it.
        ..Default::default()
    };

    // Note that H7 also lets you select VOS range, and will give you an error if you select an
    // invalid range for your HCLK speed. VOS0 is required for full speed.
    // clock_cfg.vos_range = VosRange::VOS0;
    // You can use `Clocks::full_speed()` to configure an H743 etc at 480Mhz.

    // Change the default wakeup from Stop mode to be HSI instead of MSI. (L4 and L5 only)
    clock_cfg.stop_wuck = StopWuck::Hsi;

    // Set up PLL using a 4Mhz HSE:
    clock_cfg.input_src = InputSrc::Pll(PllSrc::Hse(4_000_000));

    // Enable the Clock Security System (CSS)
    clock_cfg.security_system = true;

    // Bypass HSE output
    clock_cfg.hse_bypass = true;

    // Enable HSI48 (eg L4, L5, G4 etc)
    clock_cfg.hse48_on = true;

    // Change MSI speed (L4, L5 only)
    clock_cfg.change_msi_speed(MsiRange::R2M);

    // (L4 and L5 only) If you'd like to use MSI for the USB clock source, run this function.
    // Do not run it if using MSI for the input source or PLL source. You must also have
    // `clk48_src: Clk48Src::MSI` in the clock cfg, which is the default for L4 and L5.
    clocks_cfg.enable_msi_48();

    // Change  PLL prescalers:
    clock_cfg.pllm = Pllm::Div4;
    clock_cfg.plln = 22;
    clock_cfg.pllr = Pllm::Div4;

    // Change some of the peripheral prescalers
    clock_cfg.apb1prescaler = ApbPrescaler::Div2;

    // Enable the Clock Recovery System (CRS), to automatically trim the HSI48 on variants
    // that include it. (eg STM32l4x2 and L4x3, L5, G4, and H7)
    clocks::enable_crs(CrsSyncSrc::Usb); // CrsSyncSrc::OtgHs on H7.

    // If you need to modify functionality not supported by this library,
    // you can make register writes directly  using the PAC. If you find missing functionality
    // you find useful, consider making an issue or PR on Github.
    // For example, to set I2C1 to use HSI as its source, on L4:
    dp.RCC
        .ccipr
        .modify(|_, w| unsafe { w.i2c1sel().bits(0b10) });

    // Configure clock registers. The previous creation and modification of `clock_cfg`
    // only set up a configuration struct; `Clocks::setup` performs the MCU operations.
    clock_cfg.setup().unwrap();

    // Show speeds.
    defmt::println!("Speeds: {:?}", clock_cfg.calc_speeds());

    loop {
        low_power::sleep_now();
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
