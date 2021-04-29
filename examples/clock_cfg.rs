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

use stm32_hal2::{
    clocks::{self, ApbPrescaler, Clocks, InputSrc, MsiRng, PllSrc, Pllm, Pllr},
    low_power, pac,
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // Set up a default setting.  See documentation on Rust docs for details about
    // what this does (it depends on the MCU), but it usually runs the core and most
    // peripheral clocks at the max rated speed, using HSI as the input source.

    // For details on what these fields are, and
    let mut clock_cfg = Clocks::default();

    // Or something like this to change a few of the defaults
    let clock_cfg = Clocks {
        input_src: InputSrc::Hsi,
        stop_wuck: StopWuck::Hsi,
        ..Clocks::default()
    };

    // Set it up to use the HSI, with no PLL. This will result in a reduced speed:
    clock_cfg.input_src = InputSrc::Hsi;

    // Set it up to use a 8MHze HSE, with no PLL. This will result in a reduced speed:
    clock_cfg.input_src = InputSrc::Hse(8_000_000);

    // If you set a 8Mhz HSE as above, you may need to reduce the default PLLM value to compensate
    // compared to a 16Mhz HSI:
    clock_cfg.pllm = Pllm::Div2;

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
    clock_cfg.change_msi_speed(MsiRange::R2M, &mut dp.RCC);

    // (L4 and L5 only) If you'd like to use MSI for the USB clock source, run this function.
    // Do not run it if using MSI for the input source or PLL source. You must also have
    // `clk48_src: Clk48Src::MSI` in the clock cfg, which is the default for L4 and L5.
    clocks_cfg.enable_msi_48(&mut dp.RCC);

    // Change  PLL prescalers:
    clock_cfg.pllm = Pllm::Div4;
    clock_cfg.plln = 22;
    clock_cfg.pllr = Pllm::Div4;

    // Change some of the peripheral prescalers
    clock_cfg.apb1prescaler = ApbPrescaler::Div2;

    // If you need to modify functionality not supported by this library,
    // you can make register writes directly  using the PAC. If you find missing functionality
    // you find useful, consider making an issue or PR on Github.
    // For example, to set I2C1 to use HSI as its source, on L4:
    dp.RCC
        .ccipr
        .modify(|_, w| unsafe { w.i2c1sel().bits(0b10) });

    // Configure clock registers.
    if clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Show speeds.
    defmt::info!("Speeds: {:?}", clock_cfg.calc_speeds());

    loop {
        low_power::sleep_now(&mut SCB);
    }
}
