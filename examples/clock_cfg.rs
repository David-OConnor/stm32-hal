//! For project structure and debugging boilerplate, see the `synax_overview` example.
//! Clock config varies significantly by family
//! Reference the Cube Mx interactive clock tree tool, or the clock tree in the reference
//! manual for a visual overview.

#![no_main]
#![no_std]

use cortex_m_rt::entry;

use stm32_hal2::{
    clocks::{ApbPrescaler, Clocks, InputSrc, Pllm},
    low_power, pac,
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let mut clock_cfg = Clocks::default();

    // Bypass HSE output
    clock_cfg.hse_bypass = true;

    // Enable HSI48 (eg L4, L5, G4 etc)
    clock_cfg.hse48_on = true;

    // Set HSE as the input source, with frequency in Hz
    clock_cfg.input_src = InputSrc::Hse(8_000_000);

    // Change  PLL prescalers:
    clock_cfg.pllm = Pllm::Div2;
    clock_cfg.plln = 22;

    // Change some of the peripheral prescalers
    clock_cfg.apb1prescaler = ApbPrescaler::Div2;

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
