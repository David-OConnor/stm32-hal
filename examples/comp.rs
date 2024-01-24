//! This module includes an overview of Comparator features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

// These lines are part of our setup for debug printing.
// Cortex-M Import
use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use defmt_rtt as _;
// Importing library
use hal::{
    clocks::Clocks,
    comp::{self, Comp, CompConfig, CompDevice},
    gpio::{Pin, PinMode, Port},
    pac,
};
use panic_probe as _;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    // Setting Up Clock
    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    // Setting Up GPIO (Not really needed)
    let _pin = Pin::new(Port::B, 2, PinMode::Analog);

    // Setting Up Delay
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Setting Up Comparator
    // Comparator Configuration
    let cfg = CompConfig {
        // No Hysterysis
        hyst: comp::Hysterisis::NoHysterisis,
        // Using internal Vref as negative input
        // e.g. (1.22V) in STM32L47xx, STM32L48xx, STM32L49xx and STM32L4Axx.
        // Consult Reference Manual for all negative input.
        inmsel: comp::InvertingInput::Vref,
        // Using Io2 as positive input
        // e.g. (PB2) for COMP1 in STM32L47xx, STM32L48xx, STM32L49xx and STM32L4Axx.
        // Consult Reference Manual for all positive input.
        inpsel: comp::NonInvertingInput::Io2,
        // Don't invert output high when inverting input < noninverting and etc.
        polarity: comp::OutputPolarity::NotInverted,
        // High Power Consumption (lowest propagation delay)
        pwrmode: comp::PowerMode::HighSpeed,
    };
    // Creating Comparator device using COMP1
    let mut comparator = Comp::new(CompDevice::One, cfg);
    // Starting Comparator
    comparator.start().unwrap();

    loop {
        // Reading and Printing Output
        let output = comparator.get_output_level();
        defmt::println!("{}", output);
        delay.delay_ms(1000u32);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
