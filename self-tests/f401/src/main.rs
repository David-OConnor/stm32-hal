//! This minimial example causes an LED to blink using a (blocking) systick delay. It's
//! the canonical "Hello world" of embedded programming. It demonstrates project structure,
//! printing text to the console, using systick delays, and setting GPIO state.

#![deny(warnings)]
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime
use rtt_target;

use hal::{
    self,
    clocks::Clocks,
    gpio::{Pin, PinMode, Port},
    pac,
};

// Import the panic handler
use panic_probe as _;

// This marks the entrypoint of our application.

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    defmt::println!("Hello, world!");

    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clock_cfg` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/latest/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    let mut led = Pin::new(Port::A, 5, PinMode::Output);

    loop {
        led.set_low();
        defmt::debug!("Output pin is low.");
        delay.delay_ms(1_000);
        led.set_high();
        defmt::debug!("Output pin is high.");
        delay.delay_ms(1_000);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
