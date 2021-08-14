#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime
use stm32_hal2::{
    self,
    clocks::{Clocks},
    gpio::{Pin, PinMode, Port},
    pac,
};

use defmt_rtt as _;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    stm32_hal2::debug_workaround();

    defmt::info!("Hello, world!");

    // once your stm32-hal2 version is >= 0.2.12, you can safely switch
    // to the following initialization:
    //
    // let clock_cfg = Clocks::default();
    //
    // until then, you have to use this one (otherwise the code will crash):
    let clock_cfg = Clocks {
        plln: 84,
        ..Default::default()
    };
    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    let mut led = Pin::new(Port::C, 13, PinMode::Output);

    // Now, enjoy the lightshow!
    loop {
        defmt::debug!("Our demo is alive");
        led.set_low();
        delay.delay_ms(1000_u32);
        led.set_high();
        delay.delay_ms(1000_u32);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
