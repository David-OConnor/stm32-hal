#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal::{
    clocks::{ClockCfg, Clocks},
    dac::{Dac, Channel as DacChannel, Bits as DacBits},
    delay::Delay,
    event::Timeout,
    flash::Flash,
    pac,
    rtc::{Rtc, RtcClockSource, RtcConfig},
    timer::{Event::TimeOut, Timer},
};

use embedded_hal::prelude::*;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clocks = Clocks::default();

    if clocks.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    // Setup a delay, based on the cortex_m systick.
    let mut delay = Delay::new(cp.SYST, &clocks);

    // Set up the realtime clock.
    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.RCC,
        &mut dp.PWR,
        RtcConfig::default().clock_source(RtcClockSource::Lse), // .bypass_lse_output(true),
    );

    // Read from and write to the flash memory:
    let mut flash = Flash::new(dp.FLASH);

    flash.erase_page(FLASH_PAGE).ok();
    flash.write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    // Set up an I2C peripheral.
    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100_000, &clocks, &mut dp.RCC);

    // Set up the Digital-to-analog converter
    let mut dac = Dac::new(dp.DAC, dac_pin, DacChannel::One, Bits::TwelveR, 3.3);
    dac.enable(&mut dp.RCC);

    // Set up and start a timer; set it to fire interrupts.
    let mut timer_1 = Timer::tim3(dp.TIM3, 0.2, &clocks, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    loop {}
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
