#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal::{
    clocks,
    delay::Delay,
    event::Timeout,
    flash::Flash,
    pac,
    rtc::{Rtc, RtcClockSource, RtcConfig},
    timer::Timer,
};

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

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // todo: Temporary workaround due to a bug in `probe-rs`. Remove these lines once it's fixed.
    dp.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
    dp.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
    dp.DBGMCU.cr.modify(|_, w| w.dbg_standby().set_bit());

    let clocks = clocks::Clocks::hsi_preset();

    if clocks.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    let mut delay = Delay::new(cp.SYST, &clocks.systick());

    let mut rtc = Rtc::new(
        dp.RTC,
        &mut dp.RCC,
        &mut dp.PWR,
        RtcConfig::default().clock_source(RtcClockSource::Lse), // .bypass_lse_output(true),
    );

    // Read from adn write to the flash memory:
    let mut flash = Flash::new(dp.FLASH);

    flash.as_mut().unwrap().erase_page(FLASH_PAGE).ok();
    flash.as_mut().unwrap().write_page(10, &[1, 2, 3]).ok();

    let flash_contents = flash.read(10, 0);

    let mut timer_1 = Timer::tim3(dp.TIM3, 0.2, &clocks, &mut dp.RCC);
    timer.listen(TimeOut); // Enable update event interrupts.

    loop {}
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
