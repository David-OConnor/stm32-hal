//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicUsize, Ordering},
};

use cortex_m::{self, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use stm32_hal::{
    clocks::Clocks,
    low_power::{self, StopMode},
    pac,
    rtc::{Rtc, RtcClockSource, RtcConfig},
};

make_globals!((RTC, Rtc));

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    clock_cfg.setup().unwrap();

    // Set up the realtime clock.
    let mut rtc = Rtc::new(
        dp.RTC,
        RtcConfig {
            clock_source: RtcClockSource::Lse,
            bypass_lse_output: true, // eg if using a SMD oscillator.
            ..Default::default()
        },
    );

    rtc.set_12h_fmt(); // Optionally, use 12-hour format.

    // Set the RTC to trigger an interrupt every 30 seconds.
    rtc.set_wakeup(30.);

    // Store the RTC in a global variable that we can access in interrupts, using
    // critical sections.
    with(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
    });

    // Unmask the interrupt line.
    unsafe {
        NVIC::unmask(pac::Interrupt::RTC_WKUP);
    }

    loop {
        // The RTC uses Chrono for dates, times, and datetimes. All of these are naive.
        let date = rtc.get_date();
        let time = rtc.get_time();
        let dt = rtc.get_datetime();

        // See also: `get_seconds()`, `get_day()` etc.
        let hours = rtc.get_hours();

        // Enter a low power mode.
        low_power::stop(StopMode::One);

        // Turn back on the PLL, which is disabled by setting `stop` mode.
        clock_cfg.reselect_input();
    }
}

// todo: Alarms.

#[interrupt]
/// RTC wakeup handler
fn RTC_WKUP() {
    with(|cs| {
        // Reset pending bit for interrupt line
        unsafe {
            (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr20().set_bit());
        }
        access_global!(RTC, rtc, cs);
        rtc.clear_wakeup_flag();

        // Do something.
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
