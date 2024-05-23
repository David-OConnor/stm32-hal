//! This minimial example causes an LED to blink using a (non-blocking) timer interrupt.
//! It demonstrates basic GPIO use, timers, and RTIC project structure, with resources, and an interrupt handler.

#![no_std]
#![no_main]


use cortex_m::asm;

use hal::{
    self,
    clocks::{Clocks},
    gpio::{Pin, PinMode, Port},
    timer::{Timer, TimerInterrupt},
    pac::{self, TIM5},
};

use defmt_rtt as _;
use panic_probe as _;

const BLINK_FREQ: f32 = 1.; // seconds

#[rtic::app(device = pac, peripherals = false)]
mod app {
    // This line is required to allow imports inside an RTIC module.
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        timer: Timer<TIM5>,
        led_pin: Pin,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = pac::Peripherals::take().unwrap();

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let led_pin = Pin::new(Port::A, 1, PinMode::Output);

        let mut timer = Timer::new_tim5(dp.TIM5, BLINK_FREQ * 2., Default::default(), &clock_cfg);
        timer.enable_interrupt(TimerInterrupt::Update);
        timer.enable();

        (
            Shared {},
            Local { timer, led_pin },
            init::Monotonics(),
        )
    }

    #[idle()]
     fn idle(cx: idle::Context) -> ! {
         loop {
            asm::nop();
        }
     }

    #[task(binds = TIM5, local=[timer, led_pin], priority = 1)]
    /// When the timer's counter expires, toggle the pin connected to the LED.
    fn blink_isr(mut cx: blink_isr::Context) {
        cx.local.timer.clear_interrupt(TimerInterrupt::Update);
        // Or: timer::clear_update_interrupt(5);

        if cx.local.led_pin.is_low() {
            cx.local.led_pin.set_high();
        } else {
            cx.local.led_pin.set_low();
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
