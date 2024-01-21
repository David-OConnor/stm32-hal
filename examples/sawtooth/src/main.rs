//! This minimial example creates a sawtooth waveform on DAC1 OUT1
//! TIM4 interrupt blinks an LED on PA5, and is used as a reset trigger source for the DAC
//! TIM2 is configured to run at 1kHz, and is used as an increment trigger source for the DAC

#![no_std]
#![no_main]


use cortex_m::asm;

use hal::{
    self,
    clocks::{Clocks},
    gpio::{Pin, PinMode, Port},
    timer::{Timer, BasicTimer, TimerInterrupt, MasterModeSelection},
    pac::{self, TIM4, TIM2},
    dac::{Dac, DacChannel, SawtoothConfig, SawtoothDirection, Trigger},
};

use defmt_rtt as _;
use panic_probe as _;

const BLINK_FREQ: f32 = 1. / 0.1;

#[rtic::app(device = pac, peripherals = false)]
mod app {
    // This line is required to allow imports inside an RTIC module.
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        timer: Timer<TIM4>,
        inc_timer: Timer<TIM2>,
        led_pin: Pin,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = pac::Peripherals::take().unwrap();

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let led_pin = Pin::new(Port::A, 5, PinMode::Output);

        let mut timer = Timer::new_tim4(dp.TIM4, BLINK_FREQ * 2., Default::default(), &clock_cfg);
        timer.enable_interrupt(TimerInterrupt::Update);

        // Enable TRGO from TIM4 to provide a reset source to the DAC Sawtooth wave generator
        timer.regs.cr2.modify(|_, w| unsafe { w.mms().bits(MasterModeSelection::Update as u8) });
        timer.enable();

        let mut inc_timer = Timer::new_tim2(dp.TIM2, 1000.0, Default::default(), &clock_cfg);

        // Enable TRGO from TIM2 to provide an increment source to the DAC Sawtooth wave generator
        inc_timer.regs.cr2.modify(|_, w| unsafe { w.mms().bits(MasterModeSelection::Update as u8) });
        inc_timer.enable();


        // Set DAC1 OUT1 pin PA4 to Analog
        Pin::new(Port::A, 4, PinMode::Analog);

        let mut dac1 = Dac::new(dp.DAC1, Default::default(), 3.3);

        let sawtooth_cfg = SawtoothConfig {
            increment_trigger: Trigger::Tim2,
            reset_trigger: Trigger::Tim4,
            initial: 0,
            increment: 1000,
            direction: SawtoothDirection::Rising
        };

        dac1.generate_sawtooth(DacChannel::C1, sawtooth_cfg);
        dac1.enable(DacChannel::C1);

        (
            Shared {},
            Local { inc_timer, timer, led_pin },
            init::Monotonics(),
        )
    }

    #[idle()]
     fn idle(cx: idle::Context) -> ! {
         loop {
            asm::nop();
        }
     }

    #[task(binds = TIM4, local=[timer, led_pin], priority = 1)]
    /// When the timer's counter expires, toggle the pin connected to the LED.
    fn blink_isr(mut cx: blink_isr::Context) {
        cx.local.timer.clear_interrupt(TimerInterrupt::Update);

        if cx.local.led_pin.is_low() {
            cx.local.led_pin.set_high();
        } else {
            cx.local.led_pin.set_low();
        }
    }

    #[task(binds = TIM2, local=[inc_timer], priority = 2)]
    fn inc_isr(mut cx: inc_isr::Context) {
        cx.local.inc_timer.clear_interrupt(TimerInterrupt::Update);
        defmt::println!("Inc");
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
