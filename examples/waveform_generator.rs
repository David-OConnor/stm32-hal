//! Generates a sine waveform output from the DAC. Demonstrates using of timers to trigger
//! the DAC, and a DMA circular buffer to transfer data from memory to the DAC with minimal
//! CPU intervention.
//!
//! You can view output between ground and PA4 with an oscilloscope, or a pair of headphones or
//! speakers. If using headphones or speakers, make sure you can adjust their volume; the default
//! output volume will be too high.
//!
//! Tested on an STM32H743ZI, via STM32H7 Nucleo dev board.
//!
//! See [STM32 AN3126](https://www.st.com/resource/en/application_note/cd00259245-audio-and-waveform-generation-using-the-dac-in-stm32-products-stmicroelectronics.pdf),
//! "Audio and waveform generation using the DAC in STM32 products."

#![no_main]
#![no_std]
#![allow(non_snake_case)]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicUsize, Ordering},
};

use cortex_m::{self, delay::Delay, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use defmt_rtt as _; // global logger
use hal::{
    self,
    clocks::Clocks,
    dac::{Dac, DacBits, DacChannel, Trigger},
    debug_workaround,
    dma::{self, Dma, DmaChannel},
    gpio::{OutputType, Pin, PinMode, Port},
    low_power, pac,
    timer::{BasicTimer, MasterModeSelection, TimerDevice},
};
use panic_probe as _;

// Length of the lookup table used to generate sin waves etc.
const LUT_LEN: usize = 256;

// A lookup table for sin(x), over one period, using values from 0 - 4095; ie the full range of
// the STM32 12-bit onboard DAC. Compared to computation, this is faster, at the expense of memory use.
pub static SIN_X: [u16; crate::LUT_LEN] = [
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
    3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697, 1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
    1264, 1218, 1172, 1127, 1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565,
    530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176, 156, 137, 120, 103, 88,
    74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1, 2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103,
    120, 137, 156, 176, 197, 219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600,
    636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127, 1172, 1218, 1264, 1311,
    1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997,
];

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // Set `dac_pin` to analog mode, to prevent parasitic power use.
    // DAC1_OUT1 is on PA4. DAC1_OUT2 is on PA5.
    let _dac_pin = Pin::new(Port::A, 4, PinMode::Analog);
}

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let clock_cfg = Clocks::default();
    clock_cfg
        .setup(&mut dp.RCC, &mut dp.FLASH, &mut dp.PWR, &mut dp.SYSCFG)
        .unwrap();

    debug_workaround();

    // Set up pins with appropriate modes.
    setup_pins();

    // The output frequency of the DAC.
    let out_wave_freq = 2000.;
    let timer_freq = out_wave_freq * LUT_LEN as f32;

    // Tim6 and Tim7 are internally connected to the DAC and are able to drive it through their
    // trigger outputs.
    // We set this timer to the sample rate in Hz.
    // This timer triggers a transfer of one word from the DMA buffer into the DAC output register.
    let mut dac_timer = BasicTimer::new(dp.TIM6, TimerDevice::T6, timer_freq, &clock_cfg);

    //  The update event is selected as a trigger output (TRGO). For instance a
    // master timer can then be used as a prescaler for a slave timer.
    dac_timer.set_mastermode(MasterModeSelection::Update);

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    let mut dac = Dac::new(dp.DAC, Default::default(), 3.3);
    dac.calibrate_buffer(DacChannel::C1, &mut delay);
    dac.set_trigger(DacChannel::C1, Trigger::Tim6);

    let mut dma = Dma::new(dp.DMA1);

    dma::mux(DmaChannel::C3, dma::DmaInput::DacCh1);

    // Load the Sine LUT into a DMA circular buffer, which will send a 16-byte word of data
    // to the DAC on each timer trigger. Because it's a circular buffer, it will start at
    // the first value again once complete.
    let channel_cfg = dma::ChannelCfg {
        circular: dma::Circular::Enabled,
        ..Default::default()
    };

    unsafe {
        dac.write_dma(
            &lut::SIN_X,
            DacChannel::C1,
            DmaChannel::C3,
            channel_cfg,
            &mut dma,
        );
    }

    dac.enable(DacChannel::C1);
    dac_timer.enable();

    loop {
        low_power::csleep(&mut cp.SCB);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
