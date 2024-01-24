//! This example demonstrates how to read from a PDM microphone, and output to the DAC.
//! It passes the signal through with no processing, other than reducing bit resolution.
//! Demonstrates use of DFSDM, DAC, DMA, and timers. Uses RTIC.
//!
//! It uses double-buffered input and output to prevent conflicts between reading and
//! writing the buffers. Uses circular DMA for both input and output.
//!
//! In practice, you might perform some DSP between input and output. For example, using
//! the CMSIS-DSP library wrapped with Bindgen.
//!
//! You'd also likely use the SAI peripheral in conjunction with an external codec for output.
//! We use DAC here for simplicity, but it's only 12-bits, and may be too noisy.
//!
//! Additionally, DFSDM has poor frequency response when sampled down to the final sample rate.
//! To avoid this, you may wish to sample it down to FS * 4, then use a FIR lowpass filter and decimator
//! (Such as the one in CMSIS-DSP) to arrive at the final sample rate.
//!
//! Alternatively, you can use the SAI peripheral for PDM mic input, with software decimation
//! and lowpass. Or, use external hardware for the PDM-PCM conversion, and pass to SAI as TDM or I2S.

#![no_main]
#![no_std]
// We use DSP terms that doesn't follow Rust's naming conventions.
#![allow(non_snake_case)]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{self, AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::{self, asm, delay::Delay, peripheral::NVIC};
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use defmt_rtt as _; // global logger
use hal::{
    self,
    clocks::{Clocks, HclkPrescaler, InputSrc, PllCfg, PllSrc, SaiSrc, VosRange},
    dac::{Dac, DacBits, DacChannel, Trigger},
    debug_workaround,
    dfsdm::{self, Dfsdm, DfsdmChannel, DfsdmConfig, Filter},
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaPeriph},
    gpio::{OutputType, Pin, PinMode, Port, Pull},
    pac::{self, interrupt, DAC, DFSDM, DMA1, SAI1},
    prelude::*,
    timer::{BasicTimer, MasterModeSelection, Timer, TimerInterrupt},
};
use panic_probe as _;

const FS: u32 = 48_000;

const BLOCK_SIZE: usize = 500; // 1000 @ 48kHz = 21ms

// Double buffers
static mut INPUT_BUF_L: [u32; BLOCK_SIZE * 2] = [0; BLOCK_SIZE * 2];
static mut INPUT_BUF_R: [u32; BLOCK_SIZE * 2] = [0; BLOCK_SIZE * 2];

// Output buffers are temporarily u16 to deal with the 12-bit DAC.
static mut OUTPUT_BUF_L: [u16; BLOCK_SIZE * 2] = [0; BLOCK_SIZE * 2];
static mut OUTPUT_BUF_R: [u16; BLOCK_SIZE * 2] = [0; BLOCK_SIZE * 2];

// If `x_BUF_WRITING_RIGHT` is true, we're reading from the left half of the buffer, and writing from
// the right. If false, we're reading from the right half and writing to the left.
// Note that if using an external DAC from the SAI peripheral, we don't need this, since
// input and output would be on the same clock.
static OUTPUT_BUF_WRITING_RIGHT: AtomicBool = AtomicBool::new(false);

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // Set `dac_pin` to analog mode, to prevent parasitic power use.
    // DAC1_OUT1 is on PA4. DAC1_OUT2 is on PA5.
    let _dac_pin_l = Pin::new(Port::A, 4, PinMode::Analog);
    let _dac_pin_r = Pin::new(Port::A, 5, PinMode::Analog);

    let _dfsdm_ckout = Pin::new(Port::D, 3, PinMode::Alt(3));
    let _dfsdm_data1 = Pin::new(Port::D, 6, PinMode::Alt(4));
}

#[rtic::app(device = pac)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        #[lock_free]
        dma: Dma<DMA1>,
        #[lock_free]
        dfsdm: Dfsdm<DFSDM>,
    }

    #[local]
    struct Local {
        dac: Dac<DAC>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let mut cp: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let mut dp: pac::Peripherals = cx.device;

        // Set up clocks.

        // This example is for H743V with HSI. You'll have to modify it for your MCU.
        // Make sure to set up your audio clock (eg SAI1) appropriately here.
        let clock_cfg = Clocks {
            pll2: PllCfg {
                enabled: true,
                pllp_en: true,

                // 48Khz x 32 bits-per-word x 2 words = 3.072Mhz
                divn: 250,
                divp: 81,

                ..PllCfg::disabled()
            },
            sai1_src: SaiSrc::Pll2P,

            ..Default::default()
        };

        clock_cfg.setup().unwrap();

        debug_workaround();

        // Improves performance, at a cost of slightly increased power use.
        // May be required to prevent sound problems.
        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();

        // Set up pins with appropriate modes.
        setup_pins();

        // Tim6 and Tim7 are internally connected to the DAC and are able to drive it through their
        // trigger outputs.
        let mut dac_timer = BasicTimer::new(
            dp.TIM6,
            clock_cfg.sai1_speed() as f32 / (64. * 2.),
            &clock_cfg,
        );

        // The update event is selected as a trigger output (TRGO). For instance a
        // master timer can then be used as a prescaler for a slave timer.
        dac_timer.set_mastermode(MasterModeSelection::Update);

        let mut dac = Dac::new(dp.DAC, DacBits::TwelveR, 3.3);
        dac.set_trigger(DacChannel::C1, Trigger::Tim6);

        // This timer allows us to periodically view the output waveform, eg for inspection and
        // plotting.
        let mut read_timer = Timer::new_tim3(dp.TIM3, 1., &clock_cfg);
        read_timer.enable_interrupt(TimerInterrupt::Update);

        let cfg = DfsdmConfig {
            sampling_freq: 48_000,
            ..Default::default()
        };

        let mut dfsdm = Dfsdm::new(dp.DFSDM, cfg, &clock_cfg);
        dfsdm.setup_pdm_mics(DfsdmChannel::C1);

        let mut dma = Dma::new(dp.DMA1);

        dma::mux(DmaPeriph::Dma1, DmaChannel::C0, dma::DmaInput::Dfsdm1F0);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, dma::DmaInput::Dfsdm1F11);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, dma::DmaInput::DacCh1);

        dma.enable_interrupt(DmaChannel::C0, DmaInterrupt::HalfTransfer);
        dma.enable_interrupt(DmaChannel::C0, DmaInterrupt::TransferComplete);

        dma.enable_interrupt(DmaChannel::C2, DmaInterrupt::HalfTransfer);
        dma.enable_interrupt(DmaChannel::C2, DmaInterrupt::TransferComplete);

        unsafe {
            dfsdm.read_dma(
                &mut INPUT_BUF_L,
                Filter::F0,
                DmaChannel::C0,
                dma::ChannelCfg {
                    circular: dma::Circular::Enabled,
                    priority: dma::Priority::High,
                    ..Default::default()
                },
                &mut dma,
            );
        }

        unsafe {
            dfsdm.read_dma(
                &mut INPUT_BUF_R,
                Filter::F1,
                DmaChannel::C1,
                dma::ChannelCfg {
                    circular: dma::Circular::Enabled,
                    priority: dma::Priority::High,
                    ..Default::default()
                },
                &mut dma,
            );
        }

        dac.enable(DacChannel::C1);
        dac.enable(DacChannel::C2);
        dac_timer.enable();
        read_timer.enable();

        unsafe {
            dac.write_dma(
                &OUTPUT_BUF_L,
                DacChannel::C1,
                DmaChannel::C2,
                dma::ChannelCfg {
                    circular: dma::Circular::Enabled,
                    ..Default::default()
                },
                &mut dma,
            );
        }

        dfsdm.enable_filter(Filter::F0, DfsdmChannel::C1);
        dfsdm.enable_filter(Filter::F1, DfsdmChannel::C0);
        dfsdm.enable();

        dfsdm.start_conversion(Filter::F0);
        dfsdm.start_conversion(Filter::F1);

        (Shared { dfsdm, dma }, Local { dac }, init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    #[task(binds = DMA1_STR0, shared = [dma], priority = 2)]
    /// DFSDM left channel DMA ISR
    fn dfsdm_isr(cx: dfsdm_isr::Context) {
        let mut input_modifier = 0;
        let mut output_modifier = 0;

        if cx.shared.dma.regs.lisr.read().tcif0().bit_is_set() {
            // Transfer complete: DFSDM will now write to left half of the buffer,
            // so read from the buffer's right half.
            cx.shared
                .dma
                .clear_interrupt(DmaChannel::C0, DmaInterrupt::TransferComplete);
            input_modifier = BLOCK_SIZE;
        } else {
            cx.shared
                .dma
                .clear_interrupt(DmaChannel::C0, DmaInterrupt::HalfTransfer);
        }

        if OUTPUT_BUF_WRITING_RIGHT.load(Ordering::Acquire) == true {
            output_modifier = BLOCK_SIZE;
        }

        // This is a placeholder for DSP. It sets the output in a suitable range for the DAC.
        unsafe {
            for j in 0..BLOCK_SIZE {
                // todo: Not sure why this is scaling this way.
                OUTPUT_BUF_L[j + output_modifier] =
                    ((INPUT_BUF_L[j + input_modifier] >> 14) - 2_200) as u16;
            }
        }
    }

    #[task(binds = DMA1_STR2, shared = [dma], priority = 2)]
    /// DAC DMA ISR
    fn dac_isr(cx: dac_isr::Context) {
        if cx.shared.dma.regs.lisr.read().tcif2().bit_is_set() {
            // Transfer complete: DAC will now output from the left half of the buffer,
            // so write to the buffer's right half.
            cx.shared
                .dma
                .clear_interrupt(DmaChannel::C2, DmaInterrupt::TransferComplete);
            OUTPUT_BUF_WRITING_RIGHT.store(true, Ordering::Release);
        } else {
            cx.shared
                .dma
                .clear_interrupt(DmaChannel::C2, DmaInterrupt::HalfTransfer);
            OUTPUT_BUF_WRITING_RIGHT.store(false, Ordering::Release);
        }
    }

    #[task(binds = TIM3, priority = 1)]
    fn timer_isr(cx: timer_isr::Context) {
        unsafe { (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        // Try pasting this output into Python etc to plot!
        unsafe {
            defmt::println!("Output buffer: {:?}", OUTPUT_BUF_L);
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
