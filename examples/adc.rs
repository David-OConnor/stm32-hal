//! This module includes an overview of ADC features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    adc::{Adc, AdcChannel, AdcDevice, AdcInterrupt, Align, ClockMode, InputType, OperationMode},
    clocks::Clocks,
    delay::Delay,
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaWriteBuf},
    gpio::{Edge, PinMode, PinNum},
    low_power, pac,
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    if clock_cfg.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        defmt::error!("Unable to configure clocks due to a speed error.")
    };

    let mut delay = Delay::new(cp.SYST, &clock_cfg);

    let chan_num = 2;

    // Enable the GPIOB port.
    let mut gpiob = GpioB::new(dp.GPIOB, &mut dp.RCC);

    // Configure the ADC pin in analog mode. (This is the default state for some STM32 families,
    // but not all)
    let _adc_pin = gpiob.new_pin(PinNum::P0, PinMode::Analog);

    let mut adc = Adc::new_adc1(
        dp.ADC1,
        AdcDevice::One,
        &mut dp.ADC_COMMON,
        ClockMode::default(),
        &clock_cfg,
        &mut dp.RCC,
    );

    // 1: Confiuration options:

    // Set a channel to a specific position in a sequence:
    adc.set_sequence(1, 2); // Set channel 1 to be the second position in the sequence.

    // Set the length of the sequence to read. (ie number of channels).
    adc.set_sequence_len(2);

    // If on MCUs that support MUXING, like L5 and G, choose the appropriate channel:
    // dma::mux(DmaChannel::C1, MuxInput::Adc1, &mut dp.DMAMUX);
    adc.read_dma(&buf, DmaChannel::C1, &mut dma);

    // Set up differential mode:
    adc.set_input_type(chan_num, InputType::Differential);

    // Change the sample rate:
    adc.set_sample_time(chan_num, SampleTime::T2);

    // Set left align mode:
    adc.set_align(Align::Left);

    adc.enable_interrupt(AdcInterrupt::EndOfSequence);

    // 2: Set up DMA, for nonblocking (generally faster) conversion transfers:
    let mut dma = Dma::new(&mut dp.DMA1, &dp.RCC);

    let mut dma_buf = DmaWriteBuf { buf: &[0_u16; 1] };

    // Begin a DMA transfer. Note that the `DmaChannel` we pass here is only used on
    // MCUs that use `DMAMUX`, eg L5, G0, and G4. For those, you need to run `mux`, to
    // set the channel: `dma::mux(DmaChannel::C1, MuxInput::Adc1, &mut dp.DMAMUX);
    badc.read_dma(&mut buf, chan_num, DmaChannel::C1, &mut dma);

    // Wait for the transfer to complete. Ie by reading in the channel's transfer-complete interrupt,
    // which is enabled by the `read_dma` command.  For this example, we block until ready
    while !dma.transfer_is_complete(DmaChannel::C1) {}
    dma.stop(DmaChannel::C1);

    defmt::info!("Reading: {:?}", &dma_buf.buf);

    // 3: Alternatively, we can take readings without DMA. This provides a simpler, memory-safe API,
    // and is compatible with the `embedded_hal::adc::OneShot trait.

    // Take a OneShot reading from channel 3. (Note that the Embedded HAL trait is also available,
    // for use in embedded drivers). Channels for EH usage are included: `stm32hal2::adc::AdcChannel::C3`
    let reading = adc.read(chan_num);

    // Or, start reading in continuous mode, reading a single channel
    adc.start_conversion(&[chan_num], OperationMode::Continuous);

    // Or, set up multiple channels in a sequence:
    adc.start_conversion([1, 2, 3], OperationMode::Continuous);
    // Read from the ADC's latest (continuously-running) conversion:
    let reading = adc.read_result();

    loop {
        low_power::sleep_now(&mut SCB);
    }
}
