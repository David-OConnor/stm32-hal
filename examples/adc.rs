//! This module includes an overview of ADC features available.
//! For project structure and debugging boilerplate, see the `synax_overview` example.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::{
    delay::Delay,
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use stm32_hal2::{
    adc::{
        Adc, AdcChannel, AdcDevice, AdcInterrupt, Align, ClockMode, InputType, OperationMode,
        SampleTime,
    },
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInterrupt, DmaWriteBuf},
    gpio::{Pin, PinMode, Port},
    low_power, pac,
};

static mut ADC_READ_BUF: [u16; 2] = [0; 2];

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    clock_cfg.setup().unwrap();

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    let chan_num = 2;

    // Configure the ADC pin in analog mode. (This is the default state for some STM32 families,
    // but not all)
    let _adc_pin = Pin::new(Port::B, 0, PinMode::Analog);

    let mut adc = Adc::new_adc1(dp.ADC1, AdcDevice::One, Default::default(), &clock_cfg);

    // 1: Configuration options:

    // Set a channel to a specific position in a sequence:
    adc.set_sequence(1, 2); // Set channel 1 to be the second position in the sequence.

    // Set the length of the sequence to read. (ie number of channels):
    adc.set_sequence_len(2);

    // Set up differential mode:
    adc.set_input_type(chan_num, InputType::Differential);

    // Change the sample rate:
    adc.set_sample_time(chan_num, SampleTime::T2);

    // Set left align mode:
    adc.set_align(Align::Left);

    adc.enable_interrupt(AdcInterrupt::EndOfSequence);

    // 2: Set up DMA, for non-blocking transfers:
    let mut dma = Dma::new(&mut dp.DMA1, &dp.RCC);

    let mut dma_buf = [0];

    // Begin a DMA transfer. Note that the `DmaChannel` we pass here is only used on
    // MCUs that use `DMAMUX`, eg L5, G0, and G4. For those, you need to run `mux`, to
    // set the channel: `dma::mux(DmaChannel::C1, MuxInput::Adc1, &mut dp.DMAMUX);
    unsafe {
        adc.read_dma(
            &mut dma_buf,
            &[chan_num],
            DmaChannel::C1,
            Default::default(),
            &mut dma,
        )
    };

    // Wait for the transfer to complete. Ie by handling the channel's transfer-complete
    // interrupt in an ISR, which is enabled by the `read_dma` command.
    // For this example, we block until the flag is set.
    while !dma.transfer_is_complete(DmaChannel::C1) {}
    dma.stop(DmaChannel::C1);

    defmt::println!("Reading: {:?}", &dma_buf[0]);

    // Unmask the interrupt line. See the `DMA_CH1` interrupt handler below.
    unsafe { NVIC::unmask(pac::Interrupt::DMA1_CH1) }

    // 3: Example of starting a circular DMA transfer using 2 channels. This will continuously update
    // the buffer with values from channels 17 and 12. (You can set longer sequence lengths as well).
    // You can then read from the buffer at any point to get the latest reading.
    let adc_cfg = AdcConfig {
        operation_mode: adc::OperationMode::Continuous,
        ..Default::default()
    };

    let mut batt_curr_adc = Adc::new_adc2(dp.ADC2, AdcDevice::Two, adc_cfg, &clock_cfg);

    unsafe {
        batt_curr_adc.read_dma(
            &mut ADC_READ_BUF,
            &[17, 12],
            DmaChannel::C2,
            ChannelCfg {
                circular: dma::Circular::Enabled,
                ..Default::default()
            },
            &mut dma,
        );
    }

    // 4: Alternatively, we can take readings without DMA. This provides a simpler, blocking API.

    // Take a blocking reading from channel 3.
    let reading = adc.read(chan_num);

    // Convert a reading to voltage, which includes compensation for the built-in VDDA
    // reference voltage
    let voltage = adc.reading_to_voltage(reading);

    // Or, read convert multiple channels in a sequence. You can read the results once the transfer
    // complete interrupt fires.
    adc.enable_interrupt(AdcInterrupt::EndOfSequence);
    adc.start_conversion(&[1, 2, 3]);

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// This interrupt fires when the ADC transfer is complete.
fn DMA1_CH1() {
    free(|cs| {
        unsafe { (*pac::DMA1::ptr()).ifcr.write(|w| w.tcif1().set_bit()) }
        // Or, if you have access to the Dma peripheral struct:
        // dma.clear_interrupt(DmaChannel::C1);
        // dma.stop(DmaChannel::C1);
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
