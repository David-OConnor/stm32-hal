//! This example demonstrates use of SPI to configure an external sensor, read multiple regieters
//! over DMA in a single read, and apply an IIR lowpass filter to the readings using CMSIS-DSP.
//! Uses RTIC.

#![no_main]
#![no_std]

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as dsp_sys;
use cortex_m::{self, asm, delay::Delay};
use defmt::println;
use defmt_rtt as _; // global logger
use hal::{
    clocks::{self, Clocks},
    dma::{self, ChannelCfg, Dma, DmaChannel, DmaInterrupt, DmaPeriph},
    gpio::{self, Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    pac::{self, DMA1, SPI1},
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
};
use panic_probe as _;

static mut WRITE_BUF: [u8; 13] = [0; 13];

// IMU readings buffer. 3 accelerometer, and 3 gyro measurements; 2 bytes each. 0-padded on the left,
// since that's where we pass the register in the write buffer.
// This buffer is static, to ensure it lives through the life of the program.
pub static mut IMU_READINGS: [u8; 13] = [0; 13];

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: dsp_sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

mod imu {
    use cortex_m::delay::Delay;
    ///! Module for TDK ICM-426xx IMUs. Stripped down in this example to include only what we need.
    use hal::{
        dma::{Dma, DmaChannel, DmaPeriph},
        gpio::Pin,
        pac::{DMA1, SPI1},
        spi::Spi,
    };

    const GYRO_FULLSCALE: f32 = 34.90659; // 2,000 degrees/sec
    const ACCEL_FULLSCALE: f32 = 156.9056; // 16 G

    /// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
    #[derive(Clone, Copy)]
    #[repr(u8)]
    pub enum Reg {
        IntConfig = 0x14,
        PwrMgmt0 = 0x4E,
        GyroConfig0 = 0x4F,
        AccelConfig0 = 0x50,
        GyroConfig1 = 0x51,
        GyroAccelConfig0 = 0x52,
        IntSource0 = 0x65,
    }

    // We use this to determine which reg to start DMA reads
    pub const READINGS_START_ADDR: u8 = 0x80 | 0x1F; // (AccelDataX1)

    /// Utility function to write a single byte.
    fn write_one(reg: Reg, word: u8, spi: &mut Spi<SPI1>, cs: &mut Pin) {
        cs.set_low();
        spi.write(&[reg as u8, word]).ok();
        cs.set_high();
    }

    /// Configure the device.
    pub fn setup(spi: &mut Spi<SPI1>, cs: &mut Pin, delay: &mut Delay) {
        // Leave default of SPI mode 0 and 3.

        // Enable gyros and accelerometers in low noise mode.
        write_one(Reg::PwrMgmt0, 0b0000_1111, spi, cs);

        // Set gyros and accelerometers to 8kHz update rate, 2000 DPS gyro full scale range,
        // and +-16g accelerometer full scale range.
        write_one(Reg::GyroConfig0, 0b0000_0011, spi, cs);
        // "When transitioning from OFF to any of the other modes, do not issue any
        // register writes for 200µs." (Gyro and accel)
        delay.delay_us(200);

        write_one(Reg::AccelConfig0, 0b0000_0011, spi, cs);
        delay.delay_us(200);

        // (Leave default interrupt settings of active low, push pull, pulsed.)

        // Enable UI data ready interrupt routed to the INT1 pin.
        write_one(Reg::IntSource0, 0b0000_1000, spi, cs);
    }

    /// Output: m/s^2
    pub fn interpret_accel(val: i16) -> f32 {
        (val as f32 / i16::MAX as f32) * ACCEL_FULLSCALE
    }

    /// Output: rad/s
    pub fn interpret_gyro(val: i16) -> f32 {
        (val as f32 / i16::MAX as f32) * GYRO_FULLSCALE
    }

    /// Represents sensor readings from a 6-axis accelerometer + gyro.
    #[derive(Default)]
    pub struct ImuReadings {
        pub a_x: f32,
        pub a_y: f32,
        pub a_z: f32,
        pub v_pitch: f32,
        pub v_roll: f32,
        pub v_yaw: f32,
    }

    impl ImuReadings {
        /// We use this to assemble readings from the DMA buffer.
        pub fn from_buffer(buf: &[u8]) -> Self {
            // todo: Note: this mapping may be different for diff IMUs, eg if they use a different reading register ordering.
            // todo: Currently hard-set for ICM426xx.

            // Ignore byte 0; it's for the first reg passed during the `write` transfer.
            Self {
                a_x: interpret_accel(i16::from_be_bytes([buf[1], buf[2]])),
                a_y: interpret_accel(i16::from_be_bytes([buf[3], buf[4]])),
                a_z: interpret_accel(i16::from_be_bytes([buf[5], buf[6]])),
                v_pitch: interpret_gyro(i16::from_be_bytes([buf[7], buf[8]])),
                v_roll: interpret_gyro(i16::from_be_bytes([buf[9], buf[10]])),
                v_yaw: interpret_gyro(i16::from_be_bytes([buf[11], buf[12]])),
            }
        }
    }

    /// Read all 3 measurements, by commanding a DMA transfer. The transfer is closed, and readings
    /// are processed in the Transfer Complete ISR.
    pub fn read_imu_dma(starting_addr: u8, spi: &mut Spi<SPI1>, cs: &mut Pin) {
        // First byte is the first data reg, per this IMU's. Remaining bytes are empty, while
        // the MISO line transmits readings.
        // Note that we use a static buffer to ensure it lives throughout the DMA xfer.
        unsafe {
            WRITE_BUF[0] = starting_addr;
        }

        cs.set_low();

        unsafe {
            spi.transfer_dma(
                &WRITE_BUF,
                &mut crate::IMU_READINGS,
                DmaChannel::C1,
                DmaChannel::C2,
                Default::default(),
                Default::default(),
                DmaPeriph::Dma1,
            );
        }
    }
}

mod filter {
    //! This module contains filtering code for the IMU, using an IIR bessel lowpass, and IIR

    use cmsis_dsp_api as dsp_api;
    use cmsis_dsp_sys as dsp_sys;

    use crate::IirInstWrapper;

    // Filter states and coefficients are static, to ensure they live through the live of the program.
    static mut FILTER_STATE_ACCEL_X: [f32; 4] = [0.; 4];
    static mut FILTER_STATE_ACCEL_Y: [f32; 4] = [0.; 4];
    static mut FILTER_STATE_ACCEL_Z: [f32; 4] = [0.; 4];

    static mut FILTER_STATE_GYRO_PITCH: [f32; 4] = [0.; 4];
    static mut FILTER_STATE_GYRO_ROLL: [f32; 4] = [0.; 4];
    static mut FILTER_STATE_GYRO_YAW: [f32; 4] = [0.; 4];

    // Demonstration of how to generate these coefficients using Python + Scipy.
    // filter_ = scipy.signal.iirfilter(1, 40, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
    // coeffs = []
    // for row in filter_:
    //     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

    static COEFFS_LP_ACCEL: [f32; 5] = [
        0.015466291403103363,
        0.015466291403103363,
        0.0,
        0.9690674171937933,
        -0.0,
    ];

    // filter_ = signal.iirfilter(1, 40, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
    static COEFFS_LP_GYRO: [f32; 5] = [
        0.015466291403103363,
        0.015466291403103363,
        0.0,
        0.9690674171937933,
        -0.0,
    ];

    /// Store lowpass IIR filter instances, for use with lowpass and notch filters for IMU readings.
    pub struct ImuFilters {
        pub accel_x: IirInstWrapper,
        pub accel_y: IirInstWrapper,
        pub accel_z: IirInstWrapper,

        pub gyro_pitch: IirInstWrapper,
        pub gyro_roll: IirInstWrapper,
        pub gyro_yaw: IirInstWrapper,
    }

    impl ImuFilters {
        pub fn new() -> Self {
            let mut result = Self {
                accel_x: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },
                accel_y: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },
                accel_z: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },

                gyro_pitch: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },
                gyro_roll: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },
                gyro_yaw: IirInstWrapper {
                    inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
                },
            };

            unsafe {
                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.accel_x.inner,
                    &COEFFS_LP_ACCEL,
                    &mut FILTER_STATE_ACCEL_X,
                );
                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.accel_y.inner,
                    &COEFFS_LP_ACCEL,
                    &mut FILTER_STATE_ACCEL_Y,
                );
                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.accel_z.inner,
                    &COEFFS_LP_ACCEL,
                    &mut FILTER_STATE_ACCEL_Z,
                );

                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.gyro_pitch.inner,
                    &COEFFS_LP_GYRO,
                    &mut FILTER_STATE_GYRO_PITCH,
                );
                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.gyro_roll.inner,
                    &COEFFS_LP_GYRO,
                    &mut FILTER_STATE_GYRO_ROLL,
                );
                dsp_api::biquad_cascade_df1_init_f32(
                    &mut result.gyro_yaw.inner,
                    &COEFFS_LP_GYRO,
                    &mut FILTER_STATE_GYRO_YAW,
                );
            }

            result
        }

        /// Apply the filters to IMU readings, modifying in place. Block size = 1.
        pub fn apply(&mut self, data: &mut imu::ImuReadings) {
            let block_size = 1;

            let mut a_x = [0.];
            let mut a_y = [0.];
            let mut a_z = [0.];
            let mut v_pitch = [0.];
            let mut v_roll = [0.];
            let mut v_yaw = [0.];

            unsafe {
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.accel_x.inner,
                    &[data.a_x],
                    &mut a_x,
                    block_size,
                );
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.accel_y.inner,
                    &[data.a_y],
                    &mut a_y,
                    block_size,
                );
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.accel_z.inner,
                    &[data.a_z],
                    &mut a_z,
                    block_size,
                );
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.gyro_pitch.inner,
                    &[data.v_pitch],
                    &mut v_pitch,
                    block_size,
                );
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.gyro_roll.inner,
                    &[data.v_roll],
                    &mut v_roll,
                    block_size,
                );
                dsp_api::biquad_cascade_df1_f32(
                    &mut self.gyro_yaw.inner,
                    &[data.v_yaw],
                    &mut v_yaw,
                    block_size,
                );
            }

            data.a_x = a_x[0];
            data.a_y = a_y[0];
            data.a_z = a_z[0];
            data.v_pitch = v_pitch[0];
            data.v_roll = v_roll[0];
            data.v_yaw = v_yaw[0];
        }
    }
}

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // SPI1 for the IMU.
    let mut sck1 = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut miso1 = Pin::new(Port::A, 6, PinMode::Alt(5));
    let mut mosi1 = Pin::new(Port::A, 7, PinMode::Alt(5));

    sck1.output_speed(OutputSpeed::High);
    miso1.output_speed(OutputSpeed::High);
    mosi1.output_speed(OutputSpeed::High);

    // We assume here the interrupt config uses default settings active low, push pull, pulsed.
    let mut imu_interrupt = Pin::new(Port::C, 4, PinMode::Input);
    imu_interrupt.output_type(OutputType::OpenDrain);
    imu_interrupt.pull(Pull::Up);
    imu_interrupt.enable_interrupt(Edge::Falling);
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        spi1: Spi<SPI1>,
        cs_imu: Pin,
        imu_filters: filter::ImuFilters,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        // Set up clocks
        let clock_cfg = Default::default();
        clock_cfg.setup().unwrap();

        // Set up pins with appropriate modes.
        setup_pins();

        // We use SPI1 for the IMU
        // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
        let imu_baud_div = BaudRate::Div32; // Adjust this based on MCU speed, and IMU max speed.

        let imu_spi_cfg = SpiConfig {
            // Per ICM42688 and ISM330 DSs, only mode 3 is valid.
            mode: SpiMode::mode3(),
            ..Default::default()
        };

        let mut spi1 = Spi::new(dp.SPI1, imu_spi_cfg, imu_baud_div);

        let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

        let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
        imu::setup(&mut spi1, &mut cs_imu, &mut delay);

        let mut dma = Dma::new(dp.DMA1);
        // dma::enable_mux1(); // Required on G4 only.

        // Assign appropriate DMA channels to SPI transmit and receive. (Required on DMAMUX-supporting
        // MCUs only; channels are hard-coded on older ones).
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::Spi1Tx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Spi1Rx);

        // We use Spi transfer complete to know when our readings are ready.
        dma.enable_interrupt(DmaChannel::C2, DmaInterrupt::TransferComplete);

        (
            // todo: Make these local as able.
            Shared {
                spi1,
                cs_imu,
                imu_filters: filter::ImuFilters::new(),
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    /// Runs when new IMU data is ready. Trigger a DMA read.
    #[task(binds = EXTI4, shared = [cs_imu, dma, spi1], priority = 1)]
    fn imu_data_isr(cx: imu_data_isr::Context) {
        gpio::clear_exti_interrupt(4);

        (cx.shared.cs_imu, cx.shared.spi1).lock(|cs_imu, spi| {
            imu::read_imu_dma(imu::READINGS_START_ADDR, spi, cs_imu);
        });
    }

    #[task(binds = DMA1_CH2, shared = [spi1, cs_imu, imu_filters], priority = 2)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop.
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C1,
            DmaInterrupt::TransferComplete,
        );

        (cx.shared.spi).lock(|dma, spi| {
            // Note that this step is mandatory, per STM32 RM.
            spi.stop_dma(DmaChannel::C1, Some(DmaChannel::C2), DmaPeriph::Dma1);
        });

        cx.shared.cs_imu.lock(|cs| {
            cs.set_high();
        });

        let mut imu_data = imu::ImuReadings::from_buffer(unsafe { &IMU_READINGS });

        // Apply our lowpass filter.
        cx.shared.imu_filters.lock(|imu_filters| {
            imu_filters.apply(&mut imu_data);
        });
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
