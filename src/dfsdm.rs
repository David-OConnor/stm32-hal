//! Digital filter for sigma delta modulators (DFSDM) support. Seem H742 RM, chapter 30.

// todo: This module is currently a mess due to needing to support 2 separate register access
// todo syntaxes re channels and filters, depending on variant.

use core::ops::Deref;

use cfg_if::cfg_if;
use num_traits::Float; // Float rounding.

use crate::{
    clocks::Clocks,
    pac::{self, RCC},
    util::rcc_en_reset,
};

cfg_if! {
    if #[cfg(any(feature = "l4", feature = "l5", feature = "h7b3"))] {
        use crate::pac::dfsdm1 as dfsdm_p;
    } else {
        use crate::pac::dfsdm as dfsdm_p;
    }
}

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;
#[cfg(not(any(feature = "f4", feature = "l552")))]
use crate::dma::{self, ChannelCfg, DmaChannel};
use crate::pac::DMA1;

#[derive(Clone, Copy)]
pub enum Filter {
    F0,
    F1,
    //
    F2,
    //
    F3,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum DfsdmChannel {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    C6 = 6,
    C7 = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Sinc filter order. Sets FLTxFCR register, FORD field.
pub enum FilterOrder {
    /// FastSinc filter type
    FastSinc = 0,
    /// Sinc1 filter type
    Sinc1 = 1,
    /// Sinc2 filter type
    Sinc2 = 2,
    /// Sinc3 filter type
    Sinc3 = 3,
    /// Sinc4 filter type
    Sinc4 = 4,
    /// Sinc5 filter type
    Sinc5 = 5,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Toggle clock clourse between system and audio clocks. Sets CH0CFGR1 register, CKOUTSRC field.
pub enum DfsdmClockSrc {
    /// Source for output clock is from system clock
    SysClk = 0,
    /// Source for output clock is from audio clock
    /// H7:  Audio clock source is SAI1 clock selected by SAI1SEL[1:0] field in RCC configuration
    AudioClk = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// SPI clock select for a given channel.
pub enum SpiClock {
    /// Clock coming from external CKINy input - sampling point according SITP[1:0
    External = 0,
    /// Clock coming from internal CKOUT output - sampling point according SITP[1:0]
    Internal = 1,
    /// Clock coming from internal CKOUT - sampling point on each second CKOUT falling edge
    InternalFalling = 2,
    /// Clock coming from internal CKOUT output - sampling point on each second CKOUT rising edge.
    InternalRising = 3,
}

#[derive(Clone, Copy)]
/// The type of DFSDM interrupt to configure. Reference Section 30.5 of the H742 RM.
/// Enabled in FLTxCR2. register. Monitor in FLTxISR register. Cleared by writing to the
/// FLTxICR register, for most.
pub enum DfsdmInterrupt {
    /// End of injected conversion. Enabled by JEOCIE field
    EndOfInjectedConversion,
    /// End of regular conversion. Enabled by REOCIE field
    EndOfConversion,
    /// Data overrun for injected conversions. Enabled by JOVRIE field
    DataOverrunInjected,
    /// Data overrun for regular conversions. Enabled by ROVRIE field
    DataOverrun,
    /// Analog watchdog. Enabled by AWDIE field
    AnalogWatchdog,
    /// Short-circuit deector. Enabled by SCDIE field
    ShortCircuit,
    /// Channel clock absense. Enabled by CKABIE field
    ChannelClockAbsense,
}

#[derive(Clone, Copy, PartialEq)]
/// Set conversions to regular, continous, or contiuous fast mode. Sets FLTxCR1 register, RCONT
/// and FAST fields.
pub enum Continuous {
    /// The regular channel is converted just once for each conversion request
    OneShot,
    /// The regular channel is converted repeatedly after each conversion request
    Continuous,
    /// Fast conversion mode enabled.
    /// When converting a regular conversion in continuous mode, having enabled the fast mode causes
    /// each conversion (except the first) to execute faster than in standard mode. This bit has no effect on
    /// conversions which are not continuous.
    ContinuousFastMode,
}

// todo: Not sure how to handle the 24 bit signed data throughout!!

/// Configuration for the DFSDM peripheral.
/// All operations in the DFSDM peripheral are in signed format (filtering, integration, offset
/// correction, right bit shift).
pub struct DfsdmConfig {
    /// Set the clock source to Sysclk, or Audio clock. Audio clock appears to be the SAI clock source,
    /// at least on H7. (?)
    /// Defaults to Audio clock.
    pub clock_src: DfsdmClockSrc,
    /// Sampling frequency. Used to adjust clock divider. Defaults to 48kHz. (48_000_000)
    pub sampling_freq: u32,
    /// Sample as one-shot, continuous, or continuous fast-mode. Defaults to continuous fast mode.
    pub continuous: Continuous,
    /// Sinc filter order. Defaults to Sinc4.
    pub filter_order: FilterOrder,
    /// Sinc filter oversampling ratio. Also known as decimation ratio
    pub filter_oversampling_ratio: u16,
    /// Integrator oversampling ratio. Also known as averaging (?) ratio
    pub integrator_oversampling_ratio: u8,
    /// To have the result aligned to a 24-bit value, each channel defines a number of right bit shifts
    /// which will be applied on each conversion result (injected or regular) from a given channel.
    /// The data bit shift number is stored in DTRBS[4:0] bits in DFSDM_CHyCFGR2 register.
    /// The right bit-shift is rounding the result to nearest integer value. The sign of shifted result is
    /// maintained, in order to have valid 24-bit signed format of result data.
    pub right_shift_bits: u8,
    /// The offset correction allows to calibrate the external sigma-delta modulator offset error. The
    /// user configures the offset register with a signed 24-bit correction, and this register is
    /// automatically added to the output result. The offset correction value is usually the result of a
    /// calibration routine embedded within the microcontroller software that performs the offset
    /// calibration calculation and stores the correction into the offset register. Defaults to 0.
    /// This  must be a 24 bit signed integer, which Rust doesn't support. So, to make it negative, you might
    /// use something like this: `offset:(1<<24) - 1_000;`
    pub offset: u32,
    /// SPI clock select for channel y. Ie internal or external.
    pub spi_clock: SpiClock,
}

impl Default for DfsdmConfig {
    fn default() -> Self {
        Self {
            clock_src: DfsdmClockSrc::AudioClk,
            sampling_freq: 48_000,
            continuous: Continuous::ContinuousFastMode,
            filter_order: FilterOrder::Sinc4, // From the PDM mic AN
            filter_oversampling_ratio: 64,    // From the PDM mic AN
            integrator_oversampling_ratio: 1, // From the PDM mic AN
            right_shift_bits: 0, // PDM mic AN uses 0x02. 8 seems right to get to 24 bit signed?
            offset: 0,
            spi_clock: SpiClock::Internal,
        }
    }
}

/// Represents the Digital filter for sigma delta modulators (DFSDM) peripheral, for
/// interfacing with external Σ∆ modulators.
pub struct Dfsdm<R> {
    pub regs: R,
    config: DfsdmConfig,
}

impl<R> Dfsdm<R>
where
    R: Deref<Target = dfsdm_p::RegisterBlock>,
    R: Deref<Target = dfsdm_p::RegisterBlock>,
{
    /// Initialize a DFSDM peripheral, including  enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, config: DfsdmConfig, clock_cfg: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        #[cfg(not(feature = "l4"))]
        rcc_en_reset!(apb2, dfsdm1, rcc);
        #[cfg(feature = "l4")]
        rcc_en_reset!(apb2, dfsdm, rcc);

        // The frequency of this CKOUT signal is derived from DFSDM clock or from audio clock (see
        // CKOUTSRC bit in CH0CFGR1 register) divided by a predivider (see CKOUTDIV
        // bits in CH0CFGR1 register). If the output clock is stopped, then CKOUT signal is
        // set to low state (output clock can be stopped by CKOUTDIV=0 in CHyCFGR1
        // register or by DFSDMEN=0 in CH0CFGR1 register).
        // The output clock signal frequency must be in the range 0 - 20 MHz.

        // PDM mic AN: The clock divider value must respect the following formula:
        // Divider = DFSDM Clock Source / (AUDIO_SAMPLING_FREQUENCY ×
        // DECIMATION_FACTOR)
        // (Note that for a 3.072Mhz clock , 64 decimation factor and 48K audio, the divider
        // works out to 1. With a 12.288Mhz clock, and the same factors, it's 4.)

        let clock_speed = match config.clock_src {
            DfsdmClockSrc::SysClk => clock_cfg.sysclk(),
            // todo: Conflicting docs on if this is apb2 or audio clock, and I'm not sure what audio clock is
            DfsdmClockSrc::AudioClk => clock_cfg.sai1_speed(),
        };

        // We want it to round to nearest divider, not floor round. Let's say our clock speed is slightly lower than
        // the target - we might get a divider calculated to be 3.99. With integer math, that would round down to 3,
        // and be effectively wrong - hence the floats.
        let divider = clock_speed as f32
            / (config.sampling_freq * config.filter_oversampling_ratio as u32) as f32;
        let divider = divider.round() as u8;

        // 1- 255: Defines the division of system clock for the serial clock output for CKOUT signal in range 2 -
        // 256 (Divider = CKOUTDIV+1).
        // CKOUTDIV also defines the threshold for a clock absence detection.
        assert!(divider >= 2); // (1 is invalid. 2 would disable the output clock.)

        cfg_if! {
            if #[cfg(any(feature = "l5"))] {
                let cfgr1 = &regs.ch0cfgr1;
            } else if #[cfg(any(feature = "l4"))] {
                let cfgr1 = &regs.chcfg0r1;
            } else {
                let cfgr1 = &regs.ch0.cfgr1;
            }
        }

        cfgr1.modify(|_, w| unsafe {
            w.ckoutsrc().bit(config.clock_src as u8 != 0);
            w.ckoutdiv().bits(divider as u8 - 1)
        });

        // Configuring the input serial interface
        // The following parameters must be configured for the input serial interface:
        // • Output clock predivider. There is a programmable predivider to generate the output
        // clock from DFSDM clock (2 - 256). It is defined by CKOUTDIV[7:0] bits in
        // CH0CFGR1 register.
        // • Serial interface type and input clock phase. Selection of SPI or Manchester coding
        // and sampling edge of input clock. It is defined by SITP [1:0] bits in
        // CHyCFGR1 register.
        // • Input clock source. External source from CKINy pin or internal from CKOUT pin. It is
        // defined by SPICKSEL[1:0] field in CHyCFGR1 register.
        // • Final data right bit-shift. Defines the final data right bit shift to have the result aligned
        // to a 24-bit value. It is defined by DTRBS[4:0] in CHyCFGR2 register.
        // • Channel offset per channel. Defines the analog offset of a given serial channel (offset
        // of connected external Σ∆ modulator). It is defined by OFFSET[23:0] bits in
        // CHyCFGR2 register.
        // • short-circuit detector and clock absence per channel enable. To enable or disable
        // the short-circuit detector (by SCDEN bit) and the clock absence monitoring (by
        // CKABEN bit) on a given serial channel in register CHyCFGR1.
        // • Analog watchdog filter and short-circuit detector threshold settings. To configure
        // channel analog watchdog filter parameters and channel short-circuit detector
        // parameters. Configurations are defined in CHyAWSCDR register.

        Self { regs, config }
    }

    /// Enables the DFSDM peripheral.
    /// The DFSDM interface is globally enabled by setting DFSDMEN=1 in the
    /// CH0CFGR1 register. Once DFSDM is globally enabled, all input channels (y=0..7)
    /// and digital filters FLTx (x=0..3) start to work if their enable bits are set (channel
    /// enable bit CHEN in CHyCFGR1 and FLTx enable bit DFEN in
    /// FLTxCR1).
    pub fn enable(&mut self) {
        cfg_if! {
            if #[cfg(any(feature = "l5"))] {
                let cfgr1 = &self.regs.ch0cfgr1;
            } else if #[cfg(any(feature = "l4"))] {
                let cfgr1 = &self.regs.chcfg0r1;
            } else {
                let cfgr1 = &self.regs.ch0.cfgr1;
            }
        }
        cfgr1.modify(|_, w| w.dfsdmen().set_bit());
    }

    /// Disables the DFSDM peripheral.
    /// DFSDM must be globally disabled (by DFSDMEN=0 in CH0CFGR1) before
    /// stopping the system clock to enter in the STOP mode of the device
    pub fn disable(&mut self) {
        cfg_if! {
            if #[cfg(any(feature = "l5"))] {
                let cfgr1 = &self.regs.ch0cfgr1;
            } else if #[cfg(any(feature = "l4"))] {
                let cfgr1 = &self.regs.chcfg0r1;
            } else {
                let cfgr1 = &self.regs.ch0.cfgr1;
            }
        }
        cfgr1.modify(|_, w| w.dfsdmen().clear_bit());
    }

    /// Configures and enables the DFSDM filter for a given channel, and enables
    /// that channel.
    /// Digital filter x FLTx (x=0..3) is enabled by setting DFEN=1 in the
    /// FLTxCR1 register. Once FLTx is enabled (DFEN=1), both Sincx
    /// digital filter unit and integrator unit are reinitialized.
    /// Note that this function sets `DFEN`, so run it `after` configuring other settings such
    /// as DMA.
    pub fn enable_filter(&mut self, filter: Filter, channel: DfsdmChannel) {
        // Setting RCONT in the FLTxCR1 register causes regular conversions to execute in
        // continuous mode. RCONT=1 means that the channel selected by RCH[2:0] is converted
        // repeatedly after ‘1’ is written to RSWSTART.

        // The regular conversions executing in continuous mode can be stopped by writing ‘0’ to
        // RCONT. After clearing RCONT, the on-going conversion is stopped immediately.
        // In continuous mode, the data rate can be increased by setting the FAST bit in the
        // FLTxCR1 register. In this case, the filter does not need to be refilled by new fresh
        // data if converting continuously from one channel because data inside the filter is valid from
        // previously sampled continuous data. The speed increase depends on the chosen filter
        // order. The first conversion in fast mode (FAST=1) after starting a continuous conversion by
        // RSWSTART=1 takes still full time (as when FAST=0), then each subsequent conversion is
        // finished in shorter intervals.

        // DFSDM contains a Sincx
        //  type digital filter implementation. This Sincx filter performs an input
        // digital data stream filtering, which results in decreasing the output data rate (decimation)
        // and increasing the output data resolution. The Sincx
        //  digital filter is configurable in order to
        // reach the required output data rates and required output data resolution. The configurable
        // parameters are:
        // • Filter order/type: (see FORD[2:0] bits in FLTxFCR register):
        // – FastSinc
        // – Sinc1
        // – Sinc2
        // – Sinc3
        // – Sinc4
        // – Sinc5
        // • Filter oversampling/decimation ratio (see FOSR[9:0] bits in FLTxFCR
        // register):
        // – FOSR = 1-1024 - for FastSinc filter and Sincx filter x = FORD = 1..3
        // – FOSR = 1-215 - for Sincx filter x = FORD = 4
        // – FOSR = 1-73 - for Sincx filter x = FORD = 5

        // todo: RSYNC

        // todo: Macro this?

        assert!(self.config.filter_oversampling_ratio <= 1_023);

        // DFSDM on-off control
        // The DFSDM interface is globally enabled by setting DFSDMEN=1 in the
        // DFSDM_CH0CFGR1 register. Once DFSDM is globally enabled, all input channels (y=0..7)
        // and digital filters DFSDM_FLTx (x=0..3) start to work if their enable bits are set (channel
        // enable bit CHEN in DFSDM_CHyCFGR1 and DFSDM_FLTx enable bit DFEN in
        // DFSDM_FLTxCR1).
        // Digital filter x DFSDM_FLTx (x=0..3) is enabled by setting DFEN=1 in the
        // DFSDM_FLTxCR1 register. Once DFSDM_FLTx is enabled (DFEN=1), both Sincx
        // digital filter unit and integrator unit are reinitialized.

        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let fcr = &self.regs.flt0fcr;
                        let cr1 = &self.regs.flt0cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let fcr = &self.regs.dfsdm0_fcr;
                        let cr1 = &self.regs.dfsdm0_cr1;
                    } else {
                        let fcr = &self.regs.flt0.fcr;
                        let cr1 = &self.regs.flt0.cr1;
                    }
                }
                fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio - 1);
                    w.iosr().bits(self.config.integrator_oversampling_ratio - 1)
                });
                cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let fcr = &self.regs.flt1fcr;
                        let cr1 = &self.regs.flt1cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let fcr = &self.regs.dfsdm1_fcr;
                        // let cr1 = &self.regs.dfsdm1_cr1;
                    } else {
                        let fcr = &self.regs.flt1.fcr;
                        let cr1 = &self.regs.flt1.cr1;
                    }
                }

                fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio - 1);
                    w.iosr().bits(self.config.integrator_oversampling_ratio - 1)
                });
                cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let fcr = &self.regs.flt2fcr;
                        let cr1 = &self.regs.flt2cr1;
                    }  else if #[cfg(any(feature = "l4"))] {
                        let fcr = &self.regs.dfsdm2_fcr;
                        let cr1 = &self.regs.dfsdm2_cr1;
                    } else {
                        let fcr = &self.regs.flt2.fcr;
                        let cr1 = &self.regs.flt2.cr1;
                    }
                }

                fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio - 1);
                    w.iosr().bits(self.config.integrator_oversampling_ratio - 1)
                });
                cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let fcr = &self.regs.flt3fcr;
                        let cr1 = &self.regs.flt3cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let fcr = &self.regs.dfsdm3_fcr;
                        let cr1 = &self.regs.dfsdm3_cr1;
                    } else {
                        let fcr = &self.regs.flt3.fcr;
                        let cr1 = &self.regs.flt3.cr1;
                    }
                }
                fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio - 1);
                    w.iosr().bits(self.config.integrator_oversampling_ratio - 1)
                });
                cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
            }
        }

        // Channel y (y=0..7) is enabled by setting CHEN=1 in the DFSDM_CHyCFGR1 register.
        // Once the channel is enabled, it receives serial data from the external Σ∆ modulator or
        // parallel internal data sources (ADCs or CPU/DMA wire from memory).

        // Note that we must set some settings like `dtrbs` before enabling the channel.

        assert!(self.config.offset <= (1 << 24));

        match channel {
            DfsdmChannel::C0 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch0cfgr1;
                        let cfgr2 = &self.regs.ch0cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg0r1;
                        let cfgr2 = &self.regs.chcfg0r2;
                    } else {
                        let cfgr1 = &self.regs.ch0.cfgr1;
                        let cfgr2 = &self.regs.ch0.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C1 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch1cfgr1;
                        let cfgr2 = &self.regs.ch1cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg1r1;
                        let cfgr2 = &self.regs.chcfg1r2;
                    } else {
                        let cfgr1 = &self.regs.ch1.cfgr1;
                        let cfgr2 = &self.regs.ch1.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C2 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch2cfgr1;
                        let cfgr2 = &self.regs.ch2cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg3r1;
                        let cfgr2 = &self.regs.chcfg3r2;
                    } else {
                        let cfgr1 = &self.regs.ch2.cfgr1;
                        let cfgr2 = &self.regs.ch2.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C3 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch3cfgr1;
                        let cfgr2 = &self.regs.ch3cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg3r1;
                        let cfgr2 = &self.regs.chcfg3r2;
                    } else {
                        let cfgr1 = &self.regs.ch3.cfgr1;
                        let cfgr2 = &self.regs.ch3.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C4 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch4cfgr1;
                        let cfgr2 = &self.regs.ch4cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg4r1;
                        let cfgr2 = &self.regs.chcfg4r2;
                    } else {
                        let cfgr1 = &self.regs.ch4.cfgr1;
                        let cfgr2 = &self.regs.ch4.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C5 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch5cfgr1;
                        let cfgr2 = &self.regs.ch5cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg5r1;
                        let cfgr2 = &self.regs.chcfg5r2;
                    } else {
                        let cfgr1 = &self.regs.ch5.cfgr1;
                        let cfgr2 = &self.regs.ch5.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C6 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch6cfgr1;
                        let cfgr2 = &self.regs.ch6cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg6r1;
                        let cfgr2 = &self.regs.chcfg6r2;
                    } else {
                        let cfgr1 = &self.regs.ch6.cfgr1;
                        let cfgr2 = &self.regs.ch6.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
            DfsdmChannel::C7 => unsafe {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch7cfgr1;
                        let cfgr2 = &self.regs.ch7cfgr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg7r1;
                        let cfgr2 = &self.regs.chcfg7r2;
                    } else {
                        let cfgr1 = &self.regs.ch7.cfgr1;
                        let cfgr2 = &self.regs.ch7.cfgr2;
                    }
                }

                cfgr2.modify(|_, w| {
                    w.dtrbs().bits(self.config.right_shift_bits);
                    w.offset().bits(self.config.offset)
                });
                cfgr1.modify(|_, w| {
                    w.spicksel().bits(self.config.spi_clock as u8);
                    w.chen().set_bit()
                });
            },
        }
    }

    /// Disables the DFSDM peripheral.
    /// By clearing DFEN, any conversion which may be in progress is immediately stopped and
    /// FLTx is put into stop mode. All register settings remain unchanged except
    /// FLTxAWSR and FLTxISR (which are reset).
    pub fn disable_filter(&mut self, filter: Filter) {
        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt0cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm0_cr1;
                    } else {
                        let cr1 = &self.regs.flt0.cr1;
                    }
                }
                cr1.modify(|_, w| w.dfen().clear_bit())
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt1cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let cr1 = &self.regs.dfsdm1_cr1;
                    } else {
                        let cr1 = &self.regs.flt1.cr1;
                    }
                }
                cr1.modify(|_, w| w.dfen().clear_bit())
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt2cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm2_cr1;
                    } else {
                        let cr1 = &self.regs.flt2.cr1;
                    }
                }
                cr1.modify(|_, w| w.dfen().clear_bit())
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt3cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm3_cr1;
                    } else {
                        let cr1 = &self.regs.flt3.cr1;
                    }
                }
                cr1.modify(|_, w| w.dfen().clear_bit())
            }
        }
    }

    /// Configure for PDM microphone(s). Configures the left channel as the `channel` argument here,
    /// and the right channel as `channel` - 1. H742 RM, section 30.4.4
    pub fn setup_pdm_mics(&mut self, channel: DfsdmChannel) {
        // Configuration of serial channels for PDM microphone input:
        // • PDM microphone signals (data, clock) will be connected to DFSDM input serial channel
        // y (DATINy, CKOUT) pins.
        // (Handled by hardware connections and GPIO config)

        match channel {
            DfsdmChannel::C0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch0cfgr1;
                        let cfgr1b = &self.regs.ch7cfgr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg0r1;
                        let cfgr1b = &self.regs.chcfg7r1;
                    } else {
                        let cfgr1 = &self.regs.ch0.cfgr1;
                        let cfgr1b = &self.regs.ch7.cfgr1;
                    }
                }

                cfgr1.modify(|_, w| unsafe {
                    // • Channel y will be configured: CHINSEL = 0 (input from given channel pins: DATINy,
                    // CKINy).
                    w.chinsel().clear_bit();
                    // • Channel y: SITP[1:0] = 0 (rising edge to strobe data) => left audio channel on channel
                    // y.
                    w.sitp().bits(0)
                });

                cfgr1b.modify(|_, w| unsafe {
                    // • Channel (y-1) (modulo 8) will be configured: CHINSEL = 1 (input from the following
                    // channel ((y-1)+1) pins: DATINy, CKINy).
                    w.chinsel().set_bit();
                    // • Channel (y-1): SITP[1:0] = 1 (falling edge to strobe data) => right audio channel on
                    // channel y-1.
                    w.sitp().bits(1)
                });
            }
            DfsdmChannel::C1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch1cfgr1;
                        let cfgr1b = &self.regs.ch0cfgr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg1r1;
                        let cfgr1b = &self.regs.chcfg0r1;
                    } else {
                        let cfgr1 = &self.regs.ch1.cfgr1;
                        let cfgr1b = &self.regs.ch0.cfgr1;
                    }
                }

                cfgr1.modify(|_, w| unsafe {
                    w.chinsel().clear_bit();
                    w.sitp().bits(0)
                });

                cfgr1b.modify(|_, w| unsafe {
                    w.chinsel().set_bit();
                    w.sitp().bits(1)
                });
            }
            DfsdmChannel::C2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch2cfgr1;
                        let cfgr1b = &self.regs.ch1cfgr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg2r1;
                        let cfgr1b = &self.regs.chcfg1r1;
                    } else {
                        let cfgr1 = &self.regs.ch2.cfgr1;
                        let cfgr1b = &self.regs.ch1.cfgr1;
                    }
                }

                cfgr1.modify(|_, w| unsafe {
                    w.chinsel().clear_bit();
                    w.sitp().bits(0)
                });

                cfgr1b.modify(|_, w| unsafe {
                    w.chinsel().set_bit();
                    w.sitp().bits(1)
                });
            }
            DfsdmChannel::C3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cfgr1 = &self.regs.ch3cfgr1;
                        let cfgr1b = &self.regs.ch2cfgr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cfgr1 = &self.regs.chcfg3r1;
                        let cfgr1b = &self.regs.chcfg2r1;
                    } else {
                        let cfgr1 = &self.regs.ch3.cfgr1;
                        let cfgr1b = &self.regs.ch2.cfgr1;
                    }
                }

                cfgr1.modify(|_, w| unsafe {
                    w.chinsel().clear_bit();
                    w.sitp().bits(0)
                });

                cfgr1b.modify(|_, w| unsafe {
                    w.chinsel().set_bit();
                    w.sitp().bits(1)
                });
            }
            _ => unimplemented!(),
        }

        // • Two DFSDM DfsdmChannels will be assigned to channel y and channel (y-1) (to DfsdmChannel left and
        // right channels from PDM microphone).
    }

    /// Initiate a converssion. See H742 RM, section 30.4.15: Launching conversions
    pub fn start_conversion(&self, filter: Filter) {
        // todo: set up via interrupts/DMA/separate reading adn conversion etc
        // todo: regular vs injected conversions

        // Regular conversions can be launched using the following methods:
        // • Software: by writing ‘1’ to RSWSTART in the FLTxCR1 register.
        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt0cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm0_cr1;
                    } else {
                        let cr1 = &self.regs.flt0.cr1;
                    }
                }
                cr1.modify(|_, w| w.rswstart().set_bit())
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt1cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let cr1 = &regs.dfsdm1_cr1;
                    } else {
                        let cr1 = &self.regs.flt1.cr1;
                    }
                }
                cr1.modify(|_, w| w.rswstart().set_bit())
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt2cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm2_cr1;
                    } else {
                        let cr1 = &self.regs.flt2.cr1;
                    }
                }
                cr1.modify(|_, w| w.rswstart().set_bit())
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt3cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm3_cr1;
                    } else {
                        let cr1 = &self.regs.flt3.cr1;
                    }
                }
                cr1.modify(|_, w| w.rswstart().set_bit())
            }
        }

        // • Synchronous with FLT0 if RSYNC=1: for FLTx (x>0), a regular
        // conversion is automatically launched when in FLT0; a regular conversion is
        // started by software (RSWSTART=1 in FLT0CR2 register). Each regular
        // conversion in FLTx (x>0) is always executed according to its local
        // configuration settings (RCONT, RCH, etc.).
        // Only one regular conversion can be pending or ongoing at a given time. Thus, any request
        // to launch a regular conversion is ignored if another request for a regular conversion has
        // already been issued but not yet completed. A regular conversion can be pending if it was
        // interrupted by an injected conversion or if it was started while an injected conversion was in
        // progress. This pending regular conversion is then delayed and is performed when all
        // injected conversion are finished. Any delayed regular conversion is signalized by RPEND bit
        // in FLTxRDATAR register.
    }

    /// Initiate an injected conversion. See H742 RM, section 30.4.15: Launching conversions
    pub fn start_injected_conversion(&self, filter: Filter) {
        // Injected conversions can be launched using the following methods:
        // • Software: writing ‘1’ to JSWSTART in the FLTxCR1 register.
        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt0cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm0_cr1;
                    }else {
                        let cr1 = &self.regs.flt0.cr1;
                    }
                }
                cr1.modify(|_, w| w.jswstart().set_bit())
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt1cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let cr1 = &self.regs.dfsdm1_cr1;
                    } else {
                        let cr1 = &self.regs.flt1.cr1;
                    }
                }
                cr1.modify(|_, w| w.jswstart().set_bit())
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt2cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm2_cr1;
                    } else {
                        let cr1 = &self.regs.flt2.cr1;
                    }
                }
                cr1.modify(|_, w| w.jswstart().set_bit())
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt3cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm3_cr1;
                    } else {
                        let cr1 = &self.regs.flt3.cr1;
                    }
                }
                cr1.modify(|_, w| w.jswstart().set_bit())
            }
        }

        // • Trigger: JEXTSEL[4:0] selects the trigger signal while JEXTEN activates the trigger
        // and selects the active edge at the same time (see the FLTxCR1 register).
        // • Synchronous with FLT0 if JSYNC=1: for FLTx (x>0), an injected
        // conversion is automatically launched when in FLT0; the injected conversion is
        // started by software (JSWSTART=1 in FLT0CR2 register). Each injected
        // conversion in FLTx (x>0) is always executed according to its local
        // configuration settings (JSCAN, JCHG, etc.).
        // If the scan conversion is enabled (bit JSCAN=1) then, each time an injected conversion is
        // triggered, all of the selected channels in the injected group (JCHG[7:0] bits in
        // FLTxJCHGR register) are converted sequentially, starting with the lowest channel
        // (channel 0, if selected).
        // If the scan conversion is disabled (bit JSCAN=0) then, each time an injected conversion is
        // triggered, only one of the selected channels in the injected group (JCHG[7:0] bits in
        // FLTxJCHGR register) is converted and the channel selection is then moved to the
        // next selected channel. Writing to the JCHG[7:0] bits when JSCAN=0 sets the channel
        // selection to the lowest selected injected channel.
        // Only one injected conversion can be ongoing at a given time. Thus, any request to launch
        // an injected conversion is ignored if another request for an injected conversion has already
        // been issued but not yet completed.
    }

    /// Read regular conversion data from the FLTxRDATAR register. Suitable for use after a conversion is complete.
    /// "Signed data format in registers: Data is in a signed format in registers for final output data,
    /// analog watchdog, extremes detector, offset correction. The msb of output data word
    /// represents the sign of value (two’s complement format)."
    ///
    /// See H742 RM, section 30.4.13: Data unitblock for details.
    /// "The right bit-shift of final data is performed in this module because the final data width is 24-
    /// bit and data coming from the processing path can be up to 32 bits. This right bit-shift is
    /// configurable in the range 0-31 bits for each selected input channel (see DTRBS[4:0] bits in
    /// DFSDM_CHyCFGR2 register). The right bit-shift is rounding the result to nearest integer
    /// value. The sign of shifted result is maintained - to have valid 24-bit signed format of result
    /// data.
    /// In the next step, an offset correction of the result is performed. The offset correction value
    /// (OFFSET[23:0] stored in register DFSDM_CHyCFGR2) is subtracted from the output data
    /// for a given channel. Data in the OFFSET[23:0] field is set by software by the appropriate
    /// calibration routine."
    pub fn read(&self, filter: Filter) -> i32 {
        match filter {
            // We read the whole register, then convert to i32, then shift right, so we can take
            // advantage of the sign bit being in the same place in the data register as for 32-bit
            // integers in Rust. (Data is the left 24 most bits of the register. The shift discards
            // the other fields).
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt0rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm0_rdatar;
                    } else {
                        let rdatar = &self.regs.flt0.rdatar;
                    }
                }
                (rdatar.read().bits() as i32) >> 8
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => 0,
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt1rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let rdatar = &self.regs.dfsdm1_rdatar;
                    } else {
                        let rdatar = &self.regs.flt1.rdatar;
                    }
                }
                (rdatar.read().bits() as i32) >> 8
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt2rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm2_rdatar;
                    } else {
                        let rdatar = &self.regs.flt2.rdatar;
                    }
                }
                (rdatar.read().bits() as i32) >> 8
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt3rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm3_rdatar;
                    } else {
                        let rdatar = &self.regs.flt3.rdatar;
                    }
                }
                (rdatar.read().bits() as i32) >> 8
            }
        }
    }

    /// Read injected conversion data from the FLTxJDATAR register.
    /// Suitable for use after a conversion is complete.
    pub fn read_injected(&self, filter: Filter) -> i32 {
        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let jdatar = &self.regs.flt0jdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let jdatar = &self.regs.dfsdm0_jdatar;
                    } else {
                        let jdatar = &self.regs.flt0.jdatar;
                    }
                }
                (jdatar.read().bits() as i32) >> 8
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => 0,
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let jdatar = &self.regs.flt1jdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let jdatar = &self.regs.dfsdm1_jdatar;
                    } else {
                        let jdatar = &self.regs.flt1.jdatar;
                    }
                }
                (jdatar.read().bits() as i32) >> 8
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let jdatar = &self.regs.flt2jdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let jdatar = &self.regs.dfsdm2_jdatar;
                    } else {
                        let jdatar = &self.regs.flt2.jdatar;
                    }
                }
                (jdatar.read().bits() as i32) >> 8
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let jdatar = &self.regs.flt3jdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let jdatar = &self.regs.dfsdm3_jdatar;
                    } else {
                        let jdatar = &self.regs.flt3.jdatar;
                    }
                }
                (jdatar.read().bits() as i32) >> 8
            }
        }
        // todo: JDATACH to know which channel was converted??
        // todo isn't this implied to the register we choose to sue?
    }

    /// Read data from SAI with DMA. H743 RM, section 30.6: DFSDM DMA transfer
    /// To decrease the CPU intervention, conversions can be transferred into memory using a
    /// DMA transfer. A DMA transfer for injected conversions is enabled by setting bit JDMAEN=1
    /// in FLTxCR1 register. A DMA transfer for regular conversions is enabled by setting
    /// bit RDMAEN=1 in FLTxCR1 register.
    /// Note: With a DMA transfer, the interrupt flag is automatically cleared at the end of the injected or
    /// regular conversion (JEOCF or REOCF bit in FLTxISR register) because DMA is
    /// reading FLTxJDATAR or FLTxRDATAR register
    ///
    /// Note that this reads the entire rdatar register into memory, not just the rdata field.
    /// You need to shift the result 8 bits to the result after reading the values from memory
    /// to discard the other fields. (The integer signing is unchanged, since the 24-bit integer data
    /// is aligned to the left of the 32-bit register, which maps to an `i32` here.)
    #[cfg(not(any(feature = "f4", feature = "l552")))]
    pub unsafe fn read_dma(
        &mut self,
        buf: &mut [i32],
        filter: Filter,
        dma_channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma_periph: dma::DmaPeriph,
        // dma: &mut Dma<D>,
    ) {
        // where
        //     D: Deref<Target = dma_p::RegisterBlock>,
        // {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        #[cfg(feature = "f3")]
        let dma_channel = match filter {
            Filter::F0 => DmaInput::Dfsdm1F0.dma1_channel(),
            Filter::F1 => DmaInput::Dfsdm1F1.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                match filter {
                    Filter::F0 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F0),
                    Filter::F1 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F1),
                    Filter::F2 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F2),
                    Filter::F3 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F3),
                };
            }
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                match filter {
                    Filter::F0 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F0),
                    Filter::F1 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F1),
                    Filter::F2 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F2),
                    Filter::F3 => dma::channel_select(&mut regs, DmaInput::Dfsdm1F3),
                };
            }
        }

        match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt0cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm0_cr1;
                    } else {
                        let cr1 = &self.regs.flt0.cr1;
                    }
                }
                cr1.modify(|_, w| w.rdmaen().set_bit())
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt1cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let cr1 = &self.regs.dfsdm1_cr1;
                    } else {
                        let cr1 = &self.regs.flt1.cr1;
                    }
                }
                cr1.modify(|_, w| w.rdmaen().set_bit())
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt2cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm2_cr1;
                    } else {
                        let cr1 = &self.regs.flt2.cr1;
                    }
                }
                cr1.modify(|_, w| w.rdmaen().set_bit())
            }

            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr1 = &self.regs.flt3cr1;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr1 = &self.regs.dfsdm3_cr1;
                    } else {
                        let cr1 = &self.regs.flt3.cr1;
                    }
                }
                cr1.modify(|_, w| w.rdmaen().set_bit())
            }
        }

        let periph_addr = match filter {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt0rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm0_rdatar;
                    } else {
                        let rdatar = &self.regs.flt0.rdatar;
                    }
                }
                &rdatar as *const _ as u32
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => 0,
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt1rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let rdatar = &self.regs.dfsdm1_rdatar;
                    } else {
                        let rdatar = &self.regs.flt1.rdatar;
                    }
                }
                &rdatar as *const _ as u32
            }

            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt2rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm2_rdatar;
                    } else {
                        let rdatar = &self.regs.flt2.rdatar;
                    }
                }
                &rdatar as *const _ as u32
            }

            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let rdatar = &self.regs.flt3rdatar;
                    } else if #[cfg(any(feature = "l4"))] {
                        let rdatar = &self.regs.dfsdm3_rdatar;
                    } else {
                        let rdatar = &self.regs.flt3.rdatar;
                    }
                }
                &rdatar as *const _ as u32
            }
        };

        // todo: Injected support. Should just need to add the option flag and enable `jdmaen()` bits
        // todo instead of `rdmaen()`, and use the `jdatar` periph addr.

        // todo: Do we want this? If so, where?
        self.start_conversion(filter);

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        match dma_periph {
            dma::DmaPeriph::Dma1 => {
                let mut regs = unsafe { &(*DMA1::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    dma_channel,
                    periph_addr,
                    ptr as u32,
                    len,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S32, // For 24 bits
                    dma::DataSize::S32,
                    channel_cfg,
                );
            }
            dma::DmaPeriph::Dma2 => {
                let mut regs = unsafe { &(*pac::DMA2::ptr()) };
                dma::cfg_channel(
                    &mut regs,
                    dma_channel,
                    periph_addr,
                    ptr as u32,
                    len,
                    dma::Direction::ReadFromPeriph,
                    dma::DataSize::S32, // For 24 bits
                    dma::DataSize::S32,
                    channel_cfg,
                );
            }
        }
    }

    /// Enable a specific type of interrupt. See H743 RM, section 30.5: DFSDM interrupts
    pub fn enable_interrupt(&mut self, interrupt_type: DfsdmInterrupt, channel: Filter) {
        // todo: Macro to reduce DRY here?
        match channel {
            Filter::F0 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr2 = &self.regs.flt0cr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr2 = &self.regs.dfsdm0_cr2;
                    } else {
                        let cr2 = &self.regs.flt0.cr2;
                    }
                }

                cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            #[cfg(feature = "l4x6")]
            Filter::F1 => (),
            #[cfg(not(feature = "l4x6"))]
            Filter::F1 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr2 = &self.regs.flt1cr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        // let cr2 = &self.regs.dfsdm1_cr2;
                    } else {
                        let cr2 = &self.regs.flt1.cr2;
                    }
                }
                cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            Filter::F2 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr2 = &self.regs.flt1cr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr2 = &self.regs.dfsdm2_cr2;
                    } else {
                        let cr2 = &self.regs.flt2.cr2;
                    }
                }

                cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            Filter::F3 => {
                cfg_if! {
                    if #[cfg(any(feature = "l5"))] {
                        let cr2 = &self.regs.flt3cr2;
                    } else if #[cfg(any(feature = "l4"))] {
                        let cr2 = &self.regs.dfsdm3_cr2;
                    } else {
                        let cr2 = &self.regs.flt3.cr2;
                    }
                }

                cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
        }
    }

    // todo: write_dma

    /// Clears the interrupt pending flag for a specific type of interrupt. Note that to clear
    /// EndofInjectedConversion, or EndOfConversion interrupt,s read the FLTxJDATAR or FLTxRDATAR
    /// registers respectively.
    pub fn clear_interrupt(&mut self, interrupt_type: DfsdmInterrupt, channel: Filter) {

        // todo figure out what's wrong and put back.
        // match channel {
        //      DfsdmChannel::F0 => {
        //          self.regs.flt0icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf.set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              // todo: Possibly clrawltf bit for low ADW clear!
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //      DfsdmChannel::F1 => {
        //          self.regs.flt1icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf().set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //      DfsdmChannel::F2 => {
        //          self.regs.flt2icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf().set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //      DfsdmChannel::F3 => {
        //          self.regs.flt3icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf().set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //  }
    }
}
