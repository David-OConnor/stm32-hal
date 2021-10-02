//! Digital filter for sigma delta modulators (DFSDM) support. Seem H742 RM, chapter 30.

use core::ops::Deref;

use cortex_m::interrupt::free;

use crate::{
    clocks::Clocks,
    pac::{self, RCC},
    rcc_en_reset,
};

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(
    feature = "f3",
    feature = "l4",
    feature = "g4",
    feature = "h7",
    feature = "wb"
))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "f4", feature = "l5")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

#[derive(Clone, Copy)]
pub enum Filter {
    F0,
    F1,
    #[cfg(not(any(feature = "l4")))]
    F2,
    #[cfg(not(any(feature = "l4")))]
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
    /// Sinc6 filter type
    Sinc6 = 6,
    /// Sinc7 filter type
    Sinc7 = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Toggle clock clourse between system and audio clocks. Sets CH0CFGR1 register, CKOUTSRC field.
/// (todo: Typo from othe rRM??)
pub enum DfsdmClockSrc {
    /// Source for output clock is from system clock
    SysClk = 0,
    /// Source for output clock is from audio clock
    AudioClk = 1,
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

/// Configuration for the DFSDM peripheral.
pub struct DfsdmConfig {
    pub clock_src: DfsdmClockSrc,
    // pub continuous: Continuous, // todo: Fast, per Example cfg of PDM?
    pub continuous: Continuous,
    pub filter_order: FilterOrder,
    // Sinc filter oversampling ratio. Also known as decimation ratio
    pub filter_oversampling_ratio: u16,
    // Integrator oversampling ratio. Also known as averaging (?) ratio
    pub integrator_oversampling_ratio: u8,
}

impl Default for DfsdmConfig {
    fn default() -> Self {
        Self {
            clock_src: DfsdmClockSrc::AudioClk,
            continuous: Continuous::ContinuousFastMode,
            filter_order: FilterOrder::Sinc4, // From the PDM mic AN
            filter_oversampling_ratio: 64,    // From the PDM mic AN
            integrator_oversampling_ratio: 1, // From the PDM mic AN
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
    R: Deref<Target = pac::dfsdm::RegisterBlock>,
{
    /// Initialize a DFSDM peripheral, including  enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, config: DfsdmConfig) -> Self {
        free(|cs| {
            let rcc = unsafe { &(*RCC::ptr()) };
            rcc_en_reset!(apb2, dfsdm1, rcc);
        });

        // The frequency of this CKOUT signal is derived from DFSDM clock or from audio clock (see
        // CKOUTSRC bit in CH0CFGR1 register) divided by a predivider (see CKOUTDIV
        // bits in CH0CFGR1 register). If the output clock is stopped, then CKOUT signal is
        // set to low state (output clock can be stopped by CKOUTDIV=0 in CHyCFGR1
        // register or by DFSDMEN=0 in CH0CFGR1 register).
        // The output clock signal frequency must be in the range 0 - 20 MHz.

        // PDM mic AN: The clock divider value must respect the following formula:
        // Divider = DFSDM Clock Source / (AUDIO_SAMPLING_FREQUENCY ×
        // DECIMATION_FACTOR)
        // todo: Temp hardcoding audio sampling ratio to 48_000 and clock speed.
        let clock_speed = 3_072_000;
        let audio_sampling_freq = 48_000;
        let divider = clock_speed / (audio_sampling_freq * config.filter_oversampling_ratio as u32);

        // Note that for the defaults we have here, this actually works out to 1.
        regs.ch0cfgr1
            .modify(|_, w| unsafe { w.ckoutsrc().bit(config.clock_src as u8 != 0) });

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

        // todo: FOSR config for filter SINC etc.

        // todo: Continuous and fast continuous modes.

        // todo: AN says set continuous
        // todo Software trigger to start regular conversion
        // todo no injection.
        // todo Use Sing 4 or 5 filter. Set this in config. Also

        Self { regs, config }
    }

    /// Enables the DFSDM peripheral.
    /// The DFSDM interface is globally enabled by setting DFSDMEN=1 in the
    /// CH0CFGR1 register. Once DFSDM is globally enabled, all input channels (y=0..7)
    /// and digital filters FLTx (x=0..3) start to work if their enable bits are set (channel
    /// enable bit CHEN in CHyCFGR1 and FLTx enable bit DFEN in
    /// FLTxCR1).
    pub fn enable(&mut self) {
        self.regs.ch0cfgr1.modify(|_, w| w.dfsdmen().set_bit());
    }

    /// Disables the DFSDM peripheral.
    /// DFSDM must be globally disabled (by DFSDMEN=0 in CH0CFGR1) before
    /// stopping the system clock to enter in the STOP mode of the device
    pub fn disable(&mut self) {
        self.regs.ch0cfgr1.modify(|_, w| w.dfsdmen().clear_bit());
    }

    /// Configures and enables the DFSDM filter for a given channel.
    /// Digital filter x FLTx (x=0..3) is enabled by setting DFEN=1 in the
    /// FLTxCR1 register. Once FLTx is enabled (DFEN=1), both Sincx
    /// digital filter unit and integrator unit are reinitialized.
    pub fn enable_filter(&mut self, filter: Filter, channel: DfsdmChannel) {
        // Setting RCONT in the FLTxCR1 register causes regular conversions to execute in
        // continuous mode. RCONT=1 means that the channel selected by RCH[2:0] is converted
        // repeatedly after ‘1’ is written to RSWSTART.
        // RM0433 Rev 7 1153/3319
        // RM0433 Digital filter for sigma delta modulators (DFSDM)
        // 1184
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

        match filter {
            Filter::F0 => {
                self.regs.flt0cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
                self.regs.flt0fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio);
                    w.iosr().bits(self.config.integrator_oversampling_ratio)
                })
            }
            Filter::F1 => {
                self.regs.flt1cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
                self.regs.flt1fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio);
                    w.iosr().bits(self.config.integrator_oversampling_ratio)
                })
            }
            Filter::F2 => {
                self.regs.flt2cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
                self.regs.flt2fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio);
                    w.iosr().bits(self.config.integrator_oversampling_ratio)
                })
            }
            Filter::F3 => {
                self.regs.flt3cr1.modify(|_, w| unsafe {
                    w.rcont().bit(self.config.continuous != Continuous::OneShot);
                    w.fast()
                        .bit(self.config.continuous == Continuous::ContinuousFastMode);
                    w.rch().bits(channel as u8);
                    w.dfen().set_bit()
                });
                self.regs.flt3fcr.modify(|_, w| unsafe {
                    w.ford().bits(self.config.filter_order as u8);
                    w.fosr().bits(self.config.filter_oversampling_ratio);
                    w.iosr().bits(self.config.integrator_oversampling_ratio)
                })
            }
        }

        match channel {
            DfsdmChannel::C0 => self.regs.flt0cr1.modify(|_, w| unsafe {
                w.rcont().bit(self.config.continuous != Continuous::OneShot);
                w.fast()
                    .bit(self.config.continuous == Continuous::ContinuousFastMode);
                w.rch().bits(channel as u8);
                w.dfen().set_bit()
            }),
            DfsdmChannel::C1 => self.regs.flt1cr1.modify(|_, w| unsafe {
                w.rcont().bit(self.config.continuous != Continuous::OneShot);
                w.fast()
                    .bit(self.config.continuous == Continuous::ContinuousFastMode);
                w.rch().bits(channel as u8);
                w.dfen().set_bit()
            }),
            _ => unimplemented!(),
        }
    }

    /// Disables the DFSDM peripheral.
    /// By clearing DFEN, any conversion which may be in progress is immediately stopped and
    /// FLTx is put into stop mode. All register settings remain unchanged except
    /// FLTxAWSR and FLTxISR (which are reset).
    pub fn disable_filter(&mut self, channel: Filter) {
        match channel {
            Filter::F0 => self.regs.flt0cr1.modify(|_, w| w.dfen().clear_bit()),
            Filter::F1 => self.regs.flt1cr1.modify(|_, w| w.dfen().clear_bit()),
            Filter::F2 => self.regs.flt2cr1.modify(|_, w| w.dfen().clear_bit()),
            Filter::F3 => self.regs.flt3cr1.modify(|_, w| w.dfen().clear_bit()),
        }
    }

    /// Configure for PDM microphone(s). Configures the right channel as the `channel` argument here,
    /// and the left channel as `channel` - 1. So, Channel options for this argument must be C1, C2,
    /// or C3; this corresponds to the right channel. H742 RM, section 30.4.4
    pub fn setup_pdm_mics(&mut self, channel: DfsdmChannel) {
        // Configuration of serial channels for PDM microphone input:
        // • PDM microphone signals (data, clock) will be connected to DFSDM input serial channel
        // y (DATINy, CKOUT) pins.
        // (Handled by hardware connections and GPIO config)
        // • Channel y will be configured: CHINSEL = 0 (input from given channel pins: DATINy,
        // CKINy).
        // todo: Consolidate these calls if you can

        match channel {
            DfsdmChannel::C0 => self.regs.ch0cfgr1.modify(|_, w| w.chinsel().clear_bit()),
            DfsdmChannel::C1 => self.regs.ch1cfgr1.modify(|_, w| w.chinsel().clear_bit()),
            DfsdmChannel::C2 => self.regs.ch2cfgr1.modify(|_, w| w.chinsel().clear_bit()),
            DfsdmChannel::C3 => self.regs.ch3cfgr1.modify(|_, w| w.chinsel().clear_bit()),
            _ => unimplemented!(),
        }
        // • Channel (y-1) (modulo 8) will be configured: CHINSEL = 1 (input from the following
        // channel ((y-1)+1) pins: DATINy, CKINy).
        match channel {
            DfsdmChannel::C0 => self.regs.ch0cfgr1.modify(|_, w| w.chinsel().set_bit()),
            DfsdmChannel::C1 => self.regs.ch1cfgr1.modify(|_, w| w.chinsel().set_bit()),
            DfsdmChannel::C2 => self.regs.ch2cfgr1.modify(|_, w| w.chinsel().set_bit()),
            DfsdmChannel::C3 => self.regs.ch3cfgr1.modify(|_, w| w.chinsel().set_bit()),
            _ => unimplemented!(),
        }
        // • Channel y: SITP[1:0] = 0 (rising edge to strobe data) => left audio channel on channel
        // y.
        match channel {
            DfsdmChannel::C0 => self
                .regs
                .ch0cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            DfsdmChannel::C1 => self
                .regs
                .ch1cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            DfsdmChannel::C2 => self
                .regs
                .ch2cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            DfsdmChannel::C3 => self
                .regs
                .ch3cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            _ => unimplemented!(),
        }
        // • Channel (y-1): SITP[1:0] = 1 (falling edge to strobe data) => right audio channel on
        // channel y-1.
        match channel {
            DfsdmChannel::C0 => self
                .regs
                .ch0cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
            DfsdmChannel::C1 => self
                .regs
                .ch1cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
            DfsdmChannel::C2 => self
                .regs
                .ch2cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
            DfsdmChannel::C3 => self
                .regs
                .ch3cfgr1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
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
            Filter::F0 => self.regs.flt0cr1.modify(|_, w| w.rswstart().set_bit()),
            Filter::F1 => self.regs.flt1cr1.modify(|_, w| w.rswstart().set_bit()),
            Filter::F2 => self.regs.flt2cr1.modify(|_, w| w.rswstart().set_bit()),
            Filter::F3 => self.regs.flt3cr1.modify(|_, w| w.rswstart().set_bit()),
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
            Filter::F0 => self.regs.flt0cr1.modify(|_, w| w.jswstart().set_bit()),
            Filter::F1 => self.regs.flt1cr1.modify(|_, w| w.jswstart().set_bit()),
            Filter::F2 => self.regs.flt2cr1.modify(|_, w| w.jswstart().set_bit()),
            Filter::F3 => self.regs.flt3cr1.modify(|_, w| w.jswstart().set_bit()),
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
    pub fn read(&self, filter: Filter) -> u32 {
        match filter {
            Filter::F0 => self.regs.flt0rdatar.read().rdata().bits(),
            Filter::F1 => self.regs.flt1rdatar.read().rdata().bits(),
            Filter::F2 => self.regs.flt2rdatar.read().rdata().bits(),
            Filter::F3 => self.regs.flt3rdatar.read().rdata().bits(),
        }
        // todo: RDATACH to know which channel was converted??
        // todo isn't this implied to the register we choose to sue?
    }

    /// Read injected conversion data from the FLTxJDATAR register.
    /// Suitable for use after a conversion is complete.
    pub fn read_injected(&self, filter: Filter) -> u32 {
        match filter {
            Filter::F0 => self.regs.flt0jdatar.read().jdata().bits(),
            Filter::F1 => self.regs.flt1jdatar.read().jdata().bits(),
            Filter::F2 => self.regs.flt2jdatar.read().jdata().bits(),
            Filter::F3 => self.regs.flt3jdatar.read().jdata().bits(),
        }
        // todo: JDATACH to know which channel was converted??
        // todo isn't this implied to the register we choose to sue?
    }

    /// Read data from SAI with DMA. H743 RM, section 30.6: DFSDM DMA transfer
    /// To decrease the CPU intervention, conversions can be transferred into memory using a
    // DMA transfer. A DMA transfer for injected conversions is enabled by setting bit JDMAEN=1
    // in FLTxCR1 register. A DMA transfer for regular conversions is enabled by setting
    // bit RDMAEN=1 in FLTxCR1 register.
    // Note: With a DMA transfer, the interrupt flag is automatically cleared at the end of the injected or
    // regular conversion (JEOCF or REOCF bit in FLTxISR register) because DMA is
    // reading FLTxJDATAR or FLTxRDATAR register
    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    pub unsafe fn read_dma<D>(
        &mut self,
        buf: &mut [u32],
        filter: Filter,
        dma_channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_mut_ptr(), buf.len());

        // todo: DMA2 support.

        #[cfg(any(feature = "f3", feature = "l4"))]
        let channel = match sai_channel {
            Filter::F0 => DmaInput::DfsdmCh0.dma1_channel(),
            Filter::F1 => DmaInput::DfsdmCh1.dma1_channel(),
        };

        #[cfg(feature = "l4")]
        match filter {
            Filter::F0 => dma.channel_select(DmaInput::DfsdmCh0),
            Filter::F1 => dma.channel_select(DmaInput::DfsdmCh1),
        };

        match filter {
            Filter::F0 => self.regs.flt0cr1.modify(|_, w| w.jdmaen().set_bit()),
            Filter::F1 => self.regs.flt1cr1.modify(|_, w| w.jdmaen().set_bit()),
            #[cfg(not(any(feature = "l4")))]
            Filter::F2 => self.regs.flt2cr1.modify(|_, w| w.jdmaen().set_bit()),
            #[cfg(not(any(feature = "l4")))]
            Filter::F3 => self.regs.flt3cr1.modify(|_, w| w.jdmaen().set_bit()),
        }

        // todo: Injected?

        let periph_addr = match filter {
            Filter::F0 => &self.regs.flt0rdatar as *const _ as u32,
            Filter::F1 => &self.regs.flt1rdatar as *const _ as u32,
            #[cfg(not(any(feature = "l4")))]
            Filter::F2 => &self.regs.flt2rdatar as *const _ as u32,
            #[cfg(not(any(feature = "l4")))]
            Filter::F3 => &self.regs.flt3rdatar as *const _ as u32,
        };

        #[cfg(feature = "h7")]
        let len = len as u32;
        #[cfg(not(feature = "h7"))]
        let len = len as u16;

        dma.cfg_channel(
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

    /// Enable a specific type of interrupt. See H743 RM, section 30.5: DFSDM interrupts
    pub fn enable_interrupt(&mut self, interrupt_type: DfsdmInterrupt, channel: Filter) {
        // todo: Macro to reduce DRY here?
        match channel {
            Filter::F0 => {
                self.regs.flt0cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            Filter::F1 => {
                self.regs.flt1cr2.modify(|_, w| match interrupt_type {
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
                self.regs.flt2cr2.modify(|_, w| match interrupt_type {
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
                self.regs.flt3cr2.modify(|_, w| match interrupt_type {
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

    // todo: read_dma

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
