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
pub enum DfsdmChannel {
    C0,
    C1,
    C2,
    C3,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum DfsdmClockSrc {
    asdf = 0,
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

/// Configuration for the DFSDM peripheral.
pub struct DfsdmConfig {
    pub clock_src: DfsdmClockSrc,
}

impl Default for DfsdmConfig {
    fn default() -> Self {
        Self {
            clock_src: DfsdmClockSrc::asdf,
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
        // CKOUTSRC bit in DFSDM_CH0CFGR1 register) divided by a predivider (see CKOUTDIV
        // bits in DFSDM_CH0CFGR1 register). If the output clock is stopped, then CKOUT signal is
        // set to low state (output clock can be stopped by CKOUTDIV=0 in DFSDM_CHyCFGR1
        // register or by DFSDMEN=0 in DFSDM_CH0CFGR1 register).
        // The output clock signal frequency must be in the range 0 - 20 MHz.
        regs.dfsdm_chcfg0r1.modify(|_, w| unsafe {
            w.ckoutdiv().bits(0); // todo?
            w.ckoutsrc().bit(config.clock_src as u8 != 0)
        });

        // Configuring the input serial interface
        // The following parameters must be configured for the input serial interface:
        // • Output clock predivider. There is a programmable predivider to generate the output
        // clock from DFSDM clock (2 - 256). It is defined by CKOUTDIV[7:0] bits in
        // DFSDM_CH0CFGR1 register.
        // • Serial interface type and input clock phase. Selection of SPI or Manchester coding
        // and sampling edge of input clock. It is defined by SITP [1:0] bits in
        // DFSDM_CHyCFGR1 register.
        // • Input clock source. External source from CKINy pin or internal from CKOUT pin. It is
        // defined by SPICKSEL[1:0] field in DFSDM_CHyCFGR1 register.
        // • Final data right bit-shift. Defines the final data right bit shift to have the result aligned
        // to a 24-bit value. It is defined by DTRBS[4:0] in DFSDM_CHyCFGR2 register.
        // • Channel offset per channel. Defines the analog offset of a given serial channel (offset
        // of connected external Σ∆ modulator). It is defined by OFFSET[23:0] bits in
        // DFSDM_CHyCFGR2 register.
        // • short-circuit detector and clock absence per channel enable. To enable or disable
        // the short-circuit detector (by SCDEN bit) and the clock absence monitoring (by
        // CKABEN bit) on a given serial channel in register DFSDM_CHyCFGR1.
        // • Analog watchdog filter and short-circuit detector threshold settings. To configure
        // channel analog watchdog filter parameters and channel short-circuit detector
        // parameters. Configurations are defined in DFSDM_CHyAWSCDR register.

        // todo: FOSR config for filter SINC etc.

        // todo: Continuous and fast continuous modes.

        Self { regs, config }
    }

    /// Enables the DFSDM peripheral.
    /// The DFSDM interface is globally enabled by setting DFSDMEN=1 in the
    /// DFSDM_CH0CFGR1 register. Once DFSDM is globally enabled, all input channels (y=0..7)
    /// and digital filters DFSDM_FLTx (x=0..3) start to work if their enable bits are set (channel
    /// enable bit CHEN in DFSDM_CHyCFGR1 and DFSDM_FLTx enable bit DFEN in
    /// DFSDM_FLTxCR1).
    pub fn enable(&mut self) {
        self.regs
            .dfsdm_chcfg0r1
            .modify(|_, w| w.dfsdmen().set_bit());
    }

    /// Disables the DFSDM peripheral.
    /// DFSDM must be globally disabled (by DFSDMEN=0 in DFSDM_CH0CFGR1) before
    /// stopping the system clock to enter in the STOP mode of the device
    pub fn disable(&mut self) {
        self.regs
            .dfsdm_chcfg0r1
            .modify(|_, w| w.dfsdmen().clear_bit());
    }

    /// Enables the DFSDM filter for a given channel
    /// Digital filter x DFSDM_FLTx (x=0..3) is enabled by setting DFEN=1 in the
    /// DFSDM_FLTxCR1 register. Once DFSDM_FLTx is enabled (DFEN=1), both Sincx
    /// digital filter unit and integrator unit are reinitialized
    pub fn enable_filter(&mut self, channel: DfsdmChannel) {
        match channel {
            DfsdmChannel::C0 => self.regs.dfsdm0_cr1.modify(|_, w| w.dfen().set_bit()),
            DfsdmChannel::C1 => self.regs.dfsdm1_cr1.modify(|_, w| w.dfen().set_bit()),
            DfsdmChannel::C2 => self.regs.dfsdm2_cr1.modify(|_, w| w.dfen().set_bit()),
            DfsdmChannel::C3 => self.regs.dfsdm3_cr1.modify(|_, w| w.dfen().set_bit()),
        }
    }

    /// Disables the DFSDM peripheral.
    /// By clearing DFEN, any conversion which may be in progress is immediately stopped and
    /// DFSDM_FLTx is put into stop mode. All register settings remain unchanged except
    /// DFSDM_FLTxAWSR and DFSDM_FLTxISR (which are reset).
    pub fn disble_filter(&mut self, channel: DfsdmChannel) {
        match channel {
            DfsdmChannel::C0 => self.regs.dfsdm0_cr1.modify(|_, w| w.dfen().clear_bit()),
            DfsdmChannel::C1 => self.regs.dfsdm1_cr1.modify(|_, w| w.dfen().clear_bit()),
            DfsdmChannel::C2 => self.regs.dfsdm2_cr1.modify(|_, w| w.dfen().clear_bit()),
            DfsdmChannel::C3 => self.regs.dfsdm3_cr1.modify(|_, w| w.dfen().clear_bit()),
        }
    }

    /// Configure for PDM microphone(s). Configures the right channel as the `channel` argument here,
    /// and the left channel as `channel` - 1. So, Channel options for this argument must be C1, C2,
    /// or C3; this corresponds to the right channel. H742 RM, section 30.4.4
    pub fn setup_pdm_mics(&mut self, channel: DfsdmChannel) {
        let msg = "Channel selected must be 1 or higher for stereo PDM mics";
        // Configuration of serial channels for PDM microphone input:
        // • PDM microphone signals (data, clock) will be connected to DFSDM input serial channel
        // y (DATINy, CKOUT) pins.
        // (Handled by hardware connections and GPIO config)
        // • Channel y will be configured: CHINSEL = 0 (input from given channel pins: DATINy,
        // CKINy).
        match channel {
            DfsdmChannel::C0 => panic!(msg),
            DfsdmChannel::C1 => self
                .regs
                .dfsdm_chcfg1r1
                .modify(|_, w| w.chinsel().clear_bit()),
            DfsdmChannel::C2 => self
                .regs
                .dfsdm_chcfg2r1
                .modify(|_, w| w.chinsel().clear_bit()),
            DfsdmChannel::C3 => self
                .regs
                .dfsdm_chcfg3r1
                .modify(|_, w| w.chinsel().clear_bit()),
        }
        // • Channel (y-1) (modulo 8) will be configured: CHINSEL = 1 (input from the following
        // channel ((y-1)+1) pins: DATINy, CKINy).
        match channel {
            DfsdmChannel::C0 => panic!(msg),
            DfsdmChannel::C1 => self
                .regs
                .dfsdm_chcfg0r1
                .modify(|_, w| w.chinsel().set_bit()),
            DfsdmChannel::C2 => self
                .regs
                .dfsdm_chcfg1r1
                .modify(|_, w| w.chinsel().set_bit()),
            DfsdmChannel::C3 => self
                .regs
                .dfsdm_chcfg2r1
                .modify(|_, w| w.chinsel().set_bit()),
        }
        // • Channel y: SITP[1:0] = 0 (rising edge to strobe data) => left audio channel on channel
        // y.
        match channel {
            DfsdmChannel::C0 => panic!(msg),
            DfsdmChannel::C1 => self
                .regs
                .dfsdm_chcfg1r1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            DfsdmChannel::C2 => self
                .regs
                .dfsdm_chcfg2r1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
            DfsdmChannel::C3 => self
                .regs
                .dfsdm_chcfg3r1
                .modify(|_, w| unsafe { w.sitp().bits(0) }),
        }
        // • Channel (y-1): SITP[1:0] = 1 (falling edge to strobe data) => right audio channel on
        // channel y-1.
        match channel {
            DfsdmChannel::C0 => panic!(msg),
            DfsdmChannel::C1 => self
                .regs
                .dfsdm_chcfg0r1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
            DfsdmChannel::C2 => self
                .regs
                .dfsdm_chcfg1r1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
            DfsdmChannel::C3 => self
                .regs
                .dfsdm_chcfg2r1
                .modify(|_, w| unsafe { w.sitp().bits(1) }),
        }
        // • Two DFSDM filters will be assigned to channel y and channel (y-1) (to filter left and
        // right channels from PDM microphone).
    }

    /// Initiate a converssion. See H742 RM, section 30.4.15: Launching conversions
    pub fn start_conversion(&self, channel: DfsdmChannel) {
        // todo: set up via interrupts/DMA/separate reading adn conversion etc
        // todo: regular vs injected conversions

        // Regular conversions can be launched using the following methods:
        // • Software: by writing ‘1’ to RSWSTART in the DFSDM_FLTxCR1 register.
        match channel {
            DfsdmChannel::C0 => self.regs.dfsdm0_cr1.modify(|_, w| w.rswstart().set_bit()),
            DfsdmChannel::C1 => self.regs.dfsdm1_cr1.modify(|_, w| w.rswstart().set_bit()),
            DfsdmChannel::C2 => self.regs.dfsdm2_cr1.modify(|_, w| w.rswstart().set_bit()),
            DfsdmChannel::C3 => self.regs.dfsdm3_cr1.modify(|_, w| w.rswstart().set_bit()),
        }

        // • Synchronous with DFSDM_FLT0 if RSYNC=1: for DFSDM_FLTx (x>0), a regular
        // conversion is automatically launched when in DFSDM_FLT0; a regular conversion is
        // started by software (RSWSTART=1 in DFSDM_FLT0CR2 register). Each regular
        // conversion in DFSDM_FLTx (x>0) is always executed according to its local
        // configuration settings (RCONT, RCH, etc.).
        // Only one regular conversion can be pending or ongoing at a given time. Thus, any request
        // to launch a regular conversion is ignored if another request for a regular conversion has
        // already been issued but not yet completed. A regular conversion can be pending if it was
        // interrupted by an injected conversion or if it was started while an injected conversion was in
        // progress. This pending regular conversion is then delayed and is performed when all
        // injected conversion are finished. Any delayed regular conversion is signalized by RPEND bit
        // in DFSDM_FLTxRDATAR register.
    }

    /// Initiate an injected conversion. See H742 RM, section 30.4.15: Launching conversions
    pub fn start_injected_conversion(&self, channel: DfsdmChannel) {
        // Injected conversions can be launched using the following methods:
        // • Software: writing ‘1’ to JSWSTART in the DFSDM_FLTxCR1 register.
        match channel {
            DfsdmChannel::C0 => self.regs.dfsdm0_cr1.modify(|_, w| w.jswstart().set_bit()),
            DfsdmChannel::C1 => self.regs.dfsdm1_cr1.modify(|_, w| w.jswstart().set_bit()),
            DfsdmChannel::C2 => self.regs.dfsdm2_cr1.modify(|_, w| w.jswstart().set_bit()),
            DfsdmChannel::C3 => self.regs.dfsdm3_cr1.modify(|_, w| w.jswstart().set_bit()),
        }

        // • Trigger: JEXTSEL[4:0] selects the trigger signal while JEXTEN activates the trigger
        // and selects the active edge at the same time (see the DFSDM_FLTxCR1 register).
        // • Synchronous with DFSDM_FLT0 if JSYNC=1: for DFSDM_FLTx (x>0), an injected
        // conversion is automatically launched when in DFSDM_FLT0; the injected conversion is
        // started by software (JSWSTART=1 in DFSDM_FLT0CR2 register). Each injected
        // conversion in DFSDM_FLTx (x>0) is always executed according to its local
        // configuration settings (JSCAN, JCHG, etc.).
        // If the scan conversion is enabled (bit JSCAN=1) then, each time an injected conversion is
        // triggered, all of the selected channels in the injected group (JCHG[7:0] bits in
        // DFSDM_FLTxJCHGR register) are converted sequentially, starting with the lowest channel
        // (channel 0, if selected).
        // If the scan conversion is disabled (bit JSCAN=0) then, each time an injected conversion is
        // triggered, only one of the selected channels in the injected group (JCHG[7:0] bits in
        // DFSDM_FLTxJCHGR register) is converted and the channel selection is then moved to the
        // next selected channel. Writing to the JCHG[7:0] bits when JSCAN=0 sets the channel
        // selection to the lowest selected injected channel.
        // Only one injected conversion can be ongoing at a given time. Thus, any request to launch
        // an injected conversion is ignored if another request for an injected conversion has already
        // been issued but not yet completed.
    }

    /// Enable a specific type of interrupt. See H743 RM, section 30.5: DFSDM interrupts
    pub fn enable_interrupt(&mut self, interrupt_type: DfsdmInterrupt, channel: DfsdmChannel) {
        // todo: Macro to reduce DRY here?
        match channel {
            DfsdmChannel::C0 => {
                self.regs.dfsdm0_cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            DfsdmChannel::C1 => {
                self.regs.dfsdm1_cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            DfsdmChannel::C2 => {
                self.regs.dfsdm2_cr2.modify(|_, w| match interrupt_type {
                    DfsdmInterrupt::EndOfInjectedConversion => w.jeocie().set_bit(),
                    DfsdmInterrupt::EndOfConversion => w.reocie().set_bit(),
                    DfsdmInterrupt::DataOverrunInjected => w.jovrie().set_bit(),
                    DfsdmInterrupt::DataOverrun => w.rovrie().set_bit(),
                    DfsdmInterrupt::AnalogWatchdog => w.awdie().set_bit(),
                    DfsdmInterrupt::ShortCircuit => w.scdie().set_bit(),
                    DfsdmInterrupt::ChannelClockAbsense => w.ckabie().set_bit(),
                });
            }
            DfsdmChannel::C3 => {
                self.regs.dfsdm3_cr2.modify(|_, w| match interrupt_type {
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
    pub fn clear_interrupt(&mut self, interrupt_type: DfsdmInterrupt, channel: DfsdmChannel) {

        // todo figure out what's wrong and put back.
        // match channel {
        //      DfsdmChannel::C0 => {
        //          self.regs.dfsdm0_icr.write(|w| match interrupt_type {
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
        //      DfsdmChannel::C1 => {
        //          self.regs.dfsdm1_icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf().set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //      DfsdmChannel::C2 => {
        //          self.regs.dfsdm2_icr.write(|w| match interrupt_type {
        //              DfsdmInterrupt::EndOfInjectedConversion => (),
        //              DfsdmInterrupt::EndOfConversion => (),
        //              DfsdmInterrupt::DataOverrunInjected => w.clrjovrf().set_bit(),
        //              DfsdmInterrupt::DataOverrun => w.clrrovrf().set_bit(),
        //              DfsdmInterrupt::AnalogWatchdog => w.clrawhtf().set_bit(),
        //              DfsdmInterrupt::ShortCircuit => w.clrscdf().set_bit(),
        //              DfsdmInterrupt::ChannelClockAbsense => w.clrckabf().set_bit(),
        //          });
        //      }
        //      DfsdmChannel::C3 => {
        //          self.regs.dfsdm3_icr.write(|w| match interrupt_type {
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
