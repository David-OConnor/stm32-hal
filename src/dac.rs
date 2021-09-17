//! Support for the digital to Analog converter (DAC) peripheral.

use core::ops::Deref;

use cortex_m::{delay::Delay, interrupt::free};

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
};

cfg_if! {
    if #[cfg(any(all(feature = "f3", not(feature = "f302")), all(feature = "l4", not(feature = "l4x6")), feature = "g4", feature = "h7b3"))] {
        use crate::pac::dac1 as dac_p;
    } else {
        use crate::pac::dac as dac_p;
    }
}

#[cfg(feature = "g0")]
use crate::pac::dma as dma_p;
#[cfg(any(
    feature = "f3",
    feature = "l4",
    feature = "g4",
    feature = "h7",
    feature = "wb",
    feature = "wl"
))]
use crate::pac::dma1 as dma_p;

#[cfg(not(any(feature = "f4", feature = "l5")))]
use crate::dma::{self, ChannelCfg, Dma, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

#[derive(Clone, Copy)]
#[repr(u8)]
/// Sets the DAC_MCR register, Mode1 and Mode2 fields.
pub enum DacMode {
    /// DAC channel is connected to external pin with Buffer enabled
    NormExternalOnlyBufEn = 0b000,
    /// DAC channel is connected to external pin and to on chip peripherals with buffer
    /// enabled
    NormExternalAndPeriphBufEn = 0b001,
    /// DAC channel is connected to external pin with buffer disabled
    NormExternalOnlyBufDis = 0b010,
    /// DAC channel is connected to on chip peripherals with Buffer disabled
    NormExternalAndPeriphBuDis = 0b011,
    /// DAC channel is connected to external pin with Buffer enabled
    ShNormExternalOnlyBufEn = 0b100,
    /// DAC channel is connected to external pin and to on chip peripherals with buffer
    /// enabled
    ShExternalAndPeriphBufEn = 0b101,
    /// DAC channel is connected to external pin with buffer disabled
    ShNormExternalOnlyBufDis = 0b110,
    /// DAC channel is connected to on chip peripherals with Buffer disabled
    ShNormExternalAndPeriphBuDis = 0b111,
}

use cfg_if::cfg_if;

#[derive(Clone, Copy)]
/// Select the DAC to use.
pub enum DacDevice {
    // Note: F3 has up to 2 DACs. G4 has up to 4. L4, L5, G0, and H7(?) have only 1.
    // WB doesn't have a DAC(?), so it doesn't import this module.
    One,
    #[cfg(any(
        feature = "f303",
        feature = "f373",
        feature = "f3x4",
        feature = "f4",
        feature = "g4"
    ))]
    Two,
    #[cfg(feature = "g4")]
    Three,
    #[cfg(feature = "g4")]
    Four,
}

#[derive(Clone, Copy)]
/// Select the channel to output to. Most MCUs only use 2 channels.
pub enum DacChannel {
    C1,
    #[cfg(not(feature = "wl"))] // WL only has one channel.
    C2,
}

#[derive(Clone, Copy)]
/// Three options are available to set DAC precision. Sets the DHR8R1 etc register contents.
pub enum DacBits {
    /// Eight bit precision, right-aligned.
    EightR,
    /// 12-bit precision, left-aligned.
    TwelveL,
    /// 12-bit precision, right-aligned.
    TwelveR,
}

#[derive(Clone, Copy)]
#[repr(u8)]
#[cfg(not(feature = "h7"))]
/// Select a trigger, used by some features. Sets DAC_CR, TSEL1 and TSEL2 fields, for Channel 1
/// and Channel 2 triggers respectively. See L44 RM, Table 75. DAC trigger selection.
pub enum Trigger {
    /// Timer 6
    Tim6 = 0b000,
    /// Timers 3 or 8
    Tim3_8 = 0b001,
    /// Timer 7
    Tim7 = 0b010,
    /// Timer 15
    Tim5 = 0b011,
    /// Timer 2
    Tim2 = 0b100,
    /// Timer 4
    Tim4 = 0b101,
    /// Eg, for interrupts
    Exti9 = 0b110,
    /// A software trigger
    Swtrig = 0b111,
}

#[derive(Clone, Copy)]
#[repr(u8)]
#[cfg(feature = "h7")]
/// Select a trigger, used by some features. Sets DAC_CR, TSEL1 and TSEL2 fields, for Channel 1
/// and Channel 2 triggers respectively. See H743 RM, Table 225. DAC interconnection.
pub enum Trigger {
    /// A software trigger
    Swtrig = 0,
    /// Timer 1
    Tim1 = 1,
    /// Timer 2
    Tim2 = 2,
    /// Timer 3
    Tim4 = 3,
    /// Timer 4
    Tim5 = 4,
    /// Timer 5
    Tim6 = 5,
    /// Timer 6
    Tim7 = 6,
    /// Timer 7
    Tim8 = 7,
    /// Timer 8
    Tim15 = 8,
    /// High resolution timer trigger 1
    Hrtim1Trig1 = 9,
    /// High resolution timer trigger 2
    Hrtim1Trig2 = 10,
    /// Low power timer 1
    Lptim1 = 11,
    /// Low power timer 2
    Lptim2 = 12,
    /// Eg, for interrupts
    Exti9 = 13,
}

/// Represents a Digital to Analog Converter (DAC) peripheral.
pub struct Dac<R> {
    pub regs: R,
    device: DacDevice,
    bits: DacBits,
    vref: f32,
}

// todo: Calculate the VDDA vref, as you do with onboard ADCs!

impl<R> Dac<R>
where
    R: Deref<Target = dac_p::RegisterBlock>,
{
    /// Initialize a DAC peripheral, including  enabling and resetting
    /// its RCC peripheral clock. `vref` is in volts.
    pub fn new(regs: R, device: DacDevice, bits: DacBits, vref: f32) -> Self {
        free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };

            cfg_if! {
                if #[cfg(all(feature = "h7", not(feature = "h7b3")))] {
                    rcc_en_reset!(apb1, dac12, rcc);
                } else if #[cfg(feature = "f3")] {
                    match device {
                        DacDevice::One => { rcc_en_reset!(apb1, dac1, rcc); }
                        #[cfg(any(feature = "f303", feature = "f373", feature = "f3x4", feature = "f4", feature = "g4"))]
                        DacDevice::Two => { rcc_en_reset!(apb1, dac2, rcc); }
                    };
                } else if #[cfg(feature = "g4")] {
                    match device {
                        DacDevice::One => { rcc_en_reset!(ahb2, dac1, rcc); }
                        DacDevice::Two => { rcc_en_reset!(ahb2, dac2, rcc); }
                        DacDevice::Three => { rcc_en_reset!(ahb2, dac3, rcc); }
                        DacDevice::Four => { rcc_en_reset!(ahb2, dac4, rcc); }
                    };
                } else if #[cfg(feature = "f4")] {
                    // F4 only uses 1 enable, despite having 2 devices. (each with 1 channel)
                    rcc_en_reset!(apb1, dac, rcc);
                } else {
                    rcc_en_reset!(apb1, dac1, rcc);
                }
            }
        });

        // See H743 RM, Table 227 for info on the buffer.
        // todo: Currently at default setting for both channels of external pin with buffer enabled.
        // todo make this customizable
        let mode = DacMode::NormExternalOnlyBufEn;
        #[cfg(not(any(
            feature = "f3",
            feature = "f4",
            feature = "l5",
            feature = "g4",
            feature = "wl"
        )))]
        regs.mcr.modify(|_, w| unsafe {
            w.mode1().bits(mode as u8);
            w.mode2().bits(mode as u8)
        });

        Self {
            regs,
            device,
            bits,
            vref,
        }
    }

    /// Calibrate the DAC output buffer by performing a "User
    /// trimming" operation. It is useful when the VDDA/VREF+
    /// voltage or temperature differ from the factory trimming
    /// conditions.
    ///
    /// The calibration is only valid when the DAC channel is
    /// operating with the buffer enabled. If applied in other
    /// modes it has no effect.
    ///
    /// After the calibration operation, the DAC channel is
    /// disabled.
    #[cfg(not(any(feature = "f3", feature = "f4", feature = "l5", feature = "g4")))]
    pub fn calibrate_buffer(
        // This function taken from STM32H7xx-hal.
        &mut self,
        channel: DacChannel,
        delay: &mut Delay,
    ) {
        self.disable(channel);

        let mut trim = 0;

        loop {
            match channel {
                DacChannel::C1 => self
                    .regs
                    .ccr
                    .modify(|_, w| unsafe { w.otrim1().bits(trim) }),
                #[cfg(not(feature = "wl"))]
                DacChannel::C2 => self
                    .regs
                    .ccr
                    .modify(|_, w| unsafe { w.otrim2().bits(trim) }),
            }
            delay.delay_us(64);

            let cal_flag = match channel {
                DacChannel::C1 => self.regs.sr.read().cal_flag1().bit_is_set(),
                #[cfg(not(feature = "wl"))]
                DacChannel::C2 => self.regs.sr.read().cal_flag2().bit_is_set(),
            };

            if cal_flag {
                break;
            }
            trim += 1;
        }
    }

    /// Enable the DAC, for a specific channel.
    pub fn enable(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.en1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.en2().set_bit(),
        });
    }

    /// Disable the DAC, for a specific channel.
    pub fn disable(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.en1().clear_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.en2().clear_bit(),
        });
    }

    /// Set the DAC output word.
    pub fn write(&mut self, channel: DacChannel, val: u16) {
        // RM: DAC conversion
        // The DAC_DORx cannot be written directly and any data transfer to the DAC channelx must
        // be performed by loading the DAC_DHRx register (write operation to DAC_DHR8Rx,
        // DAC_DHR12Lx, DAC_DHR12Rx, DAC_DHR8RD, DAC_DHR12RD or DAC_DHR12LD).
        // Data stored in the DAC_DHRx register are automatically transferred to the DAC_DORx
        // register after one APB1 clock cycle, if no hardware trigger is selected (TENx bit in DAC_CR
        // register is reset). However, when a hardware trigger is selected (TENx bit in DAC_CR
        // register is set) and a trigger occurs, the transfer is performed three APB1 clock cycles after
        // the trigger signal.
        // When DAC_DORx is loaded with the DAC_DHRx contents, the analog output voltage
        // becomes available after a time tSETTLING that depends on the power supply voltage and the
        // analog output load.

        // todo: Should we ensure the word doesn't overflow the set `bits` value?

        // PAC issue where we need 32 bit int to write? Even though this
        // is a 12-bit field.
        let val = val as u32;

        #[cfg(any(feature = "l5", feature = "g4"))]
        match channel {
            DacChannel::C1 => match self.bits {
                DacBits::EightR => self.regs.dac_dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dac_dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dac_dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match self.bits {
                DacBits::EightR => self.regs.dac_dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dac_dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dac_dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }

        #[cfg(not(any(feature = "l5", feature = "g4")))]
        match channel {
            DacChannel::C1 => match self.bits {
                DacBits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match self.bits {
                DacBits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }
    }

    /// Send values to the DAC using DMA. Each trigger (Eg using a timer; the basic timers Tim6
    /// and Tim7 are designed for DAC triggering) sends one word from the buffer to the DAC's
    /// output.
    #[cfg(not(any(feature = "g0", feature = "f4", feature = "l5")))]
    pub unsafe fn write_dma<D>(
        &mut self,
        buf: &[u16],
        dac_channel: DacChannel,
        dma_channel: DmaChannel,
        channel_cfg: ChannelCfg,
        dma: &mut Dma<D>,
    ) where
        D: Deref<Target = dma_p::RegisterBlock>,
    {
        let (ptr, len) = (buf.as_ptr(), buf.len());

        // todo: Impl these non-DMAMUx features.
        // // L44 RM, Table 41. "DMA1 requests for each channel
        // // todo: DMA2 support.
        // #[cfg(any(feature = "f3", feature = "l4"))]
        //     let channel = match self.device {
        //     AdcDevice::One => DmaInput::Adc1.dma1_channel(),
        //     AdcDevice::Two => DmaInput::Adc2.dma1_channel(),
        //     _ => panic!("DMA on ADC beyond 2 is not supported. If it is for your MCU, please submit an issue \
        //         or PR on Github.")
        // };
        //
        // #[cfg(feature = "l4")]
        // match self.device {
        //     AdcDevice::One => dma.channel_select(DmaInput::Adc1),
        //     AdcDevice::Two => dma.channel_select(DmaInput::Adc2),
        //     _ => unimplemented!(),
        // }

        // H743 RM, section 26.4.8: DMA requests
        // Each DAC channel has a DMA capability. Two DMA channels are used to service DAC
        // channel DMA requests.

        // When an external trigger (but not a software trigger) occurs while the DMAENx bit is set, the
        // value of the DAC_DHRx register is transferred into the DAC_DORx register when the
        // transfer is complete, and a DMA request is generated.
        #[cfg(any(feature = "l5", feature = "g4"))]
        match dac_channel {
            DacChannel::C1 => self.regs.dac_cr.modify(|_, w| w.dmaen1().set_bit()),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => self.regs.dac_cr.modify(|_, w| w.dmaen2().set_bit()),
        }

        #[cfg(not(any(feature = "l5", feature = "g4")))]
        match dac_channel {
            DacChannel::C1 => self.regs.cr.modify(|_, w| w.dmaen1().set_bit()),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => self.regs.cr.modify(|_, w| w.dmaen2().set_bit()),
        }

        // In dual mode, if both DMAENx bits are set, two DMA requests are generated. If only one
        // DMA request is needed, only the corresponding DMAENx bit must be set. In this way, the
        // application can manage both DAC channels in dual mode by using one DMA request and a
        // unique DMA channel.

        // As DAC_DHRx to DAC_DORx data transfer occurred before the DMA request, the very first
        // data has to be written to the DAC_DHRx before the first trigger event occurs.

        // DMA underrun
        // The DAC DMA request is not queued so that if a second external trigger arrives before the
        // acknowledgment for the first external trigger is received (first request), then no new request
        // is issued and the DMA channelx underrun flag DMAUDRx in the DAC_SR register is set,
        // reporting the error condition. The DAC channelx continues to convert old data.

        // The software must clear the DMAUDRx flag by writing 1, clear the DMAEN bit of the used
        // DMA stream and re-initialize both DMA and DAC channelx to restart the transfer correctly.
        // The software must modify the DAC trigger conversion frequency or lighten the DMA
        // workload to avoid a new DMA underrun. Finally, the DAC conversion could be resumed by
        // enabling both DMA data transfer and conversion trigger.

        // For each DAC channelx, an interrupt is also generated if its corresponding DMAUDRIEx bit
        // in the DAC_CR register is enabled.

        #[cfg(any(feature = "l5", feature = "g4"))]
        let periph_addr = match dac_channel {
            DacChannel::C1 => match &self.bits {
                DacBits::EightR => &self.regs.dac_dhr8r1 as *const _ as u32,
                DacBits::TwelveL => &self.regs.dac_dhr12l1 as *const _ as u32,
                DacBits::TwelveR => &self.regs.dac_dhr12r1 as *const _ as u32,
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match &self.bits {
                DacBits::EightR => &self.regs.dac_dhr8r2 as *const _ as u32,
                DacBits::TwelveL => &self.regs.dac_dhr12l2 as *const _ as u32,
                DacBits::TwelveR => &self.regs.dac_dhr12r2 as *const _ as u32,
            },
        };

        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let periph_addr = match dac_channel {
            DacChannel::C1 => match &self.bits {
                DacBits::EightR => &self.regs.dhr8r1 as *const _ as u32,
                DacBits::TwelveL => &self.regs.dhr12l1 as *const _ as u32,
                DacBits::TwelveR => &self.regs.dhr12r1 as *const _ as u32,
            },
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => match &self.bits {
                DacBits::EightR => &self.regs.dhr8r2 as *const _ as u32,
                DacBits::TwelveL => &self.regs.dhr12l2 as *const _ as u32,
                DacBits::TwelveR => &self.regs.dhr12r2 as *const _ as u32,
            },
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
            dma::Direction::ReadFromMem,
            dma::DataSize::S16,
            dma::DataSize::S16,
            channel_cfg,
        );
    }

    /// Set the DAC output voltage.
    pub fn write_voltage(&mut self, channel: DacChannel, volts: f32) {
        let max_word = match self.bits {
            DacBits::EightR => 255.,
            DacBits::TwelveL => 4_095.,
            DacBits::TwelveR => 4_095.,
        };

        let val = ((volts / self.vref) * max_word) as u16;
        self.write(channel, val);
    }

    // todo: Trouble finding right `tsel` fields for l5 and WL. RM shows same as others. PAC bug?
    // todo Or is the PAC breaking the bits field into multiple bits?
    #[cfg(not(any(feature = "l5", feature = "wl")))]
    /// Select and activate a trigger. See f303 Reference manual, section 16.5.4.
    /// Each time a DAC interface detects a rising edge on the selected trigger source (refer to the
    /// table below), the last data stored into the DAC_DHRx register are transferred into the
    /// DAC_DORx register. The DAC_DORx register is updated three dac_pclk cycles after the
    /// trigger occurs.
    pub fn set_trigger(&mut self, channel: DacChannel, trigger: Trigger) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.ten1().set_bit();
                    w.tsel1().bits(trigger as u8)
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.ten2().set_bit();
                    w.tsel2().bits(trigger as u8)
                });
            }
        }
    }

    #[cfg(not(any(feature = "l5", feature = "wl")))] // See note on `set_trigger`.
    /// Independent trigger with single LFSR generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_lfsr(&mut self, channel: DacChannel, trigger: Trigger, data: u16) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        // todo: This may not be correct.
        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.mamp1().bits(0b01);
                    w.wave1().bits(0b01)
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.wave2().bits(0b01);
                    w.mamp2().bits(0b01)
                });
            }
        }
        self.set_trigger(channel, trigger);
        self.write(channel, data);
    }

    #[cfg(not(any(feature = "l5", feature = "wl")))] // See note on `set_trigger`.
    /// Independent trigger with single triangle generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_triangle(&mut self, channel: DacChannel, trigger: Trigger, data: u16) {
        // todo: This may not be correct.
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        match channel {
            DacChannel::C1 => {
                cr.modify(|_, w| unsafe {
                    w.wave1().bits(0b10);
                    w.mamp1().bits(0b10)
                });
            }
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => {
                cr.modify(|_, w| unsafe {
                    w.wave2().bits(0b10);
                    w.mamp2().bits(0b10)
                });
            }
        }
        self.set_trigger(channel, trigger);
        self.write(channel, data);
    }

    /// Enable the DMA Underrun interrupt - the only interrupt available.
    pub fn enable_interrupt(&mut self, channel: DacChannel) {
        #[cfg(any(feature = "l5", feature = "g4"))]
        let cr = &self.regs.dac_cr;
        #[cfg(not(any(feature = "l5", feature = "g4")))]
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::C1 => w.dmaudrie1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.dmaudrie2().set_bit(),
        });
    }

    #[cfg(not(any(feature = "l5", feature = "g4")))] // todo: PAC ommission? SR missing on L5/G4? In RM.
    /// Clear the DMA Underrun interrupt - the only interrupt available.
    pub fn clear_interrupt(&mut self, channel: DacChannel) {
        self.regs.sr.modify(|_, w| match channel {
            DacChannel::C1 => w.dmaudr1().set_bit(),
            #[cfg(not(feature = "wl"))]
            DacChannel::C2 => w.dmaudr2().set_bit(),
        });
    }
}
