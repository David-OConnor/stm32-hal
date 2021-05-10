//! Direct Memory Access
// todo WIP!

use core::ops::Deref;

use crate::{
    pac::{self, RCC},
    rcc_en_reset,
    traits::ClockCfg,
};

use cfg_if::cfg_if;
use paste::paste;

#[derive(Copy, Clone)]
#[repr(u8)]
/// L4 RM, 11.4.3, "DMA arbitration":
/// The priorities are managed in two stages:
/// • software: priority of each channel is configured in the DMA_CCRx register, to one of
/// the four different levels:
/// – very high
/// – high
/// – medium
/// – low
/// • hardware: if two requests have the same software priority level, the channel with the
/// lowest index gets priority. For example, channel 2 gets priority over channel 4.
/// Only write to this when the channel is disabled.
pub enum Priority {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    VeryHigh = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum DmaChannel {
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    C6 = 6,
    C7 = 7,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Set in CCR.
/// Can only be set when channel is disabled.
pub enum Direction {
    ReadFromPeriph = 0,
    ReadFromMem = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Set in CCR.
/// Can only be set when channel is disabled.
pub enum Circular {
    Disabled = 0,
    Enabled = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Peripheral and memory increment mode. (CCR PINC and MINC bits)
/// Can only be set when channel is disabled.
pub enum IncrMode { // Can only be set when channel is disabled.
    Disabled = 0,
    Enabled = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Peripheral and memory increment mode. (CCR PSIZE and MSIZE bits)
/// Can only be set when channel is disabled.
pub enum DataSize {
    S8 = 0b00,  // ie 8 bits
    S16 = 0b01,
    S32 = 0b10
}

#[derive(Copy, Clone)]
/// Interrupt type. Set in CCR using TEIE, HTIE, and TCIE bits.
/// Can only be set when channel is disabled.
pub enum DmaInterrupt {
    TransferError,
    HalfTransfer,
    TransferComplete,
}

pub struct Dma<D> {
    regs: D,
}

// We use a macro to handle the register-blocks-by-channel.
// macro_rules! hal {
//     ($chan:ident) => {
impl<D> Dma<D>
where
    D: Deref<Target = pac::dma1::RegisterBlock>,
{
    pub fn new(regs: D) -> Self {

        // todo: Enable RCC!
        // rcc_en_reset!(ahb1, [<adc $rcc_num>], rcc);

        Self { regs }
    }

    //     // fn ccr(&self) -> D
    //     match channel {
    //             DmaChannel::C1 => self.regs.ccr1,
    //             DmaChannel::C2 => self.regs.ccr2,
    //             DmaChannel::C3 => self.regs.ccr3,
    //             DmaChannel::C4 => self.regs.ccr4,
    //             DmaChannel::C5 => self.regs.ccr5,
    //             DmaChannel::C6=> self.regs.ccr6,
    //             DmaChannel::C7 => self.regs.ccr7,
    //         }
    // }
    /// Configure a DMA channel. See L4 RM 0394, section 11.4.4
    pub fn cfg_channel(
        &mut self,
        channel: DmaChannel,
        priority: Priority,
        direction: Direction,
        circular:Circular,
        periph_incr: IncrMode,
        mem_incr: IncrMode,
        periph_size: DataSize,
        mem_size: MemSize,
    ) {
        let ccr = match channel {
            // todo DRY
            DmaChannel::C1 => self.regs.ccr1,
            DmaChannel::C2 => self.regs.ccr2,
            DmaChannel::C3 => self.regs.ccr3,
            DmaChannel::C4 => self.regs.ccr4,
            DmaChannel::C5 => self.regs.ccr5,
            DmaChannel::C6 => self.regs.ccr6,
            DmaChannel::C7 => self.regs.ccr7,
        };
        // todo: Consider a config struct you can impl default with, instead
        // todo of all these args.

        // "The register fields/bits MEM2MEM, PL[1:0], MSIZE[1:0], PSIZE[1:0], MINC, PINC, and DIR
        // are read-only when EN = 1"
        ccr.modify(|_, w| w.en().clear_bit());

        // The following sequence is needed to configure a DMA channel x:
        // 1. Set the peripheral register address in the DMA_CPARx register.
        // The data is moved from/to this address to/from the memory after the peripheral event,
        // or after the channel is enabled in memory-to-memory mode.
        let cpar = match channel {
            DmaChannel::C1 => self.regs.cpar1,
            DmaChannel::C2 => self.regs.cpar2,
            DmaChannel::C3 => self.regs.cpar3,
            DmaChannel::C4 => self.regs.cpar4,
            DmaChannel::C5 => self.regs.cpar5,
            DmaChannel::C6 => self.regs.cpar6,
            DmaChannel::C7 => self.regs.cpar7,
        };
        cpar.modify(|_, w| w.asdf);

        // 2. Set the memory address in the DMA_CMARx register.
        // The data is written to/read from the memory after the peripheral event or after the
        // channel is enabled in memory-to-memory mode.
        let cmar = match channel {
            DmaChannel::C1 => self.regs.cmar1,
            DmaChannel::C2 => self.regs.cmar2,
            DmaChannel::C3 => self.regs.cmar3,
            DmaChannel::C4 => self.regs.cmar4,
            DmaChannel::C5 => self.regs.cmar5,
            DmaChannel::C6 => self.regs.cmar6,
            DmaChannel::C7 => self.regs.cmar7,
        };
        cmar.modify(|_, w| w.asdf);

        // 3. Configure the total number of data to transfer in the DMA_CNDTRx register.
        // After each data transfer, this value is decremented.
        let cndtr = match channel {
            DmaChannel::C1 => self.regs.cndtr1,
            DmaChannel::C2 => self.regs.cndtr2,
            DmaChannel::C3 => self.regs.cndtr3,
            DmaChannel::C4 => self.regs.cndtr4,
            DmaChannel::C5 => self.regs.cndtr5,
            DmaChannel::C6 => self.regs.cndtr6,
            DmaChannel::C7 => self.regs.cndtr7,
        };
        cndtr.modify(|_, w| w.asdf);

        // 4. Configure the parameters listed below in the DMA_CCRx register:
        // (These are listed below by their corresponding reg write code)

        // todo: See note about sep reg writes to disable channel, and when you need to do this.

        // 5. Activate the channel by setting the EN bit in the DMA_CCRx register.
        // A channel, as soon as enabled, may serve any DMA request from the peripheral connected
        // to this channel, or may start a memory-to-memory block transfer.
        // Note: The two last steps of the channel configuration procedure may be merged into a single
        // access to the DMA_CCRx register, to configure and enable the channel.
        // When a channel is enabled and still active (not completed), the software must perform two
        // separate write accesses to the DMA_CCRx register, to disable the channel, then to
        // reprogram the channel for another next block transfer.
        // Some fields of the DMA_CCRx register are read-only when the EN bit is set to 1

        // (later): The circular mode must not be used in memory-to-memory mode. Before enabling a
        // channel in circular mode (CIRC = 1), the software must clear the MEM2MEM bit of the
        // DMA_CCRx register. When the circular mode is activated, the amount of data to transfer is
        // automatically reloaded with the initial value programmed during the channel configuration
        // phase, and the DMA requests continue to be served

        ccr.modify(|_, w| w.mem2mem().clear_bit());

        // todo: Let user set mem2mem mode?

        ccr.modify(|_, w| {
            // – the channel priority
            w.pl.bits(Priority as u8);
            // – the data transfer direction
            // This bit [DIR] must be set only in memory-to-peripheral and peripheral-to-memory modes.
            // 0: read from peripheral
            w.dir.bit(direction as u8 != 0);
            // – the circular mode
            w.circ.bit(circular as u8 != 0);
            // – the peripheral and memory incremented mode
            w.pinc.bit(periph_inc as u8 != 0);
            w.minc.bit(periph_inc as u8 != 0);
            // – the peripheral and memory data size
            w.psize.bits(periph_size as u8);
            w.msize.bits(mem_size as u8);
            // – the interrupt enable at half and/or full transfer and/or transfer error
            // (We handle this using the `enable_interrupt` method below.)
            // (See `Step 5` above.)
            w.en().set_bit()
        });
    }

    pub fn stop(&mut self, channel: DmaChannel) {
        // L4 RM:
        // Once the software activates a channel, it waits for the completion of the programmed
        // transfer. The DMA controller is not able to resume an aborted active channel with a possible
        // suspended bus transfer.
        // To correctly stop and disable a channel, the software clears the EN bit of the DMA_CCRx
        // register. The software secures that no pending request from the peripheral is served by the
        // DMA controller before the transfer completion. The software waits for the transfer complete
        // or transfer error interrupt.
        // When a channel transfer error occurs, the EN bit of the DMA_CCRx register is cleared by
        // hardware. This EN bit can not be set again by software to re-activate the channel x, until the
        // TEIFx bit of the DMA_ISR register is set
        let ccr = match channel {
            // todo DRY
            DmaChannel::C1 => self.regs.ccr1,
            DmaChannel::C2 => self.regs.ccr2,
            DmaChannel::C3 => self.regs.ccr3,
            DmaChannel::C4 => self.regs.ccr4,
            DmaChannel::C5 => self.regs.ccr5,
            DmaChannel::C6 => self.regs.ccr6,
            DmaChannel::C7 => self.regs.ccr7,
        };
        ccr.modify(|_, w| w.en().clear_bit());
        // todo: Check for no pending request and transfer complete/error
    }

    /// Enable a specific type of interrupt.
     pub fn enable_interrupt(&mut self, channel: DmaChannel, interrupt_type: DmaInterrupt) {
        // Can only be set when the channel is disabled.
        let originally_enabled = ccr.read().en().bit_is_set();
        if originally_enabled {
            ccr.modify(|_, w| w.en().clear_bit());
            while ccr.read().en().bit_is_set() {}
        }
         match interrupt_type {
             DmaInterrupt::TransferError => ccr.modify(|_, w| w.teie.set_bit());
             DmaInterrupt::HalfTransfer => ccr.modify(|_, w| w.htie.set_bit());
             DmaInterrupt::TransferComplete => ccr.modify(|_, w| w.tcie.set_bit());
         }

        if originally_enabled {
            ccr.modify(|_, w| w.en().set_bit());
            while ccr.read().en().bit_is_clear() {}
        }

     }

     pub fn clear_interrupt(&mut self, interrupt_type: DmaInterrupt) {

     }

    // todo: Put this back if you think changing the priority is something you want to do
    // todo after initial config.
    // pub fn set_priority(&mut self, channel: DmaChannel, priority: Priority) {
    //     let ccr = match channel {
    //         // todo DRY
    //         DmaChannel::C1 => self.regs.ccr1,
    //         DmaChannel::C2 => self.regs.ccr2,
    //         DmaChannel::C3 => self.regs.ccr3,
    //         DmaChannel::C4 => self.regs.ccr4,
    //         DmaChannel::C5 => self.regs.ccr5,
    //         DmaChannel::C6 => self.regs.ccr6,
    //         DmaChannel::C7 => self.regs.ccr7,
    //     };
    //     ccr.modify(|_, w| w.asfd);
    // }
}
// }}
//
// hal!(1);
// hal!(2);
// hal!(3);
// hal!(4);
// hal!(5);
// hal!(6);
// hal!(7);
