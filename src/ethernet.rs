//! WIP. This module contains ethernet code for the H7, for use with its Synopsys ethernet
//! hardware.
//! With the `network` feature enabled, allows support for the [smoltcp stack](https://docs.rs/smoltcp/latest/smoltcp/).
//! See H743 RM, chapter 58.
//! https://www.keil.com/pack/doc/CMSIS/Driver/html/group__eth__interface__gr.html ?

use core::ops::Deref;

use critical_section::with;
use smoltcp::{
    self,
    phy::{self, Device, DeviceCapabilities, Medium},
    time::Instant,
};

use crate::{
    pac::{self, ETHERNET_DMA, ETHERNET_MAC, ETHERNET_MTL, RCC},
    util::RccPeriph,
};

/// Configuration data for Ethernet
pub struct EthConfig {}

// impl Default for EthConfig {
//     fn default() -> Self {
//         Self {
//             mode: SpiMode::mode0(),
//             comm_mode: SpiCommMode::FullDuplex,
//             slave_select: SlaveSelect::Software,
//             data_size: DataSize::D8,
//             fifo_reception_thresh: ReceptionThresh::D8,
//         }
//     }
// }

/// Represents an ethernet peripheral.
pub struct Eth<RDma, RMac, RMtl> {
    pub regs_dma: RDma,
    pub regs_mac: RMac,
    pub regs_mtl: RMtl,
    pub cfg: EthConfig,
}

impl<RDma, RMac, RMtl> Eth<RDma, RMac, RMtl>
where
    RDma: Deref<Target = pac::ethernet_dma::RegisterBlock> + RccPeriph,
    RMac: Deref<Target = pac::ethernet_mac::RegisterBlock> + RccPeriph,
    RMtl: Deref<Target = pac::ethernet_mtl::RegisterBlock> + RccPeriph,
{
    /// Initialize an ethernet peripheral, including configuration register writes, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: R, cfg: EthConfig) -> Self {
        with(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };
            R::en_reset(rcc);
        });

        Self { regs, cfg }
    }

    /// H743 RM, section 58.9.1: DMA initialization
    pub fn init_dma(&mut self) {
        // Complete the following steps to initialize the DMA:

        // 1. Provide a software reset to reset all MAC internal registers and logic (bit 0 of DMA
        // mode register (ETH_DMAMR)).
        self.regs_dma.dmamr.write(|w| w.swr().set_bit());

        // 2. Wait for the completion of the reset process (poll bit 0 of the DMA mode register
        // (ETH_DMAMR), which is cleared when the reset operation is completed).
        while self.regs_dma.dmamr.read().swr().bit_is_set() {}

        // 3. Program the following fields to initialize the System bus mode register
        // (ETH_DMASBMR):
        self.regs_dma.dmasbmr.modify(|_, w| {
            // todo
            // a)AAL
            w.aal.bits(0);
            // b) Fixed burst or undefined burst
            w.aal.bits(0);
            // c) Burst mode values in case of AHB bus interface.
            w.aal.bits(0);
            // d) If fixed length value is enabled, select the maximum burst length possible on the
            // AXI Bus (bits [7:1])
            w.aal.bits(0)
        });

        // 4. Create a transmit and a receive descriptor list. In addition, ensure that the receive
        // descriptors are owned by the DMA (set bit 31 of TDES3/RDES3 descriptor). For more
        // information on descriptors, refer to Section 58.10: Descriptors.
        // Note:
        // Descriptor address from start to end of the ring should not cross the 4GB boundary.
        //
        // 5.
        // Program ETH_DMACTXRLR and ETH_DMACRXRLR registers (see Channel Tx
        // descriptor ring length register (ETH_DMACTXRLR) and Channel Rx descriptor ring
        // length register (ETH_DMACRXRLR)). The programmed ring length must be at least 4.
        //
        // 6. Initialize receive and transmit descriptor list address with the base address of transmit
        // and receive descriptor (Channel Tx descriptor list address register
        // (ETH_DMACTXDLAR), Channel Rx descriptor list address register
        // (ETH_DMACRXDLAR)). In addition, program the transmit and receive tail pointer
        // registers that inform the DMA about the available descriptors (see Channel Tx
        // descriptor tail pointer register (ETH_DMACTXDTPR) and Channel Rx descriptor tail
        // pointer register (ETH_DMACRXDTPR)).
        //
        // 7. Program ETH_DMACCR, ETH_DMACTXCR and ETH_DMACRXCR registers (see
        // Channel control register (ETH_DMACCR) and Channel transmit control register
        // (ETH_DMACTXCR)) to configure the parameters such as the maximum burst-length
        // (PBL) initiated by the DMA, descriptor skip lengths, OSP for TxDMA, RBSZ for
        // RxDMA.
        //
        // 8. Enable the interrupts by programming the ETH_DMACIER register (see Channel
        // interrupt enable register (ETH_DMACIER)).
        //
        // 9. Start the Receive and Transmit DMAs by setting SR (bit 0) of Channel receive control
        // register (ETH_DMACRXCR) and ST (bit 0) of the ETH_DMACTXCR (see Channel
        // transmit control register (ETH_DMACTXCR)).
    }

    /// H743 RM, section 58.9.2: MTL initialization
    pub fn init_mtl(&mut self) {
        // Complete the following steps to initialize the MTL registers:

        // 1. Program the following fields to initialize the operating mode in the ETH_MTLTXQOMR
        // (see Tx queue operating mode Register (ETH_MTLTXQOMR)).
        self.regs_mtl.mtltx_qomr.modify(|_, w| {
            // a) Transmit Store And Forward (TSF) or Transmit Threshold Control (TTC) if the
            // Threshold mode is used.
            w.tsf().bits(0);
            // b) Transmit Queue Enable (TXQEN) to value 2‘b10 to enable Transmit Queue 0.
            w.txqen().bits(0);
            // c) Transmit Queue Size (TQS).
            w.tqs().bits(0)
        });

        // 2. Program the following fields to initialize the operating mode in the ETH_MTLRXQOMR
        // register (see Rx queue operating mode register (ETH_MTLRXQOMR)):
        self.regs_mtl.mtltx_qomr.modify(|_, w| {
            // a) Receive Store and Forward (RSF) or RTC if Threshold mode is used.
            w.rsf().bits(0);
            // b) Flow Control Activation and De-activation thresholds for MTL Receive FIFO (RFA
            // and RFD).
            w.rfa().bits(0);
            w.rfd().bits(0);
            // c) Error Packet and undersized good Packet forwarding enable (FEP and FUP).
            w.fep().bits(0);
            w.fup().bits(0);
            // d)Receive Queue Size (RQS).
            w.rqs().bits(0)
        });
    }

    /// H743 RM, section 58.9.3: MAC initialization
    pub fn init_mac(&mut self) {}
}
