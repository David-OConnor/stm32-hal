//! Inter-processor communication controller (IPCC).
//! Used on STM32WB for communication between cores.

use crate::pac::{self, IPCC, RCC};

// todo: C1_1 and C2_1 etc for channels instead of separate core enum?
// todo: Consider macros to reduce DRY here, re Core and Channel matching.
// todo: Consolidate match arms to reduce DRY match statements for the diff steps
// todo excecuted in a given fn.

#[derive(Clone, Copy)]
/// Represents one of six channels. We use this enum for both Core1 and Core2 channels.
pub enum IpccChannel {
    C1,
    C2,
    C3,
    C4,
    C5,
    C6,
}

#[derive(Clone, Copy)]
/// The core that's performing the requested operation. Core 1 is the M4 core, and Core 2 is the M0+ core.
pub enum Core {
    C1,
    C2,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Select Simplex (data stored separately on each core), or Half-Duplex (Data
/// is shared in a common memory location)
/// In Simplex channel mode, a dedicated memory location (used to transfer data in a single
/// direction) is assigned to the communication data. The associated channel N control bits
/// (see Table 235) are used to manage the transfer from the sending to the receiving
/// processor.
/// The Half-duplex channel mode is used when one processor sends a communication and the
/// other processor sends a response to each communication (ping-pong).
pub enum IpccMode {
    Simplex = 0,
    HalfDuplex = 1, // todo qc these
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// IPCC interrupts. Enabled in IPCC_C1CR.
pub enum IpccInterrupt {
    /// TXFIE: Processor 1 transmit channel free interrupt enable
    /// IPCC_C1TOC2SR
    /// Enable an unmasked processor 1 transmit channel free to generate a TX free interrupt.
    TxFree,
    /// RXOIE: Processor 1 receive channel occupied interrupt enable
    /// Associated with IPCC_C2TOC1SR
    /// Enable an unmasked processor 1 receive channel occupied to generate an RX occupie
    RxOccupied,
}

/// Represents an Inter-Integrated Circuit (I2C) peripheral.
pub struct Ipcc {
    pub regs: IPCC,
}

impl Ipcc {
    /// Initialize the IPCC peripheral, including enabling interrupts, and enabling and resetting
    /// its RCC peripheral clock.
    pub fn new(regs: IPCC) -> Self {
        let mut rcc = unsafe { &(*RCC::ptr()) };
        rcc.ahb3enr.modify(|_, w| w.ipccen().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.ipccrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.ipccrst().clear_bit());

        // todo?
        // rcc.ahb4enr.modify(|_, w| w.ipccen().set_bit());
        // rcc.ahb4rstr.modify(|_, w| w.ipccrst().set_bit());
        // rcc.ahb4rstr.modify(|_, w| w.ipccrst().clear_bit());

        // todo: Got this line from stm32wb-hal.
        // Single memory access delay after peripheral is enabled.
        // This dummy read uses `read_volatile` internally, so it shouldn't be removed by an optimizer.
        let _ = rcc.ahb3enr.read().ipccen();

        // Enable interrupts.
        let mut result = Self { regs };
        result.enable_interrupt(IpccInterrupt::TxFree);
        result.enable_interrupt(IpccInterrupt::RxOccupied);

        result
    }

    /// Send a message using simplex mode. Non-blocking.
    pub fn send_simplex(&mut self, core: Core, channel: IpccChannel, data: &[u8]) {
        // RM, section 37.3.2: To send communication data:
        // The sending processor checks the channel status flag CHnF:
        // – When CHnF = 0, the channel is free (last communication data retrieved by
        // receiving processor) and the new communication data can be written.
        // – When CHnF = 1, the channel is occupied (last communication data not retrieved
        // by receiving processor) and the sending processor unmasks the channel free
        // interrupt (CHnFM = 0).
        if self.channel_is_free(core, channel) {
            // self.regs.asdf.modify(|_, w| w.dasdf.bits(data));
        } else {
            // todo: Unmask interrupt?
        }
        // – On a TX free interrupt, the sending processor checks which channel became free
        // and masks the channel free interrupt (CHnFM = 1). Then the new communication
        // can take place.
        // Once the complete communication data is posted, the channel status is set to occupied
        // with CHnS. This gives memory access to the receiving processor and generates the
        // RX occupied interrupt.
    }

    /// Receive a message using simplex mode. Non-blocking.
    pub fn receive_simplex(&mut self, core: Core, channel: IpccChannel) {
        // RM, section 37.3.2: To receive a communication, the channel occupied interrupt is unmasked (CHnOM = 0):
        match core {
            Core::C1 => self.regs.c1mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1om().clear_bit(),
                IpccChannel::C2 => w.ch2om().clear_bit(),
                IpccChannel::C3 => w.ch3om().clear_bit(),
                IpccChannel::C4 => w.ch4om().clear_bit(),
                IpccChannel::C5 => w.ch5om().clear_bit(),
                IpccChannel::C6 => w.ch6om().clear_bit(),
            }),
            Core::C2 => self.regs.c2mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1om().clear_bit(),
                IpccChannel::C2 => w.ch2om().clear_bit(),
                IpccChannel::C3 => w.ch3om().clear_bit(),
                IpccChannel::C4 => w.ch4om().clear_bit(),
                IpccChannel::C5 => w.ch5om().clear_bit(),
                IpccChannel::C6 => w.ch6om().clear_bit(),
            }),
        }

        // - On a RX occupied interrupt, the receiving processor checks which channel became
        // occupied, masks the associated channel occupied interrupt (CHnOM) and reads the
        // communication data from memory.
        // - Once the complete communication data is retrieved, the channel status is cleared to
        // free with CHnC. This gives memory access back to the sending processor and may
        // generate the TX free interrupt.
        // - Once the channel status is cleared, the channel occupied interrupt is unmasked
        // (CHnOM = 0).
    }

    /// The Half-duplex channel mode is used when one processor sends a communication and the
    /// other processor sends a response to each communication (ping-pong). Blocking.
    pub fn send_half_duplex(&mut self, core: Core, channel: IpccChannel, data: &[u8]) {
        // RM, section 37.3.3: To send communication data:
        // * The sending processor waits for its response pending software variable to get 0.
        // – Once the response pending software variable is 0 the communication data is
        // posted.
        while !self.channel_is_free(core, channel) {} // todo is this right?

        //  Once the complete communication data has been posted, the channel status flag
        // CHnF is set to occupied with CHnS and the response pending software variable is set
        // to 1 (this gives memory access and generates the RX occupied interrupt to the
        // receiving processor).
        match core {
            Core::C1 => self.regs.c1scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
            Core::C2 => self.regs.c2scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
        }

        // * Once the channel status flag CHnF is set, the channel free interrupt is unmasked
        // (CHnFM = 0).
        match core {
            Core::C1 => self.regs.c1mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1fm().clear_bit(),
                IpccChannel::C2 => w.ch2fm().clear_bit(),
                IpccChannel::C3 => w.ch3fm().clear_bit(),
                IpccChannel::C4 => w.ch4fm().clear_bit(),
                IpccChannel::C5 => w.ch5fm().clear_bit(),
                IpccChannel::C6 => w.ch6fm().clear_bit(),
            }),
            Core::C2 => self.regs.c2mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1fm().clear_bit(),
                IpccChannel::C2 => w.ch2fm().clear_bit(),
                IpccChannel::C3 => w.ch3fm().clear_bit(),
                IpccChannel::C4 => w.ch4fm().clear_bit(),
                IpccChannel::C5 => w.ch5fm().clear_bit(),
                IpccChannel::C6 => w.ch6fm().clear_bit(),
            }),
        }
    }

    /// Send a half-duplex response.
    pub fn send_response_half_duplex(&mut self, core: Core, channel: IpccChannel, data: &[u8]) {
        // To send a response:
        // * The receiving processor waits for its response pending software variable to get 1.
        // – Once the response pending software variable is 1 the response is posted.
        while self.channel_is_free(core, channel) {}

        // todo: Write response here?

        // * Once the complete response is posted, the channel status flag CHnF is cleared to free
        // with CHnC and the response pending software variable is set to 0 (this gives memory
        // access and generates the TX free interrupt to the sending processor).
        // * Once the channel status flag CHnF is cleared, the channel occupied interrupt is
        // unmasked (CHnOM = 0).
        match core {
            Core::C1 => match channel {
                // Note about SCR: Listed technically as "rw" in manual, but also listed as "reads
                // always as 0". PAC reflects write-only
                IpccChannel::C1 => {
                    self.regs.c1scr.write(|w| w.ch1c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch1fm().clear_bit();
                        w.ch1om().clear_bit()
                    });
                }
                IpccChannel::C2 => {
                    self.regs.c1scr.write(|w| w.ch2c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch2fm().clear_bit();
                        w.ch2om().clear_bit()
                    });
                }
                IpccChannel::C3 => {
                    self.regs.c1scr.write(|w| w.ch3c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch3fm().clear_bit();
                        w.ch3om().clear_bit()
                    });
                }
                IpccChannel::C4 => {
                    self.regs.c1scr.write(|w| w.ch4c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch4fm().clear_bit();
                        w.ch4om().clear_bit()
                    });
                }
                IpccChannel::C5 => {
                    self.regs.c1scr.write(|w| w.ch5c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch5fm().clear_bit();
                        w.ch5om().clear_bit()
                    });
                }
                IpccChannel::C6 => {
                    self.regs.c1scr.write(|w| w.ch6c().set_bit());
                    self.regs.c1mr.modify(|_, w| {
                        w.ch6fm().clear_bit();
                        w.ch6om().clear_bit()
                    });
                }
            },
            Core::C2 => match channel {
                IpccChannel::C1 => {
                    self.regs.c2scr.write(|w| w.ch1c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch1fm().clear_bit();
                        w.ch1om().clear_bit()
                    });
                }
                IpccChannel::C2 => {
                    self.regs.c2scr.write(|w| w.ch2c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch2fm().clear_bit();
                        w.ch2om().clear_bit()
                    });
                }
                IpccChannel::C3 => {
                    self.regs.c2scr.write(|w| w.ch3c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch3fm().clear_bit();
                        w.ch3om().clear_bit()
                    });
                }
                IpccChannel::C4 => {
                    self.regs.c2scr.write(|w| w.ch4c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch4fm().clear_bit();
                        w.ch4om().clear_bit()
                    });
                }
                IpccChannel::C5 => {
                    self.regs.c2scr.write(|w| w.ch5c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch5fm().clear_bit();
                        w.ch5om().clear_bit()
                    });
                }
                IpccChannel::C6 => {
                    self.regs.c2scr.write(|w| w.ch6c().set_bit());
                    self.regs.c2mr.modify(|_, w| {
                        w.ch6fm().clear_bit();
                        w.ch6om().clear_bit()
                    });
                }
            },
        }
    }

    /// Receive in half duplex mode.
    pub fn receive_half_duplex(&mut self, core: Core, channel: IpccChannel) {
        // RM, section 37.3.3: To receive communication data the channel occupied interrupt is unmasked (CHnOM = 0):
        // * On a RX occupied interrupt, the receiving processor checks which channel became
        // occupied, masks the associated channel occupied interrupt (CHnOM) and reads the
        // communication data from the memory.
        // todo: check which channel becomes occupied??
        // todo: Handle this in an ISR?

        // * Once the complete communication data is retrieved, the response pending software
        // variable is set. The channel status is not changed, access to the memory is kept to post
        // the subsequent response.
        // todo: In ISR?
        match core {
            Core::C1 => self.regs.c1scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
            Core::C2 => self.regs.c2scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
        }

        // To receive the response the channel free interrupt is unmasked (CHnFM = 0):
        // * On a TX free interrupt, the sending processor checks which channel became free,
        // masks the associated channel free interrupt (CHnFM) and reads the response from the
        // memory.
        // * Once the complete response is retrieved, the response pending software variable is
        // cleared. The channel status is not changed, access to the memory is kept to post the
        // subsequent communication data.
    }

    /// Check whether a channel is free; ie isn't currently handling
    /// communication. This is used both as a public API, and internally.
    pub fn channel_is_free(&self, core: Core, channel: IpccChannel) -> bool {
        // RM: Once the sending processor has posted the communication data in the memory, it sets the
        // channel status flag CHnF to occupied with CHnS.
        // Once the receiving processor has retrieved the communication data from the memory, it
        // clears the channel status flag CHnF back to free with CHnC.m
        // todo: Direction! Maybe double chan count, or sep enum?
        // todo: There's subltety with direction semantics here.
        // todo currently this is for when processor 1 is transmitting.
        match core {
            Core::C1 => match channel {
                IpccChannel::C1 => self.regs.c1toc2sr.read().ch1f().bit_is_clear(),
                IpccChannel::C2 => self.regs.c1toc2sr.read().ch2f().bit_is_clear(),
                IpccChannel::C3 => self.regs.c1toc2sr.read().ch3f().bit_is_clear(),
                IpccChannel::C4 => self.regs.c1toc2sr.read().ch4f().bit_is_clear(),
                IpccChannel::C5 => self.regs.c1toc2sr.read().ch5f().bit_is_clear(),
                IpccChannel::C6 => self.regs.c1toc2sr.read().ch6f().bit_is_clear(),
            },
            Core::C2 => match channel {
                IpccChannel::C1 => self.regs.c2toc1sr.read().ch1f().bit_is_clear(),
                IpccChannel::C2 => self.regs.c2toc1sr.read().ch2f().bit_is_clear(),
                IpccChannel::C3 => self.regs.c2toc1sr.read().ch3f().bit_is_clear(),
                IpccChannel::C4 => self.regs.c2toc1sr.read().ch4f().bit_is_clear(),
                IpccChannel::C5 => self.regs.c2toc1sr.read().ch5f().bit_is_clear(),
                IpccChannel::C6 => self.regs.c2toc1sr.read().ch6f().bit_is_clear(),
            },
        }
    }

    /// Enable a specific type of IPCC interrupt. Note that there isn't an associated `clear_interrupt`
    /// function, due to the way IPCC is set up.
    pub fn enable_interrupt(&mut self, interrupt: IpccInterrupt) {
        self.regs.c1cr.modify(|_, w| match interrupt {
            IpccInterrupt::TxFree => w.txfie().set_bit(),
            IpccInterrupt::RxOccupied => w.rxoie().set_bit(),
        });
    }

    // Code below is taken from (and modified slightly) from stm32-wb-hal
    // todo: Reconcile it with your API above. Perhaps your API is better since it's directly
    // todo from the RM?

    pub fn is_tx_pending(&self, channel: IpccChannel) -> bool {
        self.channel_is_free(Core::C1, channel) && self.get_tx_channel(Core::C1, channel)
    }

    pub fn is_rx_pending(&self, channel: IpccChannel) -> bool {
        self.channel_is_free(Core::C2, channel) && self.get_rx_channel(Core::C1, channel)
    }

    pub fn get_rx_channel(&self, core: Core, channel: IpccChannel) -> bool {
        match core {
            Core::C1 => match channel {
                IpccChannel::C1 => self.regs.c1mr.read().ch1om().bit_is_clear(),
                IpccChannel::C2 => self.regs.c1mr.read().ch2om().bit_is_clear(),
                IpccChannel::C3 => self.regs.c1mr.read().ch3om().bit_is_clear(),
                IpccChannel::C4 => self.regs.c1mr.read().ch4om().bit_is_clear(),
                IpccChannel::C5 => self.regs.c1mr.read().ch5om().bit_is_clear(),
                IpccChannel::C6 => self.regs.c1mr.read().ch6om().bit_is_clear(),
            },
            Core::C2 => match channel {
                IpccChannel::C1 => self.regs.c2mr.read().ch1om().bit_is_clear(),
                IpccChannel::C2 => self.regs.c2mr.read().ch2om().bit_is_clear(),
                IpccChannel::C3 => self.regs.c2mr.read().ch3om().bit_is_clear(),
                IpccChannel::C4 => self.regs.c2mr.read().ch4om().bit_is_clear(),
                IpccChannel::C5 => self.regs.c2mr.read().ch5om().bit_is_clear(),
                IpccChannel::C6 => self.regs.c2mr.read().ch6om().bit_is_clear(),
            },
        }
    }

    pub fn get_tx_channel(&self, core: Core, channel: IpccChannel) -> bool {
        match core {
            Core::C1 => match channel {
                IpccChannel::C1 => self.regs.c1mr.read().ch1fm().bit_is_clear(),
                IpccChannel::C2 => self.regs.c1mr.read().ch2fm().bit_is_clear(),
                IpccChannel::C3 => self.regs.c1mr.read().ch3fm().bit_is_clear(),
                IpccChannel::C4 => self.regs.c1mr.read().ch4fm().bit_is_clear(),
                IpccChannel::C5 => self.regs.c1mr.read().ch5fm().bit_is_clear(),
                IpccChannel::C6 => self.regs.c1mr.read().ch6fm().bit_is_clear(),
            },
            Core::C2 => match channel {
                IpccChannel::C1 => self.regs.c2mr.read().ch1fm().bit_is_clear(),
                IpccChannel::C2 => self.regs.c2mr.read().ch2fm().bit_is_clear(),
                IpccChannel::C3 => self.regs.c2mr.read().ch3fm().bit_is_clear(),
                IpccChannel::C4 => self.regs.c2mr.read().ch4fm().bit_is_clear(),
                IpccChannel::C5 => self.regs.c2mr.read().ch5fm().bit_is_clear(),
                IpccChannel::C6 => self.regs.c2mr.read().ch6fm().bit_is_clear(),
            },
        }
    }

    pub fn set_rx_channel(&mut self, core: Core, channel: IpccChannel, enabled: bool) {
        match core {
            Core::C1 => self.regs.c1mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1om().bit(!enabled),
                IpccChannel::C2 => w.ch2om().bit(!enabled),
                IpccChannel::C3 => w.ch3om().bit(!enabled),
                IpccChannel::C4 => w.ch4om().bit(!enabled),
                IpccChannel::C5 => w.ch5om().bit(!enabled),
                IpccChannel::C6 => w.ch6om().bit(!enabled),
            }),
            Core::C2 => self.regs.c2mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1om().bit(!enabled),
                IpccChannel::C2 => w.ch2om().bit(!enabled),
                IpccChannel::C3 => w.ch3om().bit(!enabled),
                IpccChannel::C4 => w.ch4om().bit(!enabled),
                IpccChannel::C5 => w.ch5om().bit(!enabled),
                IpccChannel::C6 => w.ch6om().bit(!enabled),
            }),
        }
    }

    pub fn set_tx_channel(&mut self, core: Core, channel: IpccChannel, enabled: bool) {
        match core {
            Core::C1 => self.regs.c1mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1fm().bit(!enabled),
                IpccChannel::C2 => w.ch2fm().bit(!enabled),
                IpccChannel::C3 => w.ch3fm().bit(!enabled),
                IpccChannel::C4 => w.ch4fm().bit(!enabled),
                IpccChannel::C5 => w.ch5fm().bit(!enabled),
                IpccChannel::C6 => w.ch6fm().bit(!enabled),
            }),
            Core::C2 => self.regs.c2mr.modify(|_, w| match channel {
                IpccChannel::C1 => w.ch1fm().bit(!enabled),
                IpccChannel::C2 => w.ch2fm().bit(!enabled),
                IpccChannel::C3 => w.ch3fm().bit(!enabled),
                IpccChannel::C4 => w.ch4fm().bit(!enabled),
                IpccChannel::C5 => w.ch5fm().bit(!enabled),
                IpccChannel::C6 => w.ch6fm().bit(!enabled),
            }),
        }
    }

    /// Clears IPCC receive channel status.
    pub fn clear_flag_channel(&mut self, core: Core, channel: IpccChannel) {
        match core {
            Core::C1 => self.regs.c1scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1c().set_bit(),
                IpccChannel::C2 => w.ch2c().set_bit(),
                IpccChannel::C3 => w.ch3c().set_bit(),
                IpccChannel::C4 => w.ch4c().set_bit(),
                IpccChannel::C5 => w.ch5c().set_bit(),
                IpccChannel::C6 => w.ch6c().set_bit(),
            }),
            Core::C2 => self.regs.c2scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1c().set_bit(),
                IpccChannel::C2 => w.ch2c().set_bit(),
                IpccChannel::C3 => w.ch3c().set_bit(),
                IpccChannel::C4 => w.ch4c().set_bit(),
                IpccChannel::C5 => w.ch5c().set_bit(),
                IpccChannel::C6 => w.ch6c().set_bit(),
            }),
        }
    }

    /// Sets IPCC receive channel status.
    pub fn set_flag_channel(&mut self, core: Core, channel: IpccChannel) {
        match core {
            Core::C1 => self.regs.c1scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
            Core::C2 => self.regs.c2scr.write(|w| match channel {
                IpccChannel::C1 => w.ch1s().set_bit(),
                IpccChannel::C2 => w.ch2s().set_bit(),
                IpccChannel::C3 => w.ch3s().set_bit(),
                IpccChannel::C4 => w.ch4s().set_bit(),
                IpccChannel::C5 => w.ch5s().set_bit(),
                IpccChannel::C6 => w.ch6s().set_bit(),
            }),
        }
    }
}
