//! Based on `stm32f3xx-hal`.

//! Direct memory access (DMA) controller
//!
//! Currently DMA is only supported for STM32F303 MCUs.
//!
//! An example how to use DMA for serial, can be found at [examples/serial_dma.rs]
//!
//! [examples/serial_dma.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.1/examples/serial_dma.rs

// To learn about most of the ideas implemented here, check out the DMA section
// of the Embedonomicon: https://docs.rust-embedded.org/embedonomicon/dma.html

pub use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::{
    pac::{self, dma1::ch::cr},
    rcc::AHB,
    serial,
};
use core::{
    convert::TryFrom,
    mem,
    sync::atomic::{self, Ordering},
};

/// Extension trait to split a DMA peripheral into independent channels
pub trait DmaExt {
    /// The type to split the DMA into
    type Channels;

    /// Split the DMA into independent channels
    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

/// Trait implemented by DMA targets.
pub trait Target {
    /// Enable DMA on the target
    fn enable_dma(&mut self) {}
    /// Disable DMA on the target
    fn disable_dma(&mut self) {}
}

/// An in-progress one-shot DMA transfer
pub struct Transfer<B, C: Channel, T: Target> {
    // This is always a `Some` outside of `drop`.
    inner: Option<TransferInner<B, C, T>>,
}

impl<B, C: Channel, T: Target> Transfer<B, C, T> {
    /// Start a DMA write transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_write(mut buffer: B, mut channel: C, target: T) -> Self
    where
        B: WriteBuffer + 'static,
        T: OnChannel<C>,
    {
        // NOTE(unsafe) We don't know the concrete type of `buffer` here, all
        // we can use are its `WriteBuffer` methods. Hence the only `&mut self`
        // method we can call is `write_buffer`, which is allowed by
        // `WriteBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // NOTE(unsafe) We are using the address of a 'static WriteBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel.set_memory_address(ptr as u32, Increment::Enable) };
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromPeripheral);

        unsafe { Self::start(buffer, channel, target) }
    }

    /// Start a DMA read transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_read(buffer: B, mut channel: C, target: T) -> Self
    where
        B: ReadBuffer + 'static,
        T: OnChannel<C>,
    {
        // NOTE(unsafe) We don't know the concrete type of `buffer` here, all
        // we can use are its `ReadBuffer` methods. Hence there are no
        // `&mut self` methods we can call, so we are safe according to
        // `ReadBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer.read_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // NOTE(unsafe) We are using the address of a 'static ReadBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel.set_memory_address(ptr as u32, Increment::Enable) };
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromMemory);

        unsafe { Self::start(buffer, channel, target) }
    }

    /// # Safety
    ///
    /// Callers must ensure that:
    ///
    /// - the given buffer will be valid for the duration of the transfer
    /// - the DMA channel is configured correctly for the given target and buffer
    unsafe fn start(buffer: B, mut channel: C, mut target: T) -> Self
    where
        T: OnChannel<C>,
    {
        crate::assert!(!channel.is_enabled());

        atomic::compiler_fence(Ordering::Release);

        target.enable_dma();
        channel.enable();

        Self {
            inner: Some(TransferInner {
                buffer,
                channel,
                target,
            }),
        }
    }

    /// Is this transfer complete?
    pub fn is_complete(&self) -> bool {
        let inner = crate::unwrap!(self.inner.as_ref());
        inner.channel.event_occurred(Event::TransferComplete)
    }

    /// Stop this transfer and return ownership over its parts
    pub fn stop(mut self) -> (B, C, T) {
        let mut inner = crate::unwrap!(self.inner.take());
        inner.stop();

        (inner.buffer, inner.channel, inner.target)
    }

    /// Block until this transfer is done and return ownership over its parts
    pub fn wait(self) -> (B, C, T) {
        while !self.is_complete() {}

        self.stop()
    }
}

impl<B, C: Channel, T: Target> Drop for Transfer<B, C, T> {
    fn drop(&mut self) {
        if let Some(inner) = self.inner.as_mut() {
            inner.stop();
        }
    }
}

/// This only exists so we can implement `Drop` for `Transfer`.
struct TransferInner<B, C, T> {
    buffer: B,
    channel: C,
    target: T,
}

impl<B, C: Channel, T: Target> TransferInner<B, C, T> {
    /// Stop this transfer
    fn stop(&mut self) {
        self.channel.disable();
        self.target.disable_dma();

        atomic::compiler_fence(Ordering::SeqCst);
    }
}

/// DMA address increment mode
pub enum Increment {
    /// Enable increment
    Enable,
    /// Disable increment
    Disable,
}

impl From<Increment> for cr::PINC_A {
    fn from(inc: Increment) -> Self {
        match inc {
            Increment::Enable => cr::PINC_A::ENABLED,
            Increment::Disable => cr::PINC_A::DISABLED,
        }
    }
}

/// Channel priority level
pub enum Priority {
    /// Low
    Low,
    /// Medium
    Medium,
    /// High
    High,
    /// Very high
    VeryHigh,
}

impl From<Priority> for cr::PL_A {
    fn from(prio: Priority) -> Self {
        match prio {
            Priority::Low => cr::PL_A::LOW,
            Priority::Medium => cr::PL_A::MEDIUM,
            Priority::High => cr::PL_A::HIGH,
            Priority::VeryHigh => cr::PL_A::VERYHIGH,
        }
    }
}

/// DMA transfer direction
pub enum Direction {
    /// From memory to peripheral
    FromMemory,
    /// From peripheral to memory
    FromPeripheral,
}

impl From<Direction> for cr::DIR_A {
    fn from(dir: Direction) -> Self {
        match dir {
            Direction::FromMemory => cr::DIR_A::FROMMEMORY,
            Direction::FromPeripheral => cr::DIR_A::FROMPERIPHERAL,
        }
    }
}

/// DMA events
pub enum Event {
    /// First half of a transfer is done
    HalfTransfer,
    /// Transfer is complete
    TransferComplete,
    /// A transfer error occurred
    TransferError,
    /// Any of the above events occurred
    Any,
}

/// Trait implemented by all DMA channels
pub trait Channel: private::Channel {
    /// Is the interrupt flag for the given event set?
    fn event_occurred(&self, event: Event) -> bool;

    /// Clear the interrupt flag for the given event.
    ///
    /// Passing `Event::Any` clears all interrupt flags.
    ///
    /// Note that the the global interrupt flag is not automatically cleared
    /// even when all other flags are cleared. The only way to clear it is to
    /// call this method with `Event::Any`.
    fn clear_event(&mut self, event: Event);

    /// Reset the control registers of this channel.
    /// This stops any ongoing transfers.
    fn reset(&mut self) {
        self.ch().cr.reset();
        self.ch().ndtr.reset();
        self.ch().par.reset();
        self.ch().mar.reset();
        self.clear_event(Event::Any);
    }

    /// Set the base address of the peripheral data register from/to which the
    /// data will be read/written.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    ///
    /// # Safety
    ///
    /// Callers must ensure the given address is the address of a peripheral
    /// register that supports DMA.
    unsafe fn set_peripheral_address(&mut self, address: u32, inc: Increment) {
        crate::assert!(!self.is_enabled());

        self.ch().par.write(|w| w.pa().bits(address));
        self.ch().cr.modify(|_, w| w.pinc().variant(inc.into()));
    }

    /// Set the base address of the memory area from/to which
    /// the data will be read/written.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    ///
    /// # Safety
    ///
    /// Callers must ensure the given address is a valid memory address
    /// that will remain valid as long as at is used by DMA.
    unsafe fn set_memory_address(&mut self, address: u32, inc: Increment) {
        crate::assert!(!self.is_enabled());

        self.ch().mar.write(|w| w.ma().bits(address));
        self.ch().cr.modify(|_, w| w.minc().variant(inc.into()));
    }

    /// Set the number of words to transfer.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    fn set_transfer_length(&mut self, len: u16) {
        crate::assert!(!self.is_enabled());

        self.ch().ndtr.write(|w| w.ndt().bits(len));
    }

    /// Set the word size.
    ///
    /// # Panics
    ///
    /// Panics if the word size is not one of 8, 16, or 32 bits.
    fn set_word_size<W>(&mut self) {
        use cr::PSIZE_A::*;

        let psize = match mem::size_of::<W>() {
            1 => BITS8,
            2 => BITS16,
            4 => BITS32,
            s => crate::panic!("unsupported word size: {:?}", s),
        };

        self.ch().cr.modify(|_, w| {
            w.psize().variant(psize);
            w.msize().variant(psize)
        });
    }

    /// Set the priority level of this channel
    fn set_priority_level(&mut self, priority: Priority) {
        let pl = priority.into();
        self.ch().cr.modify(|_, w| w.pl().variant(pl));
    }

    /// Set the transfer direction
    fn set_direction(&mut self, direction: Direction) {
        let dir = direction.into();
        self.ch().cr.modify(|_, w| w.dir().variant(dir));
    }

    /// Enable the interrupt for the given event
    fn listen(&mut self, event: Event) {
        use Event::*;
        match event {
            HalfTransfer => self.ch().cr.modify(|_, w| w.htie().enabled()),
            TransferComplete => self.ch().cr.modify(|_, w| w.tcie().enabled()),
            TransferError => self.ch().cr.modify(|_, w| w.teie().enabled()),
            Any => self.ch().cr.modify(|_, w| {
                w.htie().enabled();
                w.tcie().enabled();
                w.teie().enabled()
            }),
        }
    }

    /// Disable the interrupt for the given event
    fn unlisten(&mut self, event: Event) {
        use Event::*;
        match event {
            HalfTransfer => self.ch().cr.modify(|_, w| w.htie().disabled()),
            TransferComplete => self.ch().cr.modify(|_, w| w.tcie().disabled()),
            TransferError => self.ch().cr.modify(|_, w| w.teie().disabled()),
            Any => self.ch().cr.modify(|_, w| {
                w.htie().disabled();
                w.tcie().disabled();
                w.teie().disabled()
            }),
        }
    }

    /// Start a transfer
    fn enable(&mut self) {
        self.clear_event(Event::Any);
        self.ch().cr.modify(|_, w| w.en().enabled());
    }

    /// Stop the current transfer
    fn disable(&mut self) {
        self.ch().cr.modify(|_, w| w.en().disabled());
    }

    /// Is there a transfer in progress on this channel?
    fn is_enabled(&self) -> bool {
        self.ch().cr.read().en().is_enabled()
    }
}

mod private {
    use crate::pac;

    /// Channel methods private to this module
    pub trait Channel {
        /// Return the register block for this channel
        fn ch(&self) -> &pac::dma1::CH;
    }
}

macro_rules! dma {
    (
        $DMAx:ident, $dmax:ident, $dmaxen:ident,
        channels: {
            $( $Ci:ident: (
                $chi:ident,
                $htifi:ident, $tcifi:ident, $teifi:ident, $gifi:ident,
                $chtifi:ident, $ctcifi:ident, $cteifi:ident, $cgifi:ident
            ), )+
        },
    ) => {
        paste::paste! {
            #[doc = "All associated types, traits and methods of the `" $DMAx "` peripheral."]
            pub mod $dmax {
                use super::*;
                use crate::pac::$DMAx;

                impl DmaExt for $DMAx {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        ahb.enr().modify(|_, w| w.$dmaxen().set_bit());

                        let mut channels = Channels {
                            $( $chi: $Ci { _0: () }, )+
                        };

                        channels.reset();
                        channels
                    }
                }

                /// DMA channels
                pub struct Channels {
                    $(
                        /// Channel
                        pub $chi: $Ci,
                    )+
                }

                impl Channels {
                    /// Reset the control registers of all channels.
                    /// This stops any ongoing transfers.
                    fn reset(&mut self) {
                        $( self.$chi.reset(); )+
                    }
                }

                $(
                    /// Singleton that represents a DMA channel
                    pub struct $Ci {
                        _0: (),
                    }

                    impl private::Channel for $Ci {
                        fn ch(&self) -> &pac::dma1::CH {
                            // NOTE(unsafe) $Ci grants exclusive access to this register
                            unsafe { &(*$DMAx::ptr()).$chi }
                        }
                    }

                    impl Channel for $Ci {
                        fn event_occurred(&self, event: Event) -> bool {
                            use Event::*;

                            // NOTE(unsafe) atomic read
                            let flags = unsafe { (*$DMAx::ptr()).isr.read() };
                            match event {
                                HalfTransfer => flags.$htifi().bit_is_set(),
                                TransferComplete => flags.$tcifi().bit_is_set(),
                                TransferError => flags.$teifi().bit_is_set(),
                                Any => flags.$gifi().bit_is_set(),
                            }
                        }

                        fn clear_event(&mut self, event: Event) {
                            use Event::*;

                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe {
                                &(*$DMAx::ptr()).ifcr.write(|w| match event {
                                    HalfTransfer => w.$chtifi().set_bit(),
                                    TransferComplete => w.$ctcifi().set_bit(),
                                    TransferError => w.$cteifi().set_bit(),
                                    Any => w.$cgifi().set_bit(),
                                });
                            }
                        }
                    }
                )+
            }
        }
    };

    ( $X:literal: {$($C:literal),+} ) => {
        paste::paste! {
            dma!(
                [<DMA $X>], [<dma $X>], [<dma $X en>],
                channels: {
                    $(
                        [<C $C>]:
			(
                            [<ch $C>],
                            [<htif $C>],
                            [<tcif $C>],
                            [<teif $C>],
                            [<gif $C>],
                            [<chtif $C>],
                            [<ctcif $C>],
                            [<cteif $C>],
                            [<cgif $C>]
                        ),
                    )+
                },
            );
        }
    };
}

dma!( 1: { 1,2,3,4,5,6,7 } );

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
dma!( 2: { 1,2,3,4,5 } );

/// Marker trait mapping DMA targets to their channels
///
/// # Safety
///
/// `C` must be the correct DMA channel for the peripheral implementing
/// this trait.
pub unsafe trait OnChannel<C: Channel>: Target {}

macro_rules! on_channel {
    (
        $dma:ident,
        $( $target:ty => $C:ident, )+
    ) => {
        $( unsafe impl OnChannel<$dma::$C> for $target {} )+
    };
}

on_channel!(dma1,
    serial::Rx<pac::USART1> => C5,
    serial::Tx<pac::USART1> => C4,
    serial::Rx<pac::USART2> => C6,
    serial::Tx<pac::USART2> => C7,
    serial::Rx<pac::USART3> => C3,
    serial::Tx<pac::USART3> => C2,
);
