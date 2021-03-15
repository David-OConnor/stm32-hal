//! Based on `stm32l4xx-hal`.

//! Direct Memory Access Engine

#![allow(dead_code)]

use core::fmt;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::ptr;
use core::slice;

use crate::pac::RCC;
use as_slice::AsSlice;
pub use generic_array::typenum::{self, consts};
use generic_array::{ArrayLength, GenericArray};
use stable_deref_trait::StableDeref;

use paste::paste;

#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    Overrun,
    BufferError,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

/// Frame reader "worker", access and handling of frame reads is made through this structure.
pub struct FrameReader<BUFFER, CHANNEL, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    N: ArrayLength<MaybeUninit<u8>>,
{
    buffer: BUFFER,
    channel: CHANNEL,
    matching_character: u8,
    _marker: core::marker::PhantomData<N>, // Needed to make the compiler happy
}

impl<BUFFER, CHANNEL, N> FrameReader<BUFFER, CHANNEL, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    N: ArrayLength<MaybeUninit<u8>>,
{
    pub(crate) fn new(
        buffer: BUFFER,
        channel: CHANNEL,
        matching_character: u8,
    ) -> FrameReader<BUFFER, CHANNEL, N> {
        Self {
            buffer,
            channel,
            matching_character,
            _marker: core::marker::PhantomData,
        }
    }
}

/// Frame sender "worker", access and handling of frame transmissions is made through this
/// structure.
pub struct FrameSender<BUFFER, CHANNEL, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    N: ArrayLength<MaybeUninit<u8>>,
{
    buffer: Option<BUFFER>,
    channel: CHANNEL,
    _marker: core::marker::PhantomData<N>,
}

impl<BUFFER, CHANNEL, N> FrameSender<BUFFER, CHANNEL, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    N: ArrayLength<MaybeUninit<u8>>,
{
    pub(crate) fn new(channel: CHANNEL) -> FrameSender<BUFFER, CHANNEL, N> {
        Self {
            buffer: None,
            channel,
            _marker: core::marker::PhantomData,
        }
    }
}

/// Data type for holding data frames for the Serial.
///
/// Internally used uninitialized storage, making this storage zero cost to create. It can also be
/// used with, for example, [`heapless::pool`] to create a pool of serial frames.
///
/// [`heapless::pool`]: https://docs.rs/heapless/0.5.3/heapless/pool/index.html
pub struct DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    len: u16,
    buf: GenericArray<MaybeUninit<u8>, N>,
}

impl<N> fmt::Debug for DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.read())
    }
}

impl<N> fmt::Write for DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let free = self.free();

        if s.len() > free {
            Err(fmt::Error)
        } else {
            self.write_slice(s.as_bytes());
            Ok(())
        }
    }
}

impl<N> Default for DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<N> DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    /// Creates a new node for the Serial DMA
    #[inline]
    pub fn new() -> Self {
        // Create an uninitialized array of `MaybeUninit<u8>`. The `assume_init` is
        // safe because the type we are claiming to have initialized here is a
        // bunch of `MaybeUninit`s, which do not require initialization.
        #[allow(clippy::uninit_assumed_init)]
        Self {
            len: 0,
            buf: unsafe { MaybeUninit::uninit().assume_init() },
        }
    }

    /// Gives a `&mut [u8]` slice to write into with the maximum size, the `commit` method
    /// must then be used to set the actual number of bytes written.
    ///
    /// Note that this function internally first zeros the uninitialized part of the node's buffer.
    pub fn write(&mut self) -> &mut [u8] {
        // Initialize remaining memory with a safe value
        for elem in &mut self.buf[self.len as usize..] {
            *elem = MaybeUninit::zeroed();
        }

        self.len = self.max_len() as u16;

        // NOTE(unsafe): This is safe as the operation above set the entire buffer to a valid state
        unsafe { slice::from_raw_parts_mut(self.buf.as_mut_ptr() as *mut _, self.max_len()) }
    }

    /// Used to shrink the current size of the frame, used in conjunction with `write`.
    #[inline]
    pub fn commit(&mut self, shrink_to: usize) {
        // Only shrinking is allowed to remain safe with the `MaybeUninit`
        if shrink_to < self.len as _ {
            self.len = shrink_to as _;
        }
    }

    /// Gives an uninitialized `&mut [MaybeUninit<u8>]` slice to write into, the `set_len` method
    /// must then be used to set the actual number of bytes written.
    #[inline]
    pub fn write_uninit(&mut self) -> &mut [MaybeUninit<u8>] {
        &mut self.buf
    }

    /// Used to set the current size of the frame, used in conjunction with `write_uninit` to have an
    /// interface for uninitialized memory. Use with care!
    ///
    /// # Safety
    ///
    /// NOTE(unsafe): This must be set so that the final buffer is only referencing initialized
    /// memory.
    #[inline]
    pub unsafe fn set_len(&mut self, len: usize) {
        assert!(len <= self.max_len());
        self.len = len as _;
    }

    /// Used to write data into the node, and returns how many bytes were written from `buf`.
    ///
    /// If the node is already partially filled, this will continue filling the node.
    pub fn write_slice(&mut self, buf: &[u8]) -> usize {
        let count = buf.len().min(self.free());

        // Used to write data into the `MaybeUninit`
        // NOTE(unsafe): Safe based on the size check above
        unsafe {
            ptr::copy_nonoverlapping(
                buf.as_ptr(),
                (self.buf.as_mut_ptr() as *mut u8).add(self.len.into()),
                count,
            );
        }

        self.len += count as u16;

        count
    }

    /// Clear the node of all data making it empty
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Returns a readable slice which maps to the buffers internal data
    #[inline]
    pub fn read(&self) -> &[u8] {
        // NOTE(unsafe): Safe as it uses the internal length of valid data
        unsafe { slice::from_raw_parts(self.buf.as_ptr() as *const _, self.len as usize) }
    }

    /// Returns a readable mutable slice which maps to the buffers internal data
    #[inline]
    pub fn read_mut(&mut self) -> &mut [u8] {
        // NOTE(unsafe): Safe as it uses the internal length of valid data
        unsafe { slice::from_raw_parts_mut(self.buf.as_mut_ptr() as *mut _, self.len as usize) }
    }

    /// Reads how many bytes are available
    #[inline]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Reads how many bytes are free
    #[inline]
    pub fn free(&self) -> usize {
        self.max_len() - self.len as usize
    }

    /// Get the max length of the frame
    #[inline]
    pub fn max_len(&self) -> usize {
        N::to_usize()
    }

    /// Checks if the frame is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    pub(crate) unsafe fn buffer_address_for_dma(&self) -> u32 {
        self.buf.as_ptr() as u32
    }

    #[inline]
    pub(crate) fn buffer_as_ptr(&self) -> *const MaybeUninit<u8> {
        self.buf.as_ptr()
    }

    #[inline]
    pub(crate) fn buffer_as_mut_ptr(&mut self) -> *mut MaybeUninit<u8> {
        self.buf.as_mut_ptr()
    }
}

impl<N> AsSlice for DMAFrame<N>
where
    N: ArrayLength<MaybeUninit<u8>>,
{
    type Element = u8;

    fn as_slice(&self) -> &[Self::Element] {
        self.read()
    }
}

pub struct CircBuffer<BUFFER, CHANNEL>
where
    BUFFER: 'static,
{
    buffer: BUFFER,
    channel: CHANNEL,
    readable_half: Half,
    consumed_offset: usize,
}

impl<BUFFER, CHANNEL> CircBuffer<BUFFER, CHANNEL> {
    pub(crate) fn new<H>(buf: BUFFER, chan: CHANNEL) -> Self
    where
        BUFFER: StableDeref<Target = [H; 2]> + 'static,
    {
        CircBuffer {
            buffer: buf,
            channel: chan,
            readable_half: Half::Second,
            consumed_offset: 0,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut RCC) -> Self::Channels;
}

pub struct Transfer<MODE, BUFFER, CHANNEL, PAYLOAD> {
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    channel: CHANNEL,
    payload: PAYLOAD,
}

impl<BUFFER, CHANNEL, PAYLOAD> Transfer<R, BUFFER, CHANNEL, PAYLOAD>
where
    BUFFER: StableDeref + 'static,
{
    pub(crate) fn r(buffer: BUFFER, channel: CHANNEL, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            channel,
            payload,
        }
    }
}

impl<BUFFER, CHANNEL, PAYLOAD> Transfer<W, BUFFER, CHANNEL, PAYLOAD>
where
    BUFFER: StableDeref + 'static,
{
    pub(crate) fn w(buffer: BUFFER, channel: CHANNEL, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            channel,
            payload,
        }
    }
}

impl<BUFFER, CHANNEL, PAYLOAD> Deref for Transfer<R, BUFFER, CHANNEL, PAYLOAD> {
    type Target = BUFFER;

    fn deref(&self) -> &BUFFER {
        &self.buffer
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

macro_rules! dma {
    ($($DMAX:ident: ($dma:ident, $enr:ident, $rst:ident, {
        $($CX:ident: (
            $ccrX:ident,
            $CCRX:ident,
            $cndtrX:ident,
            $CNDTRX:ident,
            $cparX:ident,
            $CPARX:ident,
            $cmarX:ident,
            $CMARX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident,
            $teifX:ident,
            $cteifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dma {
                use core::sync::atomic::{self, Ordering};
                use as_slice::{AsSlice};
                use crate::pac::{$DMAX, dma1};
                use core::mem::MaybeUninit;
                use generic_array::ArrayLength;
                use core::ops::DerefMut;
                use core::ptr;
                use stable_deref_trait::StableDeref;

                use crate::dma::{CircBuffer, FrameReader, FrameSender, DMAFrame, DmaExt, Error, Event, Half, Transfer, W};
                use crate::pac::RCC;

                use paste::paste;

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels((), $(pub $CX),+);

                $(
                    pub struct $CX;

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        #[inline]
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.cpar().write(|w|
                                unsafe { w.pa().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        #[inline]
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.cmar().write(|w|
                                unsafe { w.ma().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        #[inline]
                        pub fn set_transfer_length(&mut self, len: u16) {
                            self.cndtr().write(|w| w.ndt().bits(len));
                        }

                        /// Starts the DMA transfer
                        #[inline]
                        pub fn start(&mut self) {
                            self.ccr().modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        #[inline]
                        pub fn stop(&mut self) {
                            self.ifcr().write(|w| w.$cgifX().set_bit());
                            self.ccr().modify(|_, w| w.en().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        #[inline]
                        pub fn in_progress(&self) -> bool {
                            self.isr().$tcifX().bit_is_clear()
                        }

                        #[inline]
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ccr().modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.ccr().modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        #[inline]
                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ccr().modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ccr().modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        #[inline]
                        pub(crate) fn isr(&self) -> dma1::isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).isr.read() }
                        }

                        #[inline]
                        pub(crate) fn ifcr(&self) -> &dma1::IFCR {
                            unsafe { &(*$DMAX::ptr()).ifcr }
                        }

                        #[inline]
                        pub(crate) fn ccr(&mut self) -> &dma1::$CCRX {
                            unsafe { &(*$DMAX::ptr()).$ccrX }
                        }

                        #[inline]
                        pub(crate) fn cndtr(&mut self) -> &dma1::$CNDTRX {
                            unsafe { &(*$DMAX::ptr()).$cndtrX }
                        }

                        #[inline]
                        pub(crate) fn cpar(&mut self) -> &dma1::$CPARX {
                            unsafe { &(*$DMAX::ptr()).$cparX }
                        }

                        #[inline]
                        pub(crate) fn cmar(&mut self) -> &dma1::$CMARX {
                            unsafe { &(*$DMAX::ptr()).$cmarX }
                        }

                        #[inline]
                        pub(crate) fn cselr(&mut self) -> &dma1::CSELR {
                            unsafe { &(*$DMAX::ptr()).cselr }
                        }

                        #[inline]
                        pub(crate) fn get_cndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).$cndtrX.read().bits() }
                        }

                    }

                    impl<BUFFER, N> FrameSender<BUFFER, $CX, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        N: ArrayLength<MaybeUninit<u8>>,
                    {
                        /// This method should be called in the transfer complete interrupt of the
                        /// DMA, will return the sent frame if the transfer was truly completed.
                        pub fn transfer_complete_interrupt(
                            &mut self,
                        ) -> Option<BUFFER> {

                            // Clear ISR flag (Transfer Complete)
                            if !self.channel.in_progress() {
                                self.channel.ifcr().write(|w| w.$ctcifX().set_bit());
                            } else {
                                // The old transfer is not complete
                                return None;
                            }

                            self.channel.stop();

                            // NOTE(compiler_fence) operations on the DMA should not be reordered
                            // before the next statement, takes the buffer from the DMA transfer.
                            atomic::compiler_fence(Ordering::SeqCst);

                            // Return the old buffer for the user to do what they want with it
                            self.buffer.take()
                        }

                        /// Returns `true` if there is an ongoing transfer.
                        #[inline]
                        pub fn ongoing_transfer(&self) -> bool {
                            self.buffer.is_some()
                        }

                        /// Send a frame. Will return `Err(frame)` if there was already an ongoing
                        /// transaction or if the buffer has not been read out.
                        pub fn send(
                            &mut self,
                            frame: BUFFER,
                        ) -> Result<(), BUFFER> {
                            if self.ongoing_transfer() {
                                // The old transfer is not complete
                                return Err(frame);
                            }

                            let new_buf = &*frame;
                            self.channel.set_memory_address(new_buf.buffer_as_ptr() as u32, true);
                            self.channel.set_transfer_length(new_buf.len() as u16);

                            // If there has been an error, clear the error flag to let the next
                            // transaction start
                            if self.channel.isr().$teifX().bit_is_set() {
                                self.channel.ifcr().write(|w| w.$cteifX().set_bit());
                            }

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts the DMA transfer
                            atomic::compiler_fence(Ordering::Release);

                            self.channel.start();

                            self.buffer = Some(frame);

                            Ok(())
                        }
                    }

                    impl<BUFFER, N> FrameReader<BUFFER, $CX, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        N: ArrayLength<MaybeUninit<u8>>,
                    {
                        /// This function should be called from the transfer complete interrupt of
                        /// the corresponding DMA channel.
                        ///
                        /// Returns the full buffer received by the USART.
                        #[inline]
                        pub fn transfer_complete_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, false)
                        }

                        /// This function should be called from the character match interrupt of
                        /// the corresponding USART
                        ///
                        /// Returns the buffer received by the USART, including the matching
                        /// character.
                        #[inline]
                        pub fn character_match_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, true)
                        }

                        /// This function should be called from the receiver timeout interrupt of
                        /// the corresponding USART
                        ///
                        /// Returns the buffer received by the USART.
                        #[inline]
                        pub fn receiver_timeout_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, false)
                        }

                        fn internal_interrupt(
                            &mut self,
                            mut next_frame: BUFFER,
                            character_match_interrupt: bool,
                        ) -> BUFFER {
                            let old_buf = &mut *self.buffer;
                            let new_buf = &mut *next_frame;
                            new_buf.clear();

                            // Clear ISR flag (Transfer Complete)
                            if !self.channel.in_progress() {
                                self.channel.ifcr().write(|w| w.$ctcifX().set_bit());
                            } else if character_match_interrupt {
                                // 1. If DMA not done and there was a character match interrupt,
                                // let the DMA flush a little and then halt transfer.
                                //
                                // This is to alleviate the race condition between the character
                                // match interrupt and the DMA memory transfer.
                                let left_in_buffer = self.channel.get_cndtr() as usize;

                                for _ in 0..5 {
                                    let now_left = self.channel.get_cndtr() as usize;

                                    if left_in_buffer - now_left >= 4 {
                                        // We have gotten 4 extra characters flushed
                                        break;
                                    }
                                }
                            }

                            self.channel.stop();

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts a new DMA transfer
                            atomic::compiler_fence(Ordering::SeqCst);

                            let left_in_buffer = self.channel.get_cndtr() as usize;
                            let got_data_len = old_buf.max_len() - left_in_buffer; // How many bytes we got
                            unsafe {
                                old_buf.set_len(got_data_len);
                            }

                            // 2. Check DMA race condition by finding matched character, and that
                            //    the length is larger than 0
                            let len = if character_match_interrupt && got_data_len > 0 {
                                let search_buf = old_buf.read();

                                // Search from the end
                                let ch = self.matching_character;
                                if let Some(pos) = search_buf.iter().rposition(|&x| x == ch) {
                                    pos+1
                                } else {
                                    // No character match found
                                    0
                                }
                            } else {
                                old_buf.len()
                            };

                            // 3. Start DMA again
                            let diff = if len < got_data_len {
                                // We got some extra characters in the from the new frame, move
                                // them into the new buffer
                                let diff = got_data_len - len;

                                let new_buf_ptr = new_buf.buffer_as_mut_ptr();
                                let old_buf_ptr = old_buf.buffer_as_ptr();

                                // new_buf[0..diff].copy_from_slice(&old_buf[len..got_data_len]);
                                unsafe {
                                    ptr::copy_nonoverlapping(old_buf_ptr.add(len), new_buf_ptr, diff);
                                }

                                diff
                            } else {
                                0
                            };

                            self.channel.set_memory_address(unsafe { new_buf.buffer_as_ptr().add(diff) } as u32, true);
                            self.channel.set_transfer_length((new_buf.max_len() - diff) as u16);
                            let received_buffer = core::mem::replace(&mut self.buffer, next_frame);

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts the DMA transfer
                            atomic::compiler_fence(Ordering::Release);

                            self.channel.start();

                            // 4. Return full frame
                            received_buffer
                        }
                    }

                    impl<B> CircBuffer<B, $CX> {

                        /// Return the partial contents of the buffer half being written
                        pub fn partial_peek<R, F, H, T>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&[T], Half) -> Result<(usize, R), ()>,
                            B: StableDeref<Target = [H; 2]> + 'static,
                            H: AsSlice<Element=T>,
                        {
                            // this inverts expectation and returns the half being _written_
                            let buf = match self.readable_half {
                                Half::First => &self.buffer[1],
                                Half::Second => &self.buffer[0],
                            };
                            //                          ,- half-buffer
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- pending=11
                            let pending = self.channel.get_cndtr() as usize; // available bytes in _whole_ buffer
                            let slice = buf.as_slice();
                            let capacity = slice.len(); // capacity of _half_ a buffer
                            //     <--- capacity=10 --->
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            let pending = if pending > capacity {
                                pending - capacity
                            } else {
                                pending
                            };
                            //                          ,- half-buffer
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- pending=1
                            let end = capacity - pending;
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- end=9
                            //             ^- consumed_offset=4
                            //             [y y y y y] <-- slice
                            let slice = &buf.as_slice()[self.consumed_offset..end];
                            match f(slice, self.readable_half) {
                                Ok((l, r)) => { self.consumed_offset += l; Ok(r) },
                                Err(_) => Err(Error::BufferError),
                            }
                        }

                        /// Peeks into the readable half of the buffer
                        /// Returns the result of the closure
                        pub fn peek<R, F, H, T>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&[T], Half) -> R,
                            B: StableDeref<Target = [H; 2]> + 'static,
                            H: AsSlice<Element=T>,
                        {
                            let half_being_read = self.readable_half()?;
                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };
                            let slice = &buf.as_slice()[self.consumed_offset..];
                            self.consumed_offset = 0;
                            Ok(f(slice, half_being_read))
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let isr = self.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.channel.ifcr().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.channel.ifcr().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, $CX, PAYLOAD> {
                        pub fn is_done(&self) -> bool {
                            self.channel.isr().$tcifX().bit_is_set()
                        }

                        pub fn wait(mut self) -> (BUFFER, $CX, PAYLOAD) {
                            // XXX should we check for transfer errors here?
                            // The manual says "A DMA transfer error can be generated by reading
                            // from or writing to a reserved address space". I think it's impossible
                            // to get to that state with our type safe API and *safe* Rust.
                            while !self.is_done() {}

                            self.channel.ifcr().write(|w| w.$cgifX().set_bit());

                            self.channel.ccr().modify(|_, w| w.en().clear_bit());

                            // TODO can we weaken this compiler barrier?
                            // NOTE(compiler_fence) operations on `buffer` should not be reordered
                            // before the previous statement, which marks the DMA transfer as done
                            atomic::compiler_fence(Ordering::SeqCst);

                            (self.buffer, self.channel, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<W, &'static mut BUFFER, $CX, PAYLOAD> {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsSlice<Element=T>,
                        {
                            let pending = self.channel.get_cndtr() as usize;

                            let capacity = self.buffer.as_slice().len();

                            &self.buffer.as_slice()[..(capacity - pending)]
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, rcc: &mut RCC) -> Channels {
                        // todo: H7!!
                        cfg_if::cfg_if! {
                            if #[cfg(feature = "f3")] {
                                paste! {
                                    rcc.[<ahb1 enr>].modify(|_, w| w.[<$dma en>]().set_bit());
                                }
                            } else if #[cfg(any(feature = "l4", feature = "l5"))] {
                                paste! {
                                    // We use `$enr` and $rst, since we only add `1` after for apb1.
                                    // This isn't required on f3.
                                    rcc.[<ahb1 $enr>].modify(|_, w| w.[<$dma en>]().set_bit());
                                }
                            }
                        }

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$ccrX.reset();
                        )+

                        Channels((), $($CX { }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, enr, rstr1, {
        C1: (
            ccr1, CCR1,
            cndtr1, CNDTR1,
            cpar1, CPAR1,
            cmar1, CMAR1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1,
            teif1, cteif1
        ),
        C2: (
            ccr2, CCR2,
            cndtr2, CNDTR2,
            cpar2, CPAR2,
            cmar2, CMAR2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2,
            teif2, cteif2
        ),
        C3: (
            ccr3, CCR3,
            cndtr3, CNDTR3,
            cpar3, CPAR3,
            cmar3, CMAR3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3,
            teif3, cteif3
        ),
        C4: (
            ccr4, CCR4,
            cndtr4, CNDTR4,
            cpar4, CPAR4,
            cmar4, CMAR4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4,
            teif4, cteif4
        ),
        C5: (
            ccr5, CCR5,
            cndtr5, CNDTR5,
            cpar5, CPAR5,
            cmar5, CMAR5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5,
            teif5, cteif5
        ),
        C6: (
            ccr6, CCR6,
            cndtr6, CNDTR6,
            cpar6, CPAR6,
            cmar6, CMAR6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6,
            teif6, cteif6
        ),
        C7: (
            ccr7, CCR7,
            cndtr7, CNDTR7,
            cpar7, CPAR7,
            cmar7, CMAR7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7,
            teif7, cteif7
        ),
    }),
    DMA2: (dma2, enr, rstr1, {
        C1: (
            ccr1, CCR1,
            cndtr1, CNDTR1,
            cpar1, CPAR1,
            cmar1, CMAR1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1,
            teif1, cteif1
        ),
        C2: (
            ccr2, CCR2,
            cndtr2, CNDTR2,
            cpar2, CPAR2,
            cmar2, CMAR2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2,
            teif2, cteif2
        ),
        C3: (
            ccr3, CCR3,
            cndtr3, CNDTR3,
            cpar3, CPAR3,
            cmar3, CMAR3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3,
            teif3, cteif3
        ),
        C4: (
            ccr4, CCR4,
            cndtr4, CNDTR4,
            cpar4, CPAR4,
            cmar4, CMAR4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4,
            teif4, cteif4
        ),
        C5: (
            ccr5, CCR5,
            cndtr5, CNDTR5,
            cpar5, CPAR5,
            cmar5, CMAR5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5,
            teif5, cteif5
        ),
        C6: (
            ccr6, CCR6,
            cndtr6, CNDTR6,
            cpar6, CPAR6,
            cmar6, CMAR6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6,
            teif6, cteif6
        ),
        C7: (
            ccr7, CCR7,
            cndtr7, CNDTR7,
            cpar7, CPAR7,
            cmar7, CMAR7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7,
            teif7, cteif7
        ),
    }),
}
