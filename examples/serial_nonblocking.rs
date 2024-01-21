//! This example demonstrates how to use interrupts to read and write UART (serial)
//! data. We take advantage of global static Mutexes as buffers that can be accessed
//! from interrupt concept to read and write data as it's ready, allowing the CPU to
//! perform other tasks while waiting.
//!
//! Note: For many cases when reading or writing multiple words, DMA should be the
//! first choice, to minimize CPU use.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use hal::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaConfig, DmaPeriph},
    gpio::{Pin, PinMode, Port},
    low_power,
    pac::{self, interrupt},
    usart::{Usart, UsartConfig, UsartDevice, UsartInterrupt},
};

const BUF_SIZE: usize = 10;

// Set up static global variables, for sharing state between interrupt contexts and the main loop.
// Initialize `UART` to `NONE`, since we need to declare the global before setting up the peripheral.
static UART: Mutex<RefCell<Option<Usart<pac::USART1>>>> = Mutex::new(RefCell::new(None));
static READ_BUF: Mutex<RefCell<[u8; BUF_SIZE]>> = Mutex::new(RefCell::new([0; BUF_SIZE]));
static READ_I: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

// If using DMA, we use a static buffer to avoid lifetime problems.
static mut RX_BUF: [u8; BUF_SIZE] = [0; BUF_SIZE];
const DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
const DMA_CH: DmaChannel = DmaChannel::C1;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    // Configure pins for UART, according to the user manual.
    let _uart_tx = Pin::new(Port::A, 9, PinMode::Alt(7));
    let _uart_rx = Pin::new(Port::A, 10, PinMode::Alt(7));

    // Set up the USART1 peripheral.
    let mut uart = Usart::new(
        dp.USART1,
        UsartDevice::One,
        9_600,
        UsartConfig::default(),
        &clock_cfg,
    );

    uart.enable_interrupt(UsartInterrupt::ReadNotEmpty);

    unsafe {
        // Unmask interrupt lines associated with the USART1.
        NVIC::unmask(interrupt::USART1);
    }

    with(|cs| {
        // Now that we've initialized the USART peripheral, make it global.
        UART.borrow(cs).replace(Some(uart));
    });

    // Alternative approach using DMA. Note that the specifics of how you implement this
    // will depend on the format of data you are reading and writing. Specifically, pay
    // attention to how you know when a message starts and ends, if not of a fixed size.
    let mut dma = Dma::new(dp.DMA1);
    // This DMA MUX step isn't required on F3, F4, and most L4 variants.
    dma::mux(DMA_PERIPH, DMA_CH, DmaInput::Usart1Tx);
    dma.enable_interrupt(DMA_CH, DmaInterrupt::TransferComplete);

    // Example of how to start a DMA transfer:
    unsafe {
        uart.read_dma(&mut RX_BUF, DMA_CH, ChannelCfg::default(), DMA_PERIPH);
    }

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
/// Non-blocking USART read interrupt handler; read to a global buffer one byte
/// at a time as we receive them.
fn USART1() {
    with(|cs| {
        let mut u = UART.borrow(cs).borrow_mut();
        let uart = u.as_mut().unwrap();

        // Clear the interrupt flag, to prevent this ISR from repeatedly firing
        uart.clear_interrupt(UsartInterrupt::ReadNotEmpty);

        let mut buf = READ_BUF.borrow(cs).borrow_mut();

        let i = READ_I.borrow(cs);
        let i_val = i.get();
        if i_val == BUF_SIZE {
            // todo: signal end of read.
        }

        buf[i_val] = uart.read_one();
        i.set(i_val + 1);
    });
}

#[interrupt]
/// The transfer complete interrupt for our alternative, DMA-based approach. Note that even when
/// using DMA, you may want to use a UART interrupt to end the transfer.
fn DMA1_CH1() {
    with(|cs| {
        let mut u = UART.borrow(cs).borrow_mut();
        let uart = u.as_mut().unwrap();

        // Clear the interrupt flag, to prevent this ISR from repeatedly firing
        dma::clear_interrupt(DMA_PERIPH, DMA_CH, DmaInterrupt::TransferComplete);

        // (Handle the data, which is now populated in `RX_BUF`.)

        // You may need to explicitly stop the transfer, to prevent future transfers from failing.
        // This is true for writes; not sure if required for reads as well.
        dma::stop(DMA_PERIPH, DMA_CH);

        // Start a  new transfer, if appropriate for the protocol you're using.
        unsafe {
            uart.read_dma(&mut RX_BUF, DMA_CH, ChannelCfg::default(), DMA_PERIPH);
        }
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
