//! This module demonstrates basic reads and writes to and from onboard flash memory.

#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use critical_section::{with, Mutex};
use hal::{
    clocks::Clocks,
    flash::{Bank, Flash},
    low_power,
};

// Make sure the starting page isn't outside the available page range, and isn't in part of the
// flash memory taken up by your program. If you're not sure what to do, put something towards the end.
// Check your Reference Manual, FLASH section near the top for info on flash page size, and number
// of pages. Called "sector" on H7 and F4.
const FLASH_PAGE: usize = 254;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::Default();
    clock_cfg.setup().unwrap();

    let mut flash = Flash::new(dp.FLASH);

    // Before any writes take place, you must erase the page the writes will take place on.
    // On H7, replace `page` with `sector`.
    flash.erase_page(Bank::B1, FLASH_PAGE).ok();

    // Note that if using L5, or some variants of G4 or L4, you can configure FLASH to operate in
    // dual bank mode by setting the DBANK option bit. This library won't set that for you, but you
    // can set it manually, then configure the `Flash` struct with the `dual_bank` field. By default,
    // it assumes single-bank config. Note that H7 is always dual bank, and other variants are always
    // single-bank. (F4 excepted)

    // Once a page is erased, we can write to it. Writes take place internally in 32-bit intervals
    // (H7 excepted; uses varying write sizes), but our API uses 8-bit buffers.
    flash.write_page(Bank::B1, FLASH_PAGE, &[1, 2, 3, 4]);

    // Alternatively, we can erase and write together using this function:
    flash.erase_write_page(Bank::B1, FLASH_PAGE, &[1, 2, 3, 4]);

    // We can read to a u8 buffer using the `read` method:
    let mut read_buf = [u8; 4];
    flash.read(Bank::B1, FLASH_PAGE, 0, read_buf);

    // Note: Some debugging setups will crash during flash page erases or writes. You can check if
    // this is the case (after receieving an `SwdAPWait` message etc by inspecing the FLASH written
    // using `Stm32CubeProgrammer`, and verifying that your program is still running through other means.

    loop {
        low_power::sleep_now();
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
