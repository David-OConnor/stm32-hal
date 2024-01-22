blinky tutorial using stm32-hal2
================================

This tutorial will guide you in setting up stm32-hal2 library and doing the mandatory 'blinky' demo. If you want to skip the learning part, simply clone the contents of this directory - it contains everything that you need to start blinking an LED. Otherwise, read along to build the demo from absolute scratch

What you will need:
- an stm32 board. This example uses a [blackpill development board from WeAct](https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0), however
  obviously you can pick any board you like. I chose this board in particular because it features an usb type C port
- a flasher / debugger kit (optional, but recommended). This example uses ST-Link V2. The reason why this is optional is that you can still compile your code, extract it's
  raw binary part and upload it manually to the board, but the process might require a bit of dexterity - one needs to press BOOT and RST buttons at the same time, then
  let go of the BOOT button and hopefully the board would boot into DFU / flashing mode. You would have to do this each time you'd want to flash a new version of your code,
  therefore it's easy to imagine that using a flasher is much, much quicker

First of all, let's understand couple of tools that take part in the whole process:

- `target` - it's a set ot tools for creating the binary in the format understood by the board. This particular example will use `thumbv7em-none-eabihf`.
- `probe-rs` - takes care of talking to your flasher / debugger utility. If you don't plan using it, just do not install it as a dependency.
- `stm32-hal2` - takes care of setting up the board, exposing it's interfaces under friendly names and so on. Please keep in mind that because this library
  can talk to several boards with a unison API, you need to know what kind of board you have. It will be shown later
- `defmt` - is a logging utility that can be configured to cooperate with `probe-rs`. This means that you will be able to push debug messages trough the ST-Link and see them
  on the terminal. Your binary size will increase slightly.

Adding rust target
==================
Even if you have the code written, it needs be compiled by a proper compiler. This means that when you simply do `rust build` and push the binary to your board, it will most likely fail (unless you're compiling your code on the same CPU as your target board). First, take a look at the page that describes this tutorial's board: [WeAct blackpill](https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0). On the "microcontroller core" section you can see "Arm Cortex M4". Now let's visit this page: [rust embedded](https://docs.rust-embedded.org/book/intro/install.html) and scroll down to *Cortex-M4F and M7F with hardware floating point (ARMv7E-M architecture)*. There's a command there
which will allow you to install the appropriate tooling. Let's execute that:

```bash
rustup target add thumbv7em-none-eabihf
```
Setting up the code
===================

Let's start by generating a new cargo project:

```bash
cargo new blinky
```

Great. Now comes one of the important parts - `memory.x`. This is a file that will inform rust runtime about your board properties - namely flash and RAM size. Thanks to that, rust will know where to put the machine code. Again, when you look on the WeAct board site, on the `Internal memories` section, you will see:

- FLASH	512KiB
- SRAM	96KiB

great. Let's put that into `memory.x` file - you need to create it in the root directory of your project:

```bash
cd blinky
touch memory.x
```

and here's the contents:

```
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K 
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}
```

Dependencies
------------

There are couple of mandatory dependencies and couple of optional ones. Let's go trough them one by one:

- `cortex-m` - takes care of low level stuff of the Cortex-M family. this means registers, interrupts and so on
- `cortex-m-rt` - takes care of putting rust runtime on the board.

Let's incorporate that into `Cargo.toml`:

```
[dependencies]
cortex-m = "0.7.3"
cortex-m-rt = "0.7.0"
```

Now the very important part - stm32-hal2 dependency. As I already mentioned before, this library can talk to multiple boards with a similar API. This means that when you're configuring your project, you need to make sure
that it is using the proper microcontroller family. But how to check that? You can start by visiting [stm32-hal2 Cargo.toml](https://github.com/David-OConnor/stm32-hal/blob/main/Cargo.toml). Scroll down to `features` section. You will be able to see two distinct set of features - these that end with `rt` (e.g. `f3rt`, `f4rt` and so on) and these that do not (e.g. `f301`, `f405` and so on). In layman's terms, you need the runtime if you're doing a running demo and you don't need it if you're writing a driver. Therefore, because we're making a blinking demo here we do need it. But which one? again, [WeAct blackpill page](https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0) will tell us. Search for `Microcontroller` section. The *Part* has a value of `STM32F401CEU6`. skip `STM32` as it's the manufacturer prefix and what you're left with will be `F401CEU6`. This means that in `stm32-hal2` set of features we need to look for `F401` feature. And look, it is there:

```
f401 = ["stm32f4/stm32f401", "f4"]
```

great, keep that in memory. Now, the corresponding runtime would also start with F4. Is there any that would match our set of features? Yes, there is!:

```
f4rt = ["stm32f4/rt"]
```

Amazing! Let's put that to our own `Cargo.toml`:

```
hal = { package = "stm32-hal2", version = "^0.2.11", features = ["f401", "f4rt"]}
```

Great! Now, we've told cargo that we want to install stm32-hal2 with these particular features. Just so that we're on the same page here, here's a complete `Cargo.toml` to this point:

```
[package]
name = "blinky"
version = "0.1.0"
edition = "2018"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
cortex-m = "0.7.3"
cortex-m-rt = "0.7.0"
hal = { package = "stm32-hal2", version = "^0.2.11", features = ["f401", "f4rt"]}
```

Let's do `cargo build` now, just to make sure that everything is set up correctly. (Mind you, there isn't any meaingful code in `src` directory just yet!)

```
cargo build
    Updating crates.io index
  Downloaded volatile-register v0.2.1
  Downloaded 1 crate (7.5 KB) in 1.16s
   Compiling proc-macro2 v1.0.28
   ( .... )
   Compiling blinky v0.1.0 (/home/toudi/projects/stm32/blinky)
    Finished dev [unoptimized + debuginfo] target(s) in 20.19s
```

Nice! Let's add one last final touch and that would be to cargo config:

```bash
mkdir .cargo
touch .cargo/config
```

and here's the contents:

```
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"
```

the `Tlink.x` that you see mentioned in the config is responsible for including `memory.x` file that we created earlier and putting code to appropriate places.

Now, after these changes are made, let's try to build the project again:

```bash 
cargo build
   Compiling nb v1.0.0
   ( ... )
   Compiling blinky v0.1.0 (/home/toudi/projects/stm32/blinky)
error[E0463]: can't find crate for `std`
  |
  = note: the `thumbv7em-none-eabihf` target may not support the standard library
  = note: `std` is required by `blinky` because it does not declare `#![no_std]`

error: aborting due to previous error

For more information about this error, try `rustc --explain E0463`.
error: could not compile `blinky`

To learn more, run the command again with --verbose.

```

Oops! Remember how I mentioned that there is no meaningful code in src? That's the reason why rust is complaining now. Let's create an actual hello, world:

```rust
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime
use hal::{
    self,
    clocks::{Clocks, InputSrc},
    gpio::{Pin, PinMode, Port},
    pac,
};

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    // Port::C, 13 because the LED is described as PC13 on WeAct blackpill page
    let mut led = Pin::new(Port::C, 13, PinMode::Output);

    // Now, enjoy the lightshow!
    loop {
        led.set_low();
        delay.delay_ms(1_000);
        led.set_high();
        delay.delay_ms(1_000);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
```

First, try to compile the code:

```bash
cargo build
```

if this succeeds then you've got couple of options:

Manual uploading the binary via DFU mode
----------------------------------------
If you don't have the ST-Link you can still push the binary to your board by using the DFU mode. In order to do this, please do the following steps:

1) `cargo build --release`
2) `arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/blinky blinky.bin` (you will need to install gcc for arm, it's a separate step and it's slightly out of scope for this tutorial)
3) switch your board in DFU mode. This means - connect it via USB cable, then push BOOT0 and RST buttons at the same time (they are layed out opposite of each other), hold it like so for a second, then let go of the RST (keep pushing BOOT0), hold it like that for a second and lastly let go of the BOOT0 button.
4) make sure that you can see your board in DFU mode in syslog like so (`dmesg` command):

```
[32252.856084] usb 1-8.3: new full-speed USB device number 26 using xhci_hcd
[32252.933393] usb 1-8.3: New USB device found, idVendor=0483, idProduct=df11, bcdDevice=22.00
[32252.933405] usb 1-8.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[32252.933411] usb 1-8.3: Product: STM32  BOOTLOADER
[32252.933415] usb 1-8.3: Manufacturer: STMicroelectronics
[32252.933418] usb 1-8.3: SerialNumber: [ ... ]
```

Believe me, it's easier said than done!

5) flash your code with dfu-util (agagin, that's a separate step to install this tool):

```
sudo dfu-util -a0 -s 0x08000000 -D blinky.bin
```

6) push the RST button.

Using cargo run
---------------
This method assumes that you have ST-Link and that it's connected to your board. First of all, you need to make a slight adjustment to your `.cargo/config` file:

```
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "probe-rs run --chip STM32F401CEUx" # to list chips, run `probe-run --list-chips.`
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"
```
1) issue the following command: `cargo run --release`

Using cargo-flash
-----------------
Cargo flash works similarly to `cargo run` but it does not involve using ST-Link for debugging purposes, it simply flashes the binary and reboots the board.

cargo flash should pick up your chip from cargo config file, but if it does not do that, you can pass it via command-line argument like so:

```bash
cargo flash --release --chip STM32F401CEUx
    Finished release [optimized] target(s) in 0.01s
    Flashing /home/toudi/projects/stm32/blinky/target/thumbv7em-none-eabihf/release/blinky
     Erasing sectors ✔ [00:00:00] [###########################################] 16.00KiB/16.00KiB @ 43.05KiB/s (eta 0s )
 Programming pages   ✔ [00:00:00] [###########################################] 14.00KiB/14.00KiB @  9.44KiB/s (eta 0s )
    Finished in 0.641s
```

Final touches - sending diagnostic messages to the console.
-----------------------------------------------------------
Let's talk about `defmt` for a moment. It's a logging library that can talk to ST-Link connection so that you could see some messages. Let's start by adding `defmt` to our Config.toml:

```
[dependencies]
defmt = "0.2.3"
defmt-rtt = "0.2.0"

[features]
default = [
    "defmt-default",
]

defmt-default = []
```

you will also need to make changes to your .cargo/config file:

```
rustflags = [
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=-Tdefmt.x",
]
```

finally, modify your src/main.rs:

```rust
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime
use hal::{
    self,
    clocks::{Clocks},
    gpio::{Pin, PinMode, Port},
    pac,
};

use defmt_rtt as _;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    // this line is required if you want to take advantage of ST-Link
    hal::debug_workaround();

    defmt::println!("Hello, world!");

    let clock_cfg = Clocks::default();
    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
    let mut led = Pin::new(Port::C, 13, PinMode::Output);

    // Now, enjoy the lightshow!
    loop {
        defmt::debug!("Our demo is alive");
        led.set_low();
        delay.delay_ms(1000_u32);
        led.set_high();
        delay.delay_ms(1000_u32);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
```

Now if you run the code with `cargo run --release` you will see the following output:

```
cargo run --release
   Compiling blinky v0.1.0 (/home/toudi/projects/stm32/blinky)
    Finished release [optimized] target(s) in 0.14s
     Running `probe-rs run --chip STM32F401CEUx target/thumbv7em-none-eabihf/release/blinky`
(HOST) WARN  insufficient DWARF info; compile your program with `debug = 2` to enable location info
(HOST) INFO  flashing program (14.56 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
 INFO  Hello, world!
^C────────────────────────────────────────────────────────────────────────────────

```
You will notice that the `debug` message was not shown. That is because `defmt-default` only shows `info` messages. If you want to see `debug` as well, you need to change `defmt-default` into `defmt-debug` in your `Cargo.toml`.

Code optimizations, i.e. making the binary smaller
--------------------------------------------------

You can also apply the following changes to Cargo.toml to make the code smaller:

```
[profile.release]
lto = 'fat'
opt-level = 3
```

This concludes the tutorial. You can see other examples for a much more elaborated Cargo.toml and main.rs, but this one should get you up an running.

Known pitfals
-------------
If your stm32-hal2 version is lower than 0.2.12, replace this line:

```
let clock_cfg = Clocks::default();
```

with this one instead:

```
let clock_cfg = Clocks {
    plln: 84,
    ..Default::default()
};
```

otherwise the code will crash when you'll try to run it.