# Conductivity module firmware

This is the entry point for firmware for the
[AnyLeaf conductivity](https://www.anyleaf.org/ec-module) module. This device connects
to an electrical conductivity probe, and passes readings to a host over a UART serial
connection. (Example host: Raspberry Pi) It uses a simplified version of Analog Device's
[CN0411 reference design](https://www.analog.com/en/design-center/reference-designs/circuits-from-the-lab/cn0411.html),
replacing some of the analog circuitry with software, and adapted for use with an STM32.

Code in `ec.rs` is also used as a library for the same conductivity circuitry in the AnyLeaf
Water Monitor.


## MCU selection
This is set up for use with an STM32L443, but can be adapted to other STM32s, mainly by changing
the MCU-specific lines in `Cargo.toml`, `memory.x`, `.cargo/config`. As standalone firmware,
it may be more suitable for use on a simpler, low-cost device like Stm32G0 or Stm32G4.
The main requirement limiting device selection is presense of a DAC. When switching MCUs,
you will likely also need to change the relevant GPIO pin config.


## Flashing
Make sure you have `flip-link` and `probe-run` installed, using the 
[instructions here](https://github.com/knurling-rs/app-template).

To flash, run `cargo b --release --features="standalone"`, or create a binary with `cargo-binutils`
using `cargo objcopy --release -- -O binary target/firmware.bin`, then flash with a tool of your
choice, eg dfu-util, or Stm32CubeProgrammer.