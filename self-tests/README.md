# Hardware self-tests for the HAL
A collection of self-tests that run on actual hardware to ensure the HAL
behaves as expected.

## Pre-requisities
[probe-rs](https://probe.rs/) for flashing and running the tests. The probe-rs version needs
to be version 0.24 or higher.

Installation:
```sh
$ cargo install probe-rs-tools
```

### Note for musl-based systems
On musl-based system remember to disable static compilation when compiling probe-rs by instead
invoking the installation process with a `-crt-static`.

```sh
$ RUSTFLAGS="-C target-feature=-crt-static" cargo install probe-rs-tools
```

## Running the tests
The test projects **HAS** to be launched from their respective directory, otherwise Cargo
gets over-eager and uses the wrong settings.

Running the test-suite for a device (after hooking up the necessary wires) is as simple as:

```sh
$ cargo test
```

This will invoke `probe-rs` which will flash the MCU with each of the test files as a
seperate firmware and then run the tests inside.

## Writing new tests
See embedded-test's [README](https://github.com/probe-rs/embedded-test/) for how to
structure a test project.

For a great run-through of embedded testing read [japaric's](https://github.com/japaric/), of
Ferrous Systems, blog series on it:
[https://ferrous-systems.com/blog/tags/embedded-rust-testing/](https://ferrous-systems.com/blog/tags/embedded-rust-testing/)
