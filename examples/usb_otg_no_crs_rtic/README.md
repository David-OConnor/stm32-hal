# USB OTG No CRS RTIC
## Why this example exists
Not all stm32 devices have a Clock Recovery System (CRS) such as the stm32l476.
For these device families, we can't use the CRS or clocks that depend on the
CRS such as the HSI. See
https://github.com/David-OConnor/stm32-hal/issues/64 for further details.

The following example was run and tested on the STM32L476ZGTx varient.

## Running the example for the STM32L476ZGTx varient
1. Install the nix package manager by following the instructions [here](https://nixos.org/download.html).
2. Create a udev rule for the stm32 stlink following the instructions [here](https://docs.rust-embedded.org/discovery/f3discovery/03-setup/linux.html#create-etcudevrulesd99-openocdrules)
3. Reboot your machine so all OS modifactions take effect.
4. Enter the development environment subshell by navigating the the directory
   containing this readme and enter `nix-shell --pure` in the command line.
   > Note: You may find the development experience within the nix-shell to be
   > enjoyable if you drop the --pure option, however, running nix-shell without
   > the --pure option is not garunteed to work. Development is also greatly
   > improved by using direnv as used in [this](https://nix.dev/tutorials/declarative-and-reproducible-developer-environments?highlight=direnv#direnv-automatically-activating-the-environment-on-directory-change) post.
5. Compile the code by running `cargo build --bin <bin_file>` a.k.a `cargo b
   --bin <bin_file>`
6. Compile, Flash, and Run the code on the target device by running `cargo run
   --bin <bin_file>`
   a.k.a `cargo r --bin <bin_file>`

## Files needing modification to run on other stm32 varients
1. `.cargo/config.toml`. Modify target and chip.
2. `src/main.rs`. Modify the usb_dm and usb_dp pins.
