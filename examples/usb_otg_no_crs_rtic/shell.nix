{ pkgs ? import (fetchTarball https://github.com/NixOS/nixpkgs/archive/abe6ea8ac11de69b7708ca9c70e8cd007600cd73.tar.gz) {} }:

let
  rust_overlay = import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz");
  pkgs = import (builtins.fetchTarball https://github.com/NixOS/nixpkgs/archive/abe6ea8ac11de69b7708ca9c70e8cd007600cd73.tar.gz) { overlays = [ rust_overlay ]; };
in
pkgs.mkShell {
  # buildInputs is for dependencies you'd need "at run time",
  # were you to to use nix-build not nix-shell and build whatever you were working on
  buildInputs = with pkgs; [
      (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
      flip-link
      rust-analyzer
      probe-run
      stlink
      stlink-gui
      usbutils
      cargo-binutils
  ];
}

