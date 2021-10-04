//! This example demonstrates reading input using a PDM microphone using the DFSDM
//! peripheral. It uses a circular DMA buffer to continuously read and process from the microphone,
//! and outputs to the DAC using a second circular buffer.
//!
//! This example uses [RTIC](https://rtic.rs/0.5/book/en/) for its structure.
