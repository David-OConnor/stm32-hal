//! This module supports the Filter Match ACcelerator (FMAC) peripheral, which
//! allows for hardware processing of digital filters such as FIR and IIR.

// todo: Is this fixed point only?

use crate::pac::{FMAC};

pub struct Fmac {
    pub regs: FMAC,
}

impl Fmac {
    /// Create a struct used to perform operations on Flash.
    pub fn new(regs: FMAC) -> Self {
        // todo: Implement and configure dual bank mode.
        Self { regs }
    }

    /// Set up a Finite Impulse Response (FIR) filter.
    fn run_fir(&mut self, coeffs: &[f32], data: &mut [f32]) {

    }

    /// Set up an Infinite Impulse Response (IIR) filter.
    fn run_iir(&mut self, coeffs: &[f32], data: &mut [f32]) {

    }
}