//! Support for operational amplifiers.
//! WIP / non-functional

use core::ops::Deref;

use crate::pac::opamp;

/// Represents an operational amplifier peripheral.
pub struct Opamp<R> {
    pub regs: R,
}

impl<R> Opamp<R>
    where
        R: Deref<Target = opamp::RegisterBlock>,
{
    pub fn new() -> Self {

    }
}