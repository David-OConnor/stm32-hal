//! Indepdent watchdog

#[cfg(not(any(feature = "h735", feature = "h747cm4", feature = "h747cm7")))]
use crate::pac::IWDG;

#[cfg(any(feature = "h735", feature = "h747cm4", feature = "h747cm7"))]
use crate::pac::IWDG1 as IWDG;

/// Set up (enable), without window option. `timeout` is in seconds.
/// G4 RM, section 42.3.2
pub fn setup(timeout: f32) {
    unsafe {
        let regs = &(*IWDG::ptr());
        // When the window option it is not used, the IWDG can be configured as follows:
        // 1. Enable the IWDG by writing 0x0000 CCCC in the IWDG key register (IWDG_KR).
        regs.kr.write(|w| w.bits(0x0000_cccc));

        // 2.Enable register access by writing 0x0000 5555 in the IWDG key register (IWDG_KR).
        regs.kr.write(|w| w.bits(0x0000_5555));

        // 3. Write the prescaler by programming the IWDG prescaler register (IWDG_PR) from 0 to
        // 7.

        // 32kHz clock.
        // todo: Hardcoded prescaler of 32.
        regs.pr.write(|w| w.bits(0b011));

        // 4. Write the IWDG reload register (IWDG_RLR).
        // A 12-bit value. Assumes a prescaler of 0.
        let ticks_per_s = 1_000.;
        let reload_val = (ticks_per_s * timeout) as u32;

        regs.rlr.write(|w| w.bits(reload_val));

        // 5. Wait for the registers to be updated (IWDG_SR = 0x0000 0000).
        while regs.sr.read().bits() != 0 {}

        // 6. Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA).
        // (`refresh()` function
    }
}

/// Run this at an interval shorter than the countdown time to prevent a reset.
pub fn refresh() {
    unsafe {
        let regs = &(*IWDG::ptr());
        regs.rlr.write(|w| w.bits(0x0000_aaaa));
    }
}
