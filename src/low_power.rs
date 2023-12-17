//! This module contains code used to place the MCU in low power modes.
//! Reference section 5.3.3: `Low power modes` of the L4 Reference Manual.

use cfg_if::cfg_if;
use cortex_m::{asm::wfi, Peripherals};

#[cfg(any(feature = "l4", feature = "l5"))]
use crate::clocks::{Clocks, MsiRange};
#[cfg(any(feature = "l4", feature = "l5"))]
use crate::pac;
#[cfg(not(feature = "h7"))]
use crate::pac::PWR;

// See L4 Reference Manual section 5.3.6. The values correspond to the PWR_CR1 LPMS bits.
// todo PWR_CR1, LPMS field.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum StopMode {
    Zero = 0,
    One = 1,
    #[cfg(not(feature = "g4"))]
    Two = 2,
}

/// L4 RM, table 24
/// This assumes you're using MSI as the clock source, and changes speed by lowering the MSI speed.
/// You must select an MSI speed of 2Mhz or lower. Note that you may need to adjust peripheral
/// implementations that rely on system clock or APB speed.
#[cfg(any(feature = "l4", feature = "l5"))]
pub fn low_power_run(clocks: &mut Clocks, speed: MsiRange) {
    let rcc = unsafe { &(*pac::RCC::ptr()) };
    let pwr = unsafe { &(*PWR::ptr()) };

    // Decrease the system clock frequency below 2 MHz
    if speed as u8 > MsiRange::R2M as u8 {
        panic!("Selected Msi speed must be 2Mhz or lower to enter use low power run.")
    }
    clocks.change_msi_speed(speed);
    // LPR = 1
    pwr.cr1.modify(|_, w| w.lpr().set_bit())
}

/// L4 RM, table 24
/// Return to normal run mode from low-power run. Requires you to increase the clock speed
/// manually after running this.
#[cfg(any(feature = "l4", feature = "l5"))]
pub fn return_from_low_power_run() {
    let pwr = unsafe { &(*PWR::ptr()) };

    // LPR = 0
    pwr.cr1.modify(|_, w| w.lpr().clear_bit());

    // Wait until REGLPF = 0
    while pwr.sr2.read().reglpf().bit_is_set() {}

    // Increase the system clock frequency
}

/// Place the system in sleep now mode. To enter `low-power sleep now`, enter low power mode
/// (eg `low_power_mode()`) before running this. RM, table 25 and 26
#[cfg(not(feature = "h7"))]
pub fn sleep_now() {
    sleep();
}

/// F303 RM, table 19.
pub fn sleep_on_exit() {
    let mut scb = unsafe { Peripherals::steal().SCB };

    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:

    // SLEEPDEEP = 0 and SLEEPONEXIT = 1
    scb.clear_sleepdeep();
    // Sleep-on-exit: if the SLEEPONEXIT bit is set, the MCU enters Sleep mode as soon
    // as it exits the lowest priority ISR.
    scb.set_sleeponexit();

    wfi();
}

cfg_if! {
    if #[cfg(any(feature = "f3", feature = "f4"))] {
        /// Enter `Stop` mode: the middle of the 3 low-power states avail on the
        /// STM32f3.
        /// To exit:  Any EXTI Line configured in Interrupt mode (the corresponding EXTI
        /// Interrupt vector must be enabled in the NVIC). Refer to Table 82.
        /// F303 RM, table 20. F4 RM, Table 27. H742 RM, Table 38. (CSrtop on H7).
        /// Run `Clocks::reselect_input()` after to re-enable PLL etc after exiting this mode.
        pub fn stop() {
            let mut scb = unsafe { Peripherals::steal().SCB };
            let pwr = unsafe { &(*PWR::ptr()) };

            // todo: On some F4 variants, you may need to `select voltage regulator
            // todo mode by configuring LPDS, MRUDS, LPUDS and UDEN bits in PWR_CR.`
            //WFI (Wait for Interrupt) or WFE (Wait for Event) while:

            // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
            scb.set_sleepdeep();

            // Clear PDDS bit in Power Control register (PWR_CR)
            // This bit is set and cleared by software. It works together with the LPDS bit.
            // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
            // depends on the LPDS bit.
            // 1: Enter Stop mode when the CPU enters Deepsleep.
            pwr.cr.modify(|_, w| {
                w.pdds().clear_bit();
                // Select the voltage regulator mode by configuring LPDS bit in PWR_CR
                // This bit is set and cleared by software. It works together with the PDDS bit.
                // 0: Voltage regulator on during Stop mode
                // 1: Voltage regulator in low-power mode during Stop mode
                // pwr.cr.modify(|_, w| w.pdds().clear_bit());
                 w.lpds().set_bit()
            });


            wfi();
        }

        /// Enter `Standby` mode.
        /// To exit: WKUP pin rising edge, RTC alarm event’s rising edge, external Reset in
        /// NRST pin, IWDG Reset.
        /// F303 RM, table 21.
        /// Run `Clocks::reselect_input()` after to re-enable PLL etc after exiting this mode.
        pub fn standby() {
            let mut scb = unsafe { Peripherals::steal().SCB };
            let pwr = unsafe { &(*PWR::ptr()) };

            // WFI (Wait for Interrupt) or WFE (Wait for Event) while:

            // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
            scb.set_sleepdeep();

            // Set PDDS bit in Power Control register (PWR_CR)
            // This bit is set and cleared by software. It works together with the LPDS bit.
            // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
            // depends on the LPDS bit.
            // 1: Enter Standby mode when the CPU enters Deepsleep.
            pwr.cr.modify(|_, w| {
                w.pdds().set_bit();
                // Clear WUF bit in Power Control/Status register (PWR_CSR) (Must do this by setting CWUF bit in
                // PWR_CR.)
                w.cwuf().set_bit()
            });

            wfi();
        }
    } else if #[cfg(any(feature = "l4", feature = "l5", feature = "g0", feature = "g4"))] {
        /// Enter Stop 0, Stop 1, or Stop 2 modes. L4 Reference manual, section 5.3.6. Tables 27, 28, and 29.
        /// G0 RMs, tables 30, 31, 32.
        /// G4 Table 45, 47, 47.
        /// Run `Clocks::reselect_input()` after to re-enable PLL etc after exiting this mode.
        pub fn stop(mode: StopMode) {
            let mut scb = unsafe { Peripherals::steal().SCB };
            let pwr = unsafe { &(*PWR::ptr()) };

            // WFI (Wait for Interrupt) or WFE (Wait for Event) while:
            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            scb.set_sleepdeep();
            // – No interrupt (for WFI) or event (for WFE) is pending
            // – LPMS = (according to mode) in PWR_CR1
            pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(mode as u8) });

            // Or, unimplemented:
            // On Return from ISR while:
            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            // – SLEEPONEXIT = 1
            // – No interrupt is pending
            // – LPMS = “000” in PWR_CR1

            wfi();
        }


        /// Enter `Standby` mode. See L44 RM table 28. G4 table 47.
        /// Run `Clocks::reselect_input()` after to re-enable PLL etc after exiting this mode.
        pub fn standby() {
            let mut scb = unsafe { Peripherals::steal().SCB };
            let pwr = unsafe { &(*PWR::ptr()) };

            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            scb.set_sleepdeep();

            // – No interrupt (for WFI) or event (for WFE) is pending

            // – LPMS = “011” in PWR_CR1
            pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(0b011) });

            // – WUFx bits are cleared in power status register 1 (PWR_SR1)
            // (Clear by setting cwfuf bits in `pwr_scr`.)
            cfg_if! {
                if #[cfg(feature = "l4")] {
                    pwr.scr.write(|w| {
                        w.wuf1().set_bit();
                        w.wuf2().set_bit();
                        w.wuf3().set_bit();
                        w.wuf4().set_bit();
                        w.wuf5().set_bit()
                    });
                } else if #[cfg(feature = "g0")] {
                    pwr.scr.write(|w| {
                        w.cwuf1().set_bit();
                        w.cwuf2().set_bit();
                        // w.cwuf3().set_bit(); // todo: PAC ommission?
                        w.cwuf4().set_bit();
                        w.cwuf5().set_bit();
                        w.cwuf6().set_bit()
                    });
                } else {
                    pwr.scr.write(|w| {
                        w.cwuf1().set_bit();
                        w.cwuf2().set_bit();
                        w.cwuf3().set_bit();
                        w.cwuf4().set_bit();
                        w.cwuf5().set_bit()
                    });
                }
            }

            // todo: `The RTC flag corresponding to the chosen wakeup source (RTC Alarm
            // todo: A, RTC Alarm B, RTC wakeup, tamper or timestamp flags) is cleared`.

            // Or, unimplemented:
            // On return from ISR while:
            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            // – SLEEPONEXIT = 1
            // – No interrupt is pending
            // – LPMS = “011” in PWR_CR1 and
            // – WUFx bits are cleared in power status register 1 (PWR_SR1)
            // – The RTC flag corresponding to the chosen wakeup source (RTC Alarm
            // A, RTC Alarm B, RTC wakeup, tamper or timestamp flags) is cleared

            wfi();
        }

        /// Enter `Shutdown mode` mode: the lowest-power of the 3 low-power states avail. See
        /// L4 Table 31. G4 table 48.
        pub fn shutdown() {
            let mut scb = unsafe { Peripherals::steal().SCB };
            let pwr = unsafe { &(*PWR::ptr()) };

            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            scb.set_sleepdeep();
            // – No interrupt (for WFI) or event (for WFE) is pending
            // – LPMS = “011” in PWR_CR1
            pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(0b100) });
            // – WUFx bits are cleared in power status register 1 (PWR_SR1)
            // (Clear by setting cwfuf bits in `pwr_scr`.)

            // pwr.scr.write(|_, w| {
            //     w.cwuf1().set_bit();
            //     w.cwuf2().set_bit();
            //     w.cwuf3().set_bit();
            //     w.cwuf4().set_bit();
            //     w.cwuf5().set_bit();
            // })

            // Or, unimplemented:
            // On return from ISR while:
            // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
            // – SLEEPONEXT = 1
            // – No interrupt is pending
            // – LPMS = “1XX” in PWR_CR1 and
            // – WUFx bits are cleared in power status register 1 (PWR_SR1)
            // – The RTC flag corresponding to the chosen wakeup source (RTC
            // Alarm A, RTC Alarm B, RTC wakeup, tamper or timestamp flags) is
            // cleared
            wfi();
        }
    } else { // H7
        /// The CSleep mode applies only to the CPU subsystem. In CSleep mode, the CPU clock is
        /// stopped. The CPU subsystem peripheral clocks operate according to the values of
        /// PERxLPEN bits in RCC_C1_xxxxENR or RCC_DnxxxxENR. See H743 RM, Table 37.
        pub fn csleep() {
            sleep();
        }

        ///  Stops clocks on the CPU subsystem. H742 RM, Table 38.
        pub fn cstop() {
            let mut scb = unsafe { Peripherals::steal().SCB };

            // WFI (Wait for Interrupt) or WFE (Wait for Event) while:

            // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
            scb.set_sleepdeep();

            // – CPU NVIC interrupts and events cleared.
            // – All CPU EXTI Wakeup sources are cleared.

            wfi();
        }

        // /// Stops clocks on the D1 and D2 domain. H742 RM, Table 40.
        // pub fn dstop(scb: &mut SCB, pwr: &mut PWR) {
        //
        //     // -The domain CPU subsystem enters CStop.
        //     // (Don't call self.cstop, since that handles WFI and reselecting clocks as well).
        //     scb.set_sleepdeep();
        //
        //
        //     // todo?
        //     // – The CPU subsystem has an allocated peripheral in the D2 domain and
        //     // enters CStop.
        //     // – The CPU subsystem deallocated its last peripheral in the D2 domain.
        //     // – The PDDS_Dn bit for the domain selects Stop mode.
        //
        //     wfi();
        // }

        // /// Enter `Standby` mode: the lowest-power of the 3 low-power states avail on the
        // /// STM32f3.
        // /// To exit: WKUP pin rising edge, RTC alarm event’s rising edge, external Reset in
        // /// NRST pin, IWDG Reset.
        // /// RM, table 21.
        // /// Run `Clocks::reselect_input()` after to re-enable PLL etc after exiting this mode.
        // pub fn standby() {
        //     // WFI (Wait for Interrupt) or WFE (Wait for Event) while:
        //
        //     // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
        //     scb.set_sleepdeep();
        //
        //     // Set PDDS bit in Power Control register (PWR_CR)
        //     // This bit is set and cleared by software. It works together with the LPDS bit.
        //     // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
        //     // depends on the LPDS bit.
        //     // 1: Enter Standby mode when the CPU enters Deepsleep.
        //     pwr.cr.modify(|_, w| {
        //         w.pdds().set_bit();
        //         // Clear WUF bit in Power Control/Status register (PWR_CSR) (Must do this by setting CWUF bit in
        //         // PWR_CR.)
        //         w.cwuf().set_bit()
        //     });

        //     wfi();
        // }
    }
}

/// This function is used by both `sleep_now` (non-H7), and `csleep` (H7), so that the names
/// can correctly reflect functionality.
fn sleep() {
    // todo: This function is identical to `sleep_now`, but we'd like to keep the separate
    // todo name for H7.

    let mut scb = unsafe { Peripherals::steal().SCB };

    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:
    // – SLEEPDEEP = 0
    // – No interrupt (for WFI) or event (for WFE) is pending
    scb.clear_sleepdeep();

    // Or, unimplemented:
    // On return from ISR while:
    // // SLEEPDEEP = 0 and SLEEPONEXIT = 1
    // scb.clear_sleepdeep();
    // scb.set_sleeponexit();

    // Sleep-now: if the SLEEPONEXIT bit is cleared, the MCU enters Sleep mode as soon
    // as WFI or WFE instruction is executed.
    scb.clear_sleeponexit();

    wfi();
}
