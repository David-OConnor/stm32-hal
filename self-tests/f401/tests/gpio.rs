// Test file for all parts of the GPIO API.

// This test requires wires to be connected between the following pins:
// * pc12 <-> vdd
// * pd2 <-> gnd
//
// * pc8 <-> pc9

#![deny(warnings)]
#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests(setup = rtt_target::rtt_init_defmt!())]
mod tests {
    use hal::{
        clocks::Clocks,
        delay_ms,
        gpio::{OutputType, Pin, PinMode, Port, Pull},
    };

    const DELAY: u32 = 100;

    struct State {
        input: Pin,
        output: Pin,
        clocks: Clocks,
    }

    #[init]
    fn init() -> State {
        let clocks = Clocks::default();
        clocks.setup().unwrap();
        State {
            input: Pin::new(Port::C, 9, PinMode::Input),
            output: Pin::new(Port::C, 8, PinMode::Output),
            clocks,
        }
    }

    // Sanity check
    #[test]
    fn vdd_is_high() {
        let vdd = Pin::new(Port::C, 12, PinMode::Input);
        assert!(vdd.is_high());
    }

    // Sanity check
    #[test]
    fn ground_is_low() {
        let gnd = Pin::new(Port::D, 2, PinMode::Input);
        assert!(gnd.is_low());
    }

    #[test]
    fn push_pull_low(mut state: State) {
        state.output.output_type(OutputType::PushPull);
        state.output.set_low();
        defmt::assert!(state.output.is_low());
        defmt::assert!(state.input.is_low());
    }

    #[test]
    fn push_pull_high(mut state: State) {
        state.output.output_type(OutputType::PushPull);
        state.output.set_high();
        defmt::assert!(state.output.is_high());
        defmt::assert!(state.input.is_high());
    }

    #[test]
    fn pulldown_drive_input_low(mut state: State) {
        state.output.output_type(OutputType::PushPull);
        state.output.pull(Pull::Dn);
        delay_ms(DELAY, state.clocks.apb1());
        defmt::assert!(state.output.is_low());
        defmt::assert!(state.input.is_low());
    }

    #[test]
    fn pullup_drive_input_high(mut state: State) {
        state.output.output_type(OutputType::PushPull);
        state.output.pull(Pull::Up);
        state.output.set_high();
        delay_ms(DELAY, state.clocks.apb1());
        // defmt::assert!(state.output.is_high());
        defmt::assert!(state.input.is_high());
    }

    #[test]
    fn open_drain_low(mut state: State) {
        state.output.output_type(OutputType::OpenDrain);
        state.output.pull(Pull::Dn);
        delay_ms(DELAY, state.clocks.apb1());
        defmt::assert!(state.output.is_low());
        defmt::assert!(state.input.is_low());
    }

    // #[test]
    // fn open_drain_high(mut state: State) {
    //     state.output.output_type(OutputType::OpenDrain);
    //     state.output.pull(Pull::Up);
    //     delay_ms(DELAY, state.clocks.apb1());
    //     defmt::assert!(state.output.is_high());
    //     defmt::assert!(state.input.is_high());
    // }
}
