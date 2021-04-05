//! Support for operational amplifiers.
//! WIP / non-functional

// Based on `stm32g4xx-hal`

use crate::pac::{self, RCC, OPAMP};
use paste::paste;

macro_rules! opamps {
    {
        $(
            $opamp:ident
            :
            {
                inverting
                :
                {
                    $(
                        $inverting:ty
                        :
                        $inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                non_inverting
                :
                {
                    $(
                        $non_inverting:ty
                        :
                        $non_inverting_mask:tt
                    ),*
                    $(,)?
                }
                ,
                output
                :
                $output:ty
                ,
            }
        ),*
        $(,)?
    } => {
        paste!{
            /// Re-exports usefull traits.
            pub mod prelude {
                $(pub use super::$opamp::IntoFollower as _;)*
                $(pub use super::$opamp::IntoOpenLoop as _;)*
            }

            $(
                /// States for opampX.
                pub mod $opamp {

                    /// State type for disabled opamp.
                    pub struct Disabled;

                    /// State type for opamp running in voltage follower mode.
                    pub struct Follower<Input> {
                        input: Input,
                        output: Option<$output>,
                    }

                    /// State type for opamp running in open-loop mode.
                    pub struct OpenLoop<NonInverting, Inverting> {
                        non_inverting: NonInverting,
                        inverting: Inverting,
                        output: Option<$output>,
                    }

                    /// Trait for opamps that can be run in follower mode.
                    pub trait IntoFollower <IntoInput, IntoOutput, Input>
                        where
                            IntoOutput: Into<$output>,
                    {
                        /// Coonfigures the opamp as voltage follower.
                        fn follower(self, input: IntoInput, output: Option<IntoOutput>) -> Follower<Input>;
                    }

                    /// Trait for opamps that can be run in open-loop mode.
                    pub trait IntoOpenLoop <IntoNonInverting, IntoInverting, IntoOutput, NonInverting, Inverting>
                        where
                            IntoOutput: Into<$output>,
                    {
                        /// Configures the opamp for open-loop operation.
                        fn open_loop(self, non_inverting: IntoNonInverting, inverting: IntoInverting, output: Option<IntoOutput>)
                            -> OpenLoop<NonInverting, Inverting>;
                    }

                    impl<Input> Follower<Input> {

                        /// Disables the opamp and returns the resources it held.
                        pub fn disable(self) -> (Disabled, Input, Option<$output>) {
                            unsafe { (OPAMP::ptr()).[<$opamp _csr>].reset() }
                            (Disabled, self.input, self.output)
                        }

                        /// Enables the external output pin.
                        pub fn enable_output(&mut self, output:$output) {
                            self.output = Some(output);
                            unsafe {
                                (*OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().output_pin());
                            }
                        }

                        /// Disables the external output.
                        /// This will connect the opamp output to the internal ADC.
                        /// If the output was enabled, the output pin is returned.
                        pub fn disable_output(&mut self) -> Option<$output> {
                            unsafe {
                                (*OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().adcchannel());
                            }
                            self.output.take()
                        }
                    }

                    impl<NonInverting, Inverting> OpenLoop<NonInverting, Inverting> {

                        /// Disables the opamp and returns the resources it held.
                        pub fn disable(self) -> (Disabled, NonInverting, Inverting, Option<$output>) {
                            unsafe { (*OPAMP::ptr()).[<$opamp _csr>].reset() }
                            (Disabled, self.non_inverting, self.inverting, self.output)
                        }

                        /// Enables the external output pin.
                        pub fn enable_output(&mut self, output:$output) {
                            self.output = Some(output);
                            unsafe {
                                (*OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().output_pin());
                            }
                        }

                        /// Disables the external output.
                        /// This will connect the opamp output to the internal ADC.
                        /// If the output was enabled, the output pin is returned.
                        pub fn disable_output(&mut self) -> Option<$output> {
                            unsafe {
                                (*OPAMP::ptr()).[<$opamp _csr>].write(|w|
                                    w.opaintoen().adcchannel());
                            }
                            self.output.take()
                        }
                    }

                    opamps!{ @follower $opamp, $output, $($non_inverting_mask, $non_inverting),* }
                    opamps!{ @open_loop_tt $opamp, $output, $($non_inverting_mask, $non_inverting),* : ($($inverting_mask, $inverting),*) }

                }
            )*

            /// Extension trait for the OPAMP block.
            pub trait OpampEx {
                /// Splits the OPAMP block into independant access objects.
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $($opamp::Disabled,)*
                );
            }

            impl OpampEx for OPAMP {
                fn split(
                    self,
                    rcc: &mut crate::rcc::Rcc,
                ) -> (
                    $($opamp::Disabled,)*
                ) {
                    rcc.rb.apb2enr.write(|w| w.syscfgen().set_bit());

                    (
                        $($opamp::Disabled,)*
                    )
                }
            }
        }
    };

    {
        @follower
        $opamp:ident
        ,
        $output:ty
        ,
        $(
            $input_mask:tt
            ,
            $input:ty
        ),*
    } => {
        paste!{
            $(impl <IntoInput, IntoOutput> IntoFollower <IntoInput, IntoOutput, $input> for Disabled
                where
                    IntoInput: Into<$input>,
                    IntoOutput: Into<$output>,
            {
                fn follower(
                    self,
                    input: IntoInput,
                    output: Option<IntoOutput>,
                ) -> Follower<$input> {
                    let input = input.into();
                    let output = match output {
                        Some(output) => Some(output.into()),
                        None => None,
                    };
                    unsafe {
                        use pac::opamp::[<$opamp _csr>]::OPAINTOEN_A;
                        (*OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w
                                    .vp_sel()
                                    .$input_mask()
                                    .vm_sel()
                                    .output()
                                    .opaintoen()
                                    .variant(match output {
                                        Some(_) => OPAINTOEN_A::OUTPUTPIN,
                                        None => OPAINTOEN_A::ADCCHANNEL,
                                    })
                                    .opaen()
                                    .enabled()
                            );
                    }
                    Follower {input, output}
                }
            })*
        }
    };

    {
        @open_loop_tt
        $opamp:ident
        ,
        $output:ty
        ,
        $($non_inverting_mask:tt, $non_inverting:ty),*
        :
        $invertings:tt
    } => {
        $(
            opamps!{ @open_loop $opamp, $output, $non_inverting_mask, $non_inverting, $invertings }
        )*
    };

    {
        @open_loop
        $opamp:ident
        ,
        $output:ty
        ,
        $non_inverting_mask:tt
        ,
        $non_inverting:ty
        ,
        ($($inverting_mask:tt, $inverting:ty),*)
    } => {
        paste!{
            $(impl <IntoNonInverting, IntoInverting, IntoOutput> IntoOpenLoop
                <IntoNonInverting, IntoInverting, IntoOutput, $non_inverting, $inverting> for Disabled
                where
                    IntoNonInverting: Into<$non_inverting>,
                    IntoInverting: Into<$inverting>,
                    IntoOutput: Into<$output>,
            {
                fn open_loop(
                    self,
                    non_inverting: IntoNonInverting,
                    inverting: IntoInverting,
                    output: Option<IntoOutput>,
                ) -> OpenLoop<$non_inverting, $inverting> {
                    let non_inverting = non_inverting.into();
                    let inverting = inverting.into();
                    let output = match output {
                        Some(output) => Some(output.into()),
                        None => None,
                    };
                    unsafe {
                        use pac::opamp::[<$opamp _csr>]::OPAINTOEN_A;
                        (*OPAMP::ptr())
                            .[<$opamp _csr>]
                            .write(|csr_w|
                                csr_w.vp_sel()
                                    .$non_inverting_mask()
                                    .vm_sel()
                                    .$inverting_mask()
                                    .opaintoen()
                                    .variant(match output {
                                        Some(_) => OPAINTOEN_A::OUTPUTPIN,
                                        None => OPAINTOEN_A::ADCCHANNEL,
                                    })
                                    .opaen()
                                    .enabled()
                            );
                    }
                    OpenLoop {non_inverting, inverting, output}
                }
            })*
        }
    };
}

#[cfg(any(
    feature = "g4",
))]
opamps! {
    opamp1: {
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    opamp2: {
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    opamp3: {
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
}

#[cfg(any(
    feature = "g4",
))]
opamps! {
    opamp1: {
        inverting: {
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA2<crate::gpio::Analog>,
    },
    opamp2: {
        inverting: {
            crate::gpio::gpioa::PA5<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioc::PC5<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpioa::PA7<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp2,
            crate::gpio::gpiod::PD14<crate::gpio::Analog>: vinp3,
        },
        output: crate::gpio::gpioa::PA6<crate::gpio::Analog>,
    },
    opamp3: {
        inverting: {
            crate::gpio::gpiob::PB2<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB0<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB1<crate::gpio::Analog>,
    },
    opamp4: {
        inverting: {
            crate::gpio::gpiob::PB10<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiod::PD8<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD11<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB11<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB12<crate::gpio::Analog>,
    },
    opamp5: {
        inverting: {
            crate::gpio::gpiob::PB15<crate::gpio::Analog>: vinm0,
            crate::gpio::gpioa::PA3<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB14<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD12<crate::gpio::Analog>: vinp1,
            crate::gpio::gpioc::PC3<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpioa::PA8<crate::gpio::Analog>,
    },
    opamp6: {
        inverting: {
            crate::gpio::gpioa::PA1<crate::gpio::Analog>: vinm0,
            crate::gpio::gpiob::PB1<crate::gpio::Analog>: vinm1,
        },
        non_inverting: {
            crate::gpio::gpiob::PB12<crate::gpio::Analog>: vinp0,
            crate::gpio::gpiod::PD9<crate::gpio::Analog>: vinp1,
            crate::gpio::gpiob::PB13<crate::gpio::Analog>: vinp2,
        },
        output: crate::gpio::gpiob::PB11<crate::gpio::Analog>,
    },
}
