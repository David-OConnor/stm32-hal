macro_rules! impl_params {
    ($method:ident, $param_type:ident, $opcode:path) => {
        fn $method(&mut self, params: &$param_type) -> nb::Result<(), Self::Error> {
            let mut bytes = [0; $param_type::LENGTH];
            params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes)
        }
    };
}

macro_rules! impl_value_params {
    ($method:ident, $param_type:ident, $opcode:path) => {
        fn $method(&mut self, params: $param_type) -> nb::Result<(), Self::Error> {
            let mut bytes = [0; $param_type::LENGTH];
            params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes)
        }
    };
}

macro_rules! impl_validate_params {
    ($method:ident, $param_type:ident, $opcode:path) => {
        fn $method(&mut self, params: &$param_type) -> nb::Result<(), Error<Self::Error>> {
            params.validate().map_err(nb::Error::Other)?;

            let mut bytes = [0; $param_type::LENGTH];
            params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes).map_err(rewrap_error)
        }
    };
}

macro_rules! impl_variable_length_params {
    ($method:ident, $param_type:ident, $opcode:path) => {
        fn $method(&mut self, params: &$param_type) -> nb::Result<(), Self::Error> {
            let mut bytes = [0; $param_type::MAX_LENGTH];
            let len = params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes[..len])
        }
    };
}

macro_rules! impl_validate_variable_length_params {
    ($method:ident, $param_type:ident, $opcode:path) => {
        fn $method(&mut self, params: &$param_type) -> nb::Result<(), Error<Self::Error>> {
            params.validate().map_err(nb::Error::Other)?;

            let mut bytes = [0; $param_type::MAX_LENGTH];
            let len = params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes[..len])
                .map_err(rewrap_error)
        }
    };
    ($method:ident<$($genlife:lifetime),*>, $param_type:ident<$($lifetime:lifetime),*>, $opcode:path) => {
        fn $method<$($genlife),*>(
            &mut self,
            params: &$param_type<$($lifetime),*>
        ) -> nb::Result<(), Error<Self::Error>> {
            params.validate().map_err(nb::Error::Other)?;

            let mut bytes = [0; $param_type::MAX_LENGTH];
            let len = params.copy_into_slice(&mut bytes);

            self.write_command($opcode, &bytes[..len])
                .map_err(rewrap_error)
        }
    };
}

pub mod gap;
pub mod gatt;
pub mod hal;
pub mod l2cap;
