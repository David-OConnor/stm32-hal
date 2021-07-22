pub use bluetooth_hci::Opcode;

const fn ocf(cgid: u16, cid: u16) -> u16 {
    ((cgid & 0b111) << 7) | (cid & 0b111_1111)
}

const VENDOR_OGF: u16 = 0x3F;

macro_rules! opcodes {
    (
        $(
            $_cgid_comment:ident = $cgid:expr;
            {
                $(pub const $var:ident = $cid:expr;)+
            }
        )+
    ) => {
        $($(
            pub const $var: Opcode = Opcode::new(VENDOR_OGF, ocf($cgid, $cid));
        )+)+
    }
}

opcodes! {
    Hal = 0x0;
    {
        pub const HAL_GET_FIRMWARE_REVISION = 0x00;
        pub const HAL_WRITE_CONFIG_DATA = 0x0C;
        pub const HAL_READ_CONFIG_DATA = 0x0D;
        pub const HAL_SET_TX_POWER_LEVEL = 0x0F;
        pub const HAL_DEVICE_STANDBY = 0x13;
        pub const HAL_TX_TEST_PACKET_COUNT = 0x14;
        pub const HAL_START_TONE = 0x15;
        pub const HAL_STOP_TONE = 0x16;
        pub const HAL_GET_LINK_STATUS = 0x17;

        // The documentation says the OCF is 0xF8 (0b1111_1000), but that does not fit the OCF
        // length (7 bits). The C source code has 0x19, which is valid.
        pub const HAL_GET_ANCHOR_PERIOD = 0x19;
    }
    Gap = 0x1;
    {
        pub const GAP_SET_NONDISCOVERABLE = 0x01;
        pub const GAP_SET_LIMITED_DISCOVERABLE = 0x02;
        pub const GAP_SET_DISCOVERABLE = 0x03;
        pub const GAP_SET_DIRECT_CONNECTABLE = 0x04;
        pub const GAP_SET_IO_CAPABILITY = 0x05;
        pub const GAP_SET_AUTHENTICATION_REQUIREMENT = 0x06;
        pub const GAP_SET_AUTHORIZATION_REQUIREMENT = 0x07;
        pub const GAP_PASS_KEY_RESPONSE = 0x08;
        pub const GAP_AUTHORIZATION_RESPONSE = 0x09;
        pub const GAP_INIT = 0x0A;
        pub const GAP_SET_NONCONNECTABLE = 0x0B;
        pub const GAP_SET_UNDIRECTED_CONNECTABLE = 0x0C;
        pub const GAP_PERIPHERAL_SECURITY_REQUEST = 0x0D;
        pub const GAP_UPDATE_ADVERTISING_DATA = 0x0E;
        pub const GAP_DELETE_AD_TYPE = 0x0F;
        pub const GAP_GET_SECURITY_LEVEL = 0x10;
        pub const GAP_SET_EVENT_MASK = 0x11;
        pub const GAP_CONFIGURE_WHITE_LIST = 0x12;
        pub const GAP_TERMINATE = 0x13;
        pub const GAP_CLEAR_SECURITY_DATABASE = 0x14;
        pub const GAP_ALLOW_REBOND = 0x15;
        pub const GAP_START_LIMITED_DISCOVERY_PROCEDURE = 0x16;
        pub const GAP_START_GENERAL_DISCOVERY_PROCEDURE = 0x17;
        pub const GAP_START_NAME_DISCOVERY_PROCEDURE = 0x18;
        pub const GAP_START_AUTO_CONNECTION_ESTABLISHMENT = 0x19;
        pub const GAP_START_GENERAL_CONNECTION_ESTABLISHMENT = 0x1A;
        pub const GAP_START_SELECTIVE_CONNECTION_ESTABLISHMENT = 0x1B;
        pub const GAP_CREATE_CONNECTION = 0x1C;
        pub const GAP_TERMINATE_PROCEDURE = 0x1D;
        pub const GAP_START_CONNECTION_UPDATE = 0x1E;
        pub const GAP_SEND_PAIRING_REQUEST = 0x1F;
        pub const GAP_RESOLVE_PRIVATE_ADDRESS = 0x20;
        pub const GAP_SET_BROADCAST_MODE = 0x21;
        pub const GAP_START_OBSERVATION_PROCEDURE = 0x22;
        pub const GAP_GET_BONDED_DEVICES = 0x23;
        pub const GAP_IS_DEVICE_BONDED = 0x24;
    }
    Gatt = 0x2;
    {
        pub const GATT_INIT = 0x01;
        pub const GATT_ADD_SERVICE = 0x02;
        pub const GATT_INCLUDE_SERVICE = 0x03;
        pub const GATT_ADD_CHARACTERISTIC = 0x04;
        pub const GATT_ADD_CHARACTERISTIC_DESCRIPTOR = 0x05;
        pub const GATT_UPDATE_CHARACTERISTIC_VALUE = 0x06;
        pub const GATT_DELETE_CHARACTERISTIC = 0x07;
        pub const GATT_DELETE_SERVICE = 0x08;
        pub const GATT_DELETE_INCLUDED_SERVICE = 0x09;
        pub const GATT_SET_EVENT_MASK = 0x0A;
        pub const GATT_EXCHANGE_CONFIGURATION = 0x0B;
        pub const GATT_FIND_INFORMATION_REQUEST = 0x0C;
        pub const GATT_FIND_BY_TYPE_VALUE_REQUEST = 0x0D;
        pub const GATT_READ_BY_TYPE_REQUEST = 0x0E;
        pub const GATT_READ_BY_GROUP_TYPE_REQUEST = 0x0F;
        pub const GATT_PREPARE_WRITE_REQUEST = 0x10;
        pub const GATT_EXECUTE_WRITE_REQUEST = 0x11;
        pub const GATT_DISCOVER_ALL_PRIMARY_SERVICES = 0x12;
        pub const GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID = 0x13;
        pub const GATT_FIND_INCLUDED_SERVICES = 0x14;
        pub const GATT_DISCOVER_ALL_CHARACTERISTICS_OF_SERVICE = 0x15;
        pub const GATT_DISCOVER_CHARACTERISTICS_BY_UUID = 0x16;
        pub const GATT_DISCOVER_ALL_CHARACTERISTIC_DESCRIPTORS = 0x17;
        pub const GATT_READ_CHARACTERISTIC_VALUE = 0x18;
        pub const GATT_READ_CHARACTERISTIC_BY_UUID = 0x19;
        pub const GATT_READ_LONG_CHARACTERISTIC_VALUE = 0x1A;
        pub const GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES = 0x1B;
        pub const GATT_WRITE_CHARACTERISTIC_VALUE = 0x1C;
        pub const GATT_WRITE_LONG_CHARACTERISTIC_VALUE = 0x1D;
        pub const GATT_WRITE_CHARACTERISTIC_VALUE_RELIABLY = 0x1E;
        pub const GATT_WRITE_LONG_CHARACTERISTIC_DESCRIPTOR = 0x1F;
        pub const GATT_READ_LONG_CHARACTERISTIC_DESCRIPTOR = 0x20;
        pub const GATT_WRITE_CHARACTERISTIC_DESCRIPTOR = 0x21;
        pub const GATT_READ_CHARACTERISTIC_DESCRIPTOR = 0x22;
        pub const GATT_WRITE_WITHOUT_RESPONSE = 0x23;
        pub const GATT_SIGNED_WRITE_WITHOUT_RESPONSE = 0x24;
        pub const GATT_CONFIRM_INDICATION = 0x25;
        pub const GATT_WRITE_RESPONSE = 0x26;
        pub const GATT_ALLOW_READ = 0x27;
        pub const GATT_SET_SECURITY_PERMISSION = 0x28;
        pub const GATT_SET_DESCRIPTOR_VALUE = 0x29;
        pub const GATT_READ_HANDLE_VALUE = 0x2A;
        pub const GATT_READ_HANDLE_VALUE_OFFSET = 0x2B;
        pub const GATT_UPDATE_LONG_CHARACTERISTIC_VALUE = 0x2C;
    }
    L2Cap = 0x3;
    {
        pub const L2CAP_CONN_PARAM_UPDATE_REQ = 0x01;
        pub const L2CAP_CONN_PARAM_UPDATE_RESP = 0x02;
    }
}
