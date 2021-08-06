//! WIP Bluetooth Host-Controller Interfface.
//! Bsed on [Eupn's STM32WB library](https://github.com/eupn/stm32wb55)
//! Uses https://github.com/danielgallagher0/bluetooth-hci as a dependency.

//! Bluetooth HCI for STMicro's STM32WB5x Bluetooth controllers.
//!
//! # Design
//!
//! The STM32WB55 is a dual-core SoC that contains application controller (Cortex-M4F) and
//! radio coprocessor (Cortex-M0+). This crate is intended to run on application controller and
//! communicates with the BLE stack that is running on radio coprocessor. The communication is
//! performed through mailbox interface based on shared SRAM area and IPCC peripheral interrupts.
//!
//! This crate defines a public struct, [`RadioCoprocessor`] that owns the IPCC peripheral,
//! implements IPCC IRQ handlers and and implements [`bluetooth_hci::Controller`],
//! which provides access to the full Bluetooth HCI.
//!
//! STM32WB55 BLE stack implements 4.x and 5.x versions of the Bluetooth [specification].
//!
//!
//! # Vendor-Specific Commands
//!
//! STM32WB5x provides several vendor-specific commands that control the behavior of the
//! controller.
//!
//! # Vendor-Specific Events
//!
//! STM32WB5x provides several vendor-specific events that provide data related to the
//! controller. Many of these events are forwarded from the link layer, and these are documented
//! with a reference to the appropriate section of the Bluetooth specification.
//!
//! # Example
//!
//! TODO
//!
//! [specification]: https://www.bluetooth.com/specifications/bluetooth-core-specification

#![no_std]

#[macro_use]
use bitflags;
#[macro_use]
use bluetooth_hci as hci;
use byteorder;

use core::convert::TryFrom;
use hci::host::HciHeader;
use hci::Controller;

mod command;
pub mod event;
mod opcode;

pub use command::gap;
pub use command::gatt;
pub use command::hal;
pub use command::l2cap;

pub use hci::host::{AdvertisingFilterPolicy, AdvertisingType, OwnAddressType};

use crate::{
    ipcc,
    tl_mbox::{
        self,
        cmd::CmdSerial,
        {consts::TlPacketType, shci::ShciBleInitCmdParam},
    },
};

use bbqueue::{ArrayLength, Consumer, Producer};

const TX_BUF_SIZE: usize = core::mem::size_of::<CmdSerial>();

/// Handle for interfacing with the STM32WB5x radio coprocessor.
pub struct RadioCoprocessor<'buf, N: ArrayLength<u8>> {
    mbox: tl_mbox::TlMbox,
    ipcc: ipcc::Ipcc,
    config: ShciBleInitCmdParam,
    buff_producer: Producer<'buf, N>,
    buff_consumer: Consumer<'buf, N>,
    tx_buf: [u8; TX_BUF_SIZE],
    is_ble_ready: bool,
}

impl<'buf, N: ArrayLength<u8>> RadioCoprocessor<'buf, N> {
    /// Creates a new RadioCoprocessor instance to send commands to and receive events from.
    pub fn new(
        producer: Producer<'buf, N>,
        consumer: Consumer<'buf, N>,
        mbox: tl_mbox::TlMbox,
        ipcc: ipcc::Ipcc,
        config: ShciBleInitCmdParam,
    ) -> RadioCoprocessor<'buf, N> {
        RadioCoprocessor {
            mbox,
            ipcc,
            config,
            buff_consumer: consumer,
            buff_producer: producer,
            tx_buf: [0u8; TX_BUF_SIZE],
            is_ble_ready: false,
        }
    }

    fn write_command(&mut self, opcode: opcode::Opcode, params: &[u8]) -> nb::Result<(), ()> {
        const HEADER_LEN: usize = 4;
        let mut header = [0; HEADER_LEN];
        hci::host::uart::CommandHeader::new(opcode, params.len()).copy_into_slice(&mut header);

        self.write(&header, &params)
    }

    /// Call this function from `IPCC_C1_RX_IT` interrupt context.
    pub fn handle_ipcc_rx(&mut self) {
        self.mbox.interrupt_ipcc_rx_handler(&mut self.ipcc);
    }

    /// Call this function from `IPCC_C1_TX_IT` interrupt context.
    pub fn handle_ipcc_tx(&mut self) {
        self.mbox.interrupt_ipcc_tx_handler(&mut self.ipcc);
    }

    /// Call this function outside of interrupt context, for example in `main()` loop.
    /// Returns `true` if events was written and can be read with HCI `read()` function.
    /// Returns `false` if no HCI events was written.
    pub fn process_events(&mut self) -> bool {
        while let Some(evt) = self.mbox.dequeue_event() {
            let event = evt.evt();

            let mut buf = self
                .buff_producer
                .grant_exact(evt.size().expect("Known packet kind"))
                .expect("No space in buffer");

            evt.write(buf.buf()).expect("EVT_BUF_SIZE is too small");

            if event.kind() == 18 {
                tl_mbox::shci::shci_ble_init(&mut self.ipcc, self.config);
                self.is_ble_ready = true;
                buf.buf()[0] = 0x04; // Replace event code with one that is supported by HCI
            }

            buf.commit(evt.size().unwrap());
        }

        // Ignore SYS-channel "command complete" events
        if self.mbox.pop_last_cc_evt().is_some() {
            return false;
        }

        return true;
    }
}

impl<'buf, N: ArrayLength<u8>> hci::Controller for RadioCoprocessor<'buf, N> {
    type Error = ();
    type Header = bluetooth_hci::host::uart::CommandHeader;
    type Vendor = Stm32Wb5xTypes;

    fn write(&mut self, header: &[u8], payload: &[u8]) -> nb::Result<(), Self::Error> {
        let cmd_code = header[0];
        let cmd = TlPacketType::try_from(cmd_code).map_err(|_| ())?;

        self.tx_buf = [0; TX_BUF_SIZE];
        self.tx_buf[..header.len()].copy_from_slice(header);
        self.tx_buf[header.len()..(header.len() + payload.len())].copy_from_slice(payload);

        match &cmd {
            TlPacketType::AclData => {
                // Destination buffer: ble table, phci_acl_data_buffer, acldataserial field
                todo!()
            }

            TlPacketType::SysCmd => {
                // Destination buffer: SYS table, pcmdbuffer, cmdserial field
                todo!()
            }

            _ => {
                tl_mbox::ble::ble_send_cmd(&mut self.ipcc, &self.tx_buf[..]);
            }
        }

        Ok(())
    }

    fn read_into(&mut self, buffer: &mut [u8]) -> nb::Result<(), Self::Error> {
        match self.buff_consumer.read() {
            Ok(grant) => {
                if buffer.len() <= grant.buf().len() {
                    buffer.copy_from_slice(&grant.buf()[..buffer.len()]);

                    grant.release(buffer.len());

                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
            Err(bbqueue::Error::InsufficientSize) => Err(nb::Error::WouldBlock),
            Err(_other) => Err(nb::Error::Other(())),
        }
    }

    fn peek(&mut self, n: usize) -> nb::Result<u8, Self::Error> {
        match self.buff_consumer.read() {
            Ok(grant) => {
                if n >= grant.buf().len() {
                    return Err(nb::Error::WouldBlock);
                } else {
                    Ok(grant.buf()[n])
                }
            }
            Err(bbqueue::Error::InsufficientSize) => Err(nb::Error::WouldBlock),
            Err(_other) => Err(nb::Error::Other(())),
        }
    }
}

/// Specify vendor-specific extensions for the BlueNRG.
pub struct Stm32Wb5xTypes;
impl hci::Vendor for Stm32Wb5xTypes {
    type Status = event::Status;
    type Event = event::Stm32Wb5xEvent;
}

/// Master trait that encompasses all commands, and communicates over UART.
pub trait UartController<E>:
    gap::Commands<Error = E>
    + gatt::Commands<Error = E>
    + hal::Commands<Error = E>
    + l2cap::Commands<Error = E>
    + bluetooth_hci::host::uart::Hci<E, event::Stm32Wb5xEvent, event::Stm32Wb5xError>
{
}

impl<T, E> UartController<E> for T where
    T: gap::Commands<Error = E>
        + gatt::Commands<Error = E>
        + hal::Commands<Error = E>
        + l2cap::Commands<Error = E>
        + bluetooth_hci::host::uart::Hci<E, event::Stm32Wb5xEvent, event::Stm32Wb5xError>
{
}

/// Vendor-specific interpretation of the local version information from the controller.
#[derive(Clone)]
pub struct Version {
    /// Major version of the controller firmware
    pub major: u8,

    /// Minor version of the controller firmware
    pub minor: u8,

    /// Patch version of the controller firmware
    pub patch: u8,
}

/// Extension trait to convert [`hci::event::command::LocalVersionInfo`] into the BLE stack-specific
/// [`Version`] struct.
pub trait LocalVersionInfoExt {
    /// Converts LocalVersionInfo as returned by the controller into a BLE stack-specific [`Version`]
    /// struct.
    fn wireless_fw_version(&self) -> Version;
}

impl<VS> LocalVersionInfoExt for hci::event::command::LocalVersionInfo<VS> {
    fn wireless_fw_version(&self) -> Version {
        // TODO
        Version {
            major: (self.hci_revision & 0xFF) as u8,
            minor: ((self.lmp_subversion >> 4) & 0xF) as u8,
            patch: (self.lmp_subversion & 0xF) as u8,
        }
    }
}

// todo: Code below is from stm32wb55/opcode.rs
pub use hci::Opcode;

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
