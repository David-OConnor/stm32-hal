//! GAP commands and types needed for those commands.

use bluetooth_hci as hci;
use byteorder;
use nb;

use bbqueue::ArrayLength;
use byteorder::{ByteOrder, LittleEndian};
use core::time::Duration;
pub use hci::host::{AdvertisingFilterPolicy, AdvertisingType, OwnAddressType};
pub use hci::types::{ConnectionInterval, ExpectedConnectionLength, ScanWindow};
pub use hci::{BdAddr, BdAddrType};

use crate::bluetooth::{bitflags::bitflags, event, opcode, RadioCoprocessor};

/// GAP-specific commands for the [`ActiveBlueNRG`](crate::ActiveBlueNRG).
pub trait Commands {
    /// Type of communication errors.
    type Error;

    /// Set the device in non-discoverable mode. This command will disable the LL advertising and
    /// put the device in standby state.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetNonDiscoverable) event
    /// is generated.
    fn set_nondiscoverable(&mut self) -> nb::Result<(), Self::Error>;

    /// Set the device in limited discoverable mode.
    ///
    /// Limited discoverability is defined in in GAP specification volume 3, section 9.2.3. The
    /// device will be discoverable for maximum period of TGAP (lim_adv_timeout) = 180 seconds (from
    /// errata). The advertising can be disabled at any time by issuing a
    /// [`set_nondiscoverable`](Commands::set_nondiscoverable) command.
    ///
    /// # Errors
    ///
    /// - [`BadAdvertisingType`](Error::BadAdvertisingType) if
    ///   [`advertising_type`](DiscoverableParameters::advertising_type) is one of the disallowed
    ///   types:
    ///   [ConnectableDirectedHighDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedHighDutyCycle)
    ///   or
    ///   [ConnectableDirectedLowDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedLowDutyCycle).
    /// - [`BadAdvertisingInterval`](Error::BadAdvertisingInterval) if
    ///   [`advertising_interval`](DiscoverableParameters::advertising_interval) is inverted.
    ///   That is, if the min is greater than the max.
    /// - [`BadConnectionInterval`](Error::BadConnectionInterval) if
    ///   [`conn_interval`](DiscoverableParameters::conn_interval) is inverted. That is, both the
    ///   min and max are provided, and the min is greater than the max.
    ///
    /// # Generated evenst
    ///
    /// When the controller receives the command, it will generate a [command
    /// status](hci::event::Event::CommandStatus) event. The controller starts the advertising after
    /// this and when advertising timeout happens (i.e. limited discovery period has elapsed),
    /// the controller generates an [GAP Limited Discoverable
    /// Complete](event::BlueNRGEvent::GapLimitedDiscoverableTimeout) event.

    fn set_limited_discoverable<'a, 'b>(
        &mut self,
        params: &DiscoverableParameters<'a, 'b>,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Set the device in discoverable mode.
    ///
    /// Limited discoverability is defined in in GAP specification volume 3, section 9.2.4. The
    /// device will be discoverable for maximum period of TGAP (lim_adv_timeout) = 180 seconds (from
    /// errata). The advertising can be disabled at any time by issuing a
    /// [`set_nondiscoverable`](Commands::set_nondiscoverable) command.
    ///
    /// # Errors
    ///
    /// - [`BadAdvertisingType`](Error::BadAdvertisingType) if
    ///   [`advertising_type`](DiscoverableParameters::advertising_type) is one of the disallowed
    ///   types:
    ///   [ConnectableDirectedHighDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedHighDutyCycle)
    ///   or
    ///   [ConnectableDirectedLowDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedLowDutyCycle).
    /// - [`BadAdvertisingInterval`](Error::BadAdvertisingInterval) if
    ///   [`advertising_interval`](DiscoverableParameters::advertising_interval) is inverted.
    ///   That is, if the min is greater than the max.
    /// - [`BadConnectionInterval`](Error::BadConnectionInterval) if
    ///   [`conn_interval`](DiscoverableParameters::conn_interval) is inverted. That is, both the
    ///   min and max are provided, and the min is greater than the max.
    ///
    /// # Generated evenst
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetDiscoverable) event is
    /// generated.
    fn set_discoverable<'a, 'b>(
        &mut self,
        params: &DiscoverableParameters<'a, 'b>,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Set the device in direct connectable mode.
    ///
    /// Direct connectable mode is defined in GAP specification Volume 3,
    /// Section 9.3.3). Device uses direct connectable mode to advertise using either High Duty
    /// cycle advertisement events or Low Duty cycle advertisement events and the address as
    /// what is specified in the Own Address Type parameter. The Advertising Type parameter in
    /// the command specifies the type of the advertising used.
    ///
    /// When the `ms` feature is _not_ enabled, the device will be in directed connectable mode only
    /// for 1.28 seconds. If no connection is established within this duration, the device enters
    /// non discoverable mode and advertising will have to be again enabled explicitly.
    ///
    /// When the `ms` feature _is_ enabled, the advertising interval is explicitly provided in the
    /// [parameters][DirectConnectableParameters].
    ///
    /// # Errors
    ///
    /// - [`BadAdvertisingType`](Error::BadAdvertisingType) if
    ///   [`advertising_type`](DiscoverableParameters::advertising_type) is one of the disallowed
    ///   types:
    ///   [ConnectableUndirected](bluetooth_hci::host::AdvertisingType::ConnectableUndirected),
    ///   [ScannableUndirected](bluetooth_hci::host::AdvertisingType::ScannableUndirected), or
    ///   [NonConnectableUndirected](bluetooth_hci::host::AdvertisingType::NonConnectableUndirected),
    /// - (`ms` feature only) [`BadAdvertisingInterval`](Error::BadAdvertisingInterval) if
    ///   [`advertising_interval`](DiscoverableParameters::advertising_interval) is
    ///   out of range (20 ms to 10.24 s) or inverted (the min is greater than the max).
    ///
    /// # Generated evenst
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetDirectConnectable) event
    /// is generated.
    fn set_direct_connectable(
        &mut self,
        params: &DirectConnectableParameters,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Set the IO capabilities of the device.
    ///
    /// This command has to be given only when the device is not in a connected state.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetIoCapability) event is
    /// generated.
    fn set_io_capability(&mut self, capability: IoCapability) -> nb::Result<(), Self::Error>;

    /// Set the authentication requirements for the device.
    ///
    /// This command has to be given only when the device is not in a connected state.
    ///
    /// # Errors
    ///
    /// - [BadEncryptionKeySizeRange](Error::BadEncryptionKeySizeRange) if the
    ///   [`encryption_key_size_range`](AuthenticationRequirements::encryption_key_size_range) min
    ///   is greater than the max.
    /// - [BadFixedPin](Error::BadFixedPin) if the
    ///   [`fixed_pin`](AuthenticationRequirements::fixed_pin) is [Fixed](Pin::Fixed) with a value
    ///   greater than 999999.
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// - A [Command
    ///   Complete](event::command::ReturnParameters::GapSetAuthenticationRequirement) event
    ///   is generated.
    /// - If [`fixed_pin`](AuthenticationRequirements::fixed_pin) is [Request](Pin::Requested), then
    ///   a [GAP Pass Key](event::BlueNRGEvent::GapPassKeyRequest) event is generated.
    fn set_authentication_requirement(
        &mut self,
        requirements: &AuthenticationRequirements,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Set the authorization requirements of the device.
    ///
    /// This command has to be given when connected to a device if authorization is required to
    /// access services which require authorization.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// - A [Command
    ///   Complete](event::command::ReturnParameters::GapSetAuthorizationRequirement) event
    ///   is generated.
    /// - If authorization is required, then a [GAP Authorization
    ///   Request](event::BlueNRGEvent::GapAuthorizationRequest) event is generated.
    fn set_authorization_requirement(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        authorization_required: bool,
    ) -> nb::Result<(), Self::Error>;

    /// This command should be send by the host in response to the [GAP Pass Key
    /// Request](event::BlueNRGEvent::GapPassKeyRequest) event.
    ///
    /// `pin` contains the pass key which will be used during the pairing process.
    ///
    /// # Errors
    ///
    /// - [BadFixedPin](Error::BadFixedPin) if the pin is greater than 999999.
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// - A [Command Complete](event::command::ReturnParameters::GapPassKeyResponse) event is
    ///   generated.
    /// - When the pairing process completes, it will generate a
    ///   [PairingComplete](event::BlueNRGEvent::GapPairingComplete) event.
    fn pass_key_response(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        pin: u32,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// This command should be send by the host in response to the [GAP Authorization
    /// Request](event::BlueNRGEvent::GapAuthorizationRequest) event.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapAuthorizationResponse)
    /// event is generated.
    fn authorization_response(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        authorization: Authorization,
    ) -> nb::Result<(), Self::Error>;

    /// Register the GAP service with the GATT.
    ///
    /// The device name characteristic and appearance characteristic are added by default and the
    /// handles of these characteristics are returned in the [event
    /// data](event::command::GapInit).
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapInit) event is generated.
    fn init(
        &mut self,
        role: Role,
        privacy_enabled: bool,
        dev_name_characteristic_len: u8,
    ) -> nb::Result<(), Self::Error>;

    /// Register the GAP service with the GATT.
    ///
    /// This function exists to prevent name conflicts with other Commands traits' init methods.
    fn init_gap(
        &mut self,
        role: Role,
        privacy_enabled: bool,
        dev_name_characteristic_len: u8,
    ) -> nb::Result<(), Self::Error> {
        self.init(role, privacy_enabled, dev_name_characteristic_len)
    }

    /// Put the device into non-connectable mode.
    ///
    /// This mode does not support connection. The privacy setting done in the
    /// [`init`](Commands::init) command plays a role in deciding the valid
    /// parameters for this command. If privacy was not enabled, `address_type` may be
    /// [Public](AddressType::Public) or [Random](AddressType::Random).  If privacy was
    /// enabled, `address_type` may be [ResolvablePrivate](AddressType::ResolvablePrivate) or
    /// [NonResolvablePrivate](AddressType::NonResolvablePrivate).
    ///
    /// # Errors
    ///
    /// - [BadAdvertisingType](Error::BadAdvertisingType) if the advertising type is not one
    ///   of the supported modes. It must be
    ///   [ScannableUndirected](AdvertisingType::ScannableUndirected) or
    ///   (NonConnectableUndirected)[AdvertisingType::NonConnectableUndirected).
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapInit) event is generated.
    fn set_nonconnectable(
        &mut self,
        advertising_type: AdvertisingType,
        address_type: AddressType,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Put the device into undirected connectable mode.
    ///
    /// The privacy setting done in the [`init`](Commands::init) command plays a role
    /// in deciding the valid parameters for this command.
    ///
    /// # Errors
    ///
    /// - [BadAdvertisingFilterPolicy](Error::BadAdvertisingFilterPolicy) if the filter is
    ///   not one of the supported modes. It must be
    ///   [AllowConnectionAndScan](AdvertisingFilterPolicy::AllowConnectionAndScan) or
    ///   (WhiteListConnectionAllowScan)[AdvertisingFilterPolicy::WhiteListConnectionAllowScan).
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetUndirectedConnectable)
    /// event is generated.
    fn set_undirected_connectable(
        &mut self,
        filter_policy: AdvertisingFilterPolicy,
        address_type: AddressType,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// This command has to be issued to notify the central device of the security requirements of
    /// the peripheral.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event will be generated when a valid
    /// command is received. On completion of the command, i.e. when the security request is
    /// successfully transmitted to the master, a [GAP Peripheral Security
    /// Initiated](event::BlueNRGEvent::GapPeripheralSecurityInitiated) vendor-specific event
    /// will be generated.
    fn peripheral_security_request(
        &mut self,
        params: &SecurityRequestParameters,
    ) -> nb::Result<(), Self::Error>;

    /// This command can be used to update the advertising data for a particular AD type. If the AD
    /// type specified does not exist, then it is added to the advertising data. If the overall
    /// advertising data length is more than 31 octets after the update, then the command is
    /// rejected and the old data is retained.
    ///
    /// # Errors
    ///
    /// - [BadAdvertisingDataLength](Error::BadAdvertisingDataLength) if the provided data is longer
    ///   than 31 bytes.
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapUpdateAdvertisingData)
    /// event is generated.
    fn update_advertising_data(&mut self, data: &[u8]) -> nb::Result<(), Error<Self::Error>>;

    /// This command can be used to delete the specified AD type from the advertisement data if
    /// present.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapDeleteAdType) event is
    /// generated.
    fn delete_ad_type(&mut self, ad_type: AdvertisingDataType) -> nb::Result<(), Self::Error>;

    /// This command can be used to get the current security settings of the device.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapGetSecurityLevel) event is
    /// generated.
    fn get_security_level(&mut self) -> nb::Result<(), Self::Error>;

    /// Allows masking events from the GAP.
    ///
    /// The default configuration is all the events masked.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapSetEventMask) event is
    /// generated.
    fn set_event_mask(&mut self, flags: EventFlags) -> nb::Result<(), Self::Error>;

    /// Allows masking events from the GAP.
    ///
    /// This function exists to prevent name conflicts with other Commands traits' set_event_mask
    /// methods.
    fn set_gap_event_mask(&mut self, flags: EventFlags) -> nb::Result<(), Self::Error> {
        self.set_event_mask(flags)
    }

    /// Configure the controller's white list with devices that are present in the security
    /// database.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapConfigureWhiteList) event
    /// is generated.
    fn configure_white_list(&mut self) -> nb::Result<(), Self::Error>;

    /// Command the controller to terminate the connection.
    ///
    /// # Errors
    ///
    /// - [BadTerminationReason](Error::BadTerminationReason) if provided termination reason is
    ///   invalid. Valid reasons are the same as HCI [disconnect](hci::host::Hci::disconnect):
    ///   [`AuthFailure`](hci::Status::AuthFailure),
    ///   [`RemoteTerminationByUser`](hci::Status::RemoteTerminationByUser),
    ///   [`RemoteTerminationLowResources`](hci::Status::RemoteTerminationLowResources),
    ///   [`RemoteTerminationPowerOff`](hci::Status::RemoteTerminationPowerOff),
    ///   [`UnsupportedRemoteFeature`](hci::Status::UnsupportedRemoteFeature),
    ///   [`PairingWithUnitKeyNotSupported`](hci::Status::PairingWithUnitKeyNotSupported), or
    ///   [`UnacceptableConnectionParameters`](hci::Status::UnacceptableConnectionParameters).
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// The controller will generate a [command status](hci::event::Event::CommandStatus) event when
    /// the command is received and a [Disconnection
    /// Complete](hci::event::Event::DisconnectionComplete) event will be generated when the link is
    /// disconnected.
    fn terminate(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        reason: hci::Status<event::Status>,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Clear the security database. All the devices in the security database will be removed.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapClearSecurityDatabase)
    /// event is generated.
    fn clear_security_database(&mut self) -> nb::Result<(), Self::Error>;

    #[cfg(not(feature = "ms"))]
    /// This command should be given by the application when it receives the
    /// [GAP Bond Lost](::event::BlueNRGEvent::GapBondLost) event if it wants the re-bonding to happen
    /// successfully. If this command is not given on receiving the event, the bonding procedure
    /// will timeout.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](::event::command::ReturnParameters::GapAllowRebond) event is
    /// generated. Even if the command is given when it is not valid, success will be returned but
    /// internally it will have no effect.
    fn allow_rebond(&mut self) -> nb::Result<(), Self::Error>;

    #[cfg(feature = "ms")]
    /// This command should be given by the application when it receives the [GAP Bond
    /// Lost](event::BlueNRGEvent::GapBondLost) event if it wants the re-bonding to happen
    /// successfully. If this command is not given on receiving the event, the bonding procedure
    /// will timeout.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [Command Complete](event::command::ReturnParameters::GapAllowRebond) event is
    /// generated. Even if the command is given when it is not valid, success will be returned but
    /// internally it will have no effect.
    fn allow_rebond(&mut self, conn_handle: hci::ConnectionHandle) -> nb::Result<(), Self::Error>;

    /// Start the limited discovery procedure.
    ///
    /// The controller is commanded to start active scanning.  When this procedure is started, only
    /// the devices in limited discoverable mode are returned to the upper layers.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated as soon as the
    /// command is given.
    ///
    /// If [Success](hci::Status::Success) is returned in the command status, the procedure is
    /// terminated when either the upper layers issue a command to terminate the procedure by
    /// issuing the command [`terminate_procedure`](Commands::terminate_procedure) with the
    /// procedure code set to [LimitedDiscovery](event::GapProcedure::LimitedDiscovery) or a
    /// [timeout](event::BlueNRGEvent::GapLimitedDiscoverableTimeout) happens. When the
    /// procedure is terminated due to any of the above reasons, a
    /// [ProcedureComplete](event::BlueNRGEvent::GapProcedureComplete) event is returned with
    /// the procedure code set to [LimitedDiscovery](event::GapProcedure::LimitedDiscovery).
    ///
    /// The device found when the procedure is ongoing is returned to the upper layers through the
    /// [LeAdvertisingReport](hci::event::Event::LeAdvertisingReport) event.
    fn start_limited_discovery_procedure(
        &mut self,
        params: &DiscoveryProcedureParameters,
    ) -> nb::Result<(), Self::Error>;

    /// Start the general discovery procedure. The controller is commanded to start active scanning.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated as soon as the
    /// command is given.
    ///
    /// If [Success](hci::Status::Success) is returned in the command status, the procedure is
    /// terminated when either the upper layers issue a command to terminate the procedure by
    /// issuing the command [`terminate_procedure`](Commands::terminate_procedure) with the
    /// procedure code set to [GeneralDiscovery](event::GapProcedure::GeneralDiscovery) or a
    /// timeout happens. When the procedure is terminated due to any of the above reasons, a
    /// [ProcedureComplete](event::BlueNRGEvent::GapProcedureComplete) event is returned with
    /// the procedure code set to [GeneralDiscovery](event::GapProcedure::GeneralDiscovery).
    ///
    /// The device found when the procedure is ongoing is returned to the upper layers through the
    /// [LeAdvertisingReport](hci::event::Event::LeAdvertisingReport) event.
    fn start_general_discovery_procedure(
        &mut self,
        params: &DiscoveryProcedureParameters,
    ) -> nb::Result<(), Self::Error>;

    /// Start the name discovery procedure.
    ///
    /// A [LE Create Connection](hci::host::Hci::le_create_connection) call will be made to the
    /// controller by GAP with the [initiator filter
    /// policy](hci::host::ConnectionParameters::initiator_filter_policy) set to
    /// [UseAddress](hci::host::ConnectionFilterPolicy::UseAddress), to "ignore whitelist and
    /// process connectable advertising packets only for the specified device". Once a connection is
    /// established, GATT procedure is started to read the device name characteristic. When the read
    /// is completed (successfully or unsuccessfully), a
    /// [ProcedureComplete](event::BlueNRGEvent::GapProcedureComplete) event is given to the
    /// upper layer. The event also contains the name of the device if the device name was read
    /// successfully.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated Events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated as soon as the
    /// command is given. If [Success](hci::Status::Success) is returned, on completion of the
    /// procedure, a [ProcedureComplete](event::BlueNRGEvent::GapProcedureComplete) event is
    /// returned with the procedure code set to
    /// [NameDiscovery](event::GapProcedure::NameDiscovery).
    fn start_name_discovery_procedure(
        &mut self,
        params: &NameDiscoveryProcedureParameters,
    ) -> nb::Result<(), Self::Error>;

    /// Start the auto connection establishment procedure.
    ///
    /// The devices specified are added to the white list of the controller and a
    /// [`le_create_connection`](hci::host::Hci::le_create_connection) call will be made to the
    /// controller by GAP with the [initiator filter
    /// policy](hci::host::ConnectionParameters::initiator_filter_policy) set to
    /// [WhiteList](hci::host::ConnectionFilterPolicy::WhiteList), to "use whitelist to determine
    /// which advertiser to connect to". When a command is issued to terminate the procedure by
    /// upper layer, a [`le_create_connection_cancel`](hci::host::Hci::le_create_connection_cancel)
    /// call will be made to the controller by GAP.
    ///
    /// # Errors
    ///
    /// - If the [`white_list`](AutoConnectionEstablishmentParameters::white_list) is too long
    ///   (such that the serialized command would not fit in 255 bytes), a
    ///   [WhiteListTooLong](Error::WhiteListTooLong) is returned. The list cannot have more than 33
    ///   elements.
    fn start_auto_connection_establishment<'a>(
        &mut self,
        params: &AutoConnectionEstablishmentParameters<'a>,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Start a general connection establishment procedure.
    ///
    /// The host [enables scanning](hci::host::Hci::le_set_scan_enable) in the controller with the
    /// scanner [filter policy](hci::host::ScanParameters::filter_policy) set to
    /// [AcceptAll](hci::host::ScanFilterPolicy::AcceptAll), to "accept all advertising packets" and
    /// from the scanning results, all the devices are sent to the upper layer using the event [LE
    /// Advertising Report](hci::event::Event::LeAdvertisingReport). The upper layer then has to
    /// select one of the devices to which it wants to connect by issuing the command
    /// [`create_connection`](Commands::create_connection). If privacy is enabled,
    /// then either a private resolvable address or a non-resolvable address, based on the address
    /// type specified in the command is set as the scanner address but the GAP create connection
    /// always uses a private resolvable address if the general connection establishment procedure
    /// is active.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    fn start_general_connection_establishment(
        &mut self,
        params: &GeneralConnectionEstablishmentParameters,
    ) -> nb::Result<(), Self::Error>;

    /// Start a selective connection establishment procedure.
    ///
    /// The GAP adds the specified device addresses into white list and [enables
    /// scanning](hci::host::Hci::le_set_scan_enable) in the controller with the scanner [filter
    /// policy](hci::host::ScanParameters::filter_policy) set to
    /// [WhiteList](hci::host::ScanFilterPolicy::WhiteList), to "accept packets only from devices in
    /// whitelist". All the devices found are sent to the upper layer by the event [LE Advertising
    /// Report](hci::event::Event::LeAdvertisingReport). The upper layer then has to select one of
    /// the devices to which it wants to connect by issuing the command
    /// [`create_connection`](Commands::create_connection).
    ///
    /// # Errors
    ///
    /// - If the [`white_list`](SelectiveConnectionEstablishmentParameters::white_list) is too
    ///   long (such that the serialized command would not fit in 255 bytes), a
    ///   [WhiteListTooLong](Error::WhiteListTooLong) is returned. The list cannot have more than 35
    ///   elements.
    fn start_selective_connection_establishment<'a>(
        &mut self,
        params: &SelectiveConnectionEstablishmentParameters<'a>,
    ) -> nb::Result<(), Error<Self::Error>>;

    /// Start the direct connection establishment procedure.
    ///
    /// A [LE Create Connection](hci::host::Hci::le_create_connection) call will be made to the
    /// controller by GAP with the initiator [filter
    /// policy](hci::host::ConnectionParameters::initiator_filter_policy) set to
    /// [UseAddress](hci::host::ConnectionFilterPolicy::UseAddress) to "ignore whitelist and process
    /// connectable advertising packets only for the specified device". The procedure can be
    /// terminated explicitly by the upper layer by issuing the command
    /// [`terminate_procedure`](Commands::terminate_procedure). When a command is
    /// issued to terminate the procedure by upper layer, a
    /// [`le_create_connection_cancel`](hci::host::Hci::le_create_connection_cancel) call will be
    /// made to the controller by GAP.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated as soon as the
    /// command is given. If [Success](hci::Status::Success) is returned, on termination of the
    /// procedure, a [LE Connection Complete](hci::event::LeConnectionComplete) event is
    /// returned. The procedure can be explicitly terminated by the upper layer by issuing the
    /// command [`terminate_procedure`](Commands::terminate_procedure) with the procedure_code set
    /// to
    /// [DirectConnectionEstablishment](event::GapProcedure::DirectConnectionEstablishment).
    fn create_connection(&mut self, params: &ConnectionParameters) -> nb::Result<(), Self::Error>;

    /// The GAP procedure(s) specified is terminated.
    ///
    /// # Errors
    ///
    /// - [NoProcedure](Error::NoProcedure) if the bitfield is empty.
    /// - Underlying communication errors
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapTerminateProcedure) event
    /// is generated for this command. If the command was successfully processed, the status field
    /// will be [Success](hci::Status::Success) and a
    /// [ProcedureCompleted](event::BlueNRGEvent::GapProcedureComplete) event is returned
    /// with the procedure code set to the corresponding procedure.
    fn terminate_procedure(&mut self, procedure: Procedure) -> nb::Result<(), Error<Self::Error>>;

    /// Start the connection update procedure.
    ///
    /// A [`le_connection_update`](hci::host::Hci::le_connection_update) call is be made to the
    /// controller by GAP.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated as soon as the
    /// command is given. If [Success](hci::Status::Success) is returned, on completion of
    /// connection update, a
    /// [LeConnectionUpdateComplete](hci::event::Event::LeConnectionUpdateComplete) event is
    /// returned to the upper layer.
    fn start_connection_update(
        &mut self,
        params: &ConnectionUpdateParameters,
    ) -> nb::Result<(), Self::Error>;

    /// Send the SM pairing request to start a pairing process. The authentication requirements and
    /// I/O capabilities should be set before issuing this command using the
    /// [`set_io_capability`](Commands::set_io_capability) and
    /// [`set_authentication_requirement`](Commands::set_authentication_requirement)
    /// commands.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command status](hci::event::Event::CommandStatus) event is generated when the command is
    /// received. If [Success](hci::Status::Success) is returned in the command status event, a
    /// [Pairing Complete](event::BlueNRGEvent::GapPairingComplete) event is returned after
    /// the pairing process is completed.
    fn send_pairing_request(&mut self, params: &PairingRequest) -> nb::Result<(), Self::Error>;

    /// This command tries to resolve the address provided with the IRKs present in its database.
    ///
    /// If the address is resolved successfully with any one of the IRKs present in the database, it
    /// returns success and also the corresponding public/static random address stored with the IRK
    /// in the database.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapResolvePrivateAddress)
    /// event is generated. If [Success](hci::Status::Success) is returned as the status, then the
    /// address is also returned in the event.
    fn resolve_private_address(&mut self, addr: hci::BdAddr) -> nb::Result<(), Self::Error>;

    /// This command gets the list of the devices which are bonded. It returns the number of
    /// addresses and the corresponding address types and values.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapGetBondedDevices) event is
    /// generated.
    fn get_bonded_devices(&mut self) -> nb::Result<(), Self::Error>;

    #[cfg(feature = "ms")]
    /// This command puts the device into broadcast mode.
    ///
    /// # Errors
    ///
    /// - [BadAdvertisingType](Error::BadAdvertisingType) if the advertising type is not
    ///   [ScannableUndirected](hci::types::AdvertisingType::ScannableUndirected) or
    ///   [NonConnectableUndirected](hci::types::AdvertisingType::NonConnectableUndirected).
    /// - [BadAdvertisingDataLength](Error::BadAdvertisingDataLength) if the advertising data is
    ///   longer than 31 bytes.
    /// - [WhiteListTooLong](Error::WhiteListTooLong) if the length of the white list would put the
    ///   packet length over 255 bytes. The exact number of addresses that can be in the white list
    ///   can range from 35 to 31, depending on the length of the advertising data.
    /// - Underlying communication errors.
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapSetBroadcastMode) event is
    /// returned where the status indicates whether the command was successful.
    fn set_broadcast_mode(
        &mut self,
        params: &BroadcastModeParameters,
    ) -> nb::Result<(), Error<Self::Error>>;

    #[cfg(feature = "ms")]
    /// Starts an Observation procedure, when the device is in Observer Role.
    ///
    /// The host enables scanning in the controller. The advertising reports are sent to the upper
    /// layer using standard LE Advertising Report Event. See Bluetooth Core v4.1, Vol. 2, part E,
    /// Ch. 7.7.65.2, LE Advertising Report Event.
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapStartObservationProcedure)
    /// event is generated.
    fn start_observation_procedure(
        &mut self,
        params: &ObservationProcedureParameters,
    ) -> nb::Result<(), Self::Error>;

    /// The command finds whether the device, whose address is specified in the command, is
    /// bonded. If the device is using a resolvable private address and it has been bonded, then the
    /// command will return [Success](hci::Status::Success).
    ///
    /// # Errors
    ///
    /// Only underlying communication errors are reported.
    ///
    /// # Generated events
    ///
    /// A [command complete](event::command::ReturnParameters::GapIsDeviceBonded) event is
    /// generated.
    fn is_device_bonded(&mut self, addr: hci::host::PeerAddrType) -> nb::Result<(), Self::Error>;
}

impl<'buf, N: ArrayLength<u8>> Commands for RadioCoprocessor<'buf, N> {
    type Error = ();

    fn set_nondiscoverable(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_SET_NONDISCOVERABLE, &[])
    }

    impl_validate_variable_length_params!(
        set_limited_discoverable<'a, 'b>,
        DiscoverableParameters<'a, 'b>,
        opcode::GAP_SET_LIMITED_DISCOVERABLE
    );

    impl_validate_variable_length_params!(
        set_discoverable<'a, 'b>,
        DiscoverableParameters<'a, 'b>,
        opcode::GAP_SET_DISCOVERABLE
    );

    impl_validate_params!(
        set_direct_connectable,
        DirectConnectableParameters,
        opcode::GAP_SET_DIRECT_CONNECTABLE
    );

    fn set_io_capability(&mut self, capability: IoCapability) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_SET_IO_CAPABILITY, &[capability as u8])
    }

    impl_validate_params!(
        set_authentication_requirement,
        AuthenticationRequirements,
        opcode::GAP_SET_AUTHENTICATION_REQUIREMENT
    );

    fn set_authorization_requirement(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        authorization_required: bool,
    ) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 3];
        LittleEndian::write_u16(&mut bytes[0..2], conn_handle.0);
        bytes[2] = authorization_required as u8;

        self.write_command(opcode::GAP_SET_AUTHORIZATION_REQUIREMENT, &bytes)
    }

    fn pass_key_response(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        pin: u32,
    ) -> nb::Result<(), Error<Self::Error>> {
        if pin > 999_999 {
            return Err(nb::Error::Other(Error::BadFixedPin(pin)));
        }

        let mut bytes = [0; 6];
        LittleEndian::write_u16(&mut bytes[0..2], conn_handle.0);
        LittleEndian::write_u32(&mut bytes[2..6], pin);

        self.write_command(opcode::GAP_PASS_KEY_RESPONSE, &bytes)
            .map_err(rewrap_error)
    }

    fn authorization_response(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        authorization: Authorization,
    ) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 3];
        LittleEndian::write_u16(&mut bytes[0..2], conn_handle.0);
        bytes[2] = authorization as u8;

        self.write_command(opcode::GAP_AUTHORIZATION_RESPONSE, &bytes)
    }

    fn init(
        &mut self,
        role: Role,
        privacy_enabled: bool,
        dev_name_characteristic_len: u8,
    ) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 3];
        bytes[0] = role.bits();
        bytes[1] = privacy_enabled as u8;
        bytes[2] = dev_name_characteristic_len as u8;

        self.write_command(opcode::GAP_INIT, &bytes)
    }

    fn set_nonconnectable(
        &mut self,
        advertising_type: AdvertisingType,
        address_type: AddressType,
    ) -> nb::Result<(), Error<Self::Error>> {
        match advertising_type {
            AdvertisingType::ScannableUndirected | AdvertisingType::NonConnectableUndirected => (),
            _ => {
                return Err(nb::Error::Other(Error::BadAdvertisingType(
                    advertising_type,
                )));
            }
        }

        self.write_command(
            opcode::GAP_SET_NONCONNECTABLE,
            &[advertising_type as u8, address_type as u8],
        )
        .map_err(rewrap_error)
    }

    fn set_undirected_connectable(
        &mut self,
        filter_policy: AdvertisingFilterPolicy,
        address_type: AddressType,
    ) -> nb::Result<(), Error<Self::Error>> {
        match filter_policy {
            AdvertisingFilterPolicy::AllowConnectionAndScan
            | AdvertisingFilterPolicy::WhiteListConnectionAndScan => (),
            _ => {
                return Err(nb::Error::Other(Error::BadAdvertisingFilterPolicy(
                    filter_policy,
                )));
            }
        }

        self.write_command(
            opcode::GAP_SET_UNDIRECTED_CONNECTABLE,
            &[filter_policy as u8, address_type as u8],
        )
        .map_err(rewrap_error)
    }

    impl_params!(
        peripheral_security_request,
        SecurityRequestParameters,
        opcode::GAP_PERIPHERAL_SECURITY_REQUEST
    );

    fn update_advertising_data(&mut self, data: &[u8]) -> nb::Result<(), Error<Self::Error>> {
        const MAX_LENGTH: usize = 31;
        if data.len() > MAX_LENGTH {
            return Err(nb::Error::Other(Error::BadAdvertisingDataLength(
                data.len(),
            )));
        }

        let mut bytes = [0; 1 + MAX_LENGTH];
        bytes[0] = data.len() as u8;
        bytes[1..=data.len()].copy_from_slice(data);

        self.write_command(opcode::GAP_UPDATE_ADVERTISING_DATA, &bytes[0..=data.len()])
            .map_err(rewrap_error)
    }

    fn delete_ad_type(&mut self, ad_type: AdvertisingDataType) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_DELETE_AD_TYPE, &[ad_type as u8])
    }

    fn get_security_level(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_GET_SECURITY_LEVEL, &[])
    }

    fn set_event_mask(&mut self, flags: EventFlags) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 2];
        LittleEndian::write_u16(&mut bytes, flags.bits());

        self.write_command(opcode::GAP_SET_EVENT_MASK, &bytes)
    }

    fn configure_white_list(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_CONFIGURE_WHITE_LIST, &[])
    }

    fn terminate(
        &mut self,
        conn_handle: hci::ConnectionHandle,
        reason: hci::Status<event::Status>,
    ) -> nb::Result<(), Error<Self::Error>> {
        match reason {
            hci::Status::AuthFailure
            | hci::Status::RemoteTerminationByUser
            | hci::Status::RemoteTerminationLowResources
            | hci::Status::RemoteTerminationPowerOff
            | hci::Status::UnsupportedRemoteFeature
            | hci::Status::PairingWithUnitKeyNotSupported
            | hci::Status::UnacceptableConnectionParameters => (),
            _ => return Err(nb::Error::Other(Error::BadTerminationReason(reason))),
        }

        let mut bytes = [0; 3];
        LittleEndian::write_u16(&mut bytes[0..2], conn_handle.0);
        bytes[2] = reason.into();

        self.write_command(opcode::GAP_TERMINATE, &bytes)
            .map_err(rewrap_error)
    }

    fn clear_security_database(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_CLEAR_SECURITY_DATABASE, &[])
    }

    #[cfg(not(feature = "ms"))]
    fn allow_rebond(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_ALLOW_REBOND, &[])
    }

    #[cfg(feature = "ms")]
    fn allow_rebond(&mut self, conn_handle: hci::ConnectionHandle) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 2];
        LittleEndian::write_u16(&mut bytes, conn_handle.0);
        self.write_command(opcode::GAP_ALLOW_REBOND, &bytes)
    }

    impl_params!(
        start_limited_discovery_procedure,
        DiscoveryProcedureParameters,
        opcode::GAP_START_LIMITED_DISCOVERY_PROCEDURE
    );

    impl_params!(
        start_general_discovery_procedure,
        DiscoveryProcedureParameters,
        opcode::GAP_START_GENERAL_DISCOVERY_PROCEDURE
    );

    impl_params!(
        start_name_discovery_procedure,
        NameDiscoveryProcedureParameters,
        opcode::GAP_START_NAME_DISCOVERY_PROCEDURE
    );

    impl_validate_variable_length_params!(
        start_auto_connection_establishment<'a>,
        AutoConnectionEstablishmentParameters<'a>,
        opcode::GAP_START_AUTO_CONNECTION_ESTABLISHMENT
    );

    impl_params!(
        start_general_connection_establishment,
        GeneralConnectionEstablishmentParameters,
        opcode::GAP_START_GENERAL_CONNECTION_ESTABLISHMENT
    );

    impl_validate_variable_length_params!(
        start_selective_connection_establishment<'a>,
        SelectiveConnectionEstablishmentParameters<'a>,
        opcode::GAP_START_SELECTIVE_CONNECTION_ESTABLISHMENT
    );
    impl_params!(
        create_connection,
        ConnectionParameters,
        opcode::GAP_CREATE_CONNECTION
    );

    fn terminate_procedure(&mut self, procedure: Procedure) -> nb::Result<(), Error<Self::Error>> {
        if procedure.is_empty() {
            return Err(nb::Error::Other(Error::NoProcedure));
        }

        self.write_command(opcode::GAP_TERMINATE_PROCEDURE, &[procedure.bits()])
            .map_err(rewrap_error)
    }

    impl_params!(
        start_connection_update,
        ConnectionUpdateParameters,
        opcode::GAP_START_CONNECTION_UPDATE
    );

    impl_params!(
        send_pairing_request,
        PairingRequest,
        opcode::GAP_SEND_PAIRING_REQUEST
    );

    fn resolve_private_address(&mut self, addr: hci::BdAddr) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_RESOLVE_PRIVATE_ADDRESS, &addr.0)
    }

    fn get_bonded_devices(&mut self) -> nb::Result<(), Self::Error> {
        self.write_command(opcode::GAP_GET_BONDED_DEVICES, &[])
    }

    #[cfg(feature = "ms")]
    impl_validate_variable_length_params!(
        set_broadcast_mode,
        BroadcastModeParameters,
        opcode::GAP_SET_BROADCAST_MODE
    );

    #[cfg(feature = "ms")]
    impl_params!(
        start_observation_procedure,
        ObservationProcedureParameters,
        opcode::GAP_START_OBSERVATION_PROCEDURE
    );

    fn is_device_bonded(&mut self, addr: hci::host::PeerAddrType) -> nb::Result<(), Self::Error> {
        let mut bytes = [0; 7];
        addr.copy_into_slice(&mut bytes);

        self.write_command(opcode::GAP_IS_DEVICE_BONDED, &bytes)
    }
}

/// Potential errors from parameter validation.
///
/// Before some commands are sent to the controller, the parameters are validated. This type
/// enumerates the potential validation errors. Must be specialized on the types of communication
/// errors.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Error<E> {
    /// For the [GAP Set Limited Discoverable](Commands::set_limited_discoverable) and
    /// [GAP Set Discoverable](Commands::set_discoverable) commands, the connection
    /// interval is inverted (the min is greater than the max).  Return the provided min as the
    /// first element, max as the second.
    BadConnectionInterval(Duration, Duration),

    /// For the [GAP Set Limited Discoverable](Commands::set_limited_discoverable) and
    /// [GAP Set Broadcast Mode](Commands::set_broadcast_mode) commands, the advertising
    /// type is disallowed.  Returns the invalid advertising type.
    BadAdvertisingType(crate::bluetooth::AdvertisingType),

    /// For the [GAP Set Limited Discoverable](Commands::set_limited_discoverable)
    /// command, the advertising interval is inverted (that is, the max is less than the
    /// min). Includes the provided range.
    BadAdvertisingInterval(Duration, Duration),

    /// For the [GAP Set Authentication
    /// Requirement](Commands::set_authentication_requirement) command, the encryption
    /// key size range is inverted (the max is less than the min). Includes the provided range.
    BadEncryptionKeySizeRange(u8, u8),

    /// For the [GAP Set Authentication
    /// Requirement](Commands::set_authentication_requirement) and [GAP Pass Key
    /// Response](Commands::pass_key_response) commands, the provided fixed pin is out of
    /// range (must be less than or equal to 999999).  Includes the provided PIN.
    BadFixedPin(u32),

    /// For the [GAP Set Undirected Connectable](Commands::set_undirected_connectable) command, the
    /// advertising filter policy is not one of the allowed values. Only
    /// [AllowConnectionAndScan](crate::AdvertisingFilterPolicy::AllowConnectionAndScan) and
    /// [WhiteListConnectionAndScan](crate::AdvertisingFilterPolicy::WhiteListConnectionAndScan) are
    /// allowed.
    BadAdvertisingFilterPolicy(crate::bluetooth::AdvertisingFilterPolicy),

    /// For the [GAP Update Advertising Data](Commands::update_advertising_data) and [GAP
    /// Set Broadcast Mode](Commands::set_broadcast_mode) commands, the advertising data
    /// is too long. It must be 31 bytes or less. The length of the provided data is returned.
    BadAdvertisingDataLength(usize),

    /// For the [GAP Terminate](Commands::terminate) command, the termination reason was
    /// not one of the allowed reason. The reason is returned.
    BadTerminationReason(hci::Status<event::Status>),

    /// For the [GAP Start Auto Connection
    /// Establishment](Commands::start_auto_connection_establishment) or [GAP Start
    /// Selective Connection
    /// Establishment](Commands::start_selective_connection_establishment) commands, the
    /// provided [white list](AutoConnectionEstablishmentParameters::white_list) has more than 33
    /// or 35 entries, respectively, which would cause the command to be longer than 255 bytes.
    ///
    /// For the [GAP Set Broadcast Mode](Commands::set_broadcast_mode), the provided
    /// [white list](BroadcastModeParameters::white_list) the maximum number of entries ranges
    /// from 31 to 35, depending on the length of the advertising data.
    WhiteListTooLong,

    /// For the [GAP Terminate Procedure](Commands::terminate_procedure) command, the
    /// provided bitfield had no bits set.
    NoProcedure,

    /// Underlying communication error.
    Comm(E),
}

fn rewrap_error<E>(e: nb::Error<E>) -> nb::Error<Error<E>> {
    match e {
        nb::Error::WouldBlock => nb::Error::WouldBlock,
        nb::Error::Other(c) => nb::Error::Other(Error::Comm(c)),
    }
}

fn to_conn_interval_value(d: Duration) -> u16 {
    // Connection interval value: T = N * 1.25 ms
    // We have T, we need to return N.
    // N = T / 1.25 ms
    //   = 4 * T / 5 ms
    let millis = (d.as_secs() * 1000) as u32 + d.subsec_millis();
    (4 * millis / 5) as u16
}

fn to_connection_length_value(d: Duration) -> u16 {
    // Connection interval value: T = N * 0.625 ms
    // We have T, we need to return N.
    // N = T / 0.625 ms
    //   = T / 625 us
    // 1600 = 1_000_000 / 625
    (1600 * d.as_secs() as u32 + (d.subsec_micros() / 625)) as u16
}

/// Parameters for the
/// [`set_limited_discoverable`](Commands::set_limited_discoverable) and
/// [`set_discoverable`](Commands::set_discoverable) commands.
pub struct DiscoverableParameters<'a, 'b> {
    /// Advertising method for the device.
    ///
    /// Must be
    /// [ConnectableUndirected](bluetooth_hci::host::AdvertisingType::ConnectableUndirected),
    /// [ScannableUndirected](bluetooth_hci::host::AdvertisingType::ScannableUndirected), or
    /// [NonConnectableUndirected](bluetooth_hci::host::AdvertisingType::NonConnectableUndirected).
    pub advertising_type: AdvertisingType,

    /// Range of advertising for non-directed advertising.
    ///
    /// If not provided, the GAP will use default values (1.28 seconds).
    ///
    /// Range for both limits: 20 ms to 10.24 seconds.  The second value must be greater than or
    /// equal to the first.
    pub advertising_interval: Option<(Duration, Duration)>,

    /// Address type for this device.
    pub address_type: OwnAddressType,

    /// Filter policy for this device.
    pub filter_policy: AdvertisingFilterPolicy,

    /// Name of the device.
    pub local_name: Option<LocalName<'a>>,

    /// Service UUID list as defined in the Bluetooth spec, v4.1, Vol 3, Part C, Section 11.
    ///
    /// Must be 31 bytes or fewer.
    pub advertising_data: &'b [u8],

    /// Expected length of the connection to the peripheral.
    pub conn_interval: (Option<Duration>, Option<Duration>),
}

impl<'a, 'b> DiscoverableParameters<'a, 'b> {
    // 14 fixed-size parameters, one parameter of up to 31 bytes, and one of up to 248 bytes.
    const MAX_LENGTH: usize = 14 + 31 + 248;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        match self.advertising_type {
            AdvertisingType::ConnectableUndirected
            | AdvertisingType::ScannableUndirected
            | AdvertisingType::NonConnectableUndirected => (),
            _ => return Err(Error::BadAdvertisingType(self.advertising_type)),
        }

        if let Some(interval) = self.advertising_interval {
            if interval.0 > interval.1 {
                return Err(Error::BadAdvertisingInterval(interval.0, interval.1));
            }
        }

        if let (Some(min), Some(max)) = self.conn_interval {
            if min > max {
                return Err(Error::BadConnectionInterval(min, max));
            }
        }

        Ok(())
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) -> usize {
        const NO_SPECIFIC_CONN_INTERVAL: u16 = 0x0000;

        let len = self.required_len();
        assert!(len <= bytes.len());

        let no_duration = Duration::from_secs(0);
        let no_interval = (no_duration, no_duration);

        bytes[0] = self.advertising_type as u8;
        LittleEndian::write_u16(
            &mut bytes[1..],
            to_connection_length_value(self.advertising_interval.unwrap_or(no_interval).0),
        );
        LittleEndian::write_u16(
            &mut bytes[3..],
            to_connection_length_value(self.advertising_interval.unwrap_or(no_interval).1),
        );
        bytes[5] = self.address_type as u8;
        bytes[6] = self.filter_policy as u8;
        let advertising_data_len_index = match self.local_name {
            None => {
                bytes[7] = 0;
                7
            }
            Some(LocalName::Shortened(name)) => {
                const AD_TYPE_SHORTENED_LOCAL_NAME: u8 = 0x08;
                bytes[7] = 1 + name.len() as u8;
                bytes[8] = AD_TYPE_SHORTENED_LOCAL_NAME;
                bytes[9..9 + name.len()].copy_from_slice(name);
                9 + name.len()
            }
            Some(LocalName::Complete(name)) => {
                const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
                bytes[7] = 1 + name.len() as u8;
                bytes[8] = AD_TYPE_COMPLETE_LOCAL_NAME;
                bytes[9..9 + name.len()].copy_from_slice(name);
                9 + name.len()
            }
        };
        bytes[advertising_data_len_index] = self.advertising_data.len() as u8;
        bytes[(advertising_data_len_index + 1)
            ..(advertising_data_len_index + 1 + self.advertising_data.len())]
            .copy_from_slice(self.advertising_data);
        let conn_interval_index = advertising_data_len_index + 1 + self.advertising_data.len();
        LittleEndian::write_u16(
            &mut bytes[conn_interval_index..],
            if self.conn_interval.0.is_some() {
                to_conn_interval_value(self.conn_interval.0.unwrap())
            } else {
                NO_SPECIFIC_CONN_INTERVAL
            },
        );
        LittleEndian::write_u16(
            &mut bytes[(conn_interval_index + 2)..],
            if self.conn_interval.1.is_some() {
                to_conn_interval_value(self.conn_interval.1.unwrap())
            } else {
                NO_SPECIFIC_CONN_INTERVAL
            },
        );

        len
    }

    fn required_len(&self) -> usize {
        let fixed_len = 13;

        fixed_len + self.name_len() + self.advertising_data.len()
    }

    fn name_len(&self) -> usize {
        // The serialized name includes one byte indicating the type of name. That byte is not
        // included if the name is empty.
        match self.local_name {
            Some(LocalName::Shortened(bytes)) | Some(LocalName::Complete(bytes)) => 1 + bytes.len(),
            None => 0,
        }
    }
}

/// Allowed types for the local name.
pub enum LocalName<'a> {
    /// The shortened local name.
    Shortened(&'a [u8]),

    /// The complete local name.
    Complete(&'a [u8]),
}

/// Parameters for the
/// [`set_direct_connectable`](Commands::set_direct_connectable) command.
pub struct DirectConnectableParameters {
    /// Address type of this device.
    pub own_address_type: OwnAddressType,

    #[cfg(feature = "ms")]
    /// Advertising method for the device.
    ///
    /// Must be
    /// [ConnectableDirectedHighDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedHighDutyCycle),
    /// or
    /// [ConnectableDirectedLowDutyCycle](bluetooth_hci::host::AdvertisingType::ConnectableDirectedLowDutyCycle).
    pub advertising_type: AdvertisingType,

    /// Initiator's Bluetooth address.
    pub initiator_address: BdAddrType,

    #[cfg(feature = "ms")]
    /// Range of advertising interval for advertising.
    ///
    /// Range for both limits: 20 ms to 10.24 seconds.  The second value must be greater than or
    /// equal to the first.
    pub advertising_interval: (Duration, Duration),
}

impl DirectConnectableParameters {
    #[cfg(not(feature = "ms"))]
    const LENGTH: usize = 8;

    #[cfg(feature = "ms")]
    const LENGTH: usize = 13;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        #[cfg(feature = "ms")]
        {
            const MIN_DURATION: Duration = Duration::from_millis(20);
            const MAX_DURATION: Duration = Duration::from_millis(10240);

            match self.advertising_type {
                AdvertisingType::ConnectableDirectedHighDutyCycle
                | AdvertisingType::ConnectableDirectedLowDutyCycle => (),
                _ => return Err(Error::BadAdvertisingType(self.advertising_type)),
            }

            if self.advertising_interval.0 < MIN_DURATION
                || self.advertising_interval.1 > MAX_DURATION
                || self.advertising_interval.0 > self.advertising_interval.1
            {
                return Err(Error::BadAdvertisingInterval(
                    self.advertising_interval.0,
                    self.advertising_interval.1,
                ));
            }
        }

        Ok(())
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), Self::LENGTH);

        bytes[0] = self.own_address_type as u8;

        #[cfg(not(feature = "ms"))]
        {
            self.initiator_address.copy_into_slice(&mut bytes[1..8]);
        }

        #[cfg(feature = "ms")]
        {
            bytes[1] = self.advertising_type as u8;
            self.initiator_address.copy_into_slice(&mut bytes[2..9]);
            LittleEndian::write_u16(
                &mut bytes[9..],
                to_connection_length_value(self.advertising_interval.0),
            );
            LittleEndian::write_u16(
                &mut bytes[11..],
                to_connection_length_value(self.advertising_interval.1),
            );
        }
    }
}

/// I/O capabilities available for the [GAP Set I/O
/// Capability](Commands::set_io_capability) command.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum IoCapability {
    /// Display Only
    Display = 0x00,
    /// Display yes/no
    DisplayConfirm = 0x01,
    /// Keyboard Only
    Keyboard = 0x02,
    /// No Input, no output
    None = 0x03,
    /// Keyboard display
    KeyboardDisplay = 0x04,
}

/// Parameters for the [GAP Set Authentication
/// Requirement](Commands::set_authentication_requirement) command.
pub struct AuthenticationRequirements {
    /// Is MITM (man-in-the-middle) protection required?
    pub mitm_protection_required: bool,

    /// Out-of-band authentication data.
    pub out_of_band_auth: OutOfBandAuthentication,

    /// Minimum and maximum size of the encryption key.
    pub encryption_key_size_range: (u8, u8),

    /// Pin to use during the pairing process.
    pub fixed_pin: Pin,

    /// Is bonding required?
    pub bonding_required: bool,
}

impl AuthenticationRequirements {
    const LENGTH: usize = 26;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        if self.encryption_key_size_range.0 > self.encryption_key_size_range.1 {
            return Err(Error::BadEncryptionKeySizeRange(
                self.encryption_key_size_range.0,
                self.encryption_key_size_range.1,
            ));
        }

        if let Pin::Fixed(pin) = self.fixed_pin {
            if pin > 999_999 {
                return Err(Error::BadFixedPin(pin));
            }
        }

        Ok(())
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), Self::LENGTH);

        bytes[0] = self.mitm_protection_required as u8;
        match self.out_of_band_auth {
            OutOfBandAuthentication::Disabled => {
                bytes[1..18].copy_from_slice(&[0; 17]);
            }
            OutOfBandAuthentication::Enabled(data) => {
                bytes[1] = 1;
                bytes[2..18].copy_from_slice(&data);
            }
        }

        bytes[18] = self.encryption_key_size_range.0;
        bytes[19] = self.encryption_key_size_range.1;

        match self.fixed_pin {
            Pin::Requested => {
                bytes[20] = 1;
                bytes[21..25].copy_from_slice(&[0; 4]);
            }
            Pin::Fixed(pin) => {
                bytes[20] = 0;
                LittleEndian::write_u32(&mut bytes[21..25], pin);
            }
        }

        bytes[25] = self.bonding_required as u8;
    }
}

/// Options for [`out_of_band_auth`](AuthenticationRequirements::out_of_band_auth).
pub enum OutOfBandAuthentication {
    /// Out Of Band authentication not enabled
    Disabled,
    /// Out Of Band authentication enabled; includes the OOB data.
    Enabled([u8; 16]),
}

/// Options for [`fixed_pin`](AuthenticationRequirements::fixed_pin).
pub enum Pin {
    /// Do not use fixed pin during the pairing process.  In this case, GAP will generate a [GAP
    /// Pass Key Request](event::BlueNRGEvent::GapPassKeyRequest) event to the host.
    Requested,

    /// Use a fixed pin during pairing. The provided value is used as the PIN, and must be 999999 or
    /// less.
    Fixed(u32),
}

/// Options for the [GAP Authorization Response](Commands::authorization_response).
#[repr(u8)]
pub enum Authorization {
    /// Accept the connection.
    Authorized = 0x01,
    /// Reject the connection.
    Rejected = 0x02,
}

bitflags! {
    /// Roles for a [GAP service](Commands::init).
    pub struct Role: u8 {
        /// Peripheral
        const PERIPHERAL = 0x01;
        /// Broadcaster
        const BROADCASTER = 0x02;
        /// Central Device
        const CENTRAL = 0x04;
        /// Observer
        const OBSERVER = 0x08;
    }
}

/// Indicates the type of address being used in the advertising packets, for the
/// [`set_nonconnectable`](Commands::set_nonconnectable).
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AddressType {
    /// Public device address.
    Public = 0x00,
    /// Static random device address.
    Random = 0x01,
    /// Controller generates Resolvable Private Address.
    ResolvablePrivate = 0x02,
    /// Controller generates Resolvable Private Address. based on the local IRK from resolving
    /// list.
    NonResolvablePrivate = 0x03,
}

/// Parameters for the [GAP Peripheral Security
/// Request](Commands::peripheral_security_request) parameters.
pub struct SecurityRequestParameters {
    /// Handle of the connection on which the peripheral security request will
    /// be sent (ignored in peripheral-only role).
    pub conn_handle: hci::ConnectionHandle,

    /// Is bonding required?
    pub bonding: bool,

    /// Is man-in-the-middle protection required?
    pub mitm_protection: bool,
}

impl SecurityRequestParameters {
    const LENGTH: usize = 4;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), Self::LENGTH);

        LittleEndian::write_u16(&mut bytes[0..2], self.conn_handle.0);
        bytes[2] = self.bonding as u8;
        bytes[3] = self.mitm_protection as u8;
    }
}

/// Available types of advertising data.
#[repr(u8)]
pub enum AdvertisingDataType {
    /// Flags
    Flags = 0x01,
    /// 16-bit service UUID
    Uuid16 = 0x02,
    /// Complete list of 16-bit service UUIDs
    UuidCompleteList16 = 0x03,
    /// 32-bit service UUID
    Uuid32 = 0x04,
    /// Complete list of 32-bit service UUIDs
    UuidCompleteList32 = 0x05,
    /// 128-bit service UUID
    Uuid128 = 0x06,
    /// Complete list of 128-bit service UUIDs.
    UuidCompleteList128 = 0x07,
    /// Shortened local name
    ShortenedLocalName = 0x08,
    /// Complete local name
    CompleteLocalName = 0x09,
    /// Transmitter power level
    TxPowerLevel = 0x0A,
    /// Serurity Manager TK Value
    SecurityManagerTkValue = 0x10,
    /// Serurity Manager out-of-band flags
    SecurityManagerOutOfBandFlags = 0x11,
    /// Connection interval
    PeripheralConnectionInterval = 0x12,
    /// Service solicitation list, 16-bit UUIDs
    SolicitUuidList16 = 0x14,
    /// Service solicitation list, 32-bit UUIDs
    SolicitUuidList32 = 0x15,
    /// Service data
    ServiceData = 0x16,
    /// Manufacturer-specific data
    ManufacturerSpecificData = 0xFF,
}

bitflags! {
    /// Event types for [GAP Set Event Mask](Commands::set_event_mask).
    pub struct EventFlags: u16 {
        /// [Limited Discoverable](::event::BlueNRGEvent::GapLimitedDiscoverableTimeout)
        const LIMITED_DISCOVERABLE_TIMEOUT = 0x0001;
        /// [Pairing Complete](::event::BlueNRGEvent::GapPairingComplete)
        const PAIRING_COMPLETE = 0x0002;
        /// [Pass Key Request](::event::BlueNRGEvent::GapPassKeyRequest)
        const PASS_KEY_REQUEST = 0x0004;
        /// [Authorization Request](::event::BlueNRGEvent::GapAuthorizationRequest)
        const AUTHORIZATION_REQUEST = 0x0008;
        /// [Peripheral Security Initiated](::event::BlueNRGEvent::GapPeripheralSecurityInitiated).
        const PERIPHERAL_SECURITY_INITIATED = 0x0010;
        /// [Bond Lost](::event::BlueNRGEvent::GapBondLost)
        const BOND_LOST = 0x0020;
    }
}

/// Parameters for the [GAP Limited
/// Discovery](Commands::start_limited_discovery_procedure) and [GAP General
/// Discovery](Commands::start_general_discovery_procedure) procedures.
pub struct DiscoveryProcedureParameters {
    /// Scanning window for the discovery procedure.
    pub scan_window: ScanWindow,

    /// Address type of this device.
    pub own_address_type: hci::host::OwnAddressType,

    /// If true, duplicate devices are filtered out.
    pub filter_duplicates: bool,
}

impl DiscoveryProcedureParameters {
    const LENGTH: usize = 6;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), Self::LENGTH);

        self.scan_window.copy_into_slice(&mut bytes[0..4]);
        bytes[4] = self.own_address_type as u8;
        bytes[5] = self.filter_duplicates as u8;
    }
}

/// Parameters for the [GAP Name Discovery](Commands::start_name_discovery_procedure)
/// procedure.
pub struct NameDiscoveryProcedureParameters {
    /// Scanning window for the discovery procedure.
    pub scan_window: ScanWindow,

    /// Address of the connected device
    pub peer_address: hci::host::PeerAddrType,

    /// Address type of this device.
    pub own_address_type: hci::host::OwnAddressType,

    /// Connection interval parameters.
    pub conn_interval: ConnectionInterval,

    /// Expected connection length
    pub expected_connection_length: ExpectedConnectionLength,
}

impl NameDiscoveryProcedureParameters {
    const LENGTH: usize = 24;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), Self::LENGTH);

        self.scan_window.copy_into_slice(&mut bytes[0..4]);
        self.peer_address.copy_into_slice(&mut bytes[4..11]);
        bytes[11] = self.own_address_type as u8;
        self.conn_interval.copy_into_slice(&mut bytes[12..20]);
        self.expected_connection_length
            .copy_into_slice(&mut bytes[20..24]);
    }
}

/// Parameters for the [GAP Start Auto Connection
/// Establishment](Commands::start_auto_connection_establishment) command.
pub struct AutoConnectionEstablishmentParameters<'a> {
    /// Scanning window for connection establishment.
    pub scan_window: ScanWindow,

    /// Address type of this device.
    pub own_address_type: hci::host::OwnAddressType,

    /// Connection interval parameters.
    pub conn_interval: ConnectionInterval,

    /// Expected connection length
    pub expected_connection_length: ExpectedConnectionLength,

    #[cfg(not(feature = "ms"))]
    /// Reconnection address is used as our address during the procedure. The address has been
    /// previously notified to the application through the
    /// [ReconnectionAddress](::event::Event::ReconnectionAddress) event.
    pub reconnection_address: Option<hci::BdAddr>,

    /// Addresses to white-list for automatic connection.
    pub white_list: &'a [hci::host::PeerAddrType],
}

impl<'a> AutoConnectionEstablishmentParameters<'a> {
    const MAX_LENGTH: usize = 249;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        const MAX_WHITE_LIST_LENGTH: usize = 33;
        if self.white_list.len() > MAX_WHITE_LIST_LENGTH - if cfg!(feature = "ms") { 0 } else { 1 }
        {
            return Err(Error::WhiteListTooLong);
        }

        Ok(())
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) -> usize {
        let len = self.len();
        assert!(bytes.len() >= len);

        self.scan_window.copy_into_slice(&mut bytes[0..4]);
        bytes[4] = self.own_address_type as u8;
        self.conn_interval.copy_into_slice(&mut bytes[5..13]);
        self.expected_connection_length
            .copy_into_slice(&mut bytes[13..17]);

        #[cfg(not(feature = "ms"))]
        {
            if let Some(addr) = self.reconnection_address {
                bytes[17] = 1;
                bytes[18..24].copy_from_slice(&addr.0);
            } else {
                bytes[17..24].copy_from_slice(&[0; 7]);
            }
        }

        let index = if cfg!(feature = "ms") { 17 } else { 24 };

        bytes[index] = self.white_list.len() as u8;
        let index = index + 1;
        for i in 0..self.white_list.len() {
            self.white_list[i].copy_into_slice(&mut bytes[(index + 7 * i)..(index + 7 * (i + 1))]);
        }

        len
    }

    fn len(&self) -> usize {
        let reconn_addr_len = if cfg!(feature = "ms") { 0 } else { 7 };
        18 + reconn_addr_len + 7 * self.white_list.len()
    }
}

/// Parameters for the [GAP Start General Connection
/// Establishment](Commands::start_general_connection_establishment) command.
pub struct GeneralConnectionEstablishmentParameters {
    /// Scanning window for connection establishment.
    pub scan_window: ScanWindow,

    /// Address type of this device.
    pub own_address_type: hci::host::OwnAddressType,

    /// If true, only report unique devices.
    pub filter_duplicates: bool,

    #[cfg(not(feature = "ms"))]
    /// Reconnection address is used as our address during the procedure. The address has been
    /// previously notified to the application through the
    /// [ReconnectionAddress](::event::Event::ReconnectionAddress) event.
    pub reconnection_address: Option<hci::BdAddr>,
}

impl GeneralConnectionEstablishmentParameters {
    #[cfg(not(feature = "ms"))]
    const LENGTH: usize = 13;

    #[cfg(feature = "ms")]
    const LENGTH: usize = 6;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert!(bytes.len() >= Self::LENGTH);

        self.scan_window.copy_into_slice(&mut bytes[0..4]);
        bytes[4] = self.own_address_type as u8;
        bytes[5] = self.filter_duplicates as u8;

        #[cfg(not(feature = "ms"))]
        {
            if let Some(addr) = self.reconnection_address {
                bytes[6] = 1;
                bytes[7..13].copy_from_slice(&addr.0)
            } else {
                bytes[6..13].copy_from_slice(&[0; 7])
            }
        }
    }
}

/// Parameters for the [GAP Start Selective Connection
/// Establishment](Commands::start_selective_connection_establishment) command.
pub struct SelectiveConnectionEstablishmentParameters<'a> {
    /// Type of scanning
    pub scan_type: hci::host::ScanType,

    /// Scanning window for connection establishment.
    pub scan_window: ScanWindow,

    /// Address type of this device.
    pub own_address_type: hci::host::OwnAddressType,

    /// If true, only report unique devices.
    pub filter_duplicates: bool,

    /// Addresses to white-list for automatic connection.
    pub white_list: &'a [hci::host::PeerAddrType],
}

impl<'a> SelectiveConnectionEstablishmentParameters<'a> {
    const MAX_LENGTH: usize = 252;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        const MAX_WHITE_LIST_LENGTH: usize = 35;
        if self.white_list.len() > MAX_WHITE_LIST_LENGTH {
            return Err(Error::WhiteListTooLong);
        }

        Ok(())
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) -> usize {
        let len = self.len();
        assert!(bytes.len() >= len);

        bytes[0] = self.scan_type as u8;
        self.scan_window.copy_into_slice(&mut bytes[1..5]);
        bytes[5] = self.own_address_type as u8;
        bytes[6] = self.filter_duplicates as u8;
        bytes[7] = self.white_list.len() as u8;
        for i in 0..self.white_list.len() {
            self.white_list[i].copy_into_slice(&mut bytes[(8 + 7 * i)..(8 + 7 * (i + 1))]);
        }

        len
    }

    fn len(&self) -> usize {
        8 + 7 * self.white_list.len()
    }
}

/// The parameters for the [GAP Name Discovery](Commands::start_name_discovery_procedure)
/// and [GAP Create Connection](Commands::create_connection) commands are identical.
pub type ConnectionParameters = NameDiscoveryProcedureParameters;

bitflags! {
    /// Roles for a [GAP service](Commands::init).
    pub struct Procedure: u8 {
        /// [Limited Discovery](Commands::start_limited_discovery_procedure) procedure.
        const LIMITED_DISCOVERY = 0x01;
        /// [General Discovery](Commands::start_general_discovery_procedure) procedure.
        const GENERAL_DISCOVERY = 0x02;
        /// [Name Discovery](Commands::start_name_discovery_procedure) procedure.
        const NAME_DISCOVERY = 0x04;
        /// [Auto Connection Establishment](Commands::auto_connection_establishment).
        const AUTO_CONNECTION_ESTABLISHMENT = 0x08;
        /// [General Connection
        /// Establishment](Commands::general_connection_establishment).
        const GENERAL_CONNECTION_ESTABLISHMENT = 0x10;
        /// [Selective Connection
        /// Establishment](Commands::selective_connection_establishment).
        const SELECTIVE_CONNECTION_ESTABLISHMENT = 0x20;
        /// [Direct Connection
        /// Establishment](Commands::direct_connection_establishment).
        const DIRECT_CONNECTION_ESTABLISHMENT = 0x40;
        /// [Observation](Commands::start_observation_procedure) procedure.
        const OBSERVATION = 0x80;
    }
}

/// Parameters for the [`start_connection_update`](Commands::start_connection_update)
/// command.
pub struct ConnectionUpdateParameters {
    /// Handle of the connection for which the update procedure has to be started.
    pub conn_handle: hci::ConnectionHandle,

    /// Updated connection interval for the connection.
    pub conn_interval: ConnectionInterval,

    /// Expected length of connection event needed for this connection.
    pub expected_connection_length: ExpectedConnectionLength,
}

impl ConnectionUpdateParameters {
    const LENGTH: usize = 14;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        LittleEndian::write_u16(&mut bytes[0..2], self.conn_handle.0);
        self.conn_interval.copy_into_slice(&mut bytes[2..10]);
        self.expected_connection_length
            .copy_into_slice(&mut bytes[10..14]);
    }
}

/// Parameters for the [`send_pairing_request`](Commands::send_pairing_request)
/// command.
pub struct PairingRequest {
    /// Handle of the connection for which the pairing request has to be sent.
    pub conn_handle: hci::ConnectionHandle,

    /// Whether pairing request has to be sent if the device is previously bonded or not. If false,
    /// the pairing request is sent only if the device has not previously bonded.
    pub force_rebond: bool,

    /// Whether the link has to be re-encrypted after the key exchange.
    pub force_reencrypt: bool,
}

impl PairingRequest {
    const LENGTH: usize = 3;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert!(bytes.len() >= Self::LENGTH);

        LittleEndian::write_u16(&mut bytes[0..2], self.conn_handle.0);
        bytes[2] = self.force_rebond as u8 | ((self.force_reencrypt as u8) << 1);
    }
}

#[cfg(feature = "ms")]
/// Parameters for the [GAP Set Broadcast Mode](Commands::set_broadcast_mode) command.
pub struct BroadcastModeParameters<'a, 'b> {
    /// Advertising type and interval.
    ///
    /// Only the [ScannableUndirected](hci::types::AdvertisingType::ScannableUndirected) and
    /// [NonConnectableUndirected](hci::types::AdvertisingType::NonConnectableUndirected).
    pub advertising_interval: hci::types::AdvertisingInterval,

    /// Type of this device's address.
    ///
    /// A privacy enabled device uses either a [resolvable private
    /// address](AddressType::ResolvablePrivate) or a [non-resolvable
    /// private](AddressType::NonResolvablePrivate) address.
    pub own_address_type: AddressType,

    /// Advertising data used by the device when advertising.
    ///
    /// Must be 31 bytes or fewer.
    pub advertising_data: &'a [u8],

    /// Addresses to add to the white list.
    ///
    /// Each address takes up 7 bytes (1 byte for the type, 6 for the address). The full length of
    /// this packet must not exceed 255 bytes. The white list must be less than a maximum of between
    /// 31 and 35 entries, depending on the length of
    /// [`advertising_data`](BroadcastModeParameters::advertising_data). Shorter advertising data
    /// allows more white list entries.
    pub white_list: &'b [hci::host::PeerAddrType],
}

#[cfg(feature = "ms")]
impl<'a, 'b> BroadcastModeParameters<'a, 'b> {
    const MAX_LENGTH: usize = 255;

    fn validate<E>(&self) -> Result<(), Error<E>> {
        const MAX_ADVERTISING_DATA_LENGTH: usize = 31;

        match self.advertising_interval.advertising_type() {
            hci::types::AdvertisingType::ScannableUndirected
            | hci::types::AdvertisingType::NonConnectableUndirected => (),
            other => return Err(Error::BadAdvertisingType(other)),
        }

        if self.advertising_data.len() > MAX_ADVERTISING_DATA_LENGTH {
            return Err(Error::BadAdvertisingDataLength(self.advertising_data.len()));
        }

        if self.len() > Self::MAX_LENGTH {
            return Err(Error::WhiteListTooLong);
        }

        Ok(())
    }

    fn len(&self) -> usize {
        5 + // advertising_interval
            1 + // own_address_type
            1 + self.advertising_data.len() + // advertising_data
            1 + 7 * self.white_list.len() // white_list
    }

    fn copy_into_slice(&self, bytes: &mut [u8]) -> usize {
        assert!(self.len() <= bytes.len());

        self.advertising_interval.copy_into_slice(&mut bytes[0..5]);
        bytes[5] = self.own_address_type as u8;
        bytes[6] = self.advertising_data.len() as u8;
        bytes[7..7 + self.advertising_data.len()].copy_from_slice(self.advertising_data);
        bytes[7 + self.advertising_data.len()] = self.white_list.len() as u8;

        let mut index = 8 + self.advertising_data.len();
        for addr in self.white_list.iter() {
            addr.copy_into_slice(&mut bytes[index..index + 7]);
            index += 7;
        }

        index
    }
}

#[cfg(feature = "ms")]
/// Parameters for the [GAP Start Observation Procedure](Commands::start_observation_procedure)
/// command.
pub struct ObservationProcedureParameters {
    /// Scanning window.
    pub scan_window: hci::types::ScanWindow,

    /// Active or passive scanning
    pub scan_type: hci::host::ScanType,

    /// Address type of this device.
    pub own_address_type: AddressType,

    /// If true, do not report duplicate events in the [advertising
    /// report](hci::event::Event::LeAdvertisingReport).
    pub filter_duplicates: bool,
}

#[cfg(feature = "ms")]
impl ObservationProcedureParameters {
    const LENGTH: usize = 7;

    fn copy_into_slice(&self, bytes: &mut [u8]) {
        assert!(bytes.len() >= Self::LENGTH);

        self.scan_window.copy_into_slice(&mut bytes[0..4]);
        bytes[4] = self.scan_type as u8;
        bytes[5] = self.own_address_type as u8;
        bytes[6] = self.filter_duplicates as u8;
    }
}
