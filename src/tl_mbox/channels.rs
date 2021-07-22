pub mod cpu1 {
    use crate::ipcc::IpccChannel;

    pub const IPCC_BLE_CMD_CHANNEL: IpccChannel = IpccChannel::C1;
    pub const IPCC_SYSTEM_CMD_RSP_CHANNEL: IpccChannel = IpccChannel::C2;
    pub const IPCC_THREAD_OT_CMD_RSP_CHANNEL: IpccChannel = IpccChannel::C3;
    #[allow(dead_code)] // Not used currently but reserved
    pub const IPCC_MAC_802_15_4_CMD_RSP_CHANNEL: IpccChannel = IpccChannel::C3;
    #[allow(dead_code)] // Not used currently but reserved
    pub const IPCC_THREAD_CLI_CMD_CHANNEL: IpccChannel = IpccChannel::C5;
    pub const IPCC_MM_RELEASE_BUFFER_CHANNEL: IpccChannel = IpccChannel::C4;
    pub const IPCC_HCI_ACL_DATA_CHANNEL: IpccChannel = IpccChannel::C6;
}

pub mod cpu2 {
    use crate::ipcc::IpccChannel;

    pub const IPCC_BLE_EVENT_CHANNEL: IpccChannel = IpccChannel::C1;
    pub const IPCC_SYSTEM_EVENT_CHANNEL: IpccChannel = IpccChannel::C2;
    pub const IPCC_THREAD_NOTIFICATION_ACK_CHANNEL: IpccChannel = IpccChannel::C3;
    #[allow(dead_code)] // Not used currently but reserved
    pub const IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL: IpccChannel = IpccChannel::C3;
    pub const IPCC_TRACES_CHANNEL: IpccChannel = IpccChannel::C4;
    pub const IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL: IpccChannel = IpccChannel::C5;
}
