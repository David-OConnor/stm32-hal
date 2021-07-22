use crate::ipcc::Ipcc;
use crate::tl_mbox::cmd::CmdPacket;
use crate::tl_mbox::consts::TlPacketType;
use crate::tl_mbox::sys;
use crate::tl_mbox::{TL_CS_EVT_SIZE, TL_EVT_HEADER_SIZE, TL_PACKET_HEADER_SIZE, TL_SYS_TABLE};

pub const SHCI_OPCODE_BLE_INIT: u16 = 0xfc66;

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct ShciBleInitCmdParam {
    /// NOT USED CURRENTLY
    pub p_ble_buffer_address: u32,

    /// Size of the Buffer allocated in pBleBufferAddress
    pub ble_buffer_size: u32,

    pub num_attr_record: u16,
    pub num_attr_serv: u16,
    pub attr_value_arr_size: u16,
    pub num_of_links: u8,
    pub extended_packet_length_enable: u8,
    pub pr_write_list_size: u8,
    pub mb_lock_count: u8,

    pub att_mtu: u16,
    pub slave_sca: u16,
    pub master_sca: u8,
    pub ls_source: u8,
    pub max_conn_event_length: u32,
    pub hs_startup_time: u16,
    pub viterbi_enable: u8,
    pub ll_only: u8,
    pub hw_version: u8,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C, packed)]
pub struct ShciHeader {
    meta_data: [u32; 3],
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct ShciBleInitCmdPacket {
    header: ShciHeader,
    param: ShciBleInitCmdParam,
}

pub const TL_BLEEVT_CS_PACKET_SIZE: usize = TL_EVT_HEADER_SIZE + TL_CS_EVT_SIZE;
#[allow(dead_code)] // Not used currently but reserved
const TL_BLEEVT_CS_BUFFER_SIZE: usize = TL_PACKET_HEADER_SIZE + TL_BLEEVT_CS_PACKET_SIZE;

pub fn shci_ble_init(ipcc: &mut Ipcc, param: ShciBleInitCmdParam) {
    let mut packet = ShciBleInitCmdPacket {
        header: ShciHeader::default(),
        param,
    };

    let packet_ptr: *mut _ = &mut packet;

    unsafe {
        let cmd_ptr: *mut CmdPacket = packet_ptr.cast();

        (*cmd_ptr).cmdserial.cmd.cmd_code = SHCI_OPCODE_BLE_INIT;
        (*cmd_ptr).cmdserial.cmd.payload_len = core::mem::size_of::<ShciBleInitCmdParam>() as u8;

        let mut p_cmd_buffer = &mut *(*TL_SYS_TABLE.as_mut_ptr()).pcmd_buffer;
        core::ptr::write(p_cmd_buffer, *cmd_ptr);

        (*p_cmd_buffer).cmdserial.ty = TlPacketType::SysCmd as u8;

        sys::send_cmd(ipcc);
    }
}
