//! IPCC SYS (System) channel routines.
use core::mem::MaybeUninit;

use super::channels;
use crate::{
    ipcc::{Core, Ipcc},
    tl_mbox::{
        cmd::{CmdPacket, CmdSerial},
        evt,
        evt::{CcEvt, EvtBox, EvtSerial},
        unsafe_linked_list::{LST_init_head, LST_is_empty, LST_remove_head, LinkedListNode},
        HeaplessEvtQueue, SysTable, SYSTEM_EVT_QUEUE, SYS_CMD_BUF, TL_SYS_TABLE,
    },
};

pub type SysCallback = fn();

pub struct Sys {}

impl Sys {
    pub fn new(ipcc: &mut Ipcc) -> Self {
        unsafe {
            LST_init_head(SYSTEM_EVT_QUEUE.as_mut_ptr());

            TL_SYS_TABLE = MaybeUninit::new(SysTable {
                pcmd_buffer: SYS_CMD_BUF.as_mut_ptr(),
                sys_queue: SYSTEM_EVT_QUEUE.as_ptr(),
            });
        }

        ipcc.set_rx_channel(Core::C1, channels::cpu2::IPCC_SYSTEM_EVENT_CHANNEL, true);

        Sys {}
    }

    pub fn cmd_evt_handler(&self, ipcc: &mut Ipcc) -> CcEvt {
        ipcc.set_tx_channel(Core::C1, channels::cpu1::IPCC_SYSTEM_CMD_RSP_CHANNEL, false);

        // ST's command response data structure is really convoluted.
        //
        // For command response events on SYS channel,
        // the header is missing and one should:
        // 1. Interpret the content of CMD_BUFFER as CmdPacket
        // 2. Access CmdPacket's cmdserial field and interpret its content as EvtSerial
        // 3. Access EvtSerial's evt field (as Evt) and interpret its payload as CcEvt type.
        // 4. CcEvt type is the actual SHCI response.
        unsafe {
            let pcmd: *const CmdPacket = (&*TL_SYS_TABLE.as_ptr()).pcmd_buffer;
            let cmd_serial: *const CmdSerial = &(*pcmd).cmdserial;
            let evt_serial: *const EvtSerial = cmd_serial.cast();
            let cc: *const CcEvt = (*evt_serial).evt.payload.as_ptr().cast();
            *cc
        }
    }

    pub fn evt_handler(&self, ipcc: &mut Ipcc, queue: &mut HeaplessEvtQueue) {
        unsafe {
            let mut node_ptr: *mut LinkedListNode = core::ptr::null_mut();
            let node_ptr_ptr: *mut *mut LinkedListNode = &mut node_ptr;

            while !LST_is_empty(SYSTEM_EVT_QUEUE.as_mut_ptr()) {
                LST_remove_head(SYSTEM_EVT_QUEUE.as_mut_ptr(), node_ptr_ptr);

                let event: *mut evt::EvtPacket = node_ptr.cast();
                let event = EvtBox::new(event);

                queue.enqueue(event).unwrap();
            }
        }

        ipcc.clear_flag_channel(Core::C1, channels::cpu2::IPCC_SYSTEM_EVENT_CHANNEL);
    }
}

pub fn send_cmd(ipcc: &mut Ipcc) {
    ipcc.set_flag_channel(Core::C1, channels::cpu1::IPCC_SYSTEM_CMD_RSP_CHANNEL);
    ipcc.set_tx_channel(Core::C1, channels::cpu1::IPCC_SYSTEM_CMD_RSP_CHANNEL, true);
}
