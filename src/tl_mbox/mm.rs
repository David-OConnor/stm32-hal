//! MemoryManager routines.

use core::mem::MaybeUninit;

use super::channels::cpu1::IPCC_MM_RELEASE_BUFFER_CHANNEL;
use super::unsafe_linked_list::{
    LST_init_head, LST_insert_tail, LST_is_empty, LST_remove_head, LinkedListNode,
};
use super::{
    MemManagerTable, BLE_SPARE_EVT_BUF, EVT_POOL, FREE_BUF_QUEUE, LOCAL_FREE_BUF_QUEUE, POOL_SIZE,
    SYS_SPARE_EVT_BUF, TL_MEM_MANAGER_TABLE,
};

use crate::{
    ipcc::{Core, Ipcc},
    tl_mbox::{evt::EvtPacket, TL_REF_TABLE},
};

pub(super) struct MemoryManager {}

impl MemoryManager {
    pub fn new() -> Self {
        // Configure MemManager
        unsafe {
            LST_init_head(FREE_BUF_QUEUE.as_mut_ptr());
            LST_init_head(LOCAL_FREE_BUF_QUEUE.as_mut_ptr());

            TL_MEM_MANAGER_TABLE = MaybeUninit::new(MemManagerTable {
                spare_ble_buffer: BLE_SPARE_EVT_BUF.as_ptr().cast(),
                spare_sys_buffer: SYS_SPARE_EVT_BUF.as_ptr().cast(),
                blepool: EVT_POOL.as_ptr().cast(),
                blepoolsize: POOL_SIZE as u32,
                pevt_free_buffer_queue: FREE_BUF_QUEUE.as_mut_ptr(),
                traces_evt_pool: core::ptr::null(),
                tracespoolsize: 0,
            });
        }

        MemoryManager {}
    }
}

pub fn evt_drop(evt: *mut EvtPacket, ipcc: &mut Ipcc) {
    unsafe {
        let list_node: *mut _ = evt.cast();

        LST_insert_tail(LOCAL_FREE_BUF_QUEUE.as_mut_ptr(), list_node);

        let channel_is_busy = !ipcc.channel_is_free(Core::C1, IPCC_MM_RELEASE_BUFFER_CHANNEL);

        // Postpone event buffer freeing to IPCC interrupt handler
        if channel_is_busy {
            ipcc.set_tx_channel(Core::C1, IPCC_MM_RELEASE_BUFFER_CHANNEL, true);
        } else {
            send_free_buf();
            ipcc.set_flag_channel(Core::C1, IPCC_MM_RELEASE_BUFFER_CHANNEL);
        }
    }
}

/// Gives free event buffers back to the CPU2 from local buffer queue.
pub fn send_free_buf() {
    unsafe {
        let mut node_ptr: *mut LinkedListNode = core::ptr::null_mut();
        let node_ptr_ptr: *mut *mut LinkedListNode = &mut node_ptr;

        while !LST_is_empty(LOCAL_FREE_BUF_QUEUE.as_mut_ptr()) {
            LST_remove_head(LOCAL_FREE_BUF_QUEUE.as_mut_ptr(), node_ptr_ptr);
            LST_insert_tail(
                (&*(*TL_REF_TABLE.as_ptr()).mem_manager_table).pevt_free_buffer_queue,
                node_ptr,
            );
        }
    }
}

/// Free buffer channel interrupt handler.
pub fn free_buf_handler(ipcc: &mut Ipcc) {
    ipcc.set_tx_channel(Core::C1, IPCC_MM_RELEASE_BUFFER_CHANNEL, false);
    send_free_buf();
    ipcc.set_flag_channel(Core::C1, IPCC_MM_RELEASE_BUFFER_CHANNEL);
}
