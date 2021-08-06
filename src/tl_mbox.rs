//! This, and all files in the `tl_mbox` folder are from
//! [Eupn's stm32wb-hal](https://github.com/eupn/stm32wb-hal)

use core::mem::MaybeUninit;

use bit_field::BitField;
use heapless::spsc;

pub mod ble;
mod channels;
pub mod cmd;
pub mod consts;
pub mod evt;
pub mod lhci;
pub mod mm;
pub mod shci;
pub mod sys;
mod unsafe_linked_list;

use crate::{
    ipcc::Ipcc,
    tl_mbox::{
        cmd::{AclDataPacket, CmdPacket},
        evt::EvtBox,
    },
};

use unsafe_linked_list::LinkedListNode;

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct SafeBootInfoTable {
    version: u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct RssInfoTable {
    version: u32,
    memory_size: u32,
    rss_info: u32,
}

/**
 * Version
 * [0:3]   = Build - 0: Untracked - 15:Released - x: Tracked version
 * [4:7]   = branch - 0: Mass Market - x: ...
 * [8:15]  = Subversion
 * [16:23] = Version minor
 * [24:31] = Version major
 *
 * Memory Size
 * [0:7]   = Flash ( Number of 4k sector)
 * [8:15]  = Reserved ( Shall be set to 0 - may be used as flash extension )
 * [16:23] = SRAM2b ( Number of 1k sector)
 * [24:31] = SRAM2a ( Number of 1k sector)
 */
#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct WirelessFwInfoTable {
    version: u32,
    memory_size: u32,
    thread_info: u32,
    ble_info: u32,
}

impl WirelessFwInfoTable {
    pub fn version_major(&self) -> u8 {
        let version = self.clone().version;
        (version.get_bits(24..31) & 0xff) as u8
    }

    pub fn version_minor(&self) -> u8 {
        let version = self.clone().version;
        (version.clone().get_bits(16..23) & 0xff) as u8
    }

    pub fn subversion(&self) -> u8 {
        let version = self.clone().version;
        (version.clone().get_bits(8..15) & 0xff) as u8
    }

    /// Size of FLASH, expressed in number of 4K sectors.
    pub fn flash_size(&self) -> u8 {
        let memory_size = self.clone().memory_size;
        (memory_size.clone().get_bits(0..7) & 0xff) as u8
    }

    /// Size of SRAM2a, expressed in number of 1K sectors.
    pub fn sram2a_size(&self) -> u8 {
        let memory_size = self.clone().memory_size;
        (memory_size.clone().get_bits(24..31) & 0xff) as u8
    }

    /// Size of SRAM2b, expressed in number of 1K sectors.
    pub fn sram2b_size(&self) -> u8 {
        let memory_size = self.clone().memory_size;
        (memory_size.clone().get_bits(16..23) & 0xff) as u8
    }
}

#[derive(Debug, Clone)]
#[repr(C, align(4))]
pub struct DeviceInfoTable {
    pub safe_boot_info_table: SafeBootInfoTable,
    pub rss_info_table: RssInfoTable,
    pub wireless_fw_info_table: WirelessFwInfoTable,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct BleTable {
    pcmd_buffer: *mut CmdPacket,
    pcs_buffer: *const u8,
    pevt_queue: *const u8,
    phci_acl_data_buffer: *mut AclDataPacket,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct ThreadTable {
    nostack_buffer: *const u8,
    clicmdrsp_buffer: *const u8,
    otcmdrsp_buffer: *const u8,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct SysTable {
    pcmd_buffer: *mut CmdPacket,
    sys_queue: *const LinkedListNode,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct MemManagerTable {
    spare_ble_buffer: *const u8,
    spare_sys_buffer: *const u8,

    blepool: *const u8,
    blepoolsize: u32,

    pevt_free_buffer_queue: *mut LinkedListNode,

    traces_evt_pool: *const u8,
    tracespoolsize: u32,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct TracesTable {
    traces_queue: *const u8,
}

#[derive(Debug)]
#[repr(C, align(4))]
struct Mac802154Table {
    p_cmdrsp_buffer: *const u8,
    p_notack_buffer: *const u8,
    evt_queue: *const u8,
}

/// Reference table. Contains pointers to all other tables.
#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RefTable {
    pub device_info_table: *const DeviceInfoTable,
    ble_table: *const BleTable,
    thread_table: *const ThreadTable,
    sys_table: *const SysTable,
    mem_manager_table: *const MemManagerTable,
    traces_table: *const TracesTable,
    mac_802_15_4_table: *const Mac802154Table,
}

#[link_section = "TL_REF_TABLE"]
pub static mut TL_REF_TABLE: MaybeUninit<RefTable> = MaybeUninit::uninit();

#[link_section = "TL_DEVICE_INFO_TABLE"]
static mut TL_DEVICE_INFO_TABLE: MaybeUninit<DeviceInfoTable> = MaybeUninit::uninit();

#[link_section = "TL_BLE_TABLE"]
static mut TL_BLE_TABLE: MaybeUninit<BleTable> = MaybeUninit::uninit();

#[link_section = "TL_THREAD_TABLE"]
static mut TL_THREAD_TABLE: MaybeUninit<ThreadTable> = MaybeUninit::uninit();

#[link_section = "TL_SYS_TABLE"]
static mut TL_SYS_TABLE: MaybeUninit<SysTable> = MaybeUninit::uninit();

#[link_section = "TL_MEM_MANAGER_TABLE"]
static mut TL_MEM_MANAGER_TABLE: MaybeUninit<MemManagerTable> = MaybeUninit::uninit();

#[link_section = "TL_TRACES_TABLE"]
static mut TL_TRACES_TABLE: MaybeUninit<TracesTable> = MaybeUninit::uninit();

#[link_section = "TL_MAC_802_15_4_TABLE"]
static mut TL_MAC_802_15_4_TABLE: MaybeUninit<Mac802154Table> = MaybeUninit::uninit();

#[link_section = "FREE_BUF_QUEUE"]
static mut FREE_BUF_QUEUE: MaybeUninit<LinkedListNode> = MaybeUninit::uninit();

// Not in shared RAM
static mut LOCAL_FREE_BUF_QUEUE: MaybeUninit<LinkedListNode> = MaybeUninit::uninit();

#[allow(dead_code)] // Not used currently but reserved
#[link_section = "TRACES_EVT_QUEUE"]
static mut TRACES_EVT_QUEUE: MaybeUninit<LinkedListNode> = MaybeUninit::uninit();

type PacketHeader = unsafe_linked_list::LinkedListNode;

const TL_PACKET_HEADER_SIZE: usize = core::mem::size_of::<PacketHeader>();
const TL_EVT_HEADER_SIZE: usize = 3;
const TL_CS_EVT_SIZE: usize = core::mem::size_of::<evt::CsEvt>();

#[link_section = "CS_BUFFER"]
static mut CS_BUFFER: MaybeUninit<
    [u8; TL_PACKET_HEADER_SIZE + TL_EVT_HEADER_SIZE + TL_CS_EVT_SIZE],
> = MaybeUninit::uninit();

#[link_section = "EVT_QUEUE"]
static mut EVT_QUEUE: MaybeUninit<LinkedListNode> = MaybeUninit::uninit();

#[link_section = "SYSTEM_EVT_QUEUE"]
static mut SYSTEM_EVT_QUEUE: MaybeUninit<LinkedListNode> = MaybeUninit::uninit();

#[link_section = "SYS_CMD_BUF"]
pub static mut SYS_CMD_BUF: MaybeUninit<CmdPacket> = MaybeUninit::uninit();

/**
 * Queue length of BLE Event
 * This parameter defines the number of asynchronous events that can be stored in the HCI layer before
 * being reported to the application. When a command is sent to the BLE core coprocessor, the HCI layer
 * is waiting for the event with the Num_HCI_Command_Packets set to 1. The receive queue shall be large
 * enough to store all asynchronous events received in between.
 * When CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE is set to 27, this allow to store three 255 bytes long asynchronous events
 * between the HCI command and its event.
 * This parameter depends on the value given to CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE. When the queue size is to small,
 * the system may hang if the queue is full with asynchronous events and the HCI layer is still waiting
 * for a CC/CS event, In that case, the notification TL_BLE_HCI_ToNot() is called to indicate
 * to the application a HCI command did not receive its command event within 30s (Default HCI Timeout).
 */
const CFG_TLBLE_EVT_QUEUE_LENGTH: usize = 5;
const CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE: usize = 255;
const TL_BLE_EVENT_FRAME_SIZE: usize = TL_EVT_HEADER_SIZE + CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE;

const fn divc(x: usize, y: usize) -> usize {
    ((x) + (y) - 1) / (y)
}

const POOL_SIZE: usize =
    CFG_TLBLE_EVT_QUEUE_LENGTH * 4 * divc(TL_PACKET_HEADER_SIZE + TL_BLE_EVENT_FRAME_SIZE, 4);

#[link_section = "EVT_POOL"]
static mut EVT_POOL: MaybeUninit<[u8; POOL_SIZE]> = MaybeUninit::uninit();

#[link_section = "SYS_SPARE_EVT_BUF"]
static mut SYS_SPARE_EVT_BUF: MaybeUninit<[u8; TL_PACKET_HEADER_SIZE + TL_EVT_HEADER_SIZE + 255]> =
    MaybeUninit::uninit();

#[link_section = "BLE_SPARE_EVT_BUF"]
static mut BLE_SPARE_EVT_BUF: MaybeUninit<[u8; TL_PACKET_HEADER_SIZE + TL_EVT_HEADER_SIZE + 255]> =
    MaybeUninit::uninit();

#[link_section = "BLE_CMD_BUFFER"]
static mut BLE_CMD_BUFFER: MaybeUninit<CmdPacket> = MaybeUninit::uninit();

#[link_section = "HCI_ACL_DATA_BUFFER"]
//                                 fuck these "magic" numbers from ST ---v---v
static mut HCI_ACL_DATA_BUFFER: MaybeUninit<[u8; TL_PACKET_HEADER_SIZE + 5 + 251]> =
    MaybeUninit::uninit();

pub type HeaplessEvtQueue = spsc::Queue<EvtBox, heapless::consts::U32, u8, spsc::SingleCore>;

pub struct TlMbox {
    sys: sys::Sys,
    ble: ble::Ble,
    _mm: mm::MemoryManager,

    /// Current event that is produced during IPCC IRQ handler execution on SYS channel
    evt_queue: HeaplessEvtQueue,

    /// Last received Command Complete event.
    last_cc_evt: Option<evt::CcEvt>,
}

impl TlMbox {
    /// Initializes low-level transport between CPU1 and BLE stack on CPU2.
    pub fn tl_init(ipcc: &mut Ipcc) -> TlMbox {
        // Populate reference table with pointers in the shared memory
        unsafe {
            TL_REF_TABLE = MaybeUninit::new(RefTable {
                device_info_table: TL_DEVICE_INFO_TABLE.as_ptr(),
                ble_table: TL_BLE_TABLE.as_ptr(),
                thread_table: TL_THREAD_TABLE.as_ptr(),
                sys_table: TL_SYS_TABLE.as_ptr(),
                mem_manager_table: TL_MEM_MANAGER_TABLE.as_ptr(),
                traces_table: TL_TRACES_TABLE.as_ptr(),
                mac_802_15_4_table: TL_MAC_802_15_4_TABLE.as_ptr(),
            });

            TL_SYS_TABLE = MaybeUninit::zeroed();
            TL_DEVICE_INFO_TABLE = MaybeUninit::zeroed();
            TL_BLE_TABLE = MaybeUninit::zeroed();
            TL_THREAD_TABLE = MaybeUninit::zeroed();
            TL_MEM_MANAGER_TABLE = MaybeUninit::zeroed();
            TL_TRACES_TABLE = MaybeUninit::zeroed();
            TL_MAC_802_15_4_TABLE = MaybeUninit::zeroed();

            EVT_POOL = MaybeUninit::zeroed();
            SYS_SPARE_EVT_BUF = MaybeUninit::zeroed();
            BLE_SPARE_EVT_BUF = MaybeUninit::zeroed();

            CS_BUFFER = MaybeUninit::zeroed();
            BLE_CMD_BUFFER = MaybeUninit::zeroed();
            HCI_ACL_DATA_BUFFER = MaybeUninit::zeroed();
        }

        let sys = sys::Sys::new(ipcc);
        let ble = ble::Ble::new(ipcc);
        let mm = mm::MemoryManager::new();

        let evt_queue = unsafe { heapless::spsc::Queue::u8_sc() };

        TlMbox {
            sys,
            ble,
            _mm: mm,
            evt_queue,
            last_cc_evt: None,
        }
    }

    pub fn interrupt_ipcc_rx_handler(&mut self, ipcc: &mut Ipcc) {
        if ipcc.is_rx_pending(channels::cpu2::IPCC_SYSTEM_EVENT_CHANNEL) {
            self.sys.evt_handler(ipcc, &mut self.evt_queue);
        } else if ipcc.is_rx_pending(channels::cpu2::IPCC_THREAD_NOTIFICATION_ACK_CHANNEL) {
            todo!()
        } else if ipcc.is_rx_pending(channels::cpu2::IPCC_BLE_EVENT_CHANNEL) {
            self.ble.evt_handler(ipcc, &mut self.evt_queue);
        } else if ipcc.is_rx_pending(channels::cpu2::IPCC_TRACES_CHANNEL) {
            todo!()
        } else if ipcc.is_rx_pending(channels::cpu2::IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL) {
            todo!()
        }
    }

    pub fn interrupt_ipcc_tx_handler(&mut self, ipcc: &mut Ipcc) {
        if ipcc.is_tx_pending(channels::cpu1::IPCC_SYSTEM_CMD_RSP_CHANNEL) {
            self.last_cc_evt = Some(self.sys.cmd_evt_handler(ipcc));
        } else if ipcc.is_tx_pending(channels::cpu1::IPCC_THREAD_OT_CMD_RSP_CHANNEL) {
            todo!()
        } else if ipcc.is_tx_pending(channels::cpu1::IPCC_MM_RELEASE_BUFFER_CHANNEL) {
            mm::free_buf_handler(ipcc);
        } else if ipcc.is_tx_pending(channels::cpu1::IPCC_HCI_ACL_DATA_CHANNEL) {
            self.ble.acl_data_handler(ipcc);
        }
    }

    /// Returns CPU2 wireless firmware information (if present).
    pub fn wireless_fw_info(&self) -> Option<WirelessFwInfoTable> {
        let info = unsafe { &(*(*TL_REF_TABLE.as_ptr()).device_info_table).wireless_fw_info_table };

        // Zero version indicates that CPU2 wasn't active and didn't fill the information table
        if info.version != 0 {
            Some(info.clone())
        } else {
            None
        }
    }

    /// Picks single `EvtBox` from internal event queue.
    ///
    /// Internal event queue is populated in IPCC RX IRQ handler.
    pub fn dequeue_event(&mut self) -> Option<EvtBox> {
        self.evt_queue.dequeue()
    }

    /// Retrieves last Command Complete event and removes it from mailbox.
    pub fn pop_last_cc_evt(&mut self) -> Option<evt::CcEvt> {
        self.last_cc_evt.and_then(|evt| {
            self.last_cc_evt = None; // Remove event
            Some(evt)
        })
    }
}
