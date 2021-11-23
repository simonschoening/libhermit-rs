use tock_registers::interfaces::*;
use tock_registers::registers::*;
use tock_registers::{register_bitfields, register_structs};

use crate::arch::mm::VirtAddr;

register_structs! {
	/// Register offsets
	Registers {
		(0x000 => config: ReadOnly<u32, Config::Register>),
        (0x004 => _reserved1),
		(0x008 => way_enable: ReadWrite<u8>),
		(0x009 => _reserved2),
        (0x800 => way_mask_0: ReadWrite<u16>),
        (0x802 => _reserved3),
		(0x1000 => @END),
	}
}

register_bitfields! [
	// First parameter is the register width. Can be u8, u16, u32, or u64.
	u32,

	Config [
		LGBBOCKBYTES	OFFSET(24) NUMBITS(8) [],
		LGSETS	OFFSET(16) NUMBITS(8) [],
		WAYS	OFFSET(8) NUMBITS(8) [],
		BANKS	OFFSET(0) NUMBITS(8) [],
	],
];

/// Init SiFive Level 2 Cache Controller
pub fn init_cache(base: usize) {
    let cachec = VirtAddr(base as u64).as_mut_ptr::<Registers>();
    unsafe{
        let config = (*cachec).config.get();
        let way_mask_0 = (*cachec).way_mask_0.get();
        debug!("Config: {:x}", config);
        debug!("WayMask0: {:x}", way_mask_0);

        //Enable all ways:
        (*cachec).way_enable.set(0x10);
    }
}