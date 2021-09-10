use crate::drivers::net::gem::{self, GEMDriver};
use crate::drivers::net::NetworkInterface;
use crate::synch::spinlock::SpinlockIrqSave;
use alloc::vec::Vec;
// use crate::drivers::net::virtio_net::VirtioNetDriver;

use crate::arch::riscv::mm::paging;
use crate::arch::riscv::mm::{PhysAddr, VirtAddr};

static mut MMIO_DRIVERS: Vec<MmioDriver> = Vec::new();

pub enum MmioDriver {
	GEMNet(SpinlockIrqSave<GEMDriver>),
}

impl<'a> MmioDriver {
	fn get_network_driver(&self) -> Option<&SpinlockIrqSave<dyn NetworkInterface>> {
		match self {
			// Self::VirtioNet(drv) => Some(drv),
			Self::GEMNet(drv) => Some(drv),
			_ => None,
		}
	}

	// fn get_filesystem_driver(&self) -> Option<&SpinlockIrqSave<VirtioFsDriver<'a>>> {
	// 	match self {
	// 		//Self::VirtioFs(drv) => Some(drv),
	// 		_ => None,
	// 	}
	// }
}
pub fn register_driver(drv: MmioDriver) {
	unsafe {
		MMIO_DRIVERS.push(drv);
	}
}

pub fn get_network_driver() -> Option<&'static SpinlockIrqSave<dyn NetworkInterface>> {
	unsafe { MMIO_DRIVERS.iter().find_map(|drv| drv.get_network_driver()) }
}

// pub fn init_drivers() {
//     //Identitiy map GEM
//     paging::identity_map::<paging::HugePageSize>(PhysAddr(0x1009_0000), PhysAddr(0x1009_1FFF));
//     match gem::init_device() {
//         Ok(drv) => register_driver(MmioDriver::GEMNet(SpinlockIrqSave::new(drv))),
//         Err(_) => (), // could have an info which driver failed
//     }

// 	//Identitiy map PLIC
// 	paging::identity_map::<paging::HugePageSize>(PhysAddr(PLIC_BASE), PhysAddr(PLIC_END-1));
// }
