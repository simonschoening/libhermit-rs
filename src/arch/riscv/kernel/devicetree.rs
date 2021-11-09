use crate::arch::riscv::kernel::irq::init_plic;
use crate::arch::riscv::kernel::mmio::*;
use crate::arch::riscv::kernel::{get_dtb_ptr, is_uhyve};
use crate::arch::riscv::mm::{paging, PhysAddr, VirtAddr};
use alloc::vec::Vec;
use core::ptr;
use core::convert::TryFrom;
use hermit_dtb::Dtb;

use crate::drivers::net::gem::{self, GEMDriver};
use crate::synch::spinlock::SpinlockIrqSave;

struct Gem {
	base: usize,
	size: usize,
	irq: u8,
	phy_addr: u8,
	mac: [u8; 6],
}

struct Plic {
	base: usize,
	size: usize,
}

enum Device {
	GEM(Gem),
	PLIC(Plic),
}

static mut MEM_BASE: u64 = 0;

static mut DEVICES_AVAILABLE: Vec<Device> = Vec::new();

/// Inits variables based on the device tree
/// This function should only be called once
pub fn init() {
	debug!("Init devicetree");
	if !get_dtb_ptr().is_null() {
		unsafe {
			let dtb = Dtb::from_raw(get_dtb_ptr()).expect("DTB is invalid");

			let memory_reg = dtb
				.get_property("memory", "reg")
				.expect("Memory node not found in dtb");
			for i in 0..(memory_reg.len() / 2) {
				MEM_BASE <<= 8;
				MEM_BASE += memory_reg[i] as u64;
			}
		}
	}
}

/// Inits drivers based on the device tree
/// This function should only be called once
pub fn init_drivers() {
	// TODO: Implement devicetree correctly
	unsafe {
		if !get_dtb_ptr().is_null() {
			debug!("Init drivers using devicetree");
			let dtb = Dtb::from_raw(get_dtb_ptr()).expect("DTB is invalid");
			walk_nodes(&dtb, "/", 0);
		}
		for i in 0..DEVICES_AVAILABLE.len() {
			match &DEVICES_AVAILABLE[i] {
				Device::GEM(gem) => {
					//TODO: Make sure that PLIC is initialized
					debug!(
						"Init GEM at {:x}, irq: {}, phy_addr: {}",
						gem.base, gem.irq, gem.phy_addr
					);
					paging::identity_map::<paging::HugePageSize>(
						PhysAddr(gem.base as u64),
						PhysAddr((gem.base + gem.size - 1) as u64),
					);
					match gem::init_device(
						VirtAddr(gem.base as u64),
						gem.irq.into(),
						gem.phy_addr.into(),
						gem.mac,
					) {
						Ok(drv) => register_driver(MmioDriver::GEMNet(SpinlockIrqSave::new(drv))),
						Err(_) => (), // could have an info which driver failed
					}
				}
				Device::PLIC(plic) => {
					debug!("Init PLIC at {:x}, size: {:x}", plic.base, plic.size);
					paging::identity_map::<paging::HugePageSize>(
						PhysAddr(plic.base as u64),
						PhysAddr((plic.base + plic.size - 1) as u64),
					);
					init_plic(plic.base);
				}
			}
		}
		// loop {}
	}
}

fn walk_nodes<'a, 'b>(dtb: &Dtb<'a>, path: &'b str, level: usize) {
	//debug!("{}: Path: {}", level, path);
	for prop in dtb.enum_properties(path) {
		debug!(
			"{}Prop: {}: {:x?}",
			"\t".repeat(level),
			prop,
			dtb.get_property(path, prop)
		);
	}
	for node in dtb.enum_subnodes(path) {
		debug!("{}{}", "\t".repeat(level), node);
		if node.starts_with("ethernet@") {
			debug!("Found Ethernet controller");
			let path = &[path, node, "/"].concat();

			let compatible = core::str::from_utf8(
				dtb.get_property(path, "compatible")
					.expect("compatible property for ethernet not found in dtb"),
			)
			.unwrap();
			debug!("Compatible: {}", compatible);

			if compatible.contains("sifive,fu540-c000-gem") {
				let reg = dtb
					.get_property(path, "reg")
					.expect("Reg property for ethernet not found in dtb");
				let mut gem_size: u64 = 0;
				let mut gem_base: u64 = 0;
				for i in 8..16 {
					gem_size <<= 8;
					gem_size += reg[i] as u64;
				}
				for i in 0..8 {
					gem_base <<= 8;
					gem_base += reg[i] as u64;
				}

				let interrupts = dtb
					.get_property(path, "interrupts")
					.expect("interrupts property for ethernet not found in dtb");
				let irq: u8 = interrupts[3];

				let mac = dtb
					.get_property(path, "local-mac-address")
					.expect("local-mac-address property for ethernet not found in dtb");
				debug!("MAC: {:x?}",mac);
				
				let path = &[path, "ethernet-phy"].concat();
				debug!("{}", path);
				let phy = dtb
					.get_property(path, "reg")
					.expect("Reg property for ethernet-phy not found in dtb");
				let phy_addr: u8 = phy[3];

				unsafe {
					DEVICES_AVAILABLE.push(Device::GEM(Gem {
						base: gem_base as usize,
						size: gem_size as usize,
						irq: irq,
						phy_addr: phy_addr,
						mac: <[u8; 6]>::try_from(mac).expect("mac with invalid length"),
					}));
				}
			} else {
				warn!("The ethernet controller is not supported");
			}
		} else if node.starts_with("interrupt-controller@") || node.starts_with("plic@") {
			debug!("Found interrupt controller");
			let path = &[path, node, "/"].concat();

			let compatible = core::str::from_utf8(
				dtb.get_property(path, "compatible")
					.expect("compatible property for interrupt-controller not found in dtb"),
			)
			.unwrap();
			debug!("Compatible: {}", compatible);
			if compatible.contains("sifive,plic-1.0.0") {
				let reg = dtb
					.get_property(path, "reg")
					.expect("Reg property for plic not found in dtb");
				let mut plic_size: u64 = 0;
				let mut plic_base: u64 = 0;
				for i in 8..16 {
					plic_size <<= 8;
					plic_size += reg[i] as u64;
				}
				for i in 0..8 {
					plic_base <<= 8;
					plic_base += reg[i] as u64;
				}

				unsafe {
					//Insert before
					DEVICES_AVAILABLE.insert(
						0,
						Device::PLIC(Plic {
							base: plic_base as usize,
							size: plic_size as usize,
						}),
					);
				}
			} else {
				warn!("The interrupt controller is not supported");
			}
		}
		walk_nodes(&dtb, &[path, node, "/"].concat(), level + 1);
	}
}

/// Get the base address of the physical memory
/// The address will be 0x0 when nor devicetree is present
pub fn get_mem_base() -> PhysAddr {
	PhysAddr(unsafe { MEM_BASE })
}
