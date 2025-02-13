#![allow(dead_code)]

use alloc::vec::Vec;
use core::fmt;

use hermit_sync::{without_interrupts, InterruptTicketMutex};
use pci_types::{
	Bar, ConfigRegionAccess, DeviceId, EndpointHeader, InterruptLine, PciAddress, PciHeader,
	VendorId, MAX_BARS,
};

use crate::arch::mm::{PhysAddr, VirtAddr};
use crate::arch::pci::PciConfigRegion;
use crate::drivers::fs::virtio_fs::VirtioFsDriver;
#[cfg(not(target_arch = "aarch64"))]
use crate::drivers::net::rtl8139::{self, RTL8139Driver};
use crate::drivers::net::virtio_net::VirtioNetDriver;
use crate::drivers::net::NetworkInterface;
use crate::drivers::virtio::transport::pci as pci_virtio;
use crate::drivers::virtio::transport::pci::VirtioDriver;

/// Converts a given little endian coded u32 to native endian coded.
//
// INFO: As the endianness received from the device is little endian coded
// the given value must be swapped again on big endian machines. Which is done
// via the u32::to_le() method as the u32::to_be() would be a no-op in big endian
// machines. Resulting in no conversion.
#[inline]
pub(crate) fn from_pci_endian(val: u32) -> u32 {
	if cfg!(target = "big_endian") {
		val.to_le()
	} else {
		val
	}
}

/// The module contains constants specific to PCI.
#[allow(dead_code)]
pub(crate) mod constants {
	// PCI constants
	pub(crate) const PCI_MAX_BUS_NUMBER: u8 = 32;
	pub(crate) const PCI_MAX_DEVICE_NUMBER: u8 = 32;
	pub(crate) const PCI_CONFIG_ADDRESS_PORT: u16 = 0xCF8;
	pub(crate) const PCI_CONFIG_ADDRESS_ENABLE: u32 = 1 << 31;
	pub(crate) const PCI_CONFIG_DATA_PORT: u16 = 0xCFC;
	pub(crate) const PCI_CAP_ID_VNDR_VIRTIO: u32 = 0x09;
	pub(crate) const PCI_MASK_IS_DEV_BUS_MASTER: u32 = 0x0000_0004u32;
	pub(crate) const PCI_COMMAND_BUSMASTER: u32 = 1 << 2;

	/// PCI registers offset inside header,
	/// if PCI header is of type 00h.
	#[allow(dead_code, non_camel_case_types)]
	#[repr(u16)]
	pub enum RegisterHeader {
		PCI_ID_REGISTER = 0x00u16,
		PCI_COMMAND_REGISTER = 0x04u16,
		PCI_CLASS_REGISTER = 0x08u16,
		PCI_HEADER_REGISTER = 0x0Cu16,
		PCI_BAR0_REGISTER = 0x10u16,
		PCI_CAPABILITY_LIST_REGISTER = 0x34u16,
		PCI_INTERRUPT_REGISTER = 0x3Cu16,
	}

	impl From<RegisterHeader> for u16 {
		fn from(val: RegisterHeader) -> u16 {
			match val {
				RegisterHeader::PCI_ID_REGISTER => 0x00u16,
				RegisterHeader::PCI_COMMAND_REGISTER => 0x04u16,
				RegisterHeader::PCI_CLASS_REGISTER => 0x08u16,
				RegisterHeader::PCI_HEADER_REGISTER => 0x0Cu16,
				RegisterHeader::PCI_BAR0_REGISTER => 0x10u16,
				RegisterHeader::PCI_CAPABILITY_LIST_REGISTER => 0x34u16,
				RegisterHeader::PCI_INTERRUPT_REGISTER => 0x3Cu16,
			}
		}
	}

	/// PCI masks. For convenience put into an enum and provides
	/// an `Into<u32>` method for usage.
	#[allow(dead_code, non_camel_case_types)]
	#[repr(u32)]
	pub enum Masks {
		PCI_MASK_IS_BAR_IO_BAR = 0x0000_0001u32,
		PCI_MASK_IS_MEM_BASE_ADDRESS_64BIT = 0x0000_0004u32,
		PCI_MASK_IS_MEM_BAR_PREFETCHABLE = 0x0000_0008u32,
		PCI_MASK_STATUS_CAPABILITIES_LIST = 0x0000_0010u32,
		PCI_MASK_CAPLIST_POINTER = 0x0000_00FCu32,
		PCI_MASK_HEADER_TYPE = 0x007F_0000u32,
		PCI_MASK_MULTIFUNCTION = 0x0080_0000u32,
		PCI_MASK_MEM_BASE_ADDRESS = 0xFFFF_FFF0u32,
		PCI_MASK_IO_BASE_ADDRESS = 0xFFFF_FFFCu32,
	}

	impl From<Masks> for u32 {
		fn from(val: Masks) -> u32 {
			match val {
				Masks::PCI_MASK_STATUS_CAPABILITIES_LIST => 0x0000_0010u32,
				Masks::PCI_MASK_CAPLIST_POINTER => 0x0000_00FCu32,
				Masks::PCI_MASK_HEADER_TYPE => 0x007F_0000u32,
				Masks::PCI_MASK_MULTIFUNCTION => 0x0080_0000u32,
				Masks::PCI_MASK_MEM_BASE_ADDRESS => 0xFFFF_FFF0u32,
				Masks::PCI_MASK_IO_BASE_ADDRESS => 0xFFFF_FFFCu32,
				Masks::PCI_MASK_IS_MEM_BAR_PREFETCHABLE => 0x0000_0008u32,
				Masks::PCI_MASK_IS_MEM_BASE_ADDRESS_64BIT => 0x0000_0004u32,
				Masks::PCI_MASK_IS_BAR_IO_BAR => 0x0000_0001u32,
			}
		}
	}
}

pub(crate) static mut PCI_DEVICES: Vec<PciDevice<PciConfigRegion>> = Vec::new();
static mut PCI_DRIVERS: Vec<PciDriver> = Vec::new();

#[derive(Copy, Clone, Debug)]
pub(crate) struct PciDevice<T: ConfigRegionAccess> {
	address: PciAddress,
	access: T,
}

impl<T: ConfigRegionAccess> PciDevice<T> {
	pub const fn new(address: PciAddress, access: T) -> Self {
		Self { address, access }
	}

	pub fn read_register(&self, register: u16) -> u32 {
		unsafe { self.access.read(self.address, register) }
	}

	pub fn write_register(&self, register: u16, value: u32) {
		unsafe { self.access.write(self.address, register, value) }
	}

	pub fn make_bus_master(&self) {
		use crate::drivers::pci::constants::{RegisterHeader, PCI_COMMAND_BUSMASTER};

		unsafe {
			let mut command = self
				.access
				.read(self.address, RegisterHeader::PCI_COMMAND_REGISTER.into());
			command |= PCI_COMMAND_BUSMASTER;
			self.access.write(
				self.address,
				RegisterHeader::PCI_COMMAND_REGISTER.into(),
				command,
			)
		}
	}

	/// Returns the bar at bar-register `slot`.
	pub fn bar(&self, slot: u8) -> Option<Bar> {
		let header = PciHeader::new(self.address);
		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			return endpoint.bar(slot, &self.access);
		}

		None
	}

	/// Memory maps pci bar with specified index to identical location in virtual memory.
	/// no_cache determines if we set the `Cache Disable` flag in the page-table-entry.
	/// Returns (virtual-pointer, size) if successful, else None (if bar non-existent or IOSpace)
	pub fn memory_map_bar(&self, index: u8, no_cache: bool) -> Option<(VirtAddr, usize)> {
		let (address, size, prefetchable, width) = match self.bar(index) {
			Some(Bar::Io { .. }) => {
				warn!("Cannot map IOBar!");
				return None;
			}
			Some(Bar::Memory32 {
				address,
				size,
				prefetchable,
			}) => (
				u64::from(address),
				usize::try_from(size).unwrap(),
				prefetchable,
				32,
			),
			Some(Bar::Memory64 {
				address,
				size,
				prefetchable,
			}) => (address, usize::try_from(size).unwrap(), prefetchable, 64),
			_ => {
				return None;
			}
		};

		debug!(
			"Mapping bar {} at {:#x} with length {:#x}",
			index, address, size
		);

		if width != 64 {
			warn!("Currently only mapping of 64 bit bars is supported!");
			return None;
		}
		if !prefetchable {
			warn!("Currently only mapping of prefetchable bars is supported!")
		}

		// Since the bios/bootloader manages the physical address space, the address got from the bar is unique and not overlapping.
		// We therefore do not need to reserve any additional memory in our kernel.
		// Map bar into RW^X virtual memory
		let physical_address = address;
		let virtual_address = crate::mm::map(
			PhysAddr::from(physical_address),
			size,
			true,
			false,
			no_cache,
		);

		Some((virtual_address, size))
	}

	pub fn irq(&self) -> Option<InterruptLine> {
		let header = PciHeader::new(self.address);
		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			let (_pin, line) = endpoint.interrupt(&self.access);
			Some(line)
		} else {
			None
		}
	}

	pub fn bus(&self) -> u8 {
		self.address.bus()
	}

	pub fn device(&self) -> u8 {
		self.address.device()
	}

	pub fn vendor_id(&self) -> VendorId {
		let header = PciHeader::new(self.address);
		let (vendor_id, _device_id) = header.id(&self.access);
		vendor_id
	}

	pub fn device_id(&self) -> DeviceId {
		let header = PciHeader::new(self.address);
		let (_vendor_id, device_id) = header.id(&self.access);
		device_id
	}

	pub fn id(&self) -> (VendorId, DeviceId) {
		let header = PciHeader::new(self.address);
		header.id(&self.access)
	}
}

impl<T: ConfigRegionAccess> fmt::Display for PciDevice<T> {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let header = PciHeader::new(self.address);
		let header_type = header.header_type(&self.access);
		let (vendor_id, device_id) = header.id(&self.access);
		let (_dev_rev, class_id, subclass_id, _interface) = header.revision_and_class(&self.access);

		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			#[cfg(feature = "pci-ids")]
			let (class_name, vendor_name, device_name) = {
				use pci_ids::{Class, Device, FromId, Subclass};

				let class_name = Class::from_id(class_id).map_or("Unknown Class", |class| {
					class
						.subclasses()
						.find(|s| s.id() == subclass_id)
						.map(Subclass::name)
						.unwrap_or_else(|| class.name())
				});

				let (vendor_name, device_name) = Device::from_vid_pid(vendor_id, device_id)
					.map(|device| (device.vendor().name(), device.name()))
					.unwrap_or(("Unknown Vendor", "Unknown Device"));

				(class_name, vendor_name, device_name)
			};

			#[cfg(not(feature = "pci-ids"))]
			let (class_name, vendor_name, device_name) = ("Unknown Class", "Unknown Vendor", "Unknown Device");

			// Output detailed readable information about this device.
			write!(
				f,
				"{:02X}:{:02X} {} [{:02X}{:02X}]: {} {} [{:04X}:{:04X}]",
				self.address.bus(),
				self.address.device(),
				class_name,
				class_id,
				subclass_id,
				vendor_name,
				device_name,
				vendor_id,
				device_id
			)?;

			// If the devices uses an IRQ, output this one as well.
			let (_, irq) = endpoint.interrupt(&self.access);
			if irq != 0 && irq != u8::MAX {
				write!(f, ", IRQ {}", irq)?;
			}

			let mut slot: u8 = 0;
			while usize::from(slot) < MAX_BARS {
				if let Some(pci_bar) = endpoint.bar(slot, &self.access) {
					match pci_bar {
						Bar::Memory64 {
							address,
							size,
							prefetchable,
						} => {
							write!(f, ", BAR{} Memory64 {{ address: {:#X}, size: {:#X}, prefetchable: {} }}", slot, address, size, prefetchable)?;
							slot += 1;
						}
						Bar::Memory32 {
							address,
							size,
							prefetchable,
						} => {
							write!(f, ", BAR{} Memory32 {{ address: {:#X}, size: {:#X}, prefetchable: {} }}", slot, address, size, prefetchable)?;
						}
						Bar::Io { port } => {
							write!(f, ", BAR{} IO {{ port: {:#X} }}", slot, port)?;
						}
					}
				}
				slot += 1;
			}
		} else {
			// Output detailed readable information about this device.
			write!(
				f,
				"{:02X}:{:02X} {:?} [{:04X}:{:04X}]",
				self.address.bus(),
				self.address.device(),
				header_type,
				vendor_id,
				device_id
			)?;
		}

		Ok(())
	}
}

pub(crate) fn print_information() {
	infoheader!(" PCI BUS INFORMATION ");

	for adapter in unsafe { PCI_DEVICES.iter() } {
		info!("{}", adapter);
	}

	infofooter!();
}

#[allow(clippy::large_enum_variant)]
pub(crate) enum PciDriver {
	VirtioFs(InterruptTicketMutex<VirtioFsDriver>),
	VirtioNet(InterruptTicketMutex<VirtioNetDriver>),
	#[cfg(not(target_arch = "aarch64"))]
	RTL8139Net(InterruptTicketMutex<RTL8139Driver>),
}

impl PciDriver {
	fn get_network_driver(&self) -> Option<&InterruptTicketMutex<dyn NetworkInterface>> {
		match self {
			Self::VirtioNet(drv) => Some(drv),
			#[cfg(not(target_arch = "aarch64"))]
			Self::RTL8139Net(drv) => Some(drv),
			_ => None,
		}
	}

	fn get_filesystem_driver(&self) -> Option<&InterruptTicketMutex<VirtioFsDriver>> {
		match self {
			Self::VirtioFs(drv) => Some(drv),
			_ => None,
		}
	}
}

pub(crate) fn register_driver(drv: PciDriver) {
	unsafe {
		PCI_DRIVERS.push(drv);
	}
}

pub(crate) fn get_network_driver() -> Option<&'static InterruptTicketMutex<dyn NetworkInterface>> {
	unsafe { PCI_DRIVERS.iter().find_map(|drv| drv.get_network_driver()) }
}

pub(crate) fn get_filesystem_driver() -> Option<&'static InterruptTicketMutex<VirtioFsDriver>> {
	unsafe {
		PCI_DRIVERS
			.iter()
			.find_map(|drv| drv.get_filesystem_driver())
	}
}

#[cfg(not(target_arch = "aarch64"))]
pub(crate) fn init_drivers() {
	let mut nic_available = false;

	// virtio: 4.1.2 PCI Device Discovery
	without_interrupts(|| {
		for adapter in unsafe {
			PCI_DEVICES.iter().filter(|x| {
				let (vendor_id, device_id) = x.id();
				vendor_id == 0x1AF4 && (0x1000..=0x107F).contains(&device_id)
			})
		} {
			info!(
				"Found virtio network device with device id {:#x}",
				adapter.device_id()
			);

			match pci_virtio::init_device(adapter) {
				Ok(VirtioDriver::Network(drv)) => {
					nic_available = true;
					register_driver(PciDriver::VirtioNet(InterruptTicketMutex::new(drv)))
				}
				Ok(VirtioDriver::FileSystem(drv)) => {
					register_driver(PciDriver::VirtioFs(InterruptTicketMutex::new(drv)))
				}
				_ => {}
			}
		}

		// do we already found a network interface?
		#[cfg(not(target_arch = "aarch64"))]
		if !nic_available {
			// Searching for Realtek RTL8139, which is supported by Qemu
			for adapter in unsafe {
				PCI_DEVICES.iter().filter(|x| {
					let (vendor_id, device_id) = x.id();
					vendor_id == 0x10ec && (0x8138..=0x8139).contains(&device_id)
				})
			} {
				info!(
					"Found Realtek network device with device id {:#x}",
					adapter.device_id()
				);

				if let Ok(drv) = rtl8139::init_device(adapter) {
					register_driver(PciDriver::RTL8139Net(InterruptTicketMutex::new(drv)))
				}
			}
		}
	});
}

/// A module containing PCI specific errors
///
/// Errors include...
pub(crate) mod error {
	/// An enum of PciErrors
	/// typically carrying the device's id as an u16.
	#[derive(Debug)]
	pub enum PciError {
		General(u16),
		NoBar(u16),
		NoCapPtr(u16),
		BadCapPtr(u16),
		NoVirtioCaps(u16),
	}
}
