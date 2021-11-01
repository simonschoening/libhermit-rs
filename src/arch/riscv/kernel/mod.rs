// Copyright (c) 2018-2020 Stefan Lankes, RWTH Aachen University
//               2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

mod devicetree;
pub mod irq;
pub mod mmio;
pub mod pci;
pub mod percore;
pub mod processor;
mod sbi;
pub mod scheduler;
pub mod serial;
mod start;
pub mod switch;
pub mod systemtime;

pub use crate::arch::riscv::kernel::devicetree::{get_mem_base, init_drivers};
use crate::arch::riscv::kernel::percore::*;
use crate::arch::riscv::kernel::processor::lsb;
use crate::arch::riscv::kernel::serial::SerialPort;
pub use crate::arch::riscv::kernel::systemtime::get_boot_time;
use crate::arch::riscv::mm::paging;
use crate::arch::riscv::mm::physicalmem;
use crate::arch::riscv::mm::{PhysAddr, VirtAddr};
use crate::config::*;
use crate::environment;
use crate::kernel_message_buffer;
use crate::scheduler::CoreId;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt;
use core::{intrinsics, mem, ptr};
use hermit_dtb::Dtb;
use riscv::register::{fcsr, sstatus};

const SERIAL_PORT_BAUDRATE: u32 = 115200;
const BOOTINFO_MAGIC_NUMBER: u32 = 0xC0DE_CAFEu32;

static mut COM1: SerialPort = SerialPort::new(0x9000000);

// Used to store information about available harts. The index of the hart in the vector
// represents its CpuId and does not need to match its hart_id
pub static mut HARTS_AVAILABLE: Vec<usize> = Vec::new();

#[repr(C)]
struct BootInfo {
	pub magic_number: u32,
	pub version: u32,
	pub base: u64,
	pub limit: u64,
	pub image_size: u64,
	pub tls_start: u64,
	pub tls_filesz: u64,
	pub tls_memsz: u64,
	pub current_stack_address: u64,
	pub current_percore_address: u64,
	pub host_logical_addr: u64,
	pub boot_gtod: u64,
	pub mb_info: u64,
	pub cmdline: u64,
	pub cmdsize: u64,
	pub cpu_freq: u32,
	pub boot_processor: u32,
	pub cpu_online: u32,
	pub possible_cpus: u32,
	pub current_boot_id: u32,
	pub uartport: u16,
	pub single_kernel: u8,
	pub uhyve: u8,
	pub hcip: [u8; 4],
	pub hcgateway: [u8; 4],
	pub hcmask: [u8; 4],
	pub dtb_ptr: u64,
	pub hart_mask: u64,
	pub timebase_freq: u64,
}

impl fmt::Debug for BootInfo {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		writeln!(f, "magic_number 0x{:x}", self.magic_number)?;
		writeln!(f, "version 0x{:x}", self.version)?;
		writeln!(f, "base 0x{:x}", self.base)?;
		writeln!(f, "limit 0x{:x}", self.limit)?;
		writeln!(f, "tls_start 0x{:x}", self.tls_start)?;
		writeln!(f, "tls_filesz 0x{:x}", self.tls_filesz)?;
		writeln!(f, "tls_memsz 0x{:x}", self.tls_memsz)?;
		writeln!(f, "image_size 0x{:x}", self.image_size)?;
		writeln!(
			f,
			"current_stack_address 0x{:x}",
			self.current_stack_address
		)?;
		writeln!(
			f,
			"current_percore_address 0x{:x}",
			self.current_percore_address
		)?;
		writeln!(f, "host_logical_addr 0x{:x}", self.host_logical_addr)?;
		writeln!(f, "boot_gtod 0x{:x}", self.boot_gtod)?;
		writeln!(f, "cmdline 0x{:x}", self.cmdline)?;
		writeln!(f, "cmdsize 0x{:x}", self.cmdsize)?;
		writeln!(f, "cpu_freq {}", self.cpu_freq)?;
		writeln!(f, "boot_processor {}", self.boot_processor)?;
		writeln!(f, "cpu_online {}", self.cpu_online)?;
		writeln!(f, "possible_cpus {}", self.possible_cpus)?;
		writeln!(f, "current_boot_id {}", self.current_boot_id)?;
		writeln!(f, "uartport 0x{:x}", self.uartport)?;
		writeln!(f, "single_kernel {}", self.single_kernel)?;
		writeln!(f, "uhyve {}", self.uhyve)?;
		writeln!(f, "dtb_ptr {:x}", self.dtb_ptr)?;
		writeln!(f, "hart_mask {}", self.hart_mask)?;
		writeln!(f, "timebase_freq {}", self.timebase_freq)
	}
}

/// Kernel header to announce machine features
static mut BOOT_INFO: *mut BootInfo = ptr::null_mut();

// FUNCTIONS

pub fn get_image_size() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).image_size) as usize }
}

pub fn get_limit() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).limit) as usize }
}

pub fn get_mbinfo() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).mb_info) as usize }
}

#[cfg(feature = "smp")]
pub fn get_processor_count() -> u32 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).cpu_online) as u32 }
}

#[cfg(not(feature = "smp"))]
pub fn get_processor_count() -> u32 {
	1
}

pub fn get_base_address() -> VirtAddr {
	unsafe { VirtAddr(core::ptr::read_volatile(&(*BOOT_INFO).base)) }
}

pub fn get_tls_start() -> VirtAddr {
	unsafe { VirtAddr(core::ptr::read_volatile(&(*BOOT_INFO).tls_start)) }
}

pub fn get_tls_filesz() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).tls_filesz) as usize }
}

pub fn get_tls_memsz() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).tls_memsz) as usize }
}

/// Whether HermitCore is running under the "uhyve" hypervisor.
pub fn is_uhyve() -> bool {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).uhyve) != 0 }
	//false
}

/// Whether HermitCore is running alone (true) or side-by-side to Linux in Multi-Kernel mode (false).
pub fn is_single_kernel() -> bool {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).single_kernel) != 0 }
}

pub fn get_cmdsize() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).cmdsize) as usize }
}

pub fn get_cmdline() -> VirtAddr {
	VirtAddr(unsafe { core::ptr::read_volatile(&(*BOOT_INFO).cmdline) })
}

pub fn get_dtb_ptr() -> *const u8 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).dtb_ptr) as *const u8 }
}

pub fn get_hart_mask() -> u64 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).hart_mask) }
}

pub fn get_timebase_freq() -> u64 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).timebase_freq) as u64 }
}

pub fn get_current_boot_id() -> u32 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).current_boot_id) }
}

/// Earliest initialization function called by the Boot Processor.
pub fn message_output_init() {
	percore::init();

	#[cfg(not(feature = "aarch64-qemu-stdout"))]
	if environment::is_single_kernel() {
		// We can only initialize the serial port here, because VGA requires processor
		// configuration first.
		unsafe {
			COM1.init(SERIAL_PORT_BAUDRATE);
		}
	}
}

pub fn output_message_byte(byte: u8) {
	#[cfg(feature = "aarch64-qemu-stdout")]
	unsafe {
		core::ptr::write_volatile(0x3F20_1000 as *mut u8, byte);
	}
	#[cfg(not(feature = "aarch64-qemu-stdout"))]
	if environment::is_single_kernel() {
		// Output messages to the serial port and VGA screen in unikernel mode.
		unsafe {
			COM1.write_byte(byte);
		}
	} else {
		// Output messages to the kernel message buffer in multi-kernel mode.
		kernel_message_buffer::write_byte(byte);
	}
}

pub fn output_message_buf(buf: &[u8]) {
	for byte in buf {
		output_message_byte(*byte);
	}
}

/// Real Boot Processor initialization as soon as we have put the first Welcome message on the screen.
pub fn boot_processor_init() {
	devicetree::init();
	crate::mm::init();
	crate::mm::print_information();
	environment::init();
	irq::install();
	//devicetree::init_drivers();

	/*processor::detect_features();
	processor::configure();

	::mm::init();
	::mm::print_information();
	environment::init();
	gdt::init();
	gdt::add_current_core();
	idt::install();

	if !environment::is_uhyve() {
		pic::init();
	}

	irq::install();
	irq::enable();
	processor::detect_frequency();
	processor::print_information();
	systemtime::init();

	if environment::is_single_kernel() && !environment::is_uhyve() {
		pci::init();
		pci::print_information();
		acpi::init();
	}

	apic::init();
	scheduler::install_timer_handler();*/

	// TODO: PMCCNTR_EL0 is the best replacement for RDTSC on AArch64.
	// However, this test code showed that it's apparently not supported under uhyve yet.
	// Finish the boot loader for QEMU first and then run this code under QEMU, where it should be supported.
	// If that's the case, find out what's wrong with uhyve.
	/* unsafe {
		// TODO: Setting PMUSERENR_EL0 is probably not required, but find out about that
		// when reading PMCCNTR_EL0 works at all.
		let pmuserenr_el0: u32 = 1 << 0 | 1 << 2 | 1 << 3;
		llvm_asm!("msr pmuserenr_el0, $0" :: "r"(pmuserenr_el0) :: "volatile");
		debug!("pmuserenr_el0");

		// TODO: Setting PMCNTENSET_EL0 is probably not required, but find out about that
		// when reading PMCCNTR_EL0 works at all.
		let pmcntenset_el0: u32 = 1 << 31;
		llvm_asm!("msr pmcntenset_el0, $0" :: "r"(pmcntenset_el0) :: "volatile");
		debug!("pmcntenset_el0");

		// Enable PMCCNTR_EL0 using PMCR_EL0.
		let mut pmcr_el0: u32 = 0;
		llvm_asm!("mrs $0, pmcr_el0" : "=r"(pmcr_el0) :: "memory" : "volatile");
		debug!(
			"PMCR_EL0 (has RES1 bits and therefore musn't be zero): {:#X}",
			pmcr_el0
		);
		pmcr_el0 |= 1 << 0 | 1 << 2 | 1 << 6;
		llvm_asm!("msr pmcr_el0, $0" :: "r"(pmcr_el0) :: "volatile");
	}

	// Read out PMCCNTR_EL0 in an infinite loop.
	// TODO: This currently stays at zero on uhyve. Fix uhyve! :)
	loop {
		unsafe {
			let pmccntr: u64;
			llvm_asm!("mrs $0, pmccntr_el0" : "=r"(pmccntr) ::: "volatile");
			println!("Count: {}", pmccntr);
		}
	} */

	finish_processor_init();
	irq::enable();
}

/// Boots all available Application Processors on bare-metal or QEMU.
/// Called after the Boot Processor has been fully initialized along with its scheduler.
pub fn boot_application_processors() {
	// Nothing to do here yet.
}

extern "C" {
	fn _start(hart_id: usize, boot_info: &'static mut BootInfo) -> !;
}

/// Application Processor initialization
pub fn application_processor_init() {
	percore::init();
	paging::init_application_processor();
	irq::install();
	finish_processor_init();
	irq::enable();
}

fn finish_processor_init() {
	unsafe {
		sstatus::set_fs(sstatus::FS::Initial);
		//fcsr::clear_flags();
		let fcsr: usize = 0;
		asm! {
			"csrw	fcsr, {fcsr}",
			fcsr = in(reg) fcsr,
		}

		debug!("{}", fcsr::RoundingMode::RoundTowardsZero as u32);
		// asm!(
		// 	"fmv.s.x	f0, zero",
		// 	"fmv.s.x	f1, zero",
		// 	"fmv.s.x	f2, zero",
		// 	"fmv.s.x	f3, zero",
		// 	"fmv.s.x	f4, zero",
		// 	"fmv.s.x	f5, zero",
		// 	"fmv.s.x	f6, zero",
		// 	"fmv.s.x	f7, zero",
		// 	"fmv.s.x	f8, zero",
		// 	"fmv.s.x	f9, zero",
		// 	"fmv.s.x	f10, zero",
		// 	"fmv.s.x	f11, zero",
		// 	"fmv.s.x	f12, zero",
		// 	"fmv.s.x	f13, zero",
		// 	"fmv.s.x	f14, zero",
		// 	"fmv.s.x	f15, zero",
		// 	"fmv.s.x	f16, zero",
		// 	"fmv.s.x	f17, zero",
		// 	"fmv.s.x	f18, zero",
		// 	"fmv.s.x	f19, zero",
		// 	"fmv.s.x	f20, zero",
		// 	"fmv.s.x	f21, zero",
		// 	"fmv.s.x	f22, zero",
		// 	"fmv.s.x	f23, zero",
		// 	"fmv.s.x	f24, zero",
		// 	"fmv.s.x	f25, zero",
		// 	"fmv.s.x	f26, zero",
		// 	"fmv.s.x	f27, zero",
		// 	"fmv.s.x	f28, zero",
		// 	"fmv.s.x	f29, zero",
		// 	"fmv.s.x	f30, zero",
		// 	"fmv.s.x	f31, zero",
		// 	"csrw	fcsr, 0",
		// );

		// sstatus::set_fs(sstatus::FS::Initial);
	}
	info!("SSTATUS FS: {:?}", sstatus::read().fs());
	info!("FCSR: {:x?}", fcsr::read());
	/*if environment::is_uhyve() {
		// uhyve does not use apic::detect_from_acpi and therefore does not know the number of processors and
		// their APIC IDs in advance.
		// Therefore, we have to add each booted processor into the CPU_LOCAL_APIC_IDS vector ourselves.
		// Fortunately, the Core IDs are guaranteed to be sequential and match the Local APIC IDs.
		apic::add_local_apic_id(core_id() as u8);

		// uhyve also boots each processor into entry.asm itself and does not use apic::boot_application_processors.
		// Therefore, the current processor already needs to prepare the processor variables for a possible next processor.
		apic::init_next_processor_variables(core_id() + 1);
	}*/

	// This triggers apic::boot_application_processors (bare-metal/QEMU) or uhyve
	// to initialize the next processor.
	// This triggers apic::boot_application_processors (bare-metal/QEMU) or uhyve
	// to initialize the next processor.

	let current_hart_id = get_current_boot_id() as usize;

	unsafe {
		// Add hart to HARTS_AVAILABLE, the hart id is stored in current_boot_id
		HARTS_AVAILABLE.push(current_hart_id);
		info!(
			"Initialized CPU with hart_id {}",
			HARTS_AVAILABLE[percore::core_id() as usize]
		);
	}

	crate::scheduler::add_current_core();

	// Remove current hart from the hart_mask
	let new_hart_mask = get_hart_mask() & (u64::MAX - (1 << current_hart_id));
	unsafe {
		core::ptr::write_volatile(&mut (*BOOT_INFO).hart_mask, new_hart_mask);
	}

	let next_hart_index = lsb(new_hart_mask);

	if let Some(next_hart_id) = next_hart_index {
		// The current processor already needs to prepare the processor variables for a possible next processor.
		init_next_processor_variables(core_id() + 1);

		info!(
			"Starting CPU {} with hart_id {}",
			core_id() + 1,
			next_hart_id
		);

		// Changing cpu_online will cause uhyve to start the next processor
		unsafe {
			let _ = intrinsics::atomic_xadd(&mut (*BOOT_INFO).cpu_online as *mut u32, 1);

			//When running bare-metal/QEMU we use the firmware to start the next hart
			if !is_uhyve() {
				let ret = sbi::sbi_hart_start(
					next_hart_id as usize,
					_start as *const () as usize,
					BOOT_INFO as usize,
				);
				info!("sbi_hart_start: {:?}", ret);
			}
		}
	} else {
		info!("All processors are initialized");
		unsafe {
			let _ = intrinsics::atomic_xadd(&mut (*BOOT_INFO).cpu_online as *mut u32, 1);
		}
	}
}

pub fn network_adapter_init() -> i32 {
	// AArch64 supports no network adapters on bare-metal/QEMU, so return a failure code.
	-1
}

pub fn print_statistics() {}

/// Initialize the required start.rs variables for the next CPU to be booted.
pub fn init_next_processor_variables(core_id: CoreId) {
	// Allocate stack and PerCoreVariables structure for the CPU and pass the addresses.
	// Keep the stack executable to possibly support dynamically generated code on the stack (see https://security.stackexchange.com/a/47825).
	let stack = physicalmem::allocate(KERNEL_STACK_SIZE)
		.expect("Failed to allocate boot stack for new core");
	let mut boxed_percore = Box::new(PerCoreVariables::new(core_id));
	//let boxed_irq = Box::new(IrqStatistics::new());
	//let boxed_irq_raw = Box::into_raw(boxed_irq);

	unsafe {
		//IRQ_COUNTERS.insert(core_id, &(*boxed_irq_raw));
		//boxed_percore.irq_statistics = PerCoreVariable::new(boxed_irq_raw);

		core::ptr::write_volatile(&mut (*BOOT_INFO).current_stack_address, stack.as_u64());
		core::ptr::write_volatile(
			&mut (*BOOT_INFO).current_percore_address,
			Box::into_raw(boxed_percore) as u64,
		);

		info!(
			"Initialize per core data at 0x{:x} (size {} bytes)",
			core::ptr::read_volatile(&(*BOOT_INFO).current_percore_address),
			mem::size_of::<PerCoreVariables>()
		);
	}
}
