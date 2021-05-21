// Copyright (c) 2018-2020 Stefan Lankes, RWTH Aachen University
//               2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

pub mod irq;
pub mod pci;
pub mod percore;
pub mod processor;
pub mod scheduler;
pub mod serial;
mod start;
pub mod stubs;
pub mod systemtime;

use crate::arch::riscv::kernel::percore::*;
use crate::arch::riscv::kernel::serial::SerialPort;
pub use crate::arch::riscv::kernel::stubs::*;
pub use crate::arch::riscv::kernel::systemtime::get_boot_time;
use crate::arch::riscv::mm::{PhysAddr, VirtAddr};
use crate::config::*;
use crate::environment;
use crate::kernel_message_buffer;
use crate::synch::spinlock::Spinlock;
use core::ptr;
use core::fmt;

const SERIAL_PORT_BAUDRATE: u32 = 115200;
const BOOTINFO_MAGIC_NUMBER: u32 = 0xC0DE_CAFEu32;

static mut COM1: SerialPort = SerialPort::new(0x9000000);
static CPU_ONLINE: Spinlock<u32> = Spinlock::new(0);

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
		writeln!(f, "timebase_freq {}", self.timebase_freq)
	}
}

/// Kernel header to announce machine features
static mut BOOT_INFO: *mut BootInfo = ptr::null_mut();

// FUNCTIONS

global_asm!(include_str!("switch.s"));

pub fn get_image_size() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).image_size) as usize }
}

pub fn get_limit() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).limit) as usize }
}

pub fn get_mbinfo() -> usize {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).mb_info) as usize }
}

pub fn get_processor_count() -> u32 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).cpu_online) }
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
	//unsafe { core::ptr::read_volatile(&(*BOOT_INFO).uhyve) != 0 }
	false
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

pub fn get_timebase_freq() -> u64 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).timebase_freq) as u64 }
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
	crate::mm::init();
	crate::mm::print_information();
	environment::init();

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
}

/// Boots all available Application Processors on bare-metal or QEMU.
/// Called after the Boot Processor has been fully initialized along with its scheduler.
pub fn boot_application_processors() {
	// Nothing to do here yet.
}

/// Application Processor initialization
pub fn application_processor_init() {
	percore::init();
	/*processor::configure();
	gdt::add_current_core();
	idt::install();
	apic::init_x2apic();
	apic::init_local_apic();
	irq::enable();*/
	finish_processor_init();
}

fn finish_processor_init() {
	debug!("Initialized Processor");

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
	*CPU_ONLINE.lock() += 1;
}

pub fn network_adapter_init() -> i32 {
	// AArch64 supports no network adapters on bare-metal/QEMU, so return a failure code.
	-1
}

pub fn print_statistics() {}
