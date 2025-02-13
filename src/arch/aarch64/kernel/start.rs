use core::arch::asm;

use hermit_entry::boot_info::{BootInfo, RawBootInfo};
use hermit_entry::Entry;

use crate::arch::aarch64::kernel::scheduler::TaskStacks;
use crate::arch::aarch64::kernel::serial::SerialPort;
use crate::arch::aarch64::kernel::{get_processor_count, BOOT_INFO, RAW_BOOT_INFO};
use crate::KERNEL_STACK_SIZE;

extern "C" {
	static vector_table: u8;
}

/// Entrypoint - Initialize Stack pointer and Exception Table
#[no_mangle]
#[naked]
pub unsafe extern "C" fn _start(boot_info: &'static RawBootInfo, cpu_id: u32) -> ! {
	// validate signatures
	const _START: Entry = _start;
	const _PRE_INIT: Entry = pre_init;

	asm!(
		// Add stack top offset
		"mov x8, {stack_top_offset}",
		"add sp, sp, x8",

		// Jump to Rust code
		"b {pre_init}",

		stack_top_offset = const KERNEL_STACK_SIZE - TaskStacks::MARKER_SIZE,
		pre_init = sym pre_init,
		options(noreturn),
	)
}

#[inline(never)]
#[no_mangle]
unsafe extern "C" fn pre_init(boot_info: &'static RawBootInfo, cpu_id: u32) -> ! {
	unsafe {
		RAW_BOOT_INFO = Some(boot_info);
		BOOT_INFO = Some(BootInfo::from(*boot_info));
	}

	// set exception table
	asm!(
		"adrp x4, {vector_table}",
		"add  x4, x4, #:lo12:{vector_table}",
		"msr vbar_el1, x4",
		vector_table = sym vector_table,
		out("x4") _,
		options(nostack),
	);

	// Memory barrier
	asm!("dsb sy", options(nostack),);

	if cpu_id == 0 {
		crate::boot_processor_main()
	} else {
		#[cfg(not(feature = "smp"))]
		{
			error!("SMP support deactivated");
			loop {
				crate::arch::processor::halt()
			}
		}
		#[cfg(feature = "smp")]
		crate::application_processor_main();
	}
}
