// Copyright (c) 2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use crate::arch::riscv::kernel::BOOT_INFO;
use crate::scheduler::{CoreId, PerCoreScheduler};
use core::ptr;

#[no_mangle]
pub static mut PERCORE: PerCoreVariables = PerCoreVariables::new(0);

#[derive(Debug)]
pub struct PerCoreVariables {
	/// ID of the current Core.
	core_id: PerCoreVariable,
	/// Scheduler of the current Core.
	scheduler: PerCoreVariable,
	/// start address of the kernel stack
	pub kernel_stack: PerCoreVariable,
}

impl PerCoreVariables {
	pub const fn new(core_id: CoreId) -> Self {
		Self {
			core_id: PerCoreVariable::new(core_id as usize),
			scheduler: PerCoreVariable::new(0),
			kernel_stack: PerCoreVariable::new(0),
		}
	}
}

#[derive(Debug)]
#[repr(C)]
pub struct PerCoreVariable {
	data: usize,
}

pub trait PerCoreVariableMethods {
	unsafe fn get(&self) -> usize;
	unsafe fn set(&mut self, value: usize);
}

impl PerCoreVariable {
	const fn new(value: usize) -> Self {
		Self { data: value }
	}

	#[inline]
	unsafe fn offset(&self) -> usize {
		let base = &PERCORE as *const _ as usize;
		let field = self as *const _ as usize;
		field - base
	}
}

// Treat all per-core variables as 64-bit variables by default. This is true for u64, usize, pointers.
// Implement the PerCoreVariableMethods trait functions using 64-bit memory moves.
// The functions are implemented as default functions, which can be overriden in specialized implementations of the trait.
impl PerCoreVariableMethods for PerCoreVariable {
	#[inline]
	#[cfg(feature = "smp")]
	default unsafe fn get(&self) -> usize {
		let mut value: usize;
		let mut offset = self.offset();
		//llvm_asm!("movq %gs:($1), $0" : "=r"(value) : "r"(self.offset()) :: "volatile");
		asm!(
			"add {offset}, {offset}, gp",
			"ld {value}, 0({offset})",
			value = out(reg) value,
			offset = inout(reg) offset,
		);
		value
	}

	#[inline]
	#[cfg(not(feature = "smp"))]
	default unsafe fn get(&self) -> usize {
		self.data
	}

	#[inline]
	#[cfg(feature = "smp")]
	default unsafe fn set(&mut self, value: usize) {
		//llvm_asm!("movq $0, %gs:($1)" :: "r"(value), "r"(self.offset()) :: "volatile");
		let mut offset = self.offset();
		asm!(
			"add {offset}, {offset}, gp",
			"sd {value}, 0({offset})",
			value = in(reg) value,
			offset = inout(reg) offset,
		);
	}

	#[inline]
	#[cfg(not(feature = "smp"))]
	default unsafe fn set(&mut self, value: usize) {
		self.data = value;
	}
}

#[inline]
pub fn core_id() -> CoreId {
	unsafe { PERCORE.core_id.get() as u32 }
}

#[inline]
pub fn core_scheduler() -> &'static mut PerCoreScheduler {
	unsafe { &mut *(PERCORE.scheduler.get() as *mut PerCoreScheduler) }
}

#[inline]
pub fn set_core_scheduler(scheduler: *mut PerCoreScheduler) {
	unsafe {
		PERCORE.scheduler.set(scheduler as usize);
	}
}

#[inline(always)]
pub fn get_kernel_stack() -> u64 {
	unsafe { PERCORE.kernel_stack.get() as u64 }
}

#[inline]
pub fn set_kernel_stack(addr: u64) {
	unsafe { PERCORE.kernel_stack.set(addr as usize) }
}

pub fn init() {
	unsafe {
		// Store the address to the PerCoreVariables structure allocated for this core in gp.
		let mut address = core::ptr::read_volatile(&(*BOOT_INFO).current_percore_address);
		if address == 0 {
			address = &PERCORE as *const _ as u64;
		}

		asm!(
			"mv gp, {address}",
			address = in(reg) address,
		);

		//println!("percore address: {:x}, {:x?}", address, *(address as *const PerCoreVariables));
	}
}
