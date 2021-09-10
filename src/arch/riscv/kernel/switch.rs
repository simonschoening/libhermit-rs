use riscv::register::sstatus;
pub use switch_to_task as switch_to_fpu_owner;

global_asm!(include_str!("switch.s"));

extern "C" {
	pub fn switch_to_task_fp_clean(old_stack: *mut usize, new_stack: usize);
	pub fn switch_to_task_fp_dirty(old_stack: *mut usize, new_stack: usize);
}

/// Saves the floating point registers when needed and switches task
pub unsafe fn switch_to_task(old_stack: *mut usize, new_stack: usize) {
	if sstatus::read().fs() == sstatus::FS::Dirty {
		switch_to_task_fp_dirty(old_stack, new_stack);
	} else {
		switch_to_task_fp_clean(old_stack, new_stack);
	}
}
