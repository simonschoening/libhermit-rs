use alloc::alloc::{alloc, dealloc, Layout};
use core::convert::TryInto;
use core::{mem, ptr};

use crate::arch::riscv::kernel::percore::*;
use crate::arch::riscv::kernel::processor::set_oneshot_timer;
use crate::arch::riscv::mm::paging::{BasePageSize, PageSize, PageTableEntryFlags};
use crate::arch::riscv::mm::{PhysAddr, VirtAddr};
use crate::env;
use crate::scheduler::task::{Task, TaskFrame};
use crate::{DEFAULT_STACK_SIZE, KERNEL_STACK_SIZE};
use riscv::register::sie;

/* extern "C" {
	static tls_start: u8;
	static tls_end: u8;
} */

#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct State {
	/// return address register
	ra: usize,
	/// stack pointer register
	sp: usize,
	/// global pointer register
	gp: usize,
	/// thread pointer register
	tp: usize,
	/// x5 register
	x5: usize,
	/// x6 register
	x6: usize,
	/// x7 register
	x7: usize,
	/// x8 register
	x8: usize,
	/// x9 register
	x9: usize,
	/// Function arguments/return values
	a0: usize,
	/// a1 register
	a1: usize,
	/// a2 register
	a2: usize,
	/// x13 register
	x13: usize,
	/// x14 register
	x14: usize,
	/// x15 register
	x15: usize,
	/// x16 register
	x16: usize,
	/// x17 register
	x17: usize,
	/// x18 register
	x18: usize,
	/// x19 register
	x19: usize,
	/// x20 register
	x20: usize,
	/// x21 register
	x21: usize,
	/// x22 register
	x22: usize,
	/// x23 register
	x23: usize,
	/// x24 register
	x24: usize,
	/// x25 register
	x25: usize,
	/// x26 register
	x26: usize,
	/// x27 register
	x27: usize,
	/// x28 register
	x28: usize,
	/// x29 register
	x29: usize,
	/// x30 register
	x30: usize,
	/// x31 register
	x31: usize,
}

pub struct BootStack {
	/// stack for kernel tasks
	stack: VirtAddr,
	/// stack to handle interrupts
	ist0: VirtAddr,
}

pub struct CommonStack {
	/// start address of allocated virtual memory region
	virt_addr: VirtAddr,
	/// start address of allocated virtual memory region
	phys_addr: PhysAddr,
	/// total size of all stacks
	total_size: usize,
}

pub enum TaskStacks {
	Boot(BootStack),
	Common(CommonStack),
}

impl TaskStacks {
	/// Size of the debug marker at the very top of each stack.
	///
	/// We have a marker at the very top of the stack for debugging (`0xdeadbeef`), which should not be overridden.
	pub const MARKER_SIZE: usize = 0x10;

	pub fn new(size: usize) -> Self {
		let user_stack_size = if size < KERNEL_STACK_SIZE {
			KERNEL_STACK_SIZE
		} else {
			align_up!(size, BasePageSize::SIZE)
		};
		let total_size = user_stack_size + DEFAULT_STACK_SIZE + KERNEL_STACK_SIZE;
		let virt_addr = crate::arch::mm::virtualmem::allocate(total_size + 4 * BasePageSize::SIZE)
			//let virt_addr = crate::arch::mm::virtualmem::allocate(total_size)
			.expect("Failed to allocate Virtual Memory for TaskStacks");
		let phys_addr = crate::arch::mm::physicalmem::allocate(total_size)
			.expect("Failed to allocate Physical Memory for TaskStacks");

		debug!(
			"Create stacks at {:#X} with a size of {} KB",
			virt_addr,
			total_size >> 10
		);

		let mut flags = PageTableEntryFlags::empty();
		flags.normal().writable().execute_disable();

		// map IST0 into the address space
		crate::arch::mm::paging::map::<BasePageSize>(
			virt_addr + BasePageSize::SIZE,
			//virt_addr,
			phys_addr,
			KERNEL_STACK_SIZE / BasePageSize::SIZE,
			flags,
		);

		// map kernel stack into the address space
		crate::arch::mm::paging::map::<BasePageSize>(
			virt_addr + KERNEL_STACK_SIZE + 2 * BasePageSize::SIZE,
			//virt_addr + KERNEL_STACK_SIZE,
			phys_addr + KERNEL_STACK_SIZE,
			DEFAULT_STACK_SIZE / BasePageSize::SIZE,
			flags,
		);

		// map user stack into the address space
		crate::arch::mm::paging::map::<BasePageSize>(
			virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE + 3 * BasePageSize::SIZE,
			//virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE,
			phys_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE,
			user_stack_size / BasePageSize::SIZE,
			flags,
		);

		// clear user stack
		debug!("Clearing user stack...");
		unsafe {
			ptr::write_bytes(
				(virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE + 3 * BasePageSize::SIZE)
					//(virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE)
					.as_mut_ptr::<u8>(),
				0xAC,
				user_stack_size,
			);
		}

		debug!("Creating stacks finished");

		TaskStacks::Common(CommonStack {
			virt_addr,
			phys_addr,
			total_size,
		})
	}

	pub fn from_boot_stacks() -> TaskStacks {
		//let tss = unsafe { &(*PERCORE.tss.get()) };
		/*let stack = VirtAddr::from_usize(tss.rsp[0] as usize + 0x10 - KERNEL_STACK_SIZE);
		debug!("Using boot stack {:#X}", stack);
		let ist0 = VirtAddr::from_usize(tss.ist[0] as usize + 0x10 - KERNEL_STACK_SIZE);
		debug!("IST0 is located at {:#X}", ist0);*/
		let stack = VirtAddr::zero();
		let ist0 = VirtAddr::zero();

		TaskStacks::Boot(BootStack { stack, ist0 })
	}

	pub fn get_user_stack_size(&self) -> usize {
		match self {
			TaskStacks::Boot(_) => 0,
			TaskStacks::Common(stacks) => {
				stacks.total_size - DEFAULT_STACK_SIZE - KERNEL_STACK_SIZE
			}
		}
	}

	pub fn get_user_stack(&self) -> VirtAddr {
		match self {
			TaskStacks::Boot(_) => VirtAddr::zero(),
			TaskStacks::Common(stacks) => {
				stacks.virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE + 3 * BasePageSize::SIZE
				//stacks.virt_addr + KERNEL_STACK_SIZE + DEFAULT_STACK_SIZE
			}
		}
	}

	pub fn get_kernel_stack(&self) -> VirtAddr {
		match self {
			TaskStacks::Boot(stacks) => stacks.stack,
			TaskStacks::Common(stacks) => {
				stacks.virt_addr + KERNEL_STACK_SIZE + 2 * BasePageSize::SIZE
				//stacks.virt_addr + KERNEL_STACK_SIZE
			}
		}
	}

	pub fn get_kernel_stack_size(&self) -> usize {
		match self {
			TaskStacks::Boot(_) => KERNEL_STACK_SIZE,
			TaskStacks::Common(_) => DEFAULT_STACK_SIZE,
		}
	}

	pub fn get_interupt_stack(&self) -> VirtAddr {
		match self {
			TaskStacks::Boot(stacks) => stacks.ist0,
			TaskStacks::Common(stacks) => stacks.virt_addr + BasePageSize::SIZE,
			//TaskStacks::Common(stacks) => stacks.virt_addr,
		}
	}

	pub fn get_interupt_stack_size(&self) -> usize {
		KERNEL_STACK_SIZE
	}
}

impl Clone for TaskStacks {
	fn clone(&self) -> TaskStacks {
		match self {
			TaskStacks::Boot(_) => TaskStacks::new(0),
			TaskStacks::Common(stacks) => {
				TaskStacks::new(stacks.total_size - DEFAULT_STACK_SIZE - KERNEL_STACK_SIZE)
			}
		}
	}
}

impl Drop for TaskStacks {
	fn drop(&mut self) {
		// we should never deallocate a boot stack
		match self {
			TaskStacks::Boot(_) => {}
			TaskStacks::Common(stacks) => {
				debug!(
					"Deallocating stacks at {:#X} with a size of {} KB",
					stacks.virt_addr,
					stacks.total_size >> 10,
				);

				crate::arch::mm::paging::unmap::<BasePageSize>(
					stacks.virt_addr,
					stacks.total_size / BasePageSize::SIZE + 4,
					//stacks.total_size / BasePageSize::SIZE,
				);
				crate::arch::mm::virtualmem::deallocate(
					stacks.virt_addr,
					stacks.total_size + 4 * BasePageSize::SIZE,
					//stacks.total_size,
				);
				crate::arch::mm::physicalmem::deallocate(stacks.phys_addr, stacks.total_size);
			}
		}
	}
}

pub struct TaskTLS {
	address: VirtAddr,
	tp: VirtAddr,
	layout: Layout,
}

impl TaskTLS {
	pub fn new(tls_size: usize) -> Self {
		// determine the size of tdata (tls without tbss)
		let tdata_size: usize = env::get_tls_filesz();
		// Yes, it does, so we have to allocate TLS memory.
		// Allocate enough space for the given size and one more variable of type usize, which holds the tls_pointer.
		let tls_allocation_size = align_up!(tls_size, 32); // + mem::size_of::<usize>();
												   // We allocate in 128 byte granularity (= cache line size) to avoid false sharing
		let memory_size = align_up!(tls_allocation_size, 128);
		let layout =
			Layout::from_size_align(memory_size, 128).expect("TLS has an invalid size / alignment");
		let ptr = VirtAddr(unsafe { alloc(layout) as u64 });

		// The tls_pointer is the address to the end of the TLS area requested by the task.
		let tls_pointer = ptr; // + align_up!(tls_size, 32);

		unsafe {
			// Copy over TLS variables with their initial values.
			ptr::copy_nonoverlapping(
				env::get_tls_start().as_ptr::<u8>(),
				ptr.as_mut_ptr::<u8>(),
				tdata_size,
			);

			ptr::write_bytes(
				ptr.as_mut_ptr::<u8>()
					.offset(tdata_size.try_into().unwrap()),
				0,
				align_up!(tls_size, 32) - tdata_size,
			);

			// The x86-64 TLS specification also requires that the tls_pointer can be accessed at fs:0.
			// This allows TLS variable values to be accessed by "mov rax, fs:0" and a later "lea rdx, [rax+VARIABLE_OFFSET]".
			// See "ELF Handling For Thread-Local Storage", version 0.20 by Ulrich Drepper, page 12 for details.
			//
			// fs:0 is where tls_pointer points to and we have reserved space for a usize value above.
			//*(tls_pointer.as_mut_ptr::<u64>()) = tls_pointer.as_u64();
		}

		debug!(
			"Set up TLS at 0x{:x}, tdata_size 0x{:x}, tls_size 0x{:x}",
			tls_pointer, tdata_size, tls_size
		);

		Self {
			address: ptr,
			tp: tls_pointer,
			layout: layout,
		}
	}

	#[inline]
	pub fn address(&self) -> VirtAddr {
		self.address
	}

	#[inline]
	pub fn get_tp(&self) -> VirtAddr {
		self.tp
	}
}

impl Drop for TaskTLS {
	fn drop(&mut self) {
		debug!(
			"Deallocate TLS at 0x{:x} (layout {:?})",
			self.address, self.layout,
		);

		unsafe {
			dealloc(self.address.as_mut_ptr::<u8>(), self.layout);
		}
	}
}

impl Clone for TaskTLS {
	fn clone(&self) -> Self {
		TaskTLS::new(env::get_tls_memsz())
	}
}

extern "C" fn leave_task() -> ! {
	core_scheduler().exit(0);
}

#[no_mangle]
extern "C" fn task_entry(func: extern "C" fn(usize), arg: usize) {
	// Check if the task (process or thread) uses Thread-Local-Storage.
	/*let tls_size = unsafe { &tls_end as *const u8 as usize - &tls_start as *const u8 as usize };
	if tls_size > 0 {
		// Yes, it does, so we have to allocate TLS memory.
		// Allocate enough space for the given size and one more variable of type usize, which holds the tls_pointer.
		let tls_allocation_size = tls_size + mem::size_of::<usize>();
		let tls = TaskTLS::new(tls_allocation_size);

		// The tls_pointer is the address to the end of the TLS area requested by the task.
		let tls_pointer = tls.address() + tls_size;

		// TODO: Implement AArch64 TLS

		// Associate the TLS memory to the current task.
		let mut current_task_borrowed = core_scheduler().current_task.borrow_mut();
		debug!(
			"Set up TLS for task {} at address {:#X}",
			current_task_borrowed.id,
			tls.address()
		);
		current_task_borrowed.tls = Some(tls);
	}*/

	// Call the actual entry point of the task.
	//unsafe{debug!("state: {:#X?}", *((func as usize -31*8 ) as *const crate::arch::riscv::kernel::scheduler::State));}
	//panic!("Not impl");
	//println!("Task start");
	func(arg);
	//println!("Task end");

	// switch_to_kernel!();

	// Exit task
	core_scheduler().exit(0)
}

impl TaskFrame for Task {
	fn create_stack_frame(&mut self, func: extern "C" fn(usize), arg: usize) {
		// Check if the task (process or thread) uses Thread-Local-Storage.
		let tls_size = env::get_tls_memsz();
		// check is TLS is already allocated
		if self.tls.is_none() && tls_size > 0 {
			self.tls = Some(TaskTLS::new(tls_size))
		}

		unsafe {
			// Set a marker for debugging at the very top.
			let mut stack =
				self.stacks.get_kernel_stack() + self.stacks.get_kernel_stack_size() - 0x10u64;
			*stack.as_mut_ptr::<u64>() = 0xDEAD_BEEFu64;

			// Put the State structure expected by the ASM switch() function on the stack.
			stack = stack - mem::size_of::<State>();

			let state = stack.as_mut_ptr::<State>();
			ptr::write_bytes(stack.as_mut_ptr::<u8>(), 0, mem::size_of::<State>());

			if let Some(tls) = &self.tls {
				(*state).tp = tls.get_tp().as_usize();
			}
			(*state).ra = task_start as usize;
			(*state).a0 = func as usize;
			(*state).a1 = arg as usize;

			// Set the task's stack pointer entry to the stack we have just crafted.
			self.last_stack_pointer = stack;
			self.user_stack_pointer =
				self.stacks.get_user_stack() + self.stacks.get_user_stack_size() - 0x10u64;

			(*state).sp = self.last_stack_pointer.as_usize();
			(*state).a2 = self.user_stack_pointer.as_usize() - mem::size_of::<u64>();
			//debug!("state: {:#X?}", *state);
		}
		//panic!("create_stack_frame not implemented!")
	}
}

extern "C" {
	fn task_start(func: extern "C" fn(usize), arg: usize, user_stack: u64);
}

pub fn timer_handler() {
	//increment_irq_counter(apic::TIMER_INTERRUPT_NUMBER.into());
	core_scheduler().handle_waiting_tasks();
	set_oneshot_timer(None);
	core_scheduler().scheduler();
}

#[cfg(feature = "smp")]
pub fn wakeup_handler() {
	debug!("Received Wakeup Interrupt");
	//increment_irq_counter(WAKEUP_INTERRUPT_NUMBER.into());
	let core_scheduler = core_scheduler();
	core_scheduler.check_input();
	unsafe {
		sie::clear_ssoft();
	}
	if core_scheduler.is_scheduling() {
		core_scheduler.scheduler();
	}
}

#[inline(never)]
#[no_mangle]
pub fn set_current_kernel_stack() {
	core_scheduler().set_current_kernel_stack();
}
