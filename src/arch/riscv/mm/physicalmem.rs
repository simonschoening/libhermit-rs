use core::sync::atomic::{AtomicUsize, Ordering};
use core::{alloc::AllocError, convert::TryInto};

use crate::arch::riscv::kernel::get_limit;
use crate::arch::riscv::kernel::get_ram_address;
use crate::arch::riscv::mm::paging::{BasePageSize, PageSize};
use crate::arch::riscv::mm::PhysAddr;
use crate::mm;
use crate::mm::freelist::{FreeList, FreeListEntry};
use crate::synch::spinlock::SpinlockIrqSave;

static PHYSICAL_FREE_LIST: SpinlockIrqSave<FreeList> = SpinlockIrqSave::new(FreeList::new());
static TOTAL_MEMORY: AtomicUsize = AtomicUsize::new(0);

fn detect_from_limits() -> Result<(), ()> {
	let limit = get_limit();
	if unsafe { limit } == 0 {
		return Err(());
	}

	let entry = FreeListEntry::new(
		mm::kernel_end_address().as_usize(),
		get_ram_address().as_usize() + limit,
	);
	PHYSICAL_FREE_LIST.lock().list.push_back(entry);
	TOTAL_MEMORY.store(limit, Ordering::SeqCst);

	Ok(())
}

pub fn init() {
	detect_from_limits().unwrap();
}

pub fn total_memory_size() -> usize {
	TOTAL_MEMORY.load(Ordering::SeqCst)
}

pub fn allocate(size: usize) -> Result<PhysAddr, AllocError> {
	assert!(size > 0);
	assert_eq!(
		size % BasePageSize::SIZE,
		0,
		"Size {:#X} is not a multiple of {:#X}",
		size,
		BasePageSize::SIZE
	);

	Ok(PhysAddr(
		PHYSICAL_FREE_LIST
			.lock()
			.allocate(size, None)?
			.try_into()
			.unwrap(),
	))
}

pub fn allocate_aligned(size: usize, alignment: usize) -> Result<PhysAddr, AllocError> {
	assert!(size > 0);
	assert!(alignment > 0);
	assert_eq!(
		size % alignment,
		0,
		"Size {:#X} is not a multiple of the given alignment {:#X}",
		size,
		alignment
	);
	assert_eq!(
		alignment % BasePageSize::SIZE,
		0,
		"Alignment {:#X} is not a multiple of {:#X}",
		alignment,
		BasePageSize::SIZE
	);

	Ok(PhysAddr(
		PHYSICAL_FREE_LIST
			.lock()
			.allocate(size, Some(alignment))?
			.try_into()
			.unwrap(),
	))
}

/// This function must only be called from mm::deallocate!
/// Otherwise, it may fail due to an empty node pool (POOL.maintain() is called in virtualmem::deallocate)
pub fn deallocate(physical_address: PhysAddr, size: usize) {
	assert!(
		physical_address >= PhysAddr(mm::kernel_end_address().as_u64()),
		"Physical address {:#X} is not >= KERNEL_END_ADDRESS",
		physical_address
	);
	assert!(size > 0);
	assert_eq!(
		size % BasePageSize::SIZE,
		0,
		"Size {:#X} is not a multiple of {:#X}",
		size,
		BasePageSize::SIZE
	);

	PHYSICAL_FREE_LIST
		.lock()
		.deallocate(physical_address.as_usize(), size);
}

pub fn print_information() {
	PHYSICAL_FREE_LIST
		.lock()
		.print_information(" PHYSICAL MEMORY FREE LIST ");
}
