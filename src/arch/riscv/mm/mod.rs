pub mod addr;
pub mod paging;
pub mod physicalmem;
pub mod virtualmem;

pub use addr::PhysAddr;
pub use addr::VirtAddr;

pub use self::paging::init_page_tables;

pub fn init() {
	physicalmem::init();
	virtualmem::init();
}
