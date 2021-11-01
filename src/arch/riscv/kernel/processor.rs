use core::sync::atomic::spin_loop_hint;

use crate::arch::riscv::kernel::sbi;
use crate::arch::riscv::kernel::HARTS_AVAILABLE;
use crate::arch::riscv::kernel::{get_timebase_freq, is_uhyve};
use crate::scheduler::CoreId;
use core::convert::TryInto;
use riscv::asm::wfi;
use riscv::register::{sie, time};

/// Current FPU state. Saved at context switch when changed
#[repr(C, packed)]
#[derive(Debug)]
pub struct FPUState {
	/// f0 register
	f0: u64,
	/// f1 register
	f1: u64,
	/// f2 register
	f2: u64,
	/// f3 register
	f3: u64,
	/// f4 register
	f4: u64,
	/// f5 register
	f5: u64,
	/// f6 register
	f6: u64,
	/// f7 register
	f7: u64,
	/// f8 register
	f8: u64,
	/// f9 register
	f9: u64,
	/// f10 register
	f10: u64,
	/// f11 register
	f11: u64,
	/// f12 register
	f12: u64,
	/// f13 register
	f13: u64,
	/// f14 register
	f14: u64,
	/// f15 register
	f15: u64,
	/// f16 register
	f16: u64,
	/// f17 register
	f17: u64,
	/// f18 register
	f18: u64,
	/// f19 register
	f19: u64,
	/// f20 register
	f20: u64,
	/// f21 register
	f21: u64,
	/// f22 register
	f22: u64,
	/// f23 register
	f23: u64,
	/// f24 register
	f24: u64,
	/// f25 register
	f25: u64,
	/// f26 register
	f26: u64,
	/// f27 register
	f27: u64,
	/// f28 register
	f28: u64,
	/// f29 register
	f29: u64,
	/// f30 register
	f30: u64,
	/// f31 register
	f31: u64,
	/// fcsr register
	fcsr: usize,
}

impl FPUState {
	pub fn new() -> Self {
		Self {
			f0: 0,
			f1: 0,
			f2: 0,
			f3: 0,
			f4: 0,
			f5: 0,
			f6: 0,
			f7: 0,
			f8: 0,
			f9: 0,
			f10: 0,
			f11: 0,
			f12: 0,
			f13: 0,
			f14: 0,
			f15: 0,
			f16: 0,
			f17: 0,
			f18: 0,
			f19: 0,
			f20: 0,
			f21: 0,
			f22: 0,
			f23: 0,
			f24: 0,
			f25: 0,
			f26: 0,
			f27: 0,
			f28: 0,
			f29: 0,
			f30: 0,
			f31: 0,
			fcsr: 0,
		}
	}

	pub fn restore(&self) {
		unsafe {
			trace!("Restore FPUState at {:p} with {:x?}", self, self);
			asm! {
				"fld f0, (8*0)({fpu_state})",
				"fld f1, (8*1)({fpu_state})",
				"fld f2, (8*2)({fpu_state})",
				"fld f3, (8*3)({fpu_state})",
				"fld f4, (8*4)({fpu_state})",
				"fld f5, (8*5)({fpu_state})",
				"fld f6, (8*6)({fpu_state})",
				"fld f7, (8*7)({fpu_state})",
				"fld f8, (8*8)({fpu_state})",
				"fld f9, (8*9)({fpu_state})",
				"fld f10, (8*10)({fpu_state})",
				"fld f11, (8*11)({fpu_state})",
				"fld f12, (8*12)({fpu_state})",
				"fld f13, (8*13)({fpu_state})",
				"fld f14, (8*14)({fpu_state})",
				"fld f15, (8*15)({fpu_state})",
				"fld f16, (8*16)({fpu_state})",
				"fld f17, (8*17)({fpu_state})",
				"fld f18, (8*18)({fpu_state})",
				"fld f19, (8*19)({fpu_state})",
				"fld f20, (8*20)({fpu_state})",
				"fld f21, (8*21)({fpu_state})",
				"fld f22, (8*22)({fpu_state})",
				"fld f23, (8*23)({fpu_state})",
				"fld f24, (8*24)({fpu_state})",
				"fld f25, (8*25)({fpu_state})",
				"fld f26, (8*26)({fpu_state})",
				"fld f27, (8*27)({fpu_state})",
				"fld f28, (8*28)({fpu_state})",
				"fld f29, (8*29)({fpu_state})",
				"fld f30, (8*30)({fpu_state})",
				"fld f31, (8*31)({fpu_state})",
				"ld t0, (8*32)({fpu_state})",
				"fscsr t0",
				fpu_state = in(reg) self as *const _,
				out("t0") _,
			}
		}
	}

	pub fn save(&mut self) {
		unsafe {
			trace!("Save FPUState at {:p}", self);
			asm! {
				"fsd f0, (8*0)({fpu_state})",
				"fsd f1, (8*1)({fpu_state})",
				"fsd f2, (8*2)({fpu_state})",
				"fsd f3, (8*3)({fpu_state})",
				"fsd f4, (8*4)({fpu_state})",
				"fsd f5, (8*5)({fpu_state})",
				"fsd f6, (8*6)({fpu_state})",
				"fsd f7, (8*7)({fpu_state})",
				"fsd f8, (8*8)({fpu_state})",
				"fsd f9, (8*9)({fpu_state})",
				"fsd f10, (8*10)({fpu_state})",
				"fsd f11, (8*11)({fpu_state})",
				"fsd f12, (8*12)({fpu_state})",
				"fsd f13, (8*13)({fpu_state})",
				"fsd f14, (8*14)({fpu_state})",
				"fsd f15, (8*15)({fpu_state})",
				"fsd f16, (8*16)({fpu_state})",
				"fsd f17, (8*17)({fpu_state})",
				"fsd f18, (8*18)({fpu_state})",
				"fsd f19, (8*19)({fpu_state})",
				"fsd f20, (8*20)({fpu_state})",
				"fsd f21, (8*21)({fpu_state})",
				"fsd f22, (8*22)({fpu_state})",
				"fsd f23, (8*23)({fpu_state})",
				"fsd f24, (8*24)({fpu_state})",
				"fsd f25, (8*25)({fpu_state})",
				"fsd f26, (8*26)({fpu_state})",
				"fsd f27, (8*27)({fpu_state})",
				"fsd f28, (8*28)({fpu_state})",
				"fsd f29, (8*29)({fpu_state})",
				"fsd f30, (8*30)({fpu_state})",
				"fsd f31, (8*31)({fpu_state})",
				"frcsr t0",
				"sd t0, (8*32)({fpu_state})",
				fpu_state = in(reg) self as *mut _,
				out("t0") _,
			}
		}
	}
}

pub fn generate_random_number32() -> Option<u32> {
	None
}

pub fn generate_random_number64() -> Option<u64> {
	None
}

pub fn run_on_hypervisor() -> bool {
	true
}

/// Search the most significant bit, indices start at 0
#[inline(always)]
pub fn msb(value: u64) -> Option<u64> {
	if value > 0 {
		let ret: u64 = 63 - value.leading_zeros() as u64;
		Some(ret)
	} else {
		None
	}
}

/// Search the least significant bit, indices start at 0
#[inline(always)]
pub fn lsb(value: u64) -> Option<u64> {
	if value > 0 {
		let ret: u64 = value.trailing_zeros() as u64;
		Some(ret)
	} else {
		None
	}
}

/// The halt function stops the processor until the next interrupt arrives
pub fn halt() {
	unsafe {
		wfi();
	}
}

/// Shutdown the system
pub fn shutdown() -> ! {
	info!("Shutting down system");
	//SBI shutdown
	sbi::shutdown_legacy()
}

pub fn get_timer_ticks() -> u64 {
	// We simulate a timer with a 1 microsecond resolution by taking the CPU timestamp
	// and dividing it by the CPU frequency in MHz.
	get_timestamp() / u64::from(get_frequency())
}

pub fn get_frequency() -> u16 {
	(get_timebase_freq() / 1000000).try_into().unwrap()
}

#[inline]
pub fn get_timestamp() -> u64 {
	time::read64()
}

pub fn supports_1gib_pages() -> bool {
	true
}

/// Delay execution by the given number of microseconds using busy-waiting.
#[inline]
pub fn udelay(usecs: u64) {
	let end = get_timestamp() + get_frequency() as u64 * usecs;
	while get_timestamp() < end {
		spin_loop_hint();
	}
}

pub fn set_oneshot_timer(wakeup_time: Option<u64>) {
	if let Some(wt) = wakeup_time {
		debug!("Starting Timer: {:x}", get_timestamp());
		unsafe {
			sie::set_stimer();
		}
		let next_time = wt * u64::from(get_frequency());

		sbi::set_timer(next_time);
	} else {
		// Disable the Timer (and clear a pending interrupt)
		debug!("Stopping Timer");
		sbi::set_timer(u64::MAX);
	}
}

pub fn wakeup_core(core_to_wakeup: CoreId) {
	let hart_id = unsafe { HARTS_AVAILABLE[core_to_wakeup as usize] };
	debug!("Wakeup core: {} , hart_id: {}", core_to_wakeup, hart_id);
	sbi::send_ipi(1 << hart_id);
}
