// Copyright (c) 2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use core::sync::atomic::spin_loop_hint;

use riscv::asm::wfi;
use crate::arch::riscv::kernel::sbi;
use riscv::register::{time, sie};
use crate::arch::riscv::kernel::{get_timebase_freq, is_uhyve};
use crate::arch::riscv::kernel::HARTS_AVAILABLE;
use core::convert::TryInto;
use crate::scheduler::CoreId;

/// Only a dummy implementation. The state of the floating point registers are already saved/restored on context switch
pub struct FPUState {
	// dummy
}

impl FPUState {
	pub fn new() -> Self {
		Self {}
	}

	pub fn restore(&self) {
		
	}

	pub fn save(&self) {
		
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
		let ret: u64 = 63-value.leading_zeros() as u64;
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
	(get_timebase_freq()/1000000).try_into().unwrap()
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
		unsafe{
			sie::set_stimer();
		}
		let next_time = wt * u64::from(get_frequency());
		
		sbi::set_timer(next_time);
	}
	else{
		// Disable the Timer.
		debug!("Stopping Timer");
		sbi::set_timer(u64::MAX);
	}
}

pub fn wakeup_core(core_to_wakeup: CoreId) {
	let hart_id =  unsafe{
		HARTS_AVAILABLE[core_to_wakeup as usize]
	};
	debug!("Wakeup core: {} , hart_id: {}", core_to_wakeup, hart_id);
	sbi::send_ipi(1 << hart_id);
}
