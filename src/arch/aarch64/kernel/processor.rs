use core::arch::asm;
use core::hint::spin_loop;
use core::{fmt, str};

use aarch64::regs::CNTFRQ_EL0;
use hermit_dtb::Dtb;
use hermit_sync::{without_interrupts, Lazy};
use qemu_exit::QEMUExit;
use tock_registers::interfaces::Readable;

use crate::arch::aarch64::kernel::boot_info;
use crate::env;

// System counter frequency in Hz
static CPU_FREQUENCY: Lazy<CpuFrequency> = Lazy::new(|| {
	let mut cpu_frequency = CpuFrequency::new();
	unsafe {
		cpu_frequency.detect();
	}
	cpu_frequency
});

enum CpuFrequencySources {
	Invalid,
	CommandLine,
	Register,
}

impl fmt::Display for CpuFrequencySources {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match &self {
			CpuFrequencySources::CommandLine => write!(f, "Command Line"),
			CpuFrequencySources::Register => write!(f, "CNTFRQ_EL0"),
			_ => panic!("Attempted to print an invalid CPU Frequency Source"),
		}
	}
}

struct CpuFrequency {
	hz: u32,
	source: CpuFrequencySources,
}

impl CpuFrequency {
	const fn new() -> Self {
		CpuFrequency {
			hz: 0,
			source: CpuFrequencySources::Invalid,
		}
	}

	fn set_detected_cpu_frequency(
		&mut self,
		hz: u32,
		source: CpuFrequencySources,
	) -> Result<(), ()> {
		//The clock frequency must never be set to zero, otherwise a division by zero will
		//occur during runtime
		if hz > 0 {
			self.hz = hz;
			self.source = source;
			Ok(())
		} else {
			Err(())
		}
	}

	unsafe fn detect_from_cmdline(&mut self) -> Result<(), ()> {
		let mhz = env::freq().ok_or(())?;
		self.set_detected_cpu_frequency(u32::from(mhz) * 1000000, CpuFrequencySources::CommandLine)
	}

	unsafe fn detect_from_register(&mut self) -> Result<(), ()> {
		let hz = CNTFRQ_EL0.get() & 0xFFFFFFFF;
		self.set_detected_cpu_frequency(hz.try_into().unwrap(), CpuFrequencySources::Register)
	}

	unsafe fn detect(&mut self) {
		unsafe {
			self.detect_from_register()
				.or_else(|_e| self.detect_from_cmdline())
				.unwrap();
		}
	}

	fn get(&self) -> u32 {
		self.hz
	}
}

impl fmt::Display for CpuFrequency {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		write!(f, "{} Hz (from {})", self.hz, self.source)
	}
}

pub struct FPUState;

impl FPUState {
	pub fn new() -> Self {
		Self {}
	}

	pub fn restore(&self) {}

	pub fn save(&self) {}
}

pub fn seed_entropy() -> Option<[u8; 32]> {
	None
}

pub fn run_on_hypervisor() -> bool {
	true
}

/// Search the most significant bit
#[inline(always)]
pub fn msb(value: u64) -> Option<u64> {
	if value > 0 {
		let ret: u64;

		unsafe {
			asm!(
				"clz {0}, {1}",
				"sub {0}, {2}, {0}",
				out(reg) ret,
				in(reg) value,
				const 64 - 1,
				options(nostack, nomem),
			);
		}
		Some(ret)
	} else {
		None
	}
}

/// The halt function stops the processor until the next interrupt arrives
pub fn halt() {
	unsafe {
		asm!("wfi", options(nostack, nomem),);
	}
}

/// Shutdown the system
pub fn shutdown() -> ! {
	info!("Shutting down system");

	let exit_handler = qemu_exit::AArch64::new();
	exit_handler.exit_success();
}

#[inline]
pub fn get_timer_ticks() -> u64 {
	// We simulate a timer with a 1 microsecond resolution by taking the CPU timestamp
	// and dividing it by the CPU frequency in MHz.
	(1000000 * get_timestamp()) / u64::from(CPU_FREQUENCY.get())
}

#[inline]
pub fn get_frequency() -> u16 {
	(CPU_FREQUENCY.get() / 1000000).try_into().unwrap()
}

#[inline]
pub fn get_timestamp() -> u64 {
	let value: u64;

	unsafe {
		asm!(
			"mrs {value}, cntpct_el0",
			value = out(reg) value,
			options(nostack),
		);
	}

	value
}

#[inline]
pub fn supports_1gib_pages() -> bool {
	false
}

#[inline]
pub fn supports_2mib_pages() -> bool {
	false
}

/// Delay execution by the given number of microseconds using busy-waiting.
#[inline]
pub fn udelay(usecs: u64) {
	let end = get_timestamp() + get_frequency() as u64 * usecs;
	while get_timestamp() < end {
		spin_loop();
	}
}

pub fn configure() {
	// TODO: PMCCNTR_EL0 is the best replacement for RDTSC on AArch64.
	// However, this test code showed that it's apparently not supported under uhyve yet.
	// Finish the boot loader for QEMU first and then run this code under QEMU, where it should be supported.
	// If that's the case, find out what's wrong with uhyve.
	unsafe {
		// TODO: Setting PMUSERENR_EL0 is probably not required, but find out about that
		// when reading PMCCNTR_EL0 works at all.
		let pmuserenr_el0: u32 = 1 << 0 | 1 << 2 | 1 << 3;
		asm!(
			"msr pmuserenr_el0, {}",
			in(reg) pmuserenr_el0,
			options(nostack, nomem),
		);

		// TODO: Setting PMCNTENSET_EL0 is probably not required, but find out about that
		// when reading PMCCNTR_EL0 works at all.
		let pmcntenset_el0: u32 = 1 << 31;
		asm!(
			"msr pmcntenset_el0, {}",
			in(reg) pmcntenset_el0,
			options(nostack, nomem),
		);

		// Enable PMCCNTR_EL0 using PMCR_EL0.
		let mut pmcr_el0: u32 = 0;
		asm!(
			"mrs {}, pmcr_el0",
			out(reg) pmcr_el0,
			options(nostack, nomem),
		);
		debug!(
			"PMCR_EL0 (has RES1 bits and therefore mustn't be zero): {:#X}",
			pmcr_el0
		);
		pmcr_el0 |= 1 << 0 | 1 << 2 | 1 << 6;
		asm!(
			"msr pmcr_el0, {}",
			in(reg) pmcr_el0,
			options(nostack, nomem),
		);
	}
}

pub fn detect_frequency() {
	Lazy::force(&CPU_FREQUENCY);
}

#[inline]
fn __set_oneshot_timer(wakeup_time: Option<u64>) {
	if let Some(wt) = wakeup_time {
		// wt is the absolute wakeup time in microseconds based on processor::get_timer_ticks.
		let deadline = (wt * u64::from(CPU_FREQUENCY.get())) / 1000000;

		unsafe {
			asm!(
				"msr cntp_cval_el0, {value}",
				"msr cntp_ctl_el0, {enable}",
				value = in(reg) deadline,
				enable = in(reg) 1,
				options(nostack, nomem),
			);
		}
	} else {
		// disable timer
		unsafe {
			asm!(
				"msr cntp_cval_el0, {disable}",
				"msr cntp_ctl_el0, {disable}",
				disable = in(reg) 0,
				options(nostack, nomem),
			);
		}
	}
}

pub fn set_oneshot_timer(wakeup_time: Option<u64>) {
	without_interrupts(|| {
		__set_oneshot_timer(wakeup_time);
	});
}

pub fn print_information() {
	let dtb = unsafe {
		Dtb::from_raw(boot_info().hardware_info.device_tree.unwrap().get() as *const u8)
			.expect(".dtb file has invalid header")
	};

	let reg = dtb
		.get_property("/cpus/cpu@0", "compatible")
		.unwrap_or(b"unknown");

	infoheader!(" CPU INFORMATION ");
	infoentry!("Processor compatiblity", str::from_utf8(reg).unwrap());
	infoentry!("Counter frequency", *CPU_FREQUENCY);
	infofooter!();
}
