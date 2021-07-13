// Copyright (c) 2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use riscv::register::*;
use riscv::register::scause::Scause;
use riscv::asm::wfi;
use trapframe::TrapFrame;
use crate::arch::riscv::kernel::processor::set_oneshot_timer;

use crate::synch::spinlock::Spinlock;

/// base address of the PLIC, only one access at the same time is allowed
static PLIC_BASE: Spinlock<usize> = Spinlock::new(0x0);

const PLIC_PENDING_OFFSET: usize = 0x001000;
const PLIC_ENABLE_OFFSET: usize = 0x002000;

const MAX_IRQ: usize = 69;

static mut IRQ_HANDLERS: [usize; MAX_IRQ] = [0; MAX_IRQ];


/// Init Interrupts
pub fn install() {
	unsafe{
		// Intstall trap handler
		trapframe::init();
		// Enable external interrupts
		sie::set_sext();
	}
}

/// Init PLIC
pub fn init_plic(base: usize) {
	*PLIC_BASE.lock() = base;
}

/// Enable Interrupts
#[inline]
pub fn enable() {
	unsafe{
		sstatus::set_sie();
	}
}

/// Waits for the next interrupt (Only Supervisor-level software/timer interrupt for now)
/// and calls the specific handler
#[inline]
pub fn enable_and_wait() {
	unsafe{
		//Enable Supervisor-level software interrupts
		sie::set_ssoft();
		//sie::set_sext();
		debug!("Wait {:x?}", sie::read());
		loop{
			wfi();
			// Interrupts are disabled at this point, so a pending interrupt will
			// resume the execution. We still have to check if a interrupt is pending
			// because the WFI instruction could be implemented as NOP (The RISC-V Instruction Set ManualVolume II: Privileged Architecture)

			let pending_interrupts = sip::read();
			
			// debug!("sip: {:x?}", pending_interrupts);
			#[cfg(feature = "smp")]
			if pending_interrupts.ssoft() {
				//Clear pending interrupts, maybe only the bit for Supervisor-level software interrupt should be cleared
				asm!(
					"csrc sip, {ssoft_mask}",
					ssoft_mask = in(reg) 0x2,
				);
				trace!("SOFT");
				//Disable Supervisor-level software interrupt
				sie::clear_ssoft();
				//debug!("sip2: {:x?}", pending_interrupts);
				crate::arch::riscv::kernel::scheduler::wakeup_handler();
				break;
			}
			
			if pending_interrupts.sext() {
				trace!("EXT");
				external_handler();
				break;
			}
			
			if pending_interrupts.stimer() {
				// Disable Supervisor-level software interrupt
				sie::clear_ssoft();
				// Setting the timer clears the pending interrupt
				debug!("sip: {:x?}", pending_interrupts);
				set_oneshot_timer(None);
				trace!("TIMER");
				crate::arch::riscv::kernel::scheduler::timer_handler();
				break;
			}
		}

	}
}

/// Disable Interrupts
#[inline]
pub fn disable() {
	unsafe { 
		sstatus::clear_sie()
	};
}

/// Disable IRQs (nested)
///
/// Disable IRQs when unsure if IRQs were enabled at all.
/// This function together with nested_enable can be used
/// in situations when interrupts shouldn't be activated if they
/// were not activated before calling this function.
#[inline]
pub fn nested_disable() -> bool {
	let  was_enabled = sstatus::read().sie();

	disable();
	was_enabled
}

/// Enable IRQs (nested)
///
/// Can be used in conjunction with nested_disable() to only enable
/// interrupts again if they were enabled before.
#[inline]
pub fn nested_enable(was_enabled: bool) {
	if was_enabled {
		enable();
	}
}

/// Currently not needed because we use the trapframe crate
#[no_mangle]
pub extern "C" fn irq_install_handler(irq_number: u16, handler: usize) {
	unsafe{
		let base_ptr = PLIC_BASE.lock();
		debug!("Install handler for interrupt {}", irq_number);
		IRQ_HANDLERS[irq_number as usize - 1] = handler;
		// Set priority to 7 (highest)
		let prio_address = *base_ptr + irq_number as usize * 4;
		core::ptr::write_volatile(prio_address as *mut u32, 1);
		// Set Threshold to 0 (lowest)
		let thresh_address = *base_ptr + 0x20_2000;
		core::ptr::write_volatile(thresh_address as *mut u32, 0);
		// Enable irq for Hart 1 S-Mode
		let enable_address = *base_ptr + PLIC_ENABLE_OFFSET + 0x100 + ((irq_number/32)*4) as usize;
		debug!("enable_address {:x}", enable_address);
		core::ptr::write_volatile(enable_address as *mut u32, 1 << (irq_number % 32));
	}
}

// Derived from rCore: https://github.com/rcore-os/rCore
/// Dispatch and handle interrupt.
///
/// This function is called from `trap.S` which is in the trapframe crate.
#[no_mangle]
pub extern "C" fn trap_handler(tf: &mut TrapFrame) {
    use self::scause::{Exception as E, Interrupt as I, Trap};
    let scause = scause::read();
    let stval = stval::read();
	let sepc = sepc::read();
	trace!("Interrupt: {:?} ", scause.cause());
	trace!("tf: {:x?} ", tf);
	trace!("stvall: {:x}", stval);
	trace!("sepc: {:x}", sepc);
	trace!("SSTATUS FS: {:?}", sstatus::read().fs());
	trace!("FCSR: {:?}", fcsr::read());
	//loop{}
    match scause.cause() {
        Trap::Interrupt(I::SupervisorExternal) => external_handler(),
		#[cfg(feature = "smp")]
        Trap::Interrupt(I::SupervisorSoft) => crate::arch::riscv::kernel::scheduler::wakeup_handler(),
        Trap::Interrupt(I::SupervisorTimer) => crate::arch::riscv::kernel::scheduler::timer_handler(),
        //Trap::Exception(E::LoadPageFault) => page_fault(stval, tf),
        //Trap::Exception(E::StorePageFault) => page_fault(stval, tf),
        //Trap::Exception(E::InstructionPageFault) => page_fault(stval, tf),
        _ => panic!("unhandled trap {:?}", scause.cause()),
    }
    trace!("Interrupt end");
}


/// Handles external interrupts
fn external_handler() {
	unsafe {
		let handler: Option<fn()> = {
			// Claim interrupt
			let base_ptr = PLIC_BASE.lock();
			let claim_address = *base_ptr + 0x20_2004;
			let irq = core::ptr::read_volatile(claim_address as *mut u32);
			if irq != 0 {
				debug!("External INT: {}",irq);
				// Complete interrupt
				core::ptr::write_volatile(claim_address as *mut u32, irq);

				// Call handler
				if IRQ_HANDLERS[irq as usize - 1] != 0 {
					let ptr = IRQ_HANDLERS[irq as usize - 1] as *const ();
					let handler: fn() = unsafe { core::mem::transmute(ptr) };
					Some(handler)
				}
				else {
					error!("Interrupt handler not installed");
					None
				}
			}
			else {
				None
			}
		};

		if let Some(handler) = handler {
			handler();
		}
	}
}