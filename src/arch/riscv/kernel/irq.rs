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

/// Init Interrupts
pub fn install() {
	unsafe{
		// Intstall trap handler
		trapframe::init();
	}
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
		loop{
			wfi();
			// Interrupts are disabled at this point, so a pending interrupt will
			// resume the execution. We still have to check if a interrupt is pending
			// because the WFI instruction could be implemented as NOP (The RISC-V Instruction Set ManualVolume II: Privileged Architecture)

			let pending_interrupts = sip::read();
			
			//debug!("sip: {:x?}", pending_interrupts);
			#[cfg(feature = "smp")]
			if pending_interrupts.ssoft() {
				//Clear pending interrupts, maybe only the bit for Supervisor-level software interrupt should be cleared
				asm!(
					"csrw sip, zero"
				);
				//Disable Supervisor-level software interrupt
				sie::clear_ssoft();
				//debug!("sip2: {:x?}", pending_interrupts);
				crate::arch::riscv::kernel::scheduler::wakeup_handler();
				break;
			}
			
			if pending_interrupts.stimer() {
				// Disable Supervisor-level software interrupt
				sie::clear_ssoft();
				// Setting the timer clears the pending interrupt
				set_oneshot_timer(None);
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
pub extern "C" fn irq_install_handler(handler: usize) {
	info!("Install handler for interrupts");
	// TODO direct or vector?
	//unsafe{stvec::write(handler,stvec::TrapMode::Direct);}
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
    //trace!("Interrupt @ CPU{}: {:?} ", super::cpu::id(), scause.cause());
	trace!("Interrupt: {:?} ", scause.cause());
	trace!("tf: {:x?} ", tf);
	trace!("stvall: {:x}", stval);
	trace!("sepc: {:x}", sepc);
	//loop{}
    match scause.cause() {
        //Trap::Interrupt(I::SupervisorExternal) => external(),
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