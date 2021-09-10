// Copyright (c) 2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use crate::arch::riscv::kernel::BOOT_INFO;
use crate::environment;

pub fn get_boot_time() -> u64 {
	unsafe { core::ptr::read_volatile(&(*BOOT_INFO).boot_gtod) }
}
