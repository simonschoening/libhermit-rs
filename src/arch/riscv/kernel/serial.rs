// Copyright (c) 2018 Colin Finck, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

// TODO: sifive UART
use core::ptr;
use crate::arch::riscv::kernel::sbi;

pub struct SerialPort {
}

impl SerialPort {
	pub const fn new(port_address: u32) -> Self {
		Self {}
	}

	pub fn write_byte(&self, byte: u8) {
		// LF newline characters need to be extended to CRLF over a real serial port.
		if byte == b'\n' {
			sbi::console_putchar('\r' as usize);
		}

		sbi::console_putchar(byte as usize);
	}

	pub fn init(&self, baudrate: u32) {
		// We don't do anything here (yet).
	}
}
