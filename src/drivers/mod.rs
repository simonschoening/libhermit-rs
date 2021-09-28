// Copyright (c) 2019 Stefan Lankes, RWTH Aachen University
//               2020 Frederik Schulz, RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.
//
//! A module containing hermit-rs driver, hermit-rs driver trait and driver specific errors.
//!
//! The module contains ...

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// UNCOMMENTED FOR CORRECT USE STATEMENT; IS THIS CORRECT?
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#[cfg(any(
	target_arch = "riscv64",
	all(feature = "pci", not(target_arch = "aarch64"))
))]
pub mod net;

#[cfg(all(feature = "pci", not(target_arch = "aarch64")))]
pub mod virtio;

/// A common error module for drivers.
/// [DriverError](enums.drivererror.html) values will be
/// passed on to higher layers.
#[cfg(not(target_arch = "aarch64"))]
pub mod error {
	#[cfg(target_arch = "riscv64")]
	use crate::drivers::net::gem::GEMError;
	#[cfg(feature = "pci")]
	use crate::drivers::net::rtl8139::RTL8139Error;
	#[cfg(feature = "pci")]
	use crate::drivers::virtio::error::VirtioError;
	use core::fmt;

	#[derive(Debug)]
	pub enum DriverError {
		#[cfg(feature = "pci")]
		InitVirtioDevFail(VirtioError),
		#[cfg(feature = "pci")]
		InitRTL8139DevFail(RTL8139Error),
		#[cfg(target_arch = "riscv64")]
		InitGEMDevFail(GEMError),
	}

	#[cfg(feature = "pci")]
	impl From<VirtioError> for DriverError {
		fn from(err: VirtioError) -> Self {
			DriverError::InitVirtioDevFail(err)
		}
	}

	#[cfg(feature = "pci")]
	impl From<RTL8139Error> for DriverError {
		fn from(err: RTL8139Error) -> Self {
			DriverError::InitRTL8139DevFail(err)
		}
	}

	#[cfg(target_arch = "riscv64")]
	impl From<GEMError> for DriverError {
		fn from(err: GEMError) -> Self {
			DriverError::InitGEMDevFail(err)
		}
	}

	impl fmt::Display for DriverError {
		fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
			match *self {
				#[cfg(feature = "pci")]
				DriverError::InitVirtioDevFail(ref err) => {
					write!(f, "Virtio driver failed: {:?}", err)
				}
				#[cfg(feature = "pci")]
				DriverError::InitRTL8139DevFail(ref err) => {
					write!(f, "RTL8139 driver failed: {:?}", err)
				}
				#[cfg(target_arch = "riscv64")]
				DriverError::InitGEMDevFail(ref err) => {
					write!(f, "GEM driver failed: {:?}", err)
				}
			}
		}
	}
}
