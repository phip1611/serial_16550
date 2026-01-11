// SPDX-License-Identifier: MIT OR Apache-2.0

//! Provides a thin abstraction over a [`Uart16550`] for VT102-like terminal
//! emulators on the receiving side.
//!
//! This module is suited for basic use cases and toy projects, but full VT102
//! compatibility is explicitly not a goal.
//!
//! See [`Uart16550Tty`].

use crate::{Config, InvalidAddressError, Uart16550};
use crate::backend::{Backend, MmioAddress, MmioBackend, PioBackend, PortIoAddress};
use core::fmt;
use crate::spec::registers::offsets;

/// Thin abstraction over a [`Uart16550`] that helps to send Rust strings as
/// expected to the other side.
#[derive(Debug)]
pub struct Uart16550Tty<B: Backend>(Uart16550<B>);



#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
impl Uart16550Tty<PioBackend> {
    /// Creates a new [`Uart16550Tty`] backed by x86 port I/O.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the address is valid and safe to use.
    pub const unsafe fn new_port(
        base_port: u16,
        config: Config,
    ) -> Result<Self, InvalidAddressError<PortIoAddress>> {
        if base_port.checked_add(offsets::MAX as u16).is_none() {
            return Err(InvalidAddressError(PortIoAddress(base_port)));
        }

        let backend = PioBackend(PortIoAddress(base_port));
        let inner = Uart16550 { backend, config };
        Ok(Self(inner))
    }
}

impl Uart16550Tty<MmioBackend> {
    /// Creates a new [`Uart16550Tty`] backed by MMIO.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the address is valid and safe to use.
    pub unsafe fn new_mmio(
        base_address: *mut u8,
        config: Config,
    ) -> Result<Self, InvalidAddressError<MmioAddress>> {
        if base_address.is_null() {
            return Err(InvalidAddressError(MmioAddress(base_address)));
        }
        if (base_address as usize).checked_add(offsets::MAX).is_none() {
            return Err(InvalidAddressError(MmioAddress(base_address)));
        }

        let backend = MmioBackend(MmioAddress(base_address));
        let inner = Uart16550 { backend, config };
        Ok(Self(inner))
    }
}

impl<B: Backend> Uart16550Tty<B> {
    pub fn init(&mut self) {
        self.0.init().unwrap();
        self.0.test_loopback().unwrap();
        self.0.check_remote_ready_to_receive().unwrap();
    }
}

impl<B: Backend> fmt::Write for Uart16550Tty<B> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for &byte in s.as_bytes() {
            match byte {
                // backspace or delete
                8 | 0x7F => {
                    self.0.send_bytes_all(&[8]);
                    self.0.send_bytes_all(&[b' ']);
                    self.0.send_bytes_all(&[8]);
                }
                // Normal Rust newlines to terminal-compatible newlines.
                b'\n' => {
                    self.0.send_bytes_all(&[b'\r']);
                    self.0.send_bytes_all(&[b'\n']);
                }
                data => {
                    self.0.send_bytes_all(&[data]);
                }
            }
        }

        Ok(())
    }
}
