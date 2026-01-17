// SPDX-License-Identifier: MIT OR Apache-2.0

//! # uart_16550
//!
//! Simple yet highly configurable low-level driver for
//! [16550 UART devices][uart], typically known and used as serial ports or
//! COM ports. Easy integration into Rust while providing fine-grained control
//! where needed (e.g., for kernel drivers).
//!
//! The "serial device" or "COM port" in typical x86 machines is almost always
//! backed by a **16550 UART devices**, may it be physical or emulated. This
//! crate offers convenient and powerful abstractions for these devices, and
//! also works for other architectures, such as ARM or RISC-V, by offering
//! support for MMIO-mapped devices.
//!
//! Serial ports are especially useful for debugging or operating system
//! learning projects. See [`Uart16550`] to get started.
//!
//! ## Features
//!
//! - ✅ Full configure, transmit, receive, and interrupt support for UART
//!   16550–compatible devices
//! - ✅ High-level, ergonomic abstractions and types paired with support for
//!   plain integers
//! - ✅ Very easy to integrate, highly configurable when needed
//! - ✅ Validated on **real hardware** as well as across different virtual
//!   machines
//! - ✅ Fully type-safe and derived directly from the official
//!   [specification][uart]
//! - ✅ Supports both **x86 port-mapped I/O** and **memory-mapped I/O** (MMIO)
//! - ✅ `no_std`-compatible and allocation-free by design
//!
//! ## Focus, Scope & Limitations
//!
//! While serial ports are often used in conjunction with VT102-like terminal
//! emulation, the primary focus of this crate is strict specification
//! compliance and convenient direct access to the underlying hardware for
//! transmitting and receiving bytes, including all necessary device
//! configuration.
//!
//! For basic terminal-related functionality, such as newline normalization and
//! backspace handling, we provide [`Uart16550Tty`] as a **basic** convenience
//! layer.
//!
//! # Overview
//!
//! Use [`Uart16550Tty`] for a quick start. For more fine-grained low-level
//! control, please have a look at [`Uart16550`] instead.
//!
//! # Example (Minimalistic)
//!
//! ```rust,no_run
//! use uart_16550::{Config, Uart16550Tty};
//! use core::fmt::Write;
//!
//! // SAFETY: The I/O port is valid and we have exclusive access.
//! let mut uart = unsafe { Uart16550Tty::new_port(0x3f8, Config::default()).expect("should initialize device") };
//! //                                 ^ you could also use `new_mmio(0x1000 as *mut _)` here
//! uart.write_str("hello world\nhow's it going?");
//! ```
//!
//! See [`Uart16550Tty`] for more details.
//!
//! # Example (More low-level control)
//!
//! ```rust,no_run
//! use uart_16550::{Config, Uart16550};
//!
//! // SAFETY: The I/O port is valid and we have exclusive access.
//! let mut uart = unsafe { Uart16550::new_port(0x3f8, Config::default()).expect("should be valid port") };
//! //                                 ^ you could also use `new_mmio(0x1000 as *mut _)` here
//! uart.init().expect("should init device successfully");
//! uart.test_loopback().expect("should have working loopback mode");
//! uart.check_remote_ready_to_receive().expect("should have physically connected receiver");
//! uart.send_bytes_all(b"hello world!");
//! ```
//!
//! See [`Uart16550`] for more details.
//!
//! [uart]: https://en.wikipedia.org/wiki/16550_UART

#![no_std]
#![deny(
    clippy::all,
    clippy::cargo,
    clippy::nursery,
    clippy::must_use_candidate,
    clippy::missing_safety_doc,
    clippy::undocumented_unsafe_blocks,
    clippy::needless_pass_by_value
)]
#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(rustdoc::all)]

#[cfg(test)]
extern crate std;

pub use crate::config::*;
pub use crate::error::*;
pub use crate::tty::*;

use crate::backend::{Backend, MmioAddress, MmioBackend};
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
use crate::backend::{PioBackend, PortIoAddress};
use crate::spec::registers::{DLL, DLM, FCR, IER, ISR, LCR, LSR, MCR, MSR, SPR, offsets};
use crate::spec::{calc_baud_rate, calc_divisor};
use core::str::from_utf8;

pub mod spec;

mod backend;
mod config;
mod error;
mod tty;

/// Powerful abstraction over a [16550 UART device][uart] with access to
/// low-level details paired with strong flexibility for higher-level layers.
///
/// All reads and writes involving device register from/to that device operate
/// on the underlying hardware.
///
/// This type is generic over x86 port I/O and MMIO via the corresponding
/// constructors (`[Uart16550::new_port()]` and [`Uart16550::new_mmio()]`.
///
/// # Example (Minimal)
///
/// ```rust,no_run
/// use uart_16550::{Config, Uart16550};
///
/// // SAFETY: The I/O port is valid and we have exclusive access.
/// let mut uart = unsafe { Uart16550::new_port(0x3f8, Config::default()).unwrap() };
/// //                                 ^ you could also use `new_mmio(0x1000 as *mut _)` here
/// uart.init().unwrap();
/// uart.send_bytes_all(b"hello world!");
/// ```
///
/// # Example (Recommended)
///
/// ```rust,no_run
/// use uart_16550::{Config, Uart16550};
///
/// // SAFETY: The I/O port is valid and we have exclusive access.
/// let mut uart = unsafe { Uart16550::new_port(0x3f8, Config::default()).expect("should be valid port") };
/// //                                 ^ you could also use `new_mmio(0x1000 as *mut _)` here
/// uart.init().expect("should init device successfully");
/// uart.test_loopback().expect("should have working loopback mode");
/// uart.check_remote_ready_to_receive().expect("should have physically connected receiver");
/// uart.send_bytes_all(b"hello world!");
/// ```
///
/// # Sending and Receiving Data
///
/// - [`Uart16550::try_send_byte`]: try to send a single byte
/// - [`Uart16550::send_bytes`]: try to send provided bytes and return `n`
/// - [`Uart16550::send_bytes_all`]: send all provided bytes
/// - [`Uart16550::try_receive_byte`]: try to receive a single byte
/// - [`Uart16550::receive_bytes`]: try to receive bytes into a buffer and return
///   `n`
/// - [`Uart16550::receive_bytes_all`]: receive bytes until provided buffer is
///   filled
///
/// [uart]: https://en.wikipedia.org/wiki/16550_UART
#[derive(Debug)]
pub struct Uart16550<B: Backend> {
    backend: B,
    base_address: B::A,
    // The currently active config.
    config: Config,
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
impl Uart16550<PioBackend> {
    /// Creates a new [`Uart16550`] backed by x86 port I/O.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the address is valid and safe to use.
    pub const unsafe fn new_port(
        base_port: u16,
        config: Config,
    ) -> Result<Self, InvalidAddressError<PortIoAddress>> {
        let base_address = PortIoAddress(base_port);
        if base_port.checked_add(offsets::MAX as u16).is_none() {
            return Err(InvalidAddressError(base_address));
        }

        let backend = PioBackend(base_address);

        Ok(Self { backend, base_address, config })
    }
}

impl Uart16550<MmioBackend> {
    /// Creates a new [`Uart16550`] backed by MMIO.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the address is valid and safe to use.
    pub unsafe fn new_mmio(
        base_address: *mut u8,
        config: Config,
    ) -> Result<Self, InvalidAddressError<MmioAddress>> {
        let base_address = MmioAddress(base_address);
        if base_address.0.is_null() {
            return Err(InvalidAddressError(base_address));
        }
        if (base_address.0 as usize).checked_add(offsets::MAX).is_none() {
            return Err(InvalidAddressError(base_address));
        }

        let backend = MmioBackend(base_address);

        Ok(Self { backend, base_address, config })
    }
}

impl<B: Backend> Uart16550<B> {
    /* ----- Init, Setup, Tests --------------------------------------------- */

    /// Initializes the devices according to the provided [`Config`] including a
    /// few typical as well as opinionated settings so that afterwards, the
    /// device can properly receive data and send data.
    ///
    /// It is recommended to call [`Self::test_loopback`] next to check the
    /// device works.
    ///
    /// This function also tries to detect if the UART is present at all by
    /// writing a byte to the [`SPR`] register and reading from it afterwards.
    ///
    /// # Safety
    ///
    /// Callers must ensure that using this type with the underlying hardware
    /// is done only in a context where such operations are valid and safe.
    ///
    /// Further, the serial config must match the expectations of the wire and
    /// the other side. Otherwise, garbage will be received.
    pub fn init(&mut self) -> Result<(), InitError> {
        // SPR test: write something and try to read it again.
        // => detect if UART16550 is there
        // SAFETY: We operate on valid register addresses.
        unsafe {
            let write = 0x42;
            self.backend.write(offsets::SPR as u8, write);
            let read = self.backend.read(offsets::SPR as u8);

            if read != write {
                return Err(InitError::DeviceNotPresent);
            }
        }

        // Disable all interrupts (for now).
        // SAFETY: We operate on valid register addresses.
        unsafe {
            self.backend.write(offsets::IER as u8, 0);
        }

        // Set baud rate.
        // SAFETY: We operate on valid register addresses.
        unsafe {
            // Set Divisor Latch Access Bit (DLAB) to access DLL and DLM next
            self.backend.write(offsets::LCR as u8, LCR::DLAB.bits());

            let divisor = calc_divisor(
                self.config.frequency,
                self.config.baud_rate.to_integer(),
                self.config.prescaler_division_factor,
            )
            .map_err(InitError::InvalidBaudRate)?;

            let low = (divisor & 0xff) as u8;
            let high = ((divisor >> 8) & 0xff) as u8;
            self.backend.write(offsets::DLL as u8, low);
            self.backend.write(offsets::DLM as u8, high);

            // Clear DLAB
            self.backend.write(offsets::LCR as u8, 0);
        }

        // Set line control register.
        // SAFETY: We operate on valid register addresses.
        unsafe {
            let mut lcr = LCR::from_bits_retain(0);
            lcr = lcr.set_word_length(self.config.data_bits);
            if self.config.extra_stop_bits {
                lcr |= LCR::MORE_STOP_BITS;
            }
            lcr = lcr.set_parity(self.config.parity);
            // don't set break
            // don't set DLAB
            self.backend.write(offsets::LCR as u8, lcr.bits());
        }

        // Set fifo control register.
        // SAFETY: We operate on valid register addresses.
        unsafe {
            let mut fcr = FCR::from_bits_retain(0);
            if self.config.fifo_trigger_level.is_some() {
                fcr |= FCR::FIFO_ENABLE;
            }
            fcr |= FCR::RX_FIFO_RESET;
            fcr |= FCR::TX_FIFO_RESET;
            // don't set DMA mode
            if let Some(level) = self.config.fifo_trigger_level {
                fcr = fcr.set_fifo_trigger_level(level);
            }

            self.backend.write(offsets::FCR as u8, fcr.bits());
        }

        // Set modem control register.
        // SAFETY: We operate on valid register addresses.
        unsafe {
            let mut mcr = MCR::from_bits_retain(0);
            // signal that we are powered one
            mcr |= MCR::DTR;
            // signal that we are ready and configured
            mcr |= MCR::RTS;
            // enable interrupt routing to the interrupt controller
            // (so far individual interrupts are still disabled in IER)
            mcr |= MCR::OUT_2_INT_ENABLE;

            self.backend.write(offsets::MCR as u8, mcr.bits());
        }

        // Set interrupts.
        // SAFETY: We operate on valid register addresses.
        unsafe {
            self.backend
                .write(offsets::IER as u8, self.config.interrupts.bits());
        }

        Ok(())
    }

    /// Tests the device in loopback mode.
    ///
    /// The FIFO must be configured for this test.
    ///
    /// It is **recommended** to call this function **after** [`Self::init`].
    pub fn test_loopback(&mut self) -> Result<(), LoopbackFailedError> {
        /// Single test byte. Chosen arbitrarily.
        const TEST_BYTE: u8 = 42;
        /// Test message. Must be smaller then [`FIFO_SIZE`].
        const TEST_MESSAGE: &str = "hello world!";

        // SAFETY: We operate on valid register addresses.
        unsafe {
            let old_mcr = self.mcr();
            self.backend
                .write(offsets::MCR as u8, MCR::LOOP_BACK.bits());

            // First: check a single byte
            {
                self.try_send_byte(TEST_BYTE)
                    .map_err(|_| LoopbackFailedError)?;
                let read = self.try_receive_byte().map_err(|_| LoopbackFailedError)?;
                if read != TEST_BYTE {
                    return Err(LoopbackFailedError);
                }
            }

            // Now check sending and reading a whole message
            {
                let n = self.send_bytes(TEST_MESSAGE.as_bytes());
                if n != TEST_MESSAGE.len() {
                    return Err(LoopbackFailedError);
                }

                let mut read_buffer = [0_u8; TEST_MESSAGE.len()];
                let n = self.receive_bytes(&mut read_buffer);
                if n != TEST_MESSAGE.len() {
                    return Err(LoopbackFailedError);
                }
                let string = from_utf8(&read_buffer).map_err(|_| LoopbackFailedError)?;

                if string != TEST_MESSAGE {
                    return Err(LoopbackFailedError);
                }
            }

            // restore MCR
            self.backend.write(offsets::MCR as u8, old_mcr.bits());
        }

        Ok(())
    }

    /// Performs some checks to see if the UART is connected to a physical
    /// device.
    ///
    /// Once this check succeeds, one can see the connection as established.
    /// A [`InterruptType::ModemStatus`] may indicate that this check needs to
    /// be performed again.
    ///
    /// [`InterruptType::ModemStatus`]: crate::spec::registers::InterruptType::ModemStatus
    pub fn check_remote_ready_to_receive(&mut self) -> Result<(), RemoteReadyToReceiveError> {
        // SAFETY: We operate on valid register addresses.
        let msr = unsafe { self.backend.read(offsets::MSR as u8) };
        // SAFETY: All possible bits are typed.
        let msr = unsafe { MSR::from_bits(msr).unwrap_unchecked() };
        if !msr.contains(MSR::DSR) {
            return Err(RemoteReadyToReceiveError::NoRemoteConnected);
        }
        if !msr.contains(MSR::CD) {
            return Err(RemoteReadyToReceiveError::RemoteNotConfigured);
        }
        if !msr.contains(MSR::CTS) {
            return Err(RemoteReadyToReceiveError::RemoteNotClearToSend);
        }
        Ok(())
    }

    /* ----- User I/O ------------------------------------------------------- */

    /// Tries to read a raw byte from the device.
    ///
    /// This will receive whatever a remote has sent to us.
    pub fn try_receive_byte(&mut self) -> Result<u8, ByteReceiveError> {
        let lsr = self.lsr();

        if !lsr.contains(LSR::DATA_READY) {
            return Err(ByteReceiveError);
        }

        // SAFETY: We operate on valid register addresses.
        let byte = unsafe { self.backend.read(offsets::DATA as u8) };

        Ok(byte)
    }

    /// Tries to write a raw byte to the device.
    ///
    /// This will be transmitted to the remote.
    pub fn try_send_byte(&mut self, byte: u8) -> Result<(), ByteSendError> {
        let lsr = self.lsr();
        let msr = self.msr();

        if !lsr.contains(LSR::THR_EMPTY) {
            return Err(ByteSendError::NoCapacity);
        }

        if !msr.contains(MSR::CTS) {
            return Err(ByteSendError::RemoteNotClearToSend);
        }

        // SAFETY: We operate on valid register addresses.
        unsafe {
            self.backend.write(offsets::DATA as u8, byte);
        }

        Ok(())
    }

    /// Tries to receive bytes from the device and writes them into the provided
    /// buffer.
    ///
    /// This function returns the number of bytes that have been received and
    /// put into the buffer.
    pub fn receive_bytes(&mut self, buffer: &mut [u8]) -> usize {
        buffer
            .iter_mut()
            .map_while(|slot: &mut u8| {
                self.try_receive_byte().ok().map(|byte| {
                    *slot = byte;
                })
            })
            .count()
    }

    /// Tries to send bytes from the device to the remote.
    ///
    /// This function returns the number of bytes that have been sent to the
    /// remote.
    pub fn send_bytes(&mut self, buffer: &[u8]) -> usize {
        buffer
            .iter()
            .map_while(|byte: &u8| self.try_send_byte(*byte).ok())
            .count()
    }

    /// Similar to [`Self::receive_bytes`] but blocks until enough bytes were
    /// read to fully fill the buffer.
    pub fn receive_bytes_all(&mut self, buffer: &mut [u8]) {
        for slot in buffer {
            // Loop until we can fill the slot.
            loop {
                if let Ok(byte) = self.try_receive_byte() {
                    *slot = byte;
                    break;
                }
            }
        }
    }

    /// Similar to [`Self::send_bytes`] but blocks until all bytes were
    /// written entirely to the remote.
    pub fn send_bytes_all(&mut self, bytes: &[u8]) {
        for byte in bytes {
            // Loop until we can send the byte.
            loop {
                if self.try_send_byte(*byte).is_ok() {
                    break;
                }
            }
        }
    }

    /* ----- Typed Register Getters ----------------------------------------- */

    /// Fetches the current value from the [`IER`].
    pub fn ier(&mut self) -> IER {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::IER as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { IER::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`ISR`].
    pub fn isr(&mut self) -> ISR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::ISR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { ISR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`FCR`].
    pub fn fcr(&mut self) -> FCR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::FCR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { FCR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`LCR`].
    pub fn lcr(&mut self) -> LCR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::LCR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { LCR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`MCR`].
    pub fn mcr(&mut self) -> MCR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::MCR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { MCR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`LSR`].
    pub fn lsr(&mut self) -> LSR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::LSR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { LSR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`MSR`].
    pub fn msr(&mut self) -> MSR {
        // SAFETY: We operate on valid register addresses.
        let val = unsafe { self.backend.read(offsets::MSR as u8) };
        // SAFETY: All possible bits are typed.
        unsafe { MSR::from_bits(val).unwrap_unchecked() }
    }

    /// Fetches the current value from the [`SPR`].
    pub fn spr(&mut self) -> SPR {
        // SAFETY: We operate on valid register addresses.
        unsafe { self.backend.read(offsets::SPR as u8) }
    }

    /// Fetches the current values from the [`DLL`] and [`DLM`].
    ///
    /// # Safety
    ///
    /// This is unsafe as it will temporarily set the DLAB bit which may hinder
    /// or negatively influence normal operation.
    pub unsafe fn dll_dlm(&mut self) -> (DLL, DLM) {
        let old_lcr = self.lcr();
        // SAFETY: We operate on valid register addresses.
        unsafe {
            self.backend
                .write(offsets::LCR as u8, (old_lcr | LCR::DLAB).bits());

            let dll = self.backend.read(offsets::DLL as u8);
            let dlm = self.backend.read(offsets::DLM as u8);
            self.backend.write(offsets::LCR as u8, old_lcr.bits());
            (dll, dlm)
        }
    }

    /* ----- Misc ----------------------------------------------------------- */

    /// Returns the config with that the device was initialized as well as the
    /// base address.
    ///
    /// To get the values that are currently in the registers, consider using
    /// [`Self::config_register_dump`].
    pub fn config(&self) -> (&Config, B::A) {
        (&self.config, self.base_address)
    }

    /// Queries the device and returns a [`ConfigRegisterDump`].
    ///
    /// # Safety
    ///
    /// Reading on some registers may have side-effects, for example in the
    /// [`LSR`].
    pub unsafe fn config_register_dump(&mut self) -> ConfigRegisterDump {
        // SAFETY: Caller ensured access is okay.
        let (dll, dlm) = unsafe { self.dll_dlm() };
        ConfigRegisterDump {
            ier: self.ier(),
            isr: self.isr(),
            lcr: self.lcr(),
            mcr: self.mcr(),
            lsr: self.lsr(),
            msr: self.msr(),
            spr: self.spr(),
            dll,
            dlm,
        }
    }
}

/// A dump of all (readable) config registers of [`Uart16550`].
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[allow(missing_docs)]
pub struct ConfigRegisterDump {
    pub ier: IER,
    pub isr: ISR,
    pub lcr: LCR,
    pub mcr: MCR,
    pub lsr: LSR,
    pub msr: MSR,
    pub spr: SPR,
    pub dll: DLL,
    pub dlm: DLM,
}

impl ConfigRegisterDump {
    /// Returns the effective divisor.
    ///
    /// Using [`calc_baud_rate`], you can calculate the effective baud rate. You
    /// can also use [`Self::baud_rate()`].
    #[must_use]
    pub const fn divisor(&self) -> u16 {
        let dll = self.dll as u16;
        let dlm = self.dlm as u16;
        (dlm << 8) | dll
    }

    /// Returns the effective divisor.
    ///
    /// Using [`calc_baud_rate`], you can calculate the effective
    /// [`BaudRate`].
    #[must_use]
    pub fn baud_rate(&self, config: &Config) -> BaudRate {
        let divisor = self.divisor();
        let baud_rate = calc_baud_rate(
            config.frequency,
            divisor as u32,
            config.prescaler_division_factor,
        )
        .expect("should be able to calculate baud rate from the given valid values");
        BaudRate::from_integer(baud_rate)
    }
}
