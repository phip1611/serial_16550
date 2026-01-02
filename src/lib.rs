// SPDX-License-Identifier: MIT OR Apache-2.0

//! This crate provides a low-level driver for [16550 UART devices][uart], which
//! operate as serial ports and are accessible via x86 I/O port addresses.
//! These ports are commonly referred to as COM ports.
//!
//! For a few decades now, the COM port in x86 machines is almost always backed
//! by a **16550 UART devices**, may it be physical or emulated. Serial ports are
//! especially useful for debugging or operating system learning projects.
//!
//! [uart]: https://en.wikipedia.org/wiki/16550_UART
//!
//! ## Features
//!
//! - ✅ Reads and writes from/to the COM port
//! - ✅ `no_std` support
//! - ✅ Tested on **real hardware** and in virtual machines
//! - ✅ High-level abstractions and also a seamless integration with plain integers
//! - ⚠️ `x86` and `x86_64` only
//!
//! ## Scope & Limitations
//!
//! Typically, a serial port is not only used for simple ASCII transfer but
//! VT102-like terminal emulation. `serial_16550` only provides the low-level
//! hardware interface to read and write bytes. Anything regarding terminal
//! emulation, such as handling of backspace and newlines, is out of scope.
//!
//! See [`Uart16550Port`].

#![no_std]
#![deny(
    clippy::all,
    clippy::cargo,
    clippy::nursery,
    clippy::must_use_candidate
)]
#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(rustdoc::all)]
// I can't do much about that.
#![allow(clippy::multiple_crate_versions)]

#[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
compile_error!("port I/O is only available on x86 and x86_64");

use bitflags::bitflags;
use core::error::Error;
use core::fmt::{Debug, Display, Formatter};
use core::{fmt, hint};
use x86::io::{inb, outb};

/// The maximum size of the internal read and write FIFO.
///
/// Each channel has its own queue.
pub const FIFO_SIZE: usize = 16;

/// Internal frequency of the clock (by spec).
pub const FREQUENCY: u32 = 1_843_200;

/// Low-level driver for 16550 UART devices, which function as serial ports and
/// are accessible via x86 I/O port addresses. These ports are commonly referred
/// to as COM ports.
///
/// One device instance corresponds to one device, typically mapped via eight
/// I/O ports.
///
/// A port must be initialized, so don't forget to call [`Uart16550Port::init`].
/// To test your hardware, it is also recommended to call
/// [`Uart16550Port::test_loopback`] afterwards.
///
/// ## Terminal / VT-102 Handling
///
/// This type does not take care of any VT102-like terminal emulation. You need
/// to handle terminal-friendly newlines, backspace support, etc. on your own.
///
/// # Example
/// ```rust,no_run
/// use serial_16550::{SerialConfig, Uart16550Port};
/// // SAFETY: We have exclusive access to the device.
/// let mut device = unsafe { Uart16550Port::new(0x3f8) };
///
/// // Default config has sane values.
/// // SAFETY: We have exclusive access to the device.
/// unsafe { device.init(&SerialConfig::default()) }
/// // SAFETY: We have exclusive access to the device.
/// unsafe { device.test_loopback() }
///     .expect("device should operate properly");
///
/// // Write something to the device.
/// device.write_bytes_saturating(b"hello");
///
/// // Format something directly onto the device!
/// # use core::fmt::Write;
/// write!(&mut device, "{}", "hello");
///
/// // Read data from the device.
/// let mut read_buffer = [0_u8; 128];
/// let n = device.read_bytes(&mut read_buffer);
/// let read = &read_buffer[..n];
/// ```
#[derive(Debug)]
pub struct Uart16550Port(u16);

impl Uart16550Port {
    /// Creates a serial port.
    ///
    /// Don't forget to call [`Self::init`] next.
    ///
    /// # Example
    /// ```rust,no_run
    /// use serial_16550::{SerialConfig, Uart16550Port};
    /// // SAFETY: We have exclusive access to the device.
    /// let mut device = unsafe { Uart16550Port::new(0x3f8) };
    ///
    /// // Default config has sane values.
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.init(&SerialConfig::default()) }
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.test_loopback() }
    ///     .expect("device should operate properly");
    ///
    /// // Write something to the device.
    /// device.write_bytes_saturating(b"hello");
    ///
    /// // Format something directly onto the device!
    /// # use core::fmt::Write;
    /// write!(&mut device, "{}", "hello");
    ///
    /// // Read data from the device.
    ///  let mut read_buffer = [0_u8; 128];
    ///  let n = device.read_bytes(&mut read_buffer);
    ///  let read = &read_buffer[..n];
    /// ```
    ///
    /// # Safety
    ///
    /// Callers must ensure that using this type with the underlying hardware
    /// is done only in a context where such operations are valid and safe.
    #[must_use]
    pub const unsafe fn new(base: u16) -> Self {
        Self(base)
    }

    /// Returns the base port I/O address.
    #[must_use]
    #[inline]
    pub const fn base(&self) -> u16 {
        self.0
    }

    /// Initializes the devices so that afterward, data can be received and
    /// transmitted.
    ///
    /// To test your hardware, you can call [`Self::test_loopback`] to verify.
    ///
    /// # Example
    /// ```rust,no_run
    /// use serial_16550::{SerialConfig, Uart16550Port};
    /// // SAFETY: We have exclusive access to the device.
    /// let mut device = unsafe { Uart16550Port::new(0x3f8) };
    ///
    /// // Default config has sane values.
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.init(&SerialConfig::default()) }
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.test_loopback() }
    ///     .expect("device should operate properly");
    /// ```
    ///
    /// # Safety
    ///
    /// Callers must ensure that using this type with the underlying hardware
    /// is done only in a context where such operations are valid and safe.
    ///
    /// Further, the serial config must match the expectations of the wire and
    /// the other side. Otherwise, garbage will be received.
    pub unsafe fn init(&mut self, cfg: &SerialConfig) {
        self.set_interrupts(false);

        // Set baud rate registers
        unsafe {
            // Enable Divisor Latch Access Bit (DLAB):
            // Next, the first two registers map to the low and the high byte
            // of the baud rate divisor.
            outb(self.base() + reg::LINE_CTRL, 1 << 7);

            let divisor = cfg.baud_rate.raw();
            let low = divisor & 0xff;
            let high = (divisor >> 8) & 0xff;
            outb(self.base() + reg::BAUD_LOW, low as u8);
            outb(self.base() + reg::BAUD_HIGH, high as u8);

            // Disable Divisor Latch Access Bit (DLAB):
            outb(self.base() + reg::LINE_CTRL, 0);
        }

        // Set line control  register:
        // - **set:   :** data bits, stop bits
        // - **ignored:** parity, stick bit, send break
        unsafe {
            let mut line_ctrl = cfg.data_bits.raw();
            line_ctrl |= (cfg.stop_bits.raw()) << 2;
            outb(self.base() + reg::LINE_CTRL, line_ctrl);
        }

        // Set fifo control register
        unsafe {
            let mut fifo_ctrl = 0;
            // enable fifo
            fifo_ctrl |= 1;
            // receiver reset: receiver FIFO
            fifo_ctrl |= 1 << 1;
            // transmitter reset: FIFO mode
            fifo_ctrl |= 1 << 2;
            // Receiver trigger level: 14 bytes or more in fifo => interrupt
            fifo_ctrl |= 0b11 << 6;
            outb(self.base() + reg::FIFO_CTRL, fifo_ctrl);
        }

        // Set modem control register
        unsafe {
            let mut modem_ctrl = 0;
            // Device is present and powered: Data terminal ready (DTR)
            modem_ctrl |= 1 << 0;
            // Device is willing to transmit: Signal request to send
            modem_ctrl |= 1 << 1;
            // Enable interrupt routing to CPU.
            modem_ctrl |= 1 << 3;

            outb(self.base() + reg::MODEM_CTRL, modem_ctrl);
        }

        self.set_interrupts(true);
    }

    // #########################################################################
    // public helpers

    /// Enables or disables interrupts.
    #[inline]
    pub fn set_interrupts(&mut self, enabled: bool) {
        let val = if enabled { 1 } else { 0 };
        // Disable interrupts
        unsafe {
            outb(self.base() + reg::INTERRUPT_ENABLE, val);
        }
    }

    /// Tests the serial port by sending a message to itself.
    ///
    /// # Example
    /// ```rust,no_run
    /// use serial_16550::{SerialConfig, Uart16550Port};
    /// // SAFETY: We have exclusive access to the device.
    /// let mut device = unsafe { Uart16550Port::new(0x3f8) };
    ///
    /// // Default config has sane values.
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.init(&SerialConfig::default()) }
    /// // SAFETY: We have exclusive access to the device.
    /// unsafe { device.test_loopback() }
    ///     .expect("device should operate properly");
    /// ```
    ///
    /// # Safety
    ///
    /// Callers must ensure that using this type with the underlying hardware
    /// is done only in a context where such operations are valid and safe.
    ///
    /// Further, as long as this runs, the configuration of the serial device
    /// changes. No other instances should access the device in that time.
    pub unsafe fn test_loopback(&mut self) -> Result<(), LoopbackFailed> {
        self.set_interrupts(false);

        // Enable loopback.
        unsafe {
            let mut modem_ctrl = inb(self.base() + reg::MODEM_CTRL);
            // set the loop bit -> data never leaves the cip
            modem_ctrl |= 1 << 4;
            outb(self.base() + reg::MODEM_CTRL, modem_ctrl);
        }

        // Drain device: read any data that might be on the device for whatever reason first.
        while self.try_read_byte().is_ok() {}

        // Perform the actual write and read cycle
        let matches = {
            const MESSAGE: &[u8; 6] = b"hello!";
            debug_assert!(MESSAGE.len() <= FIFO_SIZE);

            self.write_bytes_saturating(MESSAGE);
            let mut buffer = [0_u8; MESSAGE.len()];
            self.read_bytes_saturating(&mut buffer);

            &buffer == MESSAGE
        };

        // Disable loopback.
        unsafe {
            let mut modem_ctrl = inb(self.base() + reg::MODEM_CTRL);
            // set the loop bit -> data never leaves the cip
            modem_ctrl &= !(1 << 4);
            outb(self.base() + reg::MODEM_CTRL, modem_ctrl);
        }

        self.set_interrupts(true);

        if matches { Ok(()) } else { Err(LoopbackFailed) }
    }

    /// Reads the current [`LineStatusFlags`].
    #[must_use]
    #[inline]
    pub fn line_status_flags(&mut self) -> LineStatusFlags {
        let raw = unsafe { inb(self.base() + reg::LINE_STS) };
        LineStatusFlags::from_bits_truncate(raw)
    }

    // #########################################################################
    // public I/O

    /// Try to read a single byte, if the underlying FIFO has data.
    ///
    /// Otherwise, an error is returned.
    #[inline]
    pub fn try_read_byte(&mut self) -> Result<u8, WouldBlock> {
        if self
            .line_status_flags()
            .contains(LineStatusFlags::DATA_READY)
        {
            let byte = unsafe { inb(self.base() + reg::DATA) };
            Ok(byte)
        } else {
            Err(WouldBlock)
        }
    }

    /// Blocking variant of [`Self::try_read_byte`].
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[must_use]
    #[inline]
    pub fn read_byte(&mut self) -> u8 {
        loop {
            if let Ok(byte) = self.try_read_byte() {
                return byte;
            }
            hint::spin_loop()
        }
    }

    /// Reads bytes as long as there is data available and returns the number
    /// of read bytes.
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[must_use]
    pub fn read_bytes(&mut self, buffer: &mut [u8]) -> usize {
        for (i, cell) in buffer.iter_mut().enumerate() {
            match self.try_read_byte() {
                Ok(byte) => *cell = byte,
                Err(_) => return i,
            }
        }

        buffer.len()
    }

    /// Reads bytes as long as it takes to fill the buffer without exiting
    /// early.
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[inline]
    pub fn read_bytes_saturating(&mut self, buffer: &mut [u8]) {
        let mut i = 0;
        while i < buffer.len() {
            match self.try_read_byte() {
                Ok(byte) => {
                    buffer[i] = byte;
                    i += 1;
                }
                Err(_) => continue,
            }
            hint::spin_loop()
        }
    }

    /// Try to write a single byte, if the underlying FIFO has capacity.
    ///
    /// Otherwise, an error is returned.
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[inline]
    pub fn try_write_byte(&mut self, byte: u8) -> Result<(), WouldBlock> {
        if self
            .line_status_flags()
            .contains(LineStatusFlags::TRANSMITTER_READY_FOR_DATA)
        {
            unsafe { outb(self.base() + reg::DATA, byte) };
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }

    /// Blocking variant of [`Self::try_write_byte`].
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[inline]
    pub fn write_byte(&mut self, byte: u8) {
        loop {
            if self.try_write_byte(byte).is_ok() {
                break;
            }
            hint::spin_loop()
        }
    }

    /// Writes all bytes into the device and only returns if all data was
    /// transmitted.
    ///
    /// ## Terminal / VT-102 Handling
    ///
    /// This type does not take care of any VT102-like terminal emulation. You
    /// need to handle terminal-friendly newlines, backspace support, etc. on
    /// your own.
    #[inline]
    pub fn write_bytes_saturating(&mut self, buffer: &[u8]) {
        for byte in buffer {
            self.write_byte(*byte);
        }
    }
}

impl fmt::Write for Uart16550Port {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_bytes_saturating(s.as_bytes());
        Ok(())
    }
}

/// Register offsets regarding the base.
#[allow(missing_docs)]
pub mod reg {
    pub const DATA: u16 = 0;
    pub const INTERRUPT_ENABLE: u16 = 1;
    pub const FIFO_CTRL: u16 = 2;
    pub const LINE_CTRL: u16 = 3;
    pub const MODEM_CTRL: u16 = 4;
    pub const LINE_STS: u16 = 5;

    /// Also called Divisor Latch Low Byte (DLL).
    pub const BAUD_LOW: u16 = 0;
    /// Also called Divisor Latch High Byte (DLH).
    pub const BAUD_HIGH: u16 = 1;
}

/// The operation would block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopbackFailed;

impl Display for LoopbackFailed {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "the loopback test failed and the device might be broken")
    }
}

impl Error for LoopbackFailed {}

/// The operation would block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WouldBlock;

impl Display for WouldBlock {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "the operation would block")
    }
}

impl Error for WouldBlock {}

/// The value was invalid in that context.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidValue<T>(pub T);

impl<T: Debug> Display for InvalidValue<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "the value was invalid: {:?}", self.0)
    }
}

impl<T: Debug> Error for InvalidValue<T> {}

/// The speed of data transmission, measured in symbols per second (or bits, in
/// the case of simple UARTs).
///
/// The low-level representation can be created via [`BaudRate::raw`].
/// Variants can be constructed either directly or from integers by using
/// [`BaudRate::try_from_raw`] and [`BaudRate::try_from_value`].
#[allow(missing_docs)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Hash)]
pub enum BaudRate {
    #[default]
    Baud115200,
    Baud57600,
    Baud38400,
    Baud9600,
    Baud1200,
    Custom(u16),
}

impl BaudRate {
    /// Returns the raw low-level representation.
    #[must_use]
    pub const fn raw(self) -> u16 /* divisor */ {
        match self {
            Self::Baud115200 => 1,
            Self::Baud57600 => 2,
            Self::Baud38400 => 3,
            Self::Baud9600 => 12,
            Self::Baud1200 => 96,
            Self::Custom(baud_rate) => {
                let divisor = FREQUENCY / (16 * baud_rate as u32);
                divisor as u16
            }
        }
    }

    /// Try to create the value from the raw low-level representation.
    ///
    /// In this case, this means from the divisor.
    pub const fn try_from_raw(raw: u8 /* divisor */) -> Result<Self, InvalidValue<u8>> {
        match raw {
            1 => Ok(Self::Baud115200),
            2 => Ok(Self::Baud57600),
            3 => Ok(Self::Baud38400),
            12 => Ok(Self::Baud9600),
            96 => Ok(Self::Baud1200),
            divisor => {
                let baud_rate = FREQUENCY / (16 * divisor as u32);
                let remainder = FREQUENCY % (16 * divisor as u32);
                if remainder != 0 {
                    Err(InvalidValue(divisor))
                } else {
                    Ok(Self::Custom(baud_rate as u16))
                }
            }
        }
    }

    /// Try to create the value from an integer representation.
    ///
    /// The value must result in a clear divisor without any remainder!
    pub const fn try_from_value(value: u32) -> Result<Self, InvalidValue<u32>> {
        match value {
            115200 => Ok(Self::Baud115200),
            57600 => Ok(Self::Baud57600),
            38400 => Ok(Self::Baud38400),
            9600 => Ok(Self::Baud9600),
            1200 => Ok(Self::Baud1200),
            baud_rate => {
                let _divisor = (FREQUENCY / (16 * baud_rate)) as u16;
                let remainder = FREQUENCY % (16 * baud_rate);
                if remainder != 0 {
                    Err(InvalidValue(baud_rate))
                } else {
                    Ok(Self::Custom(baud_rate as u16))
                }
            }
        }
    }
}

/// The number of bits in each transmitted character on the wire that carry
/// actual payload information.
///
/// The low-level representation can be created via [`DataBits::raw`].
/// Variants can be constructed either directly or from integers by using
/// [`DataBits::try_from_raw`] and [`DataBits::try_from_value`].
#[allow(missing_docs)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Hash)]
pub enum DataBits {
    Bits5,
    Bits6,
    Bits7,
    #[default]
    Bits8,
}

impl DataBits {
    /// Returns the raw low-level representation.
    #[must_use]
    pub const fn raw(self) -> u8 {
        match self {
            Self::Bits5 => 0b00,
            Self::Bits6 => 0b01,
            Self::Bits7 => 0b10,
            Self::Bits8 => 0b11,
        }
    }

    /// Try to create the value from the raw low-level representation.
    pub const fn try_from_raw(raw: u8) -> Result<Self, InvalidValue<u8>> {
        match raw {
            0b00 => Ok(Self::Bits5),
            0b01 => Ok(Self::Bits6),
            0b10 => Ok(Self::Bits7),
            0b11 => Ok(Self::Bits8),
            val => Err(InvalidValue(val)),
        }
    }

    /// Try to create the value from an integer representation.
    pub const fn try_from_value(value: u32) -> Result<Self, InvalidValue<u32>> {
        match value {
            5 => Ok(Self::Bits5),
            6 => Ok(Self::Bits6),
            7 => Ok(Self::Bits7),
            8 => Ok(Self::Bits8),
            val => Err(InvalidValue(val)),
        }
    }
}

/// Extra bits sent after each character on the wire to signal the end of
/// transmission and allow the receiver to resynchronize.
///
/// The low-level representation can be created via [`StopBits::raw`].
/// Variants can be constructed either directly or from integers by using
/// [`StopBits::try_from_raw`] and [`StopBits::try_from_value`].
#[allow(missing_docs)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Hash)]
pub enum StopBits {
    #[default]
    One,
    Two,
}

impl StopBits {
    /// Returns the raw low-level representation.
    #[must_use]
    pub const fn raw(self) -> u8 {
        match self {
            Self::One => 0,
            Self::Two => 1,
        }
    }

    /// Try to create the value from the raw low-level representation.
    pub const fn try_from_raw(raw: u8) -> Result<Self, InvalidValue<u8>> {
        match raw {
            0 => Ok(Self::One),
            1 => Ok(Self::Two),
            val => Err(InvalidValue(val)),
        }
    }

    /// Try to create the value from an integer representation.
    pub const fn try_from_value(value: u32) -> Result<Self, InvalidValue<u32>> {
        match value {
            1 => Ok(Self::One),
            2 => Ok(Self::Two),
            val => Err(InvalidValue(val)),
        }
    }
}

/// The configuration for a [`Uart16550Port`].
///
/// The default configuration uses a `8-N-1` transmission
/// (<https://en.wikipedia.org/wiki/8-N-1>) with a baud rate of 115200
/// ([`SerialConfig::default`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct SerialConfig {
    /// Divisor for the baud rate.
    pub baud_rate: BaudRate,
    /// Amount of data bits.
    pub data_bits: DataBits,
    /// Amount of stop bits.
    pub stop_bits: StopBits,
}

bitflags! {
    /// Line status flags.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    #[repr(transparent)]
    pub struct LineStatusFlags: u8 {
        /// At least one byte is ready to be read.
        const DATA_READY = 1 << 0;
        /// Receiver overrun: a byte was lost because it wasn’t read in time.
        const DATA_OVERRUN_ERROR = 1 << 1;
        /// Parity of received byte does not match the configured parity.
        const PARITY_ERROR = 1 << 2;
        /// Framing error: stop bit not detected correctly.
        const FRAMIG_ERROR = 1 << 3;
        /// Break condition detected (line held low too long).
        const BREAK_INDICATOR = 1 << 4;
        /// Can accept more bytes to be written.
        const TRANSMITTER_READY_FOR_DATA = 1 << 5;
        /// Transmitter completely empty (all bytes sent).
        const TRANSMITTER_EMPTY = 1 << 6;
        /// One or more errors present in the FIFO.
        const ONE_ERROR_IN_FIFO = 1 << 7;
    }
}

#[cfg(test)]
mod conversion_tests {
    use super::*;

    #[test]
    #[rustfmt::skip]
    fn baud_rate() {
        assert_eq!(BaudRate::try_from_raw(1), Ok(BaudRate::Baud115200));
        assert_eq!(BaudRate::try_from_raw(2), Ok(BaudRate::Baud57600));
        assert_eq!(BaudRate::try_from_raw(3), Ok(BaudRate::Baud38400));
        assert_eq!(BaudRate::try_from_raw(12), Ok(BaudRate::Baud9600));
        assert_eq!(BaudRate::try_from_raw(24), Ok(BaudRate::Custom(4800)));
        assert_eq!(BaudRate::try_from_raw(96), Ok(BaudRate::Baud1200));
        assert_eq!(BaudRate::try_from_raw(137), Err(InvalidValue(137)));

        assert_eq!(BaudRate::try_from_value(115200), Ok( BaudRate::Baud115200));
        assert_eq!(BaudRate::try_from_value(57600), Ok(BaudRate::Baud57600));
        assert_eq!(BaudRate::try_from_value(38400), Ok(BaudRate::Baud38400));
        assert_eq!(BaudRate::try_from_value(9600), Ok(BaudRate::Baud9600));
        assert_eq!(BaudRate::try_from_value(4800), Ok(BaudRate::Custom(4800)));
        assert_eq!(BaudRate::try_from_value(1200), Ok(BaudRate::Baud1200));
        assert_eq!(BaudRate::try_from_value(137), Err(InvalidValue(137)));
    }

    #[test]
    fn stop_bits() {
        assert_eq!(StopBits::try_from_raw(0), Ok(StopBits::One));
        assert_eq!(StopBits::try_from_raw(1), Ok(StopBits::Two));
        assert_eq!(StopBits::try_from_raw(2), Err(InvalidValue(2)));

        assert_eq!(StopBits::try_from_value(0), Err(InvalidValue(0)));
        assert_eq!(StopBits::try_from_value(1), Ok(StopBits::One));
        assert_eq!(StopBits::try_from_value(2), Ok(StopBits::Two));
        assert_eq!(StopBits::try_from_value(3), Err(InvalidValue(3)));
    }

    #[test]
    fn data_bits() {
        assert_eq!(DataBits::try_from_raw(0), Ok(DataBits::Bits5));
        assert_eq!(DataBits::try_from_raw(1), Ok(DataBits::Bits6));
        assert_eq!(DataBits::try_from_raw(2), Ok(DataBits::Bits7));
        assert_eq!(DataBits::try_from_raw(3), Ok(DataBits::Bits8));
        assert_eq!(DataBits::try_from_raw(4), Err(InvalidValue(4)));

        assert_eq!(DataBits::try_from_value(4), Err(InvalidValue(4)));
        assert_eq!(DataBits::try_from_value(5), Ok(DataBits::Bits5));
        assert_eq!(DataBits::try_from_value(6), Ok(DataBits::Bits6));
        assert_eq!(DataBits::try_from_value(7), Ok(DataBits::Bits7));
        assert_eq!(DataBits::try_from_value(8), Ok(DataBits::Bits8));
        assert_eq!(DataBits::try_from_value(9), Err(InvalidValue(9)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config() {
        let config_from_enum_variants = SerialConfig {
            baud_rate: BaudRate::Baud115200,
            data_bits: DataBits::Bits8,
            stop_bits: StopBits::One,
        };
        let config_from_integers = SerialConfig {
            baud_rate: BaudRate::try_from_value(115200).unwrap(),
            data_bits: DataBits::try_from_value(8).unwrap(),
            stop_bits: StopBits::try_from_value(1).unwrap(),
        };
        let config_from_raw_values = SerialConfig {
            baud_rate: BaudRate::try_from_raw(1).unwrap(),
            data_bits: DataBits::try_from_raw(0b11).unwrap(),
            stop_bits: StopBits::try_from_raw(0).unwrap(),
        };

        assert_eq!(config_from_enum_variants, SerialConfig::default());
        assert_eq!(config_from_integers, config_from_integers);
        assert_eq!(config_from_integers, config_from_raw_values);
    }
}
