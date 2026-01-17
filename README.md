# `uart_16550`

Low-level driver for [**16550 UART devices**][uart], which operate as serial ports and
are accessible via x86 I/O port addresses. These ports are commonly referred to
as COM ports.

For a few decades now, the COM port in x86 machines is almost always backed
by a **16550 UART devices**, may it be physical or emulated. Serial ports are
especially useful for debugging or operating system learning projects.

[uart]: https://en.wikipedia.org/wiki/16550_UART

## Features

- ✅ Reads and writes from/to the COM port
- ✅ `no_std` support
- ✅ Tested on **real hardware** and in virtual machines
- ✅ High-level abstractions and also a seamless integration with plain integers
- ⚠️ `x86` and `x86_64` only

## Scope & Limitations

Typically, a serial port is not only used for simple ASCII transfer but
VT102-like terminal emulation. `uart_16550` only provides the low-level
hardware interface to read and write bytes. Anything regarding terminal
emulation, such as handling of backspace and newlines, is out of scope.

## Example

```rust
fn main() {
  use uart_16550::{SerialConfig, Uart16550Port};
  let device = Uart16550Port::new(0x3f8);

  // Default config has sane values.
  // SAFETY: We have exclusive access to the device.
  unsafe { device.init(&SerialConfig::default()) }
  // SAFETY: We have exclusive access to the device.
  unsafe { device.test_loopback(&SerialConfig::default()) }

  // Write something to the device.
  device.write_bytes_saturating(b"hello");

  // Read data from the device.
  let mut read_buffer = [0_u8; 128];
  let n = device.read_bytes(&mut read_buffer);
  let read = &read[..n];
}
```

## Comparison to Ecosystem (`uart_16550` crate)

This crate is closely related to the well-established
[`uart_16550`](https://crates.io/crates/uart_16550) crate and builds upon many
of the same underlying concepts. The primary difference is a stronger focus on
flexibility and configurability, to use my crate on **real hardware**: it
allows users to explicitly configure properties such as the baud rate and
provides a structured, ergonomic API for supplying configuration values in
various representations (high-level and raw integers). In contrast to
`uart_16550`, this crate intentionally targets port-I/O–mapped devices only and
does not support MMIO-based UARTs.

Although I am a maintainer of [`rust-osdev`](https://github.com/rust-osdev), I
did not have the capacity to evolve the existing crate in the direction I had in
mind. The combination of backwards-compatibility requirements, a broader feature
scope (including terminal-related newline handling and MMIO support), and my own
preferences regarding API design and internal structure ultimately made it more
practical to develop a new library.

A particular emphasis of this crate is on transparent and well-documented device
initialization. Working through the UART bring-up in detail was both enjoyable
and instructive, and it provided an opportunity to revisit and better understand
the many control and status bits involved. The result is a clear, approachable
implementation that aims to serve not only as a practical driver, but also as an
educational resource for others working close to the hardware. :)

## License

This project is licensed under either of

- MIT license ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
