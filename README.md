# `serial_16550`

Low-level driver for **16550 UART devices**, which operate as serial ports and
are accessible via x86 I/O port addresses. These ports are commonly referred to
as COM ports.

For a few decades now, the COM port in x86 machines is almost always backed
by a **16550 UART devices**, may it be physical or emulated. Serial ports are
especially useful for debugging or operating system learning projects.

## Features

- ✅ Reads and writes from/to the COM port
- ✅ `no_std` support
- ✅ Tested on real hardware and in virtual machines
- ✅ High-level abstractions and also a seamless integration with plain integers
- ⚠️ `x86` and `x86_64` only

## Example

```rust
fn main() {
  use serial_16550::{SerialConfig, Uart16550Port};
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

## License

This project is licensed under either of

- MIT license ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
