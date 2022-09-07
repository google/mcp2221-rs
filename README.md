# MCP2221-rs

This library allows your code to control an MCP2221 or MCP2221A over USB for the
purposes of talking I2C for using the GPIO pins. It doesn't support the UART
feature of the MCP2221 although PRs are welcome.

I2C traits from the [embedded-hal](https://crates.io/crates/embedded-hal) are
used, allowing relatively easy switching between this I2C interface and
something like [linux-embedded-hal](https://crates.io/crates/linux-embedded-hal)
for devices that are supported directly by the kernel.

This library has only been tested on Linux, but it uses rusb to access the USB
bus and that supports other platforms, so there's a moderate chance it will work
on more than just Linux.

### Example

See [examples/scan_bus.rs](examples/scan_bus.rs).

### Granting access to the device

In order to allow access to the USB device, it's recommended to create a udev
rule. e.g. create `/etc/udev/rules.d/71-mcp.rules` with the following content:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="00dd", GROUP="plugdev", TAG+="uaccess"
```

This rule allows access to the device by the user physically logged into the
computer (uaccess) and to anyone in the plugdev group.

### Contributing

See [docs/contributing.md](docs/contributing.md).

### Disclaimer

This is not an officially supported Google product.

### License

This project is licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   https://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   https://opensource.org/licenses/MIT)

at your option.
