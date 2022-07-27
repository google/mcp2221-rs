// Copyright 2022 The mcp2221-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

use embedded_hal::blocking::i2c::Read;
use std::time::Duration;

fn main() {
    if let Err(error) = run() {
        println!("Error: {}", error);
    }
}

fn run() -> mcp2221::Result<()> {
    let mut config = mcp2221::Config::default();
    config.i2c_speed_hz = 400_000;
    // For talking to a peripheral we might want a higher timeout, but for
    // scanning the bus, a short timeout is good since it allows us to scan all
    // addresses more quickly.
    config.timeout = Duration::from_millis(10);
    let mut dev = mcp2221::Handle::open_first(&config)?;

    // Set GPIO pin 0 high. This is useful if your I2C bus goes through a level
    // shifter and you need to enable that level shifter in order to use the I2C
    // bus. It also serves as an example of using GPIO.
    let mut gpio_config = mcp2221::GpioConfig::default();
    gpio_config.set_direction(0, mcp2221::Direction::Output);
    gpio_config.set_value(0, true);
    dev.configure_gpio(&gpio_config)?;

    // Before we start, SDA and SCL should be high. If they're not, then either
    // the pull-up resistors are missing, the bus isn't properly connected or
    // something on the bus is holding them low. In any case, we won't be able
    // to operate.
    dev.check_bus()?;

    println!("{}", dev.get_device_info()?);

    for base_address in (0..=127).step_by(16) {
        for offset in 0..=15 {
            let address = base_address + offset;
            match dev.read(address, &mut [0u8]) {
                Ok(_) => print!("0x{:02x}", address),
                Err(_) => print!(" -- "),
            }
        }
        println!();
    }

    Ok(())
}
