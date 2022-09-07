// Copyright 2022 The mcp2221-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

//! Interface to MCP2221 and MCP2221A. Uses libusb, so no MCP2221 kernel module
//! is required. Supports I2C and GPIO.

use embedded_hal::blocking::i2c;
use std::fmt::Display;
use std::time::Duration;
use std::time::Instant;

const MCP2221A_VENDOR_ID: u16 = 0x04d8;
const MCP2221A_DEVICE_ID: u16 = 0x00dd;
const MCP2221A_INTERFACE: u8 = 2;

const COMMAND_RESET: u8 = 0x70;
const COMMAND_STATUS: u8 = 0x10;
const COMMAND_READ_DATA: u8 = 0x40;
const COMMAND_WRITE_DATA: u8 = 0x90;
const COMMAND_READ: u8 = 0x91;
const COMMAND_READ_REPEAT_START: u8 = 0x93;
const COMMAND_WRITE_NO_STOP: u8 = 0x94;
const COMMAND_GET_GPIO: u8 = 0x51;
const COMMAND_SET_SRAM_SETTINGS: u8 = 0x60;
const COMMAND_GET_SRAM_SETTINGS: u8 = 0x61;

const MCP_TRANSFER_SIZE: usize = 64;
const MCP_HEADER_SIZE: usize = 4;
const MCP_MAX_DATA_SIZE: usize = MCP_TRANSFER_SIZE - MCP_HEADER_SIZE;
const MCP_MAX_I2C_PACKET: usize = 65535;

const MCP_WRITE_ENDPOINT: u8 = 0x03; // host to device
const MCP_READ_ENDPOINT: u8 = 0x83; // device to host

const RESET_SEQUENCE: [u8; 4] = [COMMAND_RESET, 0xAB, 0xCD, 0xEF];

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(1000);

const RESET_TIMEOUT: Duration = Duration::from_millis(5000);

const I2C_BUSY: u8 = 1;

const NUM_GPIO_PINS: usize = 4;

pub type Result<T, E = Error> = core::result::Result<T, E>;

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    DeviceNotFound,
    WriteTooLarge,
    UsbError(rusb::Error),
    ReadTimeout,
    WriteTimeout,
    SetBusSpeedFailed,
    ShortRead,
    UnexpectedReadSize,
    ReadError,
    DeviceLostOnReset,
    GpioError,
    CancelFailed,
    CommandFailed,
    SclLow,
    SdaLow,
    Nack,
}

/// An MCP device that has been opened. Supports I2C and GPIO operations.
pub struct Handle {
    handle: rusb::DeviceHandle<rusb::GlobalContext>,
    recv: [u8; MCP_TRANSFER_SIZE],
    config: Config,
}

/// A supported device that hasn't yet been opened.
pub struct AvailableDevice {
    device: rusb::Device<rusb::GlobalContext>,
}

#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct Config {
    pub i2c_speed_hz: u32,
    pub reset_on_open: bool,
    pub timeout: Duration,
}

pub struct GpioConfig {
    command_buffer: [u8; MCP_TRANSFER_SIZE],
}

pub enum Direction {
    Input,
    Output,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PinState {
    Unknown,
    NotGpio,
    High,
    Low,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct PinValues {
    pub gp0: PinState,
    pub gp1: PinState,
    pub gp2: PinState,
    pub gp3: PinState,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            i2c_speed_hz: 400_000,
            reset_on_open: true,
            timeout: DEFAULT_TIMEOUT,
        }
    }
}

impl AvailableDevice {
    /// Returns all supported devices.
    pub fn list() -> Result<Vec<AvailableDevice>> {
        let mut devices = Vec::new();
        for device in rusb::devices()?.iter() {
            if let Ok(device_desc) = device.device_descriptor() {
                if device_desc.vendor_id() == MCP2221A_VENDOR_ID
                    && device_desc.product_id() == MCP2221A_DEVICE_ID
                {
                    devices.push(AvailableDevice { device });
                }
            }
        }
        Ok(devices)
    }

    /// Attempts to open the I2C device.
    pub fn open(&self, config: &Config) -> Result<Handle> {
        let mut handle = self.open_device(config)?;
        // Setting auto-detach will fail on platforms where it isn't supported
        // (Windows). So we ignore the returned error in this case. Provided no
        // driver has already claimed the device, it should be fine to continue.
        // If some driver has claimed the device, then we'll fail on the next
        // line when we try to claim it.
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(MCP2221A_INTERFACE)?;
        let mut i2c = Handle {
            handle,
            recv: [0u8; MCP_TRANSFER_SIZE],
            config: *config,
        };
        i2c.set_speed(config.i2c_speed_hz)?;
        Ok(i2c)
    }

    fn open_device(&self, config: &Config) -> Result<rusb::DeviceHandle<rusb::GlobalContext>> {
        if config.reset_on_open {
            // Record which device we're opening so that we can reobtain it
            // after we reset the device.
            let bus_number = self.device.bus_number();

            let mut handle = self.device.open()?;
            let _ = handle.set_auto_detach_kernel_driver(true);
            handle.claim_interface(MCP2221A_INTERFACE)?;
            let mut buffer = [0u8; MCP_TRANSFER_SIZE];
            buffer[..RESET_SEQUENCE.len()].copy_from_slice(&RESET_SEQUENCE);
            handle.write_interrupt(MCP_WRITE_ENDPOINT, &buffer, config.timeout)?;
            std::thread::sleep(Duration::from_millis(1000));
            let start = Instant::now();
            while start.elapsed() < RESET_TIMEOUT {
                if let Some(device) = first_device_on_bus(bus_number)? {
                    return Ok(device.open()?);
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            Err(Error::DeviceLostOnReset)
        } else {
            Ok(self.device.open()?)
        }
    }
}

/// Returns the first MCP device on the specified bus. Ideally we'd like to
/// return the same device as we had before, but the devices address changes
/// when it gets reset. Ideally, we would see what devices were on the bus
/// before we reset, then see which "new" MCP device shows up after we reset.
fn first_device_on_bus(bus_number: u8) -> Result<Option<rusb::Device<rusb::GlobalContext>>> {
    for device in AvailableDevice::list()? {
        if device.device.bus_number() == bus_number {
            return Ok(Some(device.device));
        }
    }
    Ok(None)
}

impl Handle {
    /// A shortcut that just opens the first supported device that we can find.
    pub fn open_first(config: &Config) -> Result<Self> {
        if let Some(device) = AvailableDevice::list()?.first() {
            device.open(config)
        } else {
            Err(Error::DeviceNotFound)
        }
    }

    /// Can be called on startup to check that the bus is idle. Returns an error
    /// if either SCL or SDA is being held low by something on the bus.
    pub fn check_bus(&mut self) -> Result<()> {
        self.read_status()?;
        if self.recv[22] != 1 {
            Err(Error::SclLow)
        } else if self.recv[23] != 1 {
            Err(Error::SdaLow)
        } else {
            Ok(())
        }
    }

    pub fn scl_is_high(&mut self) -> Result<bool> {
        self.read_status()?;
        Ok(self.recv[22] != 0)
    }

    pub fn sda_is_high(&mut self) -> Result<bool> {
        self.read_status()?;
        Ok(self.recv[23] != 0)
    }

    /// Sends START to `device_address`, then writes `to_write`. If `to_read` is
    /// non-empty, sends START again, then reads into `to_read`. Finally sends
    /// STOP.
    pub fn write_read_address(
        &mut self,
        device_address: u8,
        to_write: &[u8],
        to_read: &mut [u8],
    ) -> Result<()> {
        if to_write.len() > MCP_MAX_I2C_PACKET {
            return Err(Error::WriteTooLarge);
        }
        if !to_write.is_empty() {
            let write_command = if to_read.is_empty() {
                COMMAND_WRITE_DATA
            } else {
                COMMAND_WRITE_NO_STOP
            };
            for chunk in to_write.chunks(MCP_MAX_DATA_SIZE) {
                self.cmd_with_length(
                    write_command,
                    to_write.len() as u16,
                    device_address << 1,
                    chunk,
                )?;
            }
        }
        if to_read.is_empty() {
            return Ok(());
        }
        let read_command = if to_write.is_empty() {
            COMMAND_READ
        } else {
            COMMAND_READ_REPEAT_START
        };
        self.cmd_with_length(
            read_command,
            to_read.len() as u16,
            (device_address << 1) | 1,
            &[],
        )?;
        let mut read_remaining = to_read;
        while !read_remaining.is_empty() {
            let start_time = Instant::now();
            loop {
                self.cmd(COMMAND_READ_DATA, 0, 0, 0, &[])?;
                if self.recv[1] == 0 {
                    // Success
                    break;
                }
                if start_time.elapsed() >= self.config.timeout {
                    let _ = self.cancel_transfer();
                    return Err(Error::Nack);
                }
            }

            let size = self.recv[3] as usize;
            // Probably should never happen because we got a "success" code
            // above...
            if size == 127 {
                return Err(Error::ReadError);
            }
            read_remaining[..size].copy_from_slice(&self.recv[4..4 + size]);
            read_remaining = &mut read_remaining[size..];
        }
        Ok(())
    }

    pub fn configure_gpio(&mut self, commands: &GpioConfig) -> Result<()> {
        self.transfer(&commands.command_buffer)?;
        if self.recv[0] != COMMAND_SET_SRAM_SETTINGS || self.recv[1] != 0 {
            return Err(Error::GpioError);
        }
        Ok(())
    }

    pub fn get_gpio_config(&mut self) -> Result<GpioConfig> {
        let mut buffer = [0u8; MCP_TRANSFER_SIZE];
        buffer[0] = COMMAND_GET_SRAM_SETTINGS;
        self.transfer(&buffer)?;
        if self.recv[0] != COMMAND_GET_SRAM_SETTINGS || self.recv[1] != 0 {
            return Err(Error::GpioError);
        }
        let mut config = GpioConfig::default();
        config.command_buffer[8..8 + NUM_GPIO_PINS]
            .copy_from_slice(&self.recv[22..22 + NUM_GPIO_PINS]);
        Ok(config)
    }

    pub fn get_pin_values(&mut self) -> Result<PinValues> {
        let mut buffer = [0u8; MCP_TRANSFER_SIZE];
        buffer[0] = COMMAND_GET_GPIO;
        self.transfer(&buffer)?;
        if self.recv[0] != COMMAND_GET_GPIO || self.recv[1] != 0 {
            return Err(Error::ReadError);
        }
        Ok(PinValues {
            gp0: self.recv[2].into(),
            gp1: self.recv[4].into(),
            gp2: self.recv[6].into(),
            gp3: self.recv[8].into(),
        })
    }

    fn cmd_with_length(&mut self, command: u8, length: u16, arg: u8, data: &[u8]) -> Result<()> {
        let length_bytes = length.to_le_bytes();
        self.cmd_with_retry(command, length_bytes[0], length_bytes[1], arg, data)
    }

    fn cmd_with_retry(
        &mut self,
        command: u8,
        arg1: u8,
        arg2: u8,
        arg3: u8,
        data: &[u8],
    ) -> Result<()> {
        let start_time = Instant::now();
        loop {
            self.cmd(command, arg1, arg2, arg3, data)?;
            if self.recv[1] != I2C_BUSY {
                return Ok(());
            }
            if start_time.elapsed() > self.config.timeout {
                let _ = self.cancel_transfer();
                return Err(Error::ReadTimeout);
            }
        }
    }

    fn cmd(&mut self, command: u8, arg1: u8, arg2: u8, arg3: u8, data: &[u8]) -> Result<()> {
        assert!(data.len() <= MCP_MAX_DATA_SIZE);
        let mut buffer = [0u8; MCP_TRANSFER_SIZE];
        buffer[0] = command;
        buffer[1] = arg1;
        buffer[2] = arg2;
        buffer[3] = arg3;
        buffer[MCP_HEADER_SIZE..MCP_HEADER_SIZE + data.len()].copy_from_slice(data);
        self.transfer(&buffer)
    }

    fn transfer(&mut self, buffer: &[u8]) -> Result<()> {
        self.handle
            .write_interrupt(MCP_WRITE_ENDPOINT, buffer, self.config.timeout)?;

        let size =
            self.handle
                .read_interrupt(MCP_READ_ENDPOINT, &mut self.recv, self.config.timeout)?;
        if size != MCP_TRANSFER_SIZE {
            return Err(Error::ShortRead);
        }
        Ok(())
    }

    fn read_status(&mut self) -> Result<()> {
        self.cmd(COMMAND_STATUS, 0, 0, 0, &[])?;
        if self.recv[1] != 0 {
            Err(Error::CommandFailed)
        } else {
            Ok(())
        }
    }

    fn cancel_transfer(&mut self) -> Result<()> {
        self.cmd(COMMAND_STATUS, 0, 0x10, 0, &[])?;
        if self.recv[1] != 0 {
            Err(Error::CancelFailed)
        } else {
            Ok(())
        }
    }

    fn set_speed(&mut self, speed_hz: u32) -> Result<()> {
        let divisor = (12_000_000 / speed_hz - 2) as u8;
        const SET_CLOCK_DIVISOR: u8 = 0x20;
        self.cmd(COMMAND_STATUS, 0, 0, SET_CLOCK_DIVISOR, &[divisor])?;
        if self.recv[1] != 0 {
            Err(Error::SetBusSpeedFailed)
        } else {
            Ok(())
        }
    }

    /// Returns information about the device.
    pub fn get_device_info(&mut self) -> Result<DeviceInfo> {
        self.read_status()?;
        Ok(DeviceInfo {
            hardware_major: self.recv[46],
            hardware_minor: self.recv[47],
            firmware_major: self.recv[48],
            firmware_minor: self.recv[49],
        })
    }
}

impl i2c::WriteRead for Handle {
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_read_address(address, bytes, buffer)
    }
}

impl i2c::Write for Handle {
    type Error = Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_read_address(address, bytes, &mut [])
    }
}

impl i2c::Read for Handle {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.write_read_address(address, &[], buffer)
    }
}

pub struct DeviceInfo {
    hardware_major: u8,
    hardware_minor: u8,
    firmware_major: u8,
    firmware_minor: u8,
}

impl Default for GpioConfig {
    fn default() -> Self {
        let mut command_buffer = [0u8; MCP_TRANSFER_SIZE];
        command_buffer[0] = COMMAND_SET_SRAM_SETTINGS;
        // Tell MCP that we're changing GPIO settings.
        command_buffer[7] = 1 << 7;
        Self { command_buffer }
    }
}

impl GpioConfig {
    /// Sets a GPIO pin to input or output.
    pub fn set_direction(&mut self, pin: usize, direction: Direction) {
        assert!(pin < NUM_GPIO_PINS);
        let config = &mut self.command_buffer[8 + pin];
        const DIRECTION_BIT: u8 = 1 << 3;
        match direction {
            Direction::Input => *config |= DIRECTION_BIT,
            Direction::Output => *config &= !DIRECTION_BIT,
        };
    }

    /// Sets the value of an output pin.
    pub fn set_value(&mut self, pin: usize, value: bool) {
        assert!(pin < NUM_GPIO_PINS);
        let config = &mut self.command_buffer[8 + pin];
        const VALUE_BIT: u8 = 1 << 4;
        match value {
            true => *config |= VALUE_BIT,
            false => *config &= !VALUE_BIT,
        };
    }
}

impl Display for DeviceInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MCP rev {}.{} firmware {}.{}",
            self.hardware_major as char,
            self.hardware_minor as char,
            self.firmware_major as char,
            self.firmware_minor as char
        )
    }
}

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        Error::UsbError(error)
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::UsbError(e) => Some(e),
            _ => None,
        }
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::DeviceNotFound => write!(f, "Device not found"),
            Error::WriteTooLarge => write!(f, "Write too large"),
            Error::UsbError(inner) => write!(f, "USB error: {}", inner),
            Error::ReadTimeout => write!(f, "Read timeout"),
            Error::WriteTimeout => write!(f, "Write timeout"),
            Error::SetBusSpeedFailed => write!(f, "Failed to set bus speed"),
            Error::ShortRead => write!(f, "Short read"),
            Error::UnexpectedReadSize => write!(f, "Unexpected read size"),
            Error::ReadError => write!(f, "Read error"),
            Error::DeviceLostOnReset => write!(f, "Device lost on reset"),
            Error::Nack => write!(f, "NACK"),
            Error::GpioError => write!(f, "GPIO error"),
            Error::CancelFailed => write!(f, "Cancel failed"),
            Error::CommandFailed => write!(f, "Command failed"),
            Error::SclLow => write!(f, "SCL is low"),
            Error::SdaLow => write!(f, "SDA is low"),
        }
    }
}

impl From<u8> for PinState {
    fn from(value: u8) -> Self {
        match value {
            0 => PinState::Low,
            1 => PinState::High,
            0xee => PinState::NotGpio,
            _ => PinState::Unknown,
        }
    }
}
