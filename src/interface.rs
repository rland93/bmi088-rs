//! I2C interfaces

use crate::{private, Error};
use embedded_hal::i2c;

/// I2C interface
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) acc_addr: u8,
    pub(crate) gyro_addr: u8,
}

/// Possible I2C Addresses
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Addr {
    /// Accelerometer
    Acc(u8),
    /// Gyro
    Gyro(u8),
}

impl Addr {
    /// Device i2c address
    pub fn addr(&self, alt: bool) -> Self {
        match self {
            Addr::Acc(addr) => Addr::Gyro(if alt { *addr + 1 } else { *addr }),
            Addr::Gyro(addr) => Addr::Gyro(if alt { *addr } else { *addr - 1 }),
        }
    }
    /// Get the address as u8
    pub fn as_u8(&self) -> u8 {
        match self {
            Addr::Acc(addr) => *addr,
            Addr::Gyro(addr) => *addr,
        }
    }
}

/// Write Data
pub trait WriteData: private::Sealed {
    /// Error type
    type Error;
    /// Write to an u8 register on the accelerometer
    fn write_register_acc(&mut self, register: u8, data: u8) -> Result<(), Self::Error>;
    /// Write data on the accelerometer, first element is starting address.
    fn write_data_acc(&mut self, payload: &mut [u8]) -> Result<(), Self::Error>;
    /// Write to an u8 register on the gyro
    fn write_register_gyro(&mut self, register: u8, data: u8) -> Result<(), Self::Error>;
    /// Write data to the gyro, first element is starting address.
    fn write_data_gyro(&mut self, payload: &mut [u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> WriteData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;

    fn write_register_acc(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let payload: [u8; 2] = [register, data];
        let addr = self.acc_addr;
        self.i2c.write(addr, &payload).map_err(Error::Comm)
    }

    fn write_data_acc(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        let addr = self.acc_addr;
        self.i2c.write(addr, payload).map_err(Error::Comm)
    }

    fn write_register_gyro(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let payload: [u8; 2] = [register, data];
        let addr = self.gyro_addr;
        self.i2c.write(addr, &payload).map_err(Error::Comm)
    }

    fn write_data_gyro(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        let addr = self.gyro_addr;
        self.i2c.write(addr, payload).map_err(Error::Comm)
    }
}

/// Read data
pub trait ReadData: private::Sealed {
    /// Error type
    type Error;
    /// Read an u8 `register` on the accelerometer
    fn read_register_acc(&mut self, register: u8) -> Result<u8, Self::Error>;
    /// Read some `payload` data from the accelerometer from `register`
    fn read_data_acc(&mut self, register: u8, payload: &mut [u8]) -> Result<(), Self::Error>;
    /// Read an u8 `register` on the gyro
    fn read_register_gyro(&mut self, register: u8) -> Result<u8, Self::Error>;
    /// Read some `payload` data from the gyro from `register`
    fn read_data_gyro(&mut self, register: u8, payload: &mut [u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> ReadData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;
    fn read_register_acc(&mut self, register: u8) -> Result<u8, Self::Error> {
        let addr = self.acc_addr;

        let mut write = [register];
        let mut read = [0xFF];

        self.i2c
            .write_read(addr, &mut write, &mut read)
            .map_err(Error::Comm)
            .and(Ok(read[0]))
    }

    fn read_data_acc(&mut self, register: u8, payload: &mut [u8]) -> Result<(), Self::Error> {
        let len = payload.len();
        let addr = self.acc_addr;
        self.i2c
            .write_read(addr, &[register], &mut payload[0..len])
            .map_err(Error::Comm)
    }

    fn read_register_gyro(&mut self, register: u8) -> Result<u8, Self::Error> {
        let addr = self.gyro_addr;

        let mut write = [register];
        let mut read = [0xFF];

        self.i2c
            .write_read(addr, &mut write, &mut read)
            .map_err(Error::Comm)
            .and(Ok(read[0]))
    }

    fn read_data_gyro(&mut self, register: u8, payload: &mut [u8]) -> Result<(), Self::Error> {
        let len = payload.len();
        let addr = self.gyro_addr;
        self.i2c
            .write_read(addr, &[register], &mut payload[0..len])
            .map_err(Error::Comm)
    }
}
