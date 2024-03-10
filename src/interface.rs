//! I2C/SPI interfaces

use crate::{private, Error};

use embedded_hal::{
    i2c::{self, I2c},
    spi::SpiDevice,
};

const I2C_DEV_BASE_ADDR: u8 = 0x18;

/// SPI interface
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    pub(crate) spi: SPI,
}

/// I2C interface
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
}

/// Possible I2C Addresses
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Addr {
    /// Default
    Default,
    /// Alternate
    Alternate(bool),
}

impl Default for Addr {
    /// Default address
    fn default() -> Self {
        Addr::Default
    }
}

impl Addr {
    /// Device i2c address
    pub fn address(&self) -> u8 {
        match self {
            Addr::Default => I2C_DEV_BASE_ADDR,
            Addr::Alternate(true) => I2C_DEV_BASE_ADDR | 0x01,
            Addr::Alternate(false) => I2C_DEV_BASE_ADDR,
        }
    }
}

/// Write Data
pub trait WriteData: private::Sealed {
    /// Error type
    type Error;
    /// Write to an u8 register
    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Self::Error>;
    /// Write data, first element is starting address.
    fn write_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error>;
}

impl<SPI, CommE> WriteData for SpiInterface<SPI>
where
    SPI: SpiDevice<u8, Error = CommE>,
{
    type Error = Error<CommE>;
    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let payload: [u8; 2] = [register, data];
        self.spi.write(&payload).map_err(Error::Comm)
    }

    fn write_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.write(payload).map_err(Error::Comm)
    }
}

impl<I2C, E> WriteData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;

    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let payload: [u8; 2] = [register, data];
        let addr = self.address;
        self.i2c.write(addr, &payload).map_err(Error::Comm)
    }

    fn write_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        let addr = self.address;
        self.i2c.write(addr, payload).map_err(Error::Comm)
    }
}

/// Read data
pub trait ReadData: private::Sealed {
    /// Error type
    type Error;
    /// Read an u8 register
    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error>;
    /// Read some data. The first element corresponds to the starting address.
    fn read_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error>;
}

impl<SPI, CommE> ReadData for SpiInterface<SPI>
where
    SPI: SpiDevice<u8, Error = CommE>,
{
    type Error = Error<CommE>;
    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error> {
        let mut data = [register + 0x80, 0];
        self.spi.transfer_in_place(&mut data).map_err(Error::Comm)?;
        Ok(data[1])
    }

    fn read_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        payload[0] += 0x80;
        self.spi.transfer_in_place(payload).map_err(Error::Comm)?;
        Ok(())
    }
}

impl<I2C, E> ReadData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;
    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error> {
        let mut data = [0];
        let addr = self.address;
        self.i2c
            .write_read(addr, &[register], &mut data)
            .map_err(Error::Comm)
            .and(Ok(data[0]))
    }

    fn read_data(&mut self, payload: &mut [u8]) -> Result<(), Self::Error> {
        let len = payload.len();
        let addr = self.address;
        self.i2c
            .write_read(addr, &[payload[0]], &mut payload[1..len])
            .map_err(Error::Comm)
    }
}
