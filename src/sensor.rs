use crate::{
    interface::{I2cInterface, ReadData, SpiInterface, WriteData},
    reg::Registers,
    Addr, Bmi088, Error,
};

impl<SPI> Bmi088<SpiInterface<SPI>> {
    /// Create new instance of the BMI088 device communicating through SPI.
    pub fn new_with_spi(spi: SPI) -> Self {
        Bmi088 {
            iface: SpiInterface { spi },
        }
    }

    /// Destroy driver instance, return SPI device instance.
    pub fn destroy(self) -> SPI {
        self.iface.spi
    }
}

impl<I2C> Bmi088<I2cInterface<I2C>> {
    /// Create new instance of the BMI088 device communicating through I2C.
    pub fn new_with_i2c(i2c: I2C) -> Self {
        Bmi088 {
            iface: I2cInterface {
                i2c,
                address: Addr::default().address(),
            },
        }
    }

    /// Create new instance of the BMI088 device communicating through I2C with
    /// custom address.
    pub fn new_with_i2c_and_addr(i2c: I2C, address: u8) -> Self {
        Bmi088 {
            iface: I2cInterface { i2c, address },
        }
    }

    /// Destroy driver instance, return I2C device instance.
    pub fn destroy(self) -> I2C {
        self.iface.i2c
    }
}

impl<DI, CommE> Bmi088<DI>
where
    DI: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
{
    /// Get Accelerometer Chip ID
    pub fn acc_chip_id(&mut self) -> Result<u8, Error<CommE>> {
        self.iface.read_register(Registers::ACC_CHIP_ID as u8)
    }
}
