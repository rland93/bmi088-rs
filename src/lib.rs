//! Driver for Bosch Sensortec BMI088 6-axis IMU. Uses embedded-hal traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//!

#![deny(unsafe_code)]
#![no_std]

const I2C_ACC_BASE_ADDR: u8 = 0x18;
const I2C_ACC_ALT_ADDR: u8 = 0x19;
const I2C_GYRO_BASE_ADDR: u8 = 0x68;
const I2C_GYRO_ALT_ADDR: u8 = 0x69;

mod builder;
pub mod interface;
mod reg;
pub mod sensor;
pub mod types;

pub use builder::{AccConfiguration, GyroConfiguration};
pub use interface::{Addr, I2cInterface, SpiInterface};
pub use types::acc::*;
pub use types::gyro::*;
pub use types::{IntPin, PinActive, PinBehavior, Sensor3DData};

/// BMI088 device object.
#[derive(Debug)]
pub struct Bmi088<DI> {
    /// Digital interface (i2c)
    pub iface: DI,
}

mod private {
    use super::interface;
    pub trait Sealed {}

    impl<I2C> Sealed for interface::I2cInterface<I2C> {}
    impl<SPI> Sealed for interface::SpiInterface<SPI> {}
}

impl<I2C> Bmi088<interface::I2cInterface<I2C>> {
    /// Create new instance of the BMI088 device communicating through I2C.
    pub fn new_with_i2c(i2c: I2C, acc_alt: bool, gyro_alt: bool) -> Self {
        // gyro, accelerometer are addressed separately.

        let acc_addr = if !acc_alt {
            Addr::Acc(I2C_ACC_BASE_ADDR)
        } else {
            Addr::Acc(I2C_ACC_ALT_ADDR)
        };

        let gyro_addr = if !gyro_alt {
            Addr::Gyro(I2C_GYRO_BASE_ADDR)
        } else {
            Addr::Gyro(I2C_GYRO_ALT_ADDR)
        };

        Bmi088 {
            iface: interface::I2cInterface {
                i2c,
                acc_addr: acc_addr.as_u8(),
                gyro_addr: gyro_addr.as_u8(),
            },
        }
    }

    /// Destroy driver instance, return I2C device instance.
    pub fn destroy(self) -> I2C {
        self.iface.i2c
    }
}

impl<SPI> Bmi088<interface::SpiInterface<SPI>> {
    /// Create new instance of the BMI088 device communicating through SPI.
    pub fn new_with_spi(spi: SPI) -> Self {
        Self {
            iface: interface::SpiInterface { spi },
        }
    }
}
