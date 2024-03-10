//! Driver for Bosch Sensortec BMI088 6-axis IMU. Uses embedded-hal traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//!

#![deny(unsafe_code, missing_docs)]
#![no_std]

pub mod interface;
mod reg;
mod sensor;
mod types;
pub use crate::interface::Addr;
pub use crate::types::{Error, Status};

/// BMI088 device object.
#[derive(Debug)]
pub struct Bmi088<DI> {
    /// Digital interface (SPI)
    iface: DI,
}

mod private {
    use super::interface;
    pub trait Sealed {}

    impl<SPI> Sealed for interface::SpiInterface<SPI> {}
    impl<I2C> Sealed for interface::I2cInterface<I2C> {}
}
