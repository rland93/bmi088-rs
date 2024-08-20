pub mod acc;
pub mod gyro;

/// Input pin configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntPin {
    /// Input
    Input = 0b0001_0000,
    /// Output
    Output = 0b0000_1000,
}

/// Pin behavior configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinBehavior {
    /// Push-pull
    PushPull = 0b0000_0000,
    /// Open-drain
    OpenDrain = 0b0000_0100,
}

/// Pin active configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinActive {
    /// Active high
    ActiveHigh = 0b0000_0000,
    /// Active low
    ActiveLow = 0b0000_0010,
}

/// BMI088 errors.
#[derive(Debug)]
pub enum Error<CommE> {
    /// Interface communication error
    Comm(CommE),
    /// Invalid input data
    InvalidInputData,
    /// Invalid output data
    InvalidOutputData,
    /// Bad write, if a write fails
    BadWrite,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensor3DData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Sensor3DData {
    /// from a 6-byte slice, construct a Sensor3DData
    pub fn from_le_slice(bytes: &[u8]) -> Self {
        let x = i16::from_le_bytes(bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(bytes[4..6].try_into().unwrap());
        Self { x, y, z }
    }
}
