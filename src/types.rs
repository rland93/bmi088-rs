/// BMI088 errors.
#[derive(Debug)]
pub enum Error<CommE> {
    /// Interface communication error
    Comm(CommE),
    /// Invalid input data
    InvalidInputData,
    /// Bad write, if a write fails
    BadWrite,
}

/// Accelerometer Power Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccPowerConf {
    /// Normal Mode
    Active = 0x00,
    /// Suspend Mode
    Suspend = 0x03,
}

impl TryFrom<u8> for AccPowerConf {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(AccPowerConf::Active),
            0x03 => Ok(AccPowerConf::Suspend),
            _ => Err(Error::InvalidInputData),
        }
    }
}

/// Accelerometer Power Enable
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccPowerEnable {
    /// Power off
    Off = 0x00,
    /// Power on
    On = 0x04,
}

impl TryFrom<u8> for AccPowerEnable {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(AccPowerEnable::Off),
            0x04 => Ok(AccPowerEnable::On),
            _ => Err(Error::InvalidInputData),
        }
    }
}

/// Sensor Status
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Status {}

/// Accelerometer Error codes
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ErrCode {
    /// Error code
    pub error_code: bool,
    /// Fatal error
    pub fatal_error: bool,
}

impl ErrCode {
    /// ErrCode from a raw u8 register value
    pub fn from_u8(value: u8) -> Self {
        ErrCode {
            error_code: (value & 0b001_1100) != 0,
            fatal_error: (value & 0b000_0001) != 0,
        }
    }
}

/// Sensor data read selector
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensor3DData {
    /// X axis data
    pub x: i16,
    /// Y axis data
    pub y: i16,
    /// Z axis data
    pub z: i16,
}

/// Accelerometer bandwidth
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccBandwidth {
    /// osr4
    X4 = 0x00,
    /// osr2
    X2 = 0x01,
    /// none
    X1 = 0x02,
}

impl From<u8> for AccBandwidth {
    fn from(item: u8) -> Self {
        match item {
            0x00 => AccBandwidth::X4,
            0x01 => AccBandwidth::X2,
            0x02 => AccBandwidth::X1,
            _ => panic!("Invalid value for AccBandwidth"),
        }
    }
}

/// Accelerometer Data Rate
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccDataRate {
    /// 12.5 Hz
    Hz12_5 = 0x05,
    /// 25 Hz
    Hz25 = 0x06,
    /// 50 Hz
    Hz50 = 0x07,
    /// 100 Hz
    Hz100 = 0x08,
    /// 200 Hz
    Hz200 = 0x09,
    /// 400 Hz
    Hz400 = 0x0A,
    /// 800 Hz
    Hz800 = 0x0B,
    /// 1600 Hz
    Hz1600 = 0x0C,
}

impl From<u8> for AccDataRate {
    fn from(item: u8) -> Self {
        match item {
            0x05 => AccDataRate::Hz12_5,
            0x06 => AccDataRate::Hz25,
            0x07 => AccDataRate::Hz50,
            0x08 => AccDataRate::Hz100,
            0x09 => AccDataRate::Hz200,
            0x0A => AccDataRate::Hz400,
            0x0B => AccDataRate::Hz800,
            0x0C => AccDataRate::Hz1600,
            _ => panic!("Invalid value for AccDataRate"),
        }
    }
}

/// Stuct to hold Conf register values
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccConf {
    /// Bandwidth
    pub acc_bwp: AccBandwidth,
    /// Data Rate
    pub acc_odr: AccDataRate,
}

impl From<u8> for AccConf {
    fn from(item: u8) -> Self {
        let acc_bwp = AccBandwidth::from((item & 0b0111_0000) >> 4);
        let acc_odr = AccDataRate::from(item & 0b0000_1111);
        AccConf { acc_bwp, acc_odr }
    }
}

/// Accelerometer range configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccRange {
    /// ±3g
    G3 = 0x00,
    /// ±6g
    G6 = 0x01,
    /// ±12g
    G12 = 0x02,
    /// ±24g
    G24 = 0x03,
}

impl From<u8> for AccRange {
    /// Raw register to AccRange
    fn from(item: u8) -> Self {
        match item & 0b0000_0011 {
            // mask out the reserved bits
            0x00 => AccRange::G3,
            0x01 => AccRange::G6,
            0x02 => AccRange::G12,
            0x03 => AccRange::G24,
            _ => panic!("Invalid value for AccRange"),
        }
    }
}
/// Struct to hold a complete accelerometer configuration
#[derive(Debug, Clone, Copy, PartialEq)]

pub struct AccelerometerConfig {
    /// Bandwidth of the low pass filter
    pub conf: AccConf,
    /// Range
    pub acc_range: AccRange,
}
