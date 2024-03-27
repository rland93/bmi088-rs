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

impl TryFrom<u8> for AccBandwidth {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(AccBandwidth::X4),
            0x01 => Ok(AccBandwidth::X2),
            0x02 => Ok(AccBandwidth::X1),
            _ => Err(Error::InvalidInputData),
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

impl TryFrom<u8> for AccDataRate {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x05 => Ok(AccDataRate::Hz12_5),
            0x06 => Ok(AccDataRate::Hz25),
            0x07 => Ok(AccDataRate::Hz50),
            0x08 => Ok(AccDataRate::Hz100),
            0x09 => Ok(AccDataRate::Hz200),
            0x0A => Ok(AccDataRate::Hz400),
            0x0B => Ok(AccDataRate::Hz800),
            0x0C => Ok(AccDataRate::Hz1600),
            _ => Err(Error::InvalidInputData),
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

impl TryFrom<u8> for AccConf {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        let acc_bwp = AccBandwidth::try_from((item & 0b0111_0000) >> 4)?;
        let acc_odr = AccDataRate::try_from(item & 0b0000_1111)?;
        Ok(AccConf { acc_bwp, acc_odr })
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

impl TryFrom<u8> for AccRange {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item & 0b0000_0011 {
            // mask out the reserved bits
            0x00 => Ok(AccRange::G3),
            0x01 => Ok(AccRange::G6),
            0x02 => Ok(AccRange::G12),
            0x03 => Ok(AccRange::G24),
            _ => Err(Error::InvalidInputData),
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

/// Struct to hold the configuration of an input pin
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntConfiguration {
    /// Input or output
    pub int_pin: IntPin,
    /// Push-pull or open-drain
    pub int_od: PinBehavior,
    /// Active high or active low
    pub int_lvl: PinActive,
}

impl From<u8> for IntConfiguration {
    fn from(item: u8) -> Self {
        let int_pin = if (item & 0b0001_0000) != 0 {
            IntPin::Input
        } else {
            IntPin::Output
        };
        let int_od = if (item & 0b0000_0100) != 0 {
            PinBehavior::OpenDrain
        } else {
            PinBehavior::PushPull
        };
        let int_lvl = if (item & 0b0000_0010) != 0 {
            PinActive::ActiveLow
        } else {
            PinActive::ActiveHigh
        };
        IntConfiguration {
            int_pin,
            int_od,
            int_lvl,
        }
    }
}

impl From<&IntConfiguration> for u8 {
    fn from(item: &IntConfiguration) -> Self {
        let mut value = 0;
        match item.int_pin {
            IntPin::Input => value |= 0b0001_0000,
            IntPin::Output => value |= 0b0000_0000,
        }
        match item.int_od {
            PinBehavior::PushPull => value |= 0b0000_0000,
            PinBehavior::OpenDrain => value |= 0b0000_0100,
        }
        match item.int_lvl {
            PinActive::ActiveHigh => value |= 0b0000_0000,
            PinActive::ActiveLow => value |= 0b0000_0010,
        }
        value
    }
}

impl From<IntConfiguration> for u8 {
    fn from(val: IntConfiguration) -> Self {
        let mut value = 0;
        match val.int_pin {
            IntPin::Input => value |= 0b0001_0000,
            IntPin::Output => value |= 0b0000_0000,
        }
        match val.int_od {
            PinBehavior::PushPull => value |= 0b0000_0000,
            PinBehavior::OpenDrain => value |= 0b0000_0100,
        }
        match val.int_lvl {
            PinActive::ActiveHigh => value |= 0b0000_0000,
            PinActive::ActiveLow => value |= 0b0000_0010,
        }
        value
    }
}

/// Data ready map for accelerometer interrupt
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccDrdyMap {
    /// Map data ready to neither pin
    None = 0b0000_0000,
    /// Map data ready interrupt to output pin INT1
    Int1 = 0b0000_0100,
    /// Map data ready interrupt to output pin INT2
    Int2 = 0b0100_0000,
    /// Map data ready interrupt to output pin INT1 and INT2
    Int1Int2 = 0b0100_0100,
}

/// Gyroscope range configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroRange {
    /// ±2000°/s
    Dps2000 = 0x00,
    /// ±1000°/s
    Dps1000 = 0x01,
    /// ±500°/s
    Dps500 = 0x02,
    /// ±250°/s
    Dps250 = 0x03,
    /// ±125°/s
    Dps125 = 0x04,
}

impl TryFrom<u8> for GyroRange {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroRange::Dps2000),
            0x01 => Ok(GyroRange::Dps1000),
            0x02 => Ok(GyroRange::Dps500),
            0x03 => Ok(GyroRange::Dps250),
            0x04 => Ok(GyroRange::Dps125),
            _ => Err(Error::InvalidInputData),
        }
    }
}

/// Gyroscope bandwidth configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroBandwidth {
    /// 532 Hz, ODR 2000 Hz
    Hz532 = 0x00,
    /// 230 Hz, ODR 2000 Hz
    Hz230 = 0x01,
    /// 116 Hz, ODR 1000 Hz
    Hz116 = 0x02,
    /// 47 Hz, ODR 400 Hz
    Hz47 = 0x03,
    /// 23 Hz, ODR 200 Hz
    Hz23 = 0x04,
    /// 12 Hz, ODR 100 Hz
    Hz12 = 0x05,
    /// 64 Hz, ODR 200 Hz
    Hz64 = 0x06,
    /// 32 Hz, ODR 100 Hz
    Hz32 = 0x07,
}

impl TryFrom<u8> for GyroBandwidth {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroBandwidth::Hz532),
            0x01 => Ok(GyroBandwidth::Hz230),
            0x02 => Ok(GyroBandwidth::Hz116),
            0x03 => Ok(GyroBandwidth::Hz47),
            0x04 => Ok(GyroBandwidth::Hz23),
            0x05 => Ok(GyroBandwidth::Hz12),
            0x06 => Ok(GyroBandwidth::Hz64),
            0x07 => Ok(GyroBandwidth::Hz32),
            _ => Err(Error::InvalidInputData),
        }
    }
}

/// Gyroscope power mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroPowerMode {
    /// Normal mode
    Normal = 0x00,
    /// Deep suspend mode
    DeepSuspend = 0x20,
    /// Suspend mode
    Suspend = 0x80,
}

impl TryFrom<u8> for GyroPowerMode {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroPowerMode::Normal),
            0x20 => Ok(GyroPowerMode::DeepSuspend),
            0x80 => Ok(GyroPowerMode::Suspend),
            _ => Err(Error::InvalidInputData),
        }
    }
}

/// Gyroscope data ready interrupt mapping
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroDrdyMap {
    /// Map data ready to neither pin
    None = 0x00,
    /// Map data ready interrupt to output pin INT1
    Int3 = 0x01,
    /// Map data ready interrupt to output pin INT2
    Int4 = 0x80,
    /// Map data ready interrupt to output pin INT1 and INT2
    Int4Int5 = 0x81,
}

impl TryFrom<u8> for GyroDrdyMap {
    type Error = Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroDrdyMap::None),
            0x01 => Ok(GyroDrdyMap::Int3),
            0x80 => Ok(GyroDrdyMap::Int4),
            0x81 => Ok(GyroDrdyMap::Int4Int5),
            _ => Err(Error::InvalidInputData),
        }
    }
}
