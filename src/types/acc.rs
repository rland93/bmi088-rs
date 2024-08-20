/// Accelerometer Power Conf
///
/// ACC_PWR_CONF 0x7C
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccPowerConf {
    Active = 0x00,
    Suspend = 0x03,
}

impl TryFrom<u8> for AccPowerConf {
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(AccPowerConf::Active),
            0x03 => Ok(AccPowerConf::Suspend),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// Accelerometer Power Ctrl
///
/// ACC_PWR_CTRL 0x7D
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccPowerCtrl {
    Off = 0x00,
    On = 0x04,
}

impl TryFrom<u8> for AccPowerCtrl {
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(AccPowerCtrl::Off),
            0x04 => Ok(AccPowerCtrl::On),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// Accelerometer Error codes
///
/// ACC_ERR_REG 0x02
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ErrCode {
    pub error_code: bool,
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

/// Accelerometer bandwidth
///
/// ACC_CONF 0x40
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccBandwidth {
    X4 = 0b1000_0000,
    X2 = 0b1001_0000,
    X1 = 0b1010_0000,
}

impl TryFrom<u8> for AccBandwidth {
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item & 0b1111_0000 {
            0b1000_0000 => Ok(AccBandwidth::X4),
            0b1001_0000 => Ok(AccBandwidth::X2),
            0b1010_0000 => Ok(AccBandwidth::X1),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// Accelerometer Data Rate
///
/// ACC_CONF 0x40
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccDataRate {
    /// 12.5 Hz
    Hz12_5 = 0b0000_0101,
    /// 25 Hz
    Hz25 = 0b0000_0110,
    /// 50 Hz
    Hz50 = 0b0000_0111,
    /// 100 Hz
    Hz100 = 0b0000_1000,
    /// 200 Hz
    Hz200 = 0b0000_1001,
    /// 400 Hz
    Hz400 = 0b0000_1010,
    /// 800 Hz
    Hz800 = 0b0000_1011,
    /// 1600 Hz
    Hz1600 = 0b0000_1100,
}

impl TryFrom<u8> for AccDataRate {
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item & 0b0000_1111 {
            0b0000_0101 => Ok(AccDataRate::Hz12_5),
            0b0000_0110 => Ok(AccDataRate::Hz25),
            0b0000_0111 => Ok(AccDataRate::Hz50),
            0b0000_1000 => Ok(AccDataRate::Hz100),
            0b0000_1001 => Ok(AccDataRate::Hz200),
            0b0000_1010 => Ok(AccDataRate::Hz400),
            0b0000_1011 => Ok(AccDataRate::Hz800),
            0b0000_1100 => Ok(AccDataRate::Hz1600),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// Accelerometer range configuration
///
/// ACC_CONF 0x41
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
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item & 0b0000_0011 {
            // mask out the reserved bits
            0x00 => Ok(AccRange::G3),
            0x01 => Ok(AccRange::G6),
            0x02 => Ok(AccRange::G12),
            0x03 => Ok(AccRange::G24),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// FIFO configuration
///
/// FIFO_CONFIG_0 0x48
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccFifoMode {
    Fifo = 0b0000_0010,
    Stream = 0b0000_0011,
}

impl From<u8> for AccFifoMode {
    fn from(item: u8) -> Self {
        match item {
            0b0000_0010 => AccFifoMode::Fifo,
            0b0000_0011 => AccFifoMode::Stream,
            _ => AccFifoMode::Fifo,
        }
    }
}

impl From<AccFifoMode> for u8 {
    fn from(item: AccFifoMode) -> Self {
        match item {
            AccFifoMode::Fifo => 0b0000_0010,
            AccFifoMode::Stream => 0b0000_0011,
        }
    }
}

/// FIFO configuration
///
/// FIFO_CONFIG_1 0x49
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccFifoConfig1 {
    pub acc_en: bool,
    pub int1_input_en: bool,
    pub int2_input_en: bool,
}

impl From<u8> for AccFifoConfig1 {
    fn from(item: u8) -> Self {
        AccFifoConfig1 {
            acc_en: (item & 0b0100_0000) != 0,
            int1_input_en: (item & 0b0000_1000) != 0,
            int2_input_en: (item & 0b0000_0100) != 0,
        }
    }
}

impl From<AccFifoConfig1> for u8 {
    fn from(item: AccFifoConfig1) -> u8 {
        let mut value = 0b0001_0000;
        if item.acc_en {
            value |= 0b0100_0000;
        }
        if item.int1_input_en {
            value |= 0b0000_1000;
        }
        if item.int2_input_en {
            value |= 0b0000_0100;
        }
        value
    }
}

/// FIFO configuration
///
/// FIFO_CONFIG_1 0x49
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FifoExtIntPin {
    /// INT3
    Int3 = 0x00,
    /// INT4
    Int4 = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FifoExtIntS {
    pub ext_sync_mode: bool,
    pub ext_sync_sel: FifoExtIntPin,
}

impl From<u8> for FifoExtIntS {
    fn from(item: u8) -> Self {
        FifoExtIntS {
            ext_sync_mode: (item & 0b0010_0000) != 0,
            ext_sync_sel: if (item & 0b0001_0000) != 0 {
                FifoExtIntPin::Int4
            } else {
                FifoExtIntPin::Int3
            },
        }
    }
}

impl From<FifoExtIntS> for u8 {
    fn from(item: FifoExtIntS) -> Self {
        let mut value = 0b0000_0000;
        if item.ext_sync_mode {
            value |= 0b0010_0000;
        }
        match item.ext_sync_sel {
            FifoExtIntPin::Int3 => (),
            FifoExtIntPin::Int4 => value |= 0b0001_0000,
        }
        value
    }
}

/// Struct to hold the configuration of an input pin
///
/// INT1_IO_CTRL 0x53
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccIntConfiguration {
    /// Input or output
    pub int_pin: crate::types::IntPin,
    /// Push-pull or open-drain
    pub int_od: crate::types::PinBehavior,
    /// Active high or active low
    pub int_lvl: crate::types::PinActive,
}

impl From<u8> for AccIntConfiguration {
    fn from(item: u8) -> Self {
        let int_pin = if (item & 0b0001_0000) != 0 {
            crate::types::IntPin::Input
        } else {
            crate::types::IntPin::Output
        };
        let int_od = if (item & 0b0000_0100) != 0 {
            crate::types::PinBehavior::OpenDrain
        } else {
            crate::types::PinBehavior::PushPull
        };
        let int_lvl = if (item & 0b0000_0010) != 0 {
            crate::types::PinActive::ActiveLow
        } else {
            crate::types::PinActive::ActiveHigh
        };
        AccIntConfiguration {
            int_pin,
            int_od,
            int_lvl,
        }
    }
}

impl From<&AccIntConfiguration> for u8 {
    fn from(item: &AccIntConfiguration) -> Self {
        let mut value = 0;
        match item.int_pin {
            crate::types::IntPin::Input => value |= 0b0001_0000,
            crate::types::IntPin::Output => value |= 0b0000_1000,
        }
        match item.int_od {
            crate::types::PinBehavior::PushPull => value |= 0b0000_0000,
            crate::types::PinBehavior::OpenDrain => value |= 0b0000_0100,
        }
        match item.int_lvl {
            crate::types::PinActive::ActiveHigh => value |= 0b0000_0000,
            crate::types::PinActive::ActiveLow => value |= 0b0000_0010,
        }
        value
    }
}

impl From<AccIntConfiguration> for u8 {
    fn from(val: AccIntConfiguration) -> Self {
        let mut value = 0;
        match val.int_pin {
            crate::types::IntPin::Input => value |= 0b0001_0000,
            crate::types::IntPin::Output => value |= 0b0000_1000,
        }
        match val.int_od {
            crate::types::PinBehavior::PushPull => value |= 0b0000_0000,
            crate::types::PinBehavior::OpenDrain => value |= 0b0000_0100,
        }
        match val.int_lvl {
            crate::types::PinActive::ActiveHigh => value |= 0b0000_0000,
            crate::types::PinActive::ActiveLow => value |= 0b0000_0010,
        }
        value
    }
}

/// Interrupt mapping
///
/// INT_MAP_DATA 0x58
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntMap {
    Drdy,
    Fwm,
    Ffull,
    None,
}

/// Interrupt mapping
///
/// INT_MAP_DATA 0x58
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccIntMap {
    pub int1: IntMap,
    pub int2: IntMap,
}

impl From<u8> for AccIntMap {
    fn from(item: u8) -> Self {
        let int1 = match item & 0b0000_0111 {
            0b0000_0100 => IntMap::Drdy,
            0b0000_0010 => IntMap::Fwm,
            0b0000_0001 => IntMap::Ffull,
            _ => IntMap::None,
        };

        let int2 = match item & 0b0111_0000 {
            0b0100_0000 => IntMap::Drdy,
            0b0010_0000 => IntMap::Fwm,
            0b0001_0000 => IntMap::Ffull,
            _ => IntMap::None,
        };

        Self { int1, int2 }
    }
}

impl From<AccIntMap> for u8 {
    fn from(item: AccIntMap) -> u8 {
        let mut val = 0;
        val |= match item.int1 {
            IntMap::Drdy => 0b0000_0100,
            IntMap::Fwm => 0b0000_0010,
            IntMap::Ffull => 0b0000_0001,
            IntMap::None => 0b0000_0000,
        };
        val |= match item.int2 {
            IntMap::Drdy => 0b0100_0000,
            IntMap::Fwm => 0b0010_0000,
            IntMap::Ffull => 0b0001_0000,
            IntMap::None => 0b0000_0000,
        };
        defmt::debug!("int map: {=u8:#04b}", val);
        val
    }
}

/// Accelerometer FIFO Frame header
///
/// FIFO_DATA 0x26
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccIntFrameHeader {
    pub int1: bool,
    pub int2: bool,
}

impl From<u8> for AccIntFrameHeader {
    fn from(item: u8) -> Self {
        AccIntFrameHeader {
            int1: (item & 0b0000_0001) != 0,
            int2: (item & 0b0000_0010) != 0,
        }
    }
}

/// Accelerometer FIFO Frame header
///
/// FIFO_DATA 0x26
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum AccFifoFrameHeader {
    Acceleration(AccIntFrameHeader) = 0b1000_0100,
    Skip = 0b0100_0000,
    SensorTime = 0b0100_0100,
    InputConfig = 0b0100_1000,
    SampleSkip = 0b0101_0000,
    Invalid = 0b0000_0000,
    End = 0b1000_0000,
}

impl From<u8> for AccFifoFrameHeader {
    fn from(item: u8) -> Self {
        match item & 0b1111_1100 {
            0b1000_0100 => {
                let status = AccIntFrameHeader::from(item);
                AccFifoFrameHeader::Acceleration(status)
            }
            0b0100_0000 => AccFifoFrameHeader::Skip,
            0b0100_0100 => AccFifoFrameHeader::SensorTime,
            0b0100_1000 => AccFifoFrameHeader::InputConfig,
            0b0101_0000 => AccFifoFrameHeader::SampleSkip,
            0b1000_0000 => AccFifoFrameHeader::End,
            _ => AccFifoFrameHeader::Invalid,
        }
    }
}

impl AccFifoFrameHeader {
    /// number of bytes to read subsequently for the given frame type
    pub fn bytes_to_read(&self) -> usize {
        match self {
            AccFifoFrameHeader::Acceleration(_) => 6,
            AccFifoFrameHeader::Skip => 1,
            AccFifoFrameHeader::SensorTime => 3,
            AccFifoFrameHeader::InputConfig => 1,
            AccFifoFrameHeader::SampleSkip => 1,
            AccFifoFrameHeader::Invalid => 1,
            AccFifoFrameHeader::End => 1,
        }
    }
}
