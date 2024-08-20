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
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroRange::Dps2000),
            0x01 => Ok(GyroRange::Dps1000),
            0x02 => Ok(GyroRange::Dps500),
            0x03 => Ok(GyroRange::Dps250),
            0x04 => Ok(GyroRange::Dps125),
            _ => Err(crate::types::Error::InvalidInputData),
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
    type Error = crate::types::Error<()>;

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
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

/// Gyroscope power mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroPowerConf {
    /// Normal mode
    Normal = 0x00,
    /// Deep suspend mode
    DeepSuspend = 0x20,
    /// Suspend mode
    Suspend = 0x80,
}

impl TryFrom<u8> for GyroPowerConf {
    type Error = crate::types::Error<()>;

    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(GyroPowerConf::Normal),
            0x20 => Ok(GyroPowerConf::DeepSuspend),
            0x80 => Ok(GyroPowerConf::Suspend),
            _ => Err(crate::types::Error::InvalidInputData),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroIntMap {
    Drdy,
    Fifo,
    DrdyFifo,
    None,
}

/// Gyroscope data ready interrupt mapping
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroIntMapping {
    pub int3: GyroIntMap,
    pub int4: GyroIntMap,
}

impl From<u8> for GyroIntMapping {
    fn from(item: u8) -> Self {
        Self {
            int3: match item & 0b0000_0101 {
                0x05 => GyroIntMap::DrdyFifo,
                0x01 => GyroIntMap::Drdy,
                0x04 => GyroIntMap::Fifo,
                0x00 => GyroIntMap::None,
                _ => GyroIntMap::None,
            },
            int4: match item & 0b1010_0000 {
                0xA0 => GyroIntMap::DrdyFifo,
                0x80 => GyroIntMap::Drdy,
                0x20 => GyroIntMap::Fifo,
                0x00 => GyroIntMap::None,
                _ => GyroIntMap::None,
            },
        }
    }
}

impl From<GyroIntMapping> for u8 {
    fn from(item: GyroIntMapping) -> Self {
        let mut val = 0;
        match item.int3 {
            GyroIntMap::Drdy => val |= 0x01,
            GyroIntMap::Fifo => val |= 0x04,
            GyroIntMap::DrdyFifo => val |= 0x05,
            GyroIntMap::None => {}
        }
        match item.int4 {
            GyroIntMap::Drdy => val |= 0x80,
            GyroIntMap::Fifo => val |= 0x20,
            GyroIntMap::DrdyFifo => val |= 0xA0,
            GyroIntMap::None => {}
        }
        val
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroFifoModeConf {
    Fifo = 0x40,
    Stream = 0x80,
}

impl From<u8> for GyroFifoModeConf {
    fn from(item: u8) -> Self {
        match item {
            0x40 => GyroFifoModeConf::Fifo,
            0x80 => GyroFifoModeConf::Stream,
            _ => GyroFifoModeConf::Fifo,
        }
    }
}

impl From<GyroFifoModeConf> for u8 {
    fn from(item: GyroFifoModeConf) -> Self {
        match item {
            GyroFifoModeConf::Fifo => 0x40,
            GyroFifoModeConf::Stream => 0x80,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroIntEnable {
    data: bool,
    fifo: bool,
}

impl From<u8> for GyroIntEnable {
    fn from(item: u8) -> Self {
        GyroIntEnable {
            data: (item & 0b1000_0000) != 0,
            fifo: (item & 0b0100_0000) != 0,
        }
    }
}

impl From<GyroIntEnable> for u8 {
    fn from(item: GyroIntEnable) -> Self {
        let mut val = 0;
        if item.data {
            val |= 0b1000_0000;
        }
        if item.fifo {
            val |= 0b0100_0000;
        }
        val
    }
}

pub struct GyroIntPinConfiguration {
    int3_od: crate::types::PinBehavior,
    int3_lvl: crate::types::PinActive,
    int4_od: crate::types::PinBehavior,
    int4_lvl: crate::types::PinActive,
}

impl From<u8> for GyroIntPinConfiguration {
    fn from(item: u8) -> Self {
        GyroIntPinConfiguration {
            int3_od: if (item & 0b0000_0010) != 0 {
                crate::types::PinBehavior::OpenDrain
            } else {
                crate::types::PinBehavior::PushPull
            },
            int3_lvl: if (item & 0b0000_0001) != 0 {
                crate::types::PinActive::ActiveLow
            } else {
                crate::types::PinActive::ActiveHigh
            },
            int4_od: if (item & 0b0000_1000) != 0 {
                crate::types::PinBehavior::OpenDrain
            } else {
                crate::types::PinBehavior::PushPull
            },
            int4_lvl: if (item & 0b0000_0100) != 0 {
                crate::types::PinActive::ActiveLow
            } else {
                crate::types::PinActive::ActiveHigh
            },
        }
    }
}

impl From<GyroIntPinConfiguration> for u8 {
    fn from(item: GyroIntPinConfiguration) -> Self {
        let mut val = 0;
        if item.int3_od == crate::types::PinBehavior::OpenDrain {
            val |= 0b0000_0010;
        }
        if item.int3_lvl == crate::types::PinActive::ActiveLow {
            val |= 0b0000_0001;
        }
        if item.int4_od == crate::types::PinBehavior::OpenDrain {
            val |= 0b0000_1000;
        }
        if item.int4_lvl == crate::types::PinActive::ActiveLow {
            val |= 0b0000_0100;
        }
        val
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroIntPin {
    Int3 = 0x00,
    Int4 = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroExtIntS {
    enable: bool,
    source: GyroIntPin,
}

impl From<u8> for GyroExtIntS {
    fn from(item: u8) -> Self {
        GyroExtIntS {
            enable: (item & 0b0010_0000) != 0,
            source: if (item & 0b0001_0000) != 0 {
                GyroIntPin::Int4
            } else {
                GyroIntPin::Int3
            },
        }
    }
}

impl From<GyroExtIntS> for u8 {
    fn from(item: GyroExtIntS) -> Self {
        let mut val = 0;
        if item.enable {
            val |= 0b0010_0000;
        }
        if item.source == GyroIntPin::Int4 {
            val |= 0b0001_0000;
        }
        val
    }
}

pub struct GyroIntConfiguration {
    pub map: GyroIntMapping,
    pub conf: GyroIntPinConfiguration,
    pub enable: GyroIntEnable,
}
