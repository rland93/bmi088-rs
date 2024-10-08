// ignore case warnings

#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccRegisters {
    ACC_SOFTRESET = 0x7E,
    ACC_PWR_CTRL = 0x7D,
    ACC_PWR_CONF = 0x7C,
    #[allow(dead_code)]
    ACC_SELF_TEST = 0x6D,
    INT_MAP_DATA = 0x58,
    INT2_IO_CTRL = 0x54,
    INT1_IO_CTRL = 0x53,
    FIFO_CONFIG_1 = 0x49,
    FIFO_CONFIG_0 = 0x48,
    FIFO_WTM_1 = 0x47,
    FIFO_WTM_0 = 0x46,
    FIFO_DOWNS = 0x45,
    ACC_RANGE = 0x41,
    ACC_CONF = 0x40,
    FIFO_DATA = 0x26,
    FIFO_LENGTH_1 = 0x25,
    FIFO_LENGTH_0 = 0x24,
    TEMP_LSB = 0x23,
    #[allow(dead_code)]
    TEMP_MSB = 0x22,
    ACC_INT_STAT_1 = 0x1D,
    #[allow(dead_code)]
    SENSORTIME_2 = 0x1A,
    #[allow(dead_code)]
    SENSORTIME_1 = 0x19,
    #[allow(dead_code)]
    SENSORTIME_0 = 0x18,
    #[allow(dead_code)]
    ACC_Z_MSB = 0x17,
    #[allow(dead_code)]
    ACC_Z_LSB = 0x16,
    #[allow(dead_code)]
    ACC_Y_MSB = 0x15,
    #[allow(dead_code)]
    ACC_Y_LSB = 0x14,
    #[allow(dead_code)]
    ACC_X_MSB = 0x13,
    ACC_X_LSB = 0x12,
    ACC_STATUS = 0x03,
    ACC_ERR_REG = 0x02,
    ACC_CHIP_ID = 0x00,
}

pub enum GyroRegisters {
    FIFO_DATA = 0x3F,
    FIFO_CONFIG_1 = 0x3E,
    FIFO_CONFIG_0 = 0x3D,
    #[allow(dead_code)]
    GYRO_SELF_TEST = 0x3C,
    FIFO_EXT_INT_S = 0x34,
    FIFO_WM_EN = 0x1E,
    INT3_INT4_IO_MAP = 0x18,
    INT3_INT4_IO_CONF = 0x16,
    GYRO_INT_CTRL = 0x15,
    GYRO_SOFTRESET = 0x14,
    GYRO_LPM1 = 0x11,
    GYRO_BANDWIDTH = 0x10,
    GYRO_RANGE = 0x0F,
    FIFO_STATUS = 0x0E,
    GYRO_INT_STAT_1 = 0x0A,
    #[allow(dead_code)]
    RATE_Z_MSB = 0x07,
    #[allow(dead_code)]
    RATE_Z_LSB = 0x06,
    #[allow(dead_code)]
    RATE_Y_MSB = 0x05,
    #[allow(dead_code)]
    RATE_Y_LSB = 0x04,
    #[allow(dead_code)]
    RATE_X_MSB = 0x03,
    RATE_X_LSB = 0x02,
    GYRO_CHIP_ID = 0x00,
}
