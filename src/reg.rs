// ignore case warnings
#![allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq)]

pub enum AccRegisters {
    /// Accelerometer Chip ID
    ACC_CHIP_ID = 0x00,
    /// Reports sensor error conditions
    ACC_ERR_REG = 0x02,
    /// Sensor status flag
    ACC_STATUS = 0x03,
    /// Acceleration data (0x12-0x17)
    ACC_DATA = 0x12,
    /// Sensortime data (0x18-0x1A)
    ACC_SENSORTIME = 0x18,
    /// Interrupt status register
    ACC_INT_STAT_1 = 0x1D,
    /// Temperature data register (0x22-0x23)
    TEMPERATURE = 0x22,
    /// Accelerometer configuration register
    ACC_CONF = 0x40,
    /// Accelerometer range setting
    ACC_RANGE = 0x41,
    /// Configures the input/output pin INT1.
    INT1_IO_CONF = 0x53,
    /// Configures the input/output pin INT2.
    INT2_IO_CONF = 0x54,
    /// Map data ready interrupt to output pin INT1 and/or INT2
    INT1_INT2_MAP_DATA = 0x58,
    // Enables the sensor self-test signal, occurring as a steady offset to the
    // sensor output. Note that the self-test needs to be switched off actively
    // by the user (details see 4.6.1).
    // ACC_SELF_TEST = 0x6D,
    /// Switches accelerometer into suspend mode for saving power. In this mode
    /// the data acquisition is
    ACC_PWR_CONF = 0x7C,
    /// Switches accelerometer ON or OFF. Required to do after every reset in
    /// order to obtain acceleration values.
    ACC_PWR_CTRL = 0x7D,
    /// Writing a value of 0xB6 to this register resets the sensor. Following a
    /// delay of 1 ms, all configuration settings are overwritten with their
    /// reset value. The soft-reset can be triggered from any operation mode.
    ACC_SOFTRESET = 0x7E,
}

pub enum GyroRegisters {
    /// Gyro ChipID
    GYRO_CHIP_ID = 0x00,
    /// Angular Rate Data (0x02-0x07)
    GYRO_RATE_DATA = 0x02,
    /// Interrupt Status
    GYRO_INT_STAT_1 = 0x0A,
    /// Gyro Range
    GYRO_RANGE = 0x0F,
    /// Gyro Bandwidth
    GYRO_BANDWIDTH = 0x10,
    /// Selection of the main power modes. Please note that only switching
    /// between normal mode and the suspend modes is allowed, it is not possible
    /// to switch between suspend and deep suspend and vice versa.
    GYRO_LPM1 = 0x11,
    /// Gyro Soft Reset
    GYRO_SOFTRESET = 0x14,
    /// Gyro Interrupt control
    GYRO_INT_CTRL = 0x15,
    /// Int3, Int4 IO configuration
    GYRO_INT3_INT4_IO_CONF = 0x16,
    /// Map data ready interrupt to output pin INT3 and/or INT4
    GYRO_INT3_INT4_IO_MAP = 0x18,
    // Built-in self-test of gyroscope.
    // GYRO_SELF_TEST = 0x3C,
}
