pub enum Registers {
    /// Accelerometer Chip ID
    ACC_CHIP_ID = 0x00,
    /// Reports sensor error conditions
    ACC_ERR_REG = 0x02,
    /// Sensor status flag
    ACC_STATUS = 0x03,
    /// Interrupt status register
    ACC_INT_STAT_1 = 0x1D,
    /// Accelerometer configuration register
    ACC_CONF = 0x40,
    /// Accelerometer range setting
    ACC_RANGE = 0x41,
    /// Reduction of sample rate
    FIFO_DOWNS = 0x45,
    /// Sets the FIFO Mode
    FIFO_CONFIG_0 = 0x48,
    /// Select sources for the FIFO buffer
    FIFO_CONFIG_1 = 0x49,
    /// Configures the input/output pin INT1
    INT1_IO_CONF = 0x53,
    /// Configures the input/output pin INT2.
    INT2_IO_CONF = 0x54,
    /// Map data ready interrupt to output pin INT1 and/or INT2
    INT1_INT2_MAP_DATA = 0x58,
    /// Enables the sensor self-test signal, occurring as a steady offset to the
    /// sensor output. Note that the self-test needs to be switched off actively
    /// by the user (details see 4.6.1).
    ACC_SELF_TEST = 0x6D,
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
