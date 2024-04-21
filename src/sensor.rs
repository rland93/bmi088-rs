use crate::{
    interface::{ReadData, WriteData},
    reg::{AccRegisters, GyroRegisters},
    types::{
        AccConf, AccDrdyMap, AccPowerConf, AccPowerEnable, AccRange, AccelerometerConfig, ErrCode,
        GyroBandwidth, GyroDrdyMap, GyroPowerMode, GyroRange, IntConfiguration, PinActive,
        PinBehavior, Sensor3DData,
    },
    Bmi088, Error,
};

impl<DI, CommE> Bmi088<DI>
where
    DI: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
{
    /// Accelerometer chip ID. (0x00)
    ///
    /// Reads the chip ID and returns the chip ID value.
    ///
    /// # Returns
    ///
    /// - `Ok(u8)`: The chip ID value.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_chipid(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_acc(AccRegisters::ACC_CHIP_ID as u8)
    }

    /// Accelerometer error register. (0x02)
    ///
    /// Reads the error register and returns the error code.
    ///
    /// # Returns
    ///
    /// - `Ok(ErrCode)`: The error code.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_err_reg(&mut self) -> Result<ErrCode, Error<CommE>> {
        let err = self
            .iface
            .read_register_acc(AccRegisters::ACC_ERR_REG as u8)?;
        Ok(ErrCode::from_u8(err))
    }

    /// Accelerometer status register. (0x03)
    ///
    /// Read drdy. True if data ready. Also clears.
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if data is ready, `false` otherwise.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_status(&mut self) -> Result<bool, Error<CommE>> {
        let drdy = self
            .iface
            .read_register_acc(AccRegisters::ACC_STATUS as u8)?;

        Ok(((drdy & 0b1000_0000) >> 7) != 0)
    }

    /// Accelerometer data. (0x12)
    ///
    /// Reads the accelerometer data registers and returns the 3D sensor data.
    ///
    /// # Returns
    ///
    /// - `Ok(Sensor3DData)`: The 3D sensor data.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_data(&mut self) -> Result<Sensor3DData, Error<CommE>> {
        let mut data = [0xFF; 6];
        let reg = AccRegisters::ACC_DATA as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        Ok(Sensor3DData {
            x: i16::from_be_bytes([data[1], data[0]]),
            y: i16::from_be_bytes([data[3], data[2]]),
            z: i16::from_be_bytes([data[5], data[4]]),
        })
    }

    /// Sensor time register. (24-bit, 0x18-0x1A)
    ///
    /// Sensor time, in 24-bit counts. Overflows after 2^23 - 1 counts.
    ///
    /// # Returns
    ///
    /// - `Ok(u32)`: The time value.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn sensor_time_24bit(&mut self) -> Result<u32, Error<CommE>> {
        let mut data = [0xFF; 4];
        let reg = AccRegisters::ACC_SENSORTIME as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let sensortime = [0x00, data[2], data[1], data[0]];
        Ok(u32::from_be_bytes(sensortime))
    }

    /// Converts sensor time (counts) to microseconds.
    ///
    /// # Arguments
    ///
    /// - `counts`: The sensor time in counts.
    ///
    /// # Returns
    ///
    /// - `u32`: The converted time in microseconds.
    ///
    pub fn sensor_time_counts_to_us(counts: u32) -> u32 {
        let time = counts as u64 * 655360000 / ((1 << 23) - 1) as u64;
        time as u32
    }

    /// Accelerometer drdy status. (0x1D)
    ///
    /// Read and return acc drdy. Also clear the interrupt status register
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if an interrupt is active
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_drdy(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(AccRegisters::ACC_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    /// Temperature register. (0x22)
    ///
    /// Temperature value in degrees Celsius.
    ///
    /// # Returns
    ///
    /// - `Ok(f32)`: The temperature value in degrees Celsius.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn temperature(&mut self) -> Result<f32, Error<CommE>> {
        let mut data = [0xFF, 0xFF];
        let reg = AccRegisters::TEMPERATURE as u8;
        self.iface.read_data_acc(reg, &mut data)?;

        let temp_uint11 = (data[0] as u16 * 8) + (data[1] as u16 / 32);
        let temp_int11 = if temp_uint11 > 1023 {
            temp_uint11 as i16 - 2048
        } else {
            temp_uint11 as i16
        };
        let temperature = temp_int11 as f32 * 0.125 + 23.0;
        Ok(temperature)
    }

    /// Writes ACC_CONF. (0x40)
    ///
    /// NOTE:Does not write the whole AccelerometerConfig, just the subset
    /// in the ACC_CONF register.
    ///
    /// # Arguments
    ///
    /// - `conf`: The accelerometer configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    fn acc_conf_write(&mut self, conf: AccConf) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::ACC_CONF as u8;
        let res: u8 = 0b1000_0000; // reserved
        let bwp: u8 = ((conf.acc_bwp as u8) << 4) & 0b0110_0000; // [6:4]
        let odr: u8 = conf.acc_odr as u8 & 0b0000_1111; // [3:0]
        let set = res | bwp | odr;

        let mut data = [reg, set];

        self.iface.write_data_acc(&mut data)?;

        Ok(())
    }

    /// Reads the ACC_CONF (0x40) register
    ///
    /// # Returns
    ///
    /// - `Ok(AccConf)`: Accelerometer conf value
    /// - `Err(Error<CommE>)`: Read Failure
    ///
    fn acc_conf_read(&mut self) -> Result<AccConf, Error<CommE>> {
        let mut data = [0xFF];
        let reg = AccRegisters::ACC_CONF as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let conf = AccConf::try_from(data[0]).map_err(|_| Error::InvalidInputData)?;
        Ok(conf)
    }

    /// Read ACC_RANGE (0x41)
    ///
    /// # Returns
    ///
    /// - `Ok(AccRange)`: Range value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    fn acc_range_read(&mut self) -> Result<AccRange, Error<CommE>> {
        let mut data = [0xFF];
        let reg = AccRegisters::ACC_RANGE as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let range = AccRange::try_from(data[0]).map_err(|_| Error::InvalidInputData)?;
        Ok(range)
    }

    /// Write ACC_RANGE. (0x41)
    ///
    /// # Arguments
    ///
    /// - `range`: The accelerometer range to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    fn acc_range_write(&mut self, range: AccRange) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::ACC_RANGE as u8;
        // [7:2] are reserved.
        let set = (range as u8) & 0b0000_0011; // [1:0]
        self.iface.write_register_acc(reg, set)?;

        Ok(())
    }

    /// Reads the accelerometer configuration. Utility method for both
    /// ACC_CONF and ACC_RANGE.
    ///
    /// # Returns
    ///
    /// - `Ok(AccelerometerConfig)`: Accelerometer configuration
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_configuration_read(&mut self) -> Result<AccelerometerConfig, Error<CommE>> {
        let conf = self.acc_conf_read()?;
        let range = self.acc_range_read()?;
        Ok(AccelerometerConfig {
            conf,
            acc_range: range,
        })
    }

    /// Writes the accelerometer configuration. Utility method for both
    /// ACC_CONF and ACC_RANGE.
    ///
    /// # Arguments
    ///
    /// - `config`: The accelerometer configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_configuration_write(
        &mut self,
        config: AccelerometerConfig,
    ) -> Result<(), Error<CommE>> {
        self.acc_conf_write(config.conf)?;
        self.acc_range_write(config.acc_range)?;
        Ok(())
    }

    /// Reads the ACC_PWR_CONF (0x1E) register to figure out if the sensor
    /// is suspended or active.
    ///
    /// # Returns
    ///
    /// - `Ok(AccPowerConf)`: Power conf value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_wake_suspend_read(&mut self) -> Result<AccPowerConf, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(AccRegisters::ACC_PWR_CONF as u8)?;
        AccPowerConf::try_from(reg).map_err(|_| Error::InvalidInputData)
    }

    /// Writes the ACC_PWR_CONF (0x7C) register to suspend or wake the
    /// sensor.
    ///
    /// # Arguments
    ///
    /// - `conf`: The power configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_wake_suspend_write(&mut self, conf: AccPowerConf) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::ACC_PWR_CONF as u8;
        let set = conf as u8;
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Reads the ACC_PWR_CTRL (0x7C) register to figure out if the sensor
    /// is enabled or disabled.
    ///
    /// # Returns
    ///
    /// - `Ok(AccPowerEnable)`: Power enable value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_enable_read(&mut self) -> Result<AccPowerEnable, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(AccRegisters::ACC_PWR_CTRL as u8)?;
        AccPowerEnable::try_from(reg).map_err(|_| Error::InvalidInputData)
    }

    /// Writes the ACC_PWR_CTRL (0x7D) register to enable or disable the sensor.
    /// Must be called after startup to enable the sensor.
    ///
    /// # Arguments
    ///
    /// - `enable`: Power enable
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_enable_write(&mut self, enable: AccPowerEnable) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::ACC_PWR_CTRL as u8;
        let set = enable as u8;
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Soft Reset (0x7E)
    ///
    /// Sensortec recommends a delay of 1ms after a soft reset of the sensor.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_soft_reset(&mut self) -> Result<(), Error<CommE>> {
        let reg = 0xB6;
        self.iface
            .write_register_acc(AccRegisters::ACC_SOFTRESET as u8, reg)?;
        Ok(())
    }

    /// Utility method for initializing the accelerometer after a power-on
    /// reset or startup.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn init(&mut self, configuration: AccelerometerConfig) -> Result<(), Error<CommE>> {
        self.acc_configuration_write(configuration)?;
        self.acc_enable_write(AccPowerEnable::On)?;
        self.acc_wake_suspend_write(AccPowerConf::Active)?;
        Ok(())
    }

    /// Accelerometer drdy status. (0x1D) Clears drdy interrupt
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if an interrupt is active
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_drdy1(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(AccRegisters::ACC_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    /// Configure INT1 pin.
    ///
    /// # Arguments
    ///
    /// - `conf`: The interrupt configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn int1_io_conf(&mut self, conf: IntConfiguration) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::INT1_IO_CONF as u8;
        let set = conf.into();
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Configure INT2 pin.
    ///
    /// # Arguments
    ///
    /// - `conf`: The interrupt configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn int2_io_conf(&mut self, conf: IntConfiguration) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::INT2_IO_CONF as u8;
        let set = conf.into();
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Map data ready interrupt to output pin INT1 and/or INT2. (0x58)
    ///
    /// # Arguments
    ///
    /// - `map`: The interrupt configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_map_drdy(&mut self, map: AccDrdyMap) -> Result<(), Error<CommE>> {
        let reg = AccRegisters::INT1_INT2_MAP_DATA as u8;
        let set = map as u8;
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /*     GYRO REGISTERS      */

    /// Read the gyro chip ID (0x00)
    ///
    /// # Returns
    ///
    /// - `Ok(u8)`: The chip ID value.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_chipid(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_gyro(GyroRegisters::GYRO_CHIP_ID as u8)
    }

    /// Read the gyro rate data (0x02-0x07)
    ///
    /// # Returns
    ///
    /// - `Ok(Sensor3DData)`: The 3D sensor data.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_read_rate(&mut self) -> Result<Sensor3DData, Error<CommE>> {
        let mut data = [0xFF; 6];
        let reg = GyroRegisters::GYRO_RATE_DATA as u8;
        self.iface.read_data_gyro(reg, &mut data)?;
        Ok(Sensor3DData {
            x: i16::from_be_bytes([data[1], data[0]]),
            y: i16::from_be_bytes([data[3], data[2]]),
            z: i16::from_be_bytes([data[5], data[4]]),
        })
    }

    /// Read gyro drdy status (0x0A)
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if an interrupt is active
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_drdy(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_gyro(GyroRegisters::GYRO_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    /// Read gyro range (0x0F)
    ///
    /// # Returns
    ///
    /// - `Ok(GyroRange)`: Range value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_range_read(&mut self) -> Result<GyroRange, Error<CommE>> {
        let reg = GyroRegisters::GYRO_RANGE as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let range = GyroRange::try_from(data).map_err(|_| Error::InvalidInputData)?;
        Ok(range)
    }

    /// Write gyro range (0x0F)
    ///
    /// # Arguments
    ///
    /// - `range`: The gyro range to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_range_write(&mut self, range: GyroRange) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_RANGE as u8;
        let set = range as u8;
        self.iface.write_register_gyro(reg, set)?;

        Ok(())
    }

    /// Read gyro bandwidth (0x10)
    ///
    /// # Returns
    ///
    /// - `Ok(GyroBandwidth)`: Bandwidth value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_bandwidth_read(&mut self) -> Result<GyroBandwidth, Error<CommE>> {
        let reg = GyroRegisters::GYRO_BANDWIDTH as u8;
        let data = self.iface.read_register_gyro(reg)?;
        // Note: bit #7 is read-only and always ‚1‘, but has no function and can safely be ignored.
        let data_masked = data & 0b0111_1111;
        let bw = GyroBandwidth::try_from(data_masked).map_err(|_| Error::InvalidInputData)?;
        Ok(bw)
    }

    /// Write gyro bandwidth (0x10)
    ///
    /// # Arguments
    ///
    /// - `bw`: The gyro bandwidth to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_bandwidth_write(&mut self, bw: GyroBandwidth) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_BANDWIDTH as u8;
        let set = bw as u8;
        self.iface.write_register_gyro(reg, set)?;

        Ok(())
    }

    /// Read gyro power mode (0x11)
    ///
    /// # Returns
    ///
    /// - `Ok(GyroPowerMode)`: Power mode value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_power_mode_read(&mut self) -> Result<GyroPowerMode, Error<CommE>> {
        let reg = GyroRegisters::GYRO_LPM1 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let mode = GyroPowerMode::try_from(data).map_err(|_| Error::InvalidInputData)?;
        Ok(mode)
    }

    /// Write gyro power mode (0x11)
    ///
    /// # Arguments
    ///
    /// - `mode`: The gyro power mode to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_power_mode_write(&mut self, mode: GyroPowerMode) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_LPM1 as u8;
        let set = mode as u8;
        self.iface.write_register_gyro(reg, set)?;

        Ok(())
    }

    /// Perform a soft reset of the gyroscope. (0x14)
    /// Sensortec recommends a delay of 30ms after resetting the gyro.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_soft_reset(&mut self) -> Result<(), Error<CommE>> {
        let reg = 0xB6;
        self.iface
            .write_register_gyro(GyroRegisters::GYRO_SOFTRESET as u8, reg)?;
        Ok(())
    }

    /// Enable data ready interrupt on the gyro. (0x15)
    ///
    /// # Arguments
    ///
    /// - `enable`: `true` to enable, `false` to disable
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_drdy_en(&mut self, enable: bool) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT_CTRL as u8;
        let set = if enable { 0x80 } else { 0x00 };
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    /// Read interrupt 4 configuration (0x16)
    ///
    /// # Returns
    ///
    /// - `Ok((PinActive, PinBehavior))`: The pin active and behavior
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_conf_int4_read(&mut self) -> Result<(PinActive, PinBehavior), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_CONF as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let active = if data & 0b0000_0100 != 0 {
            PinActive::ActiveHigh
        } else {
            PinActive::ActiveLow
        };
        let behavior = if data & 0b0000_1000 != 0 {
            PinBehavior::OpenDrain
        } else {
            PinBehavior::PushPull
        };
        Ok((active, behavior))
    }

    /// Write interrupt 4 configuration (0x16)
    ///
    /// # Arguments
    ///
    /// - `active`: The pin active state
    /// - `behavior`: The pin behavior
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_conf_int4_write(
        &mut self,
        active: PinActive,
        behavior: PinBehavior,
    ) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_CONF as u8;
        let set = match active {
            PinActive::ActiveHigh => 0b0000_0100,
            PinActive::ActiveLow => 0b0000_0000,
        } | match behavior {
            PinBehavior::PushPull => 0b0000_0000,
            PinBehavior::OpenDrain => 0b0000_1000,
        };
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    /// Read interrupt 3 configuration (0x16)
    /// This is the same register as interrupt 3.
    ///
    /// # Returns
    ///
    /// - `Ok((PinActive, PinBehavior))`: The pin active and behavior
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_conf_int3_read(&mut self) -> Result<(PinActive, PinBehavior), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_CONF as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let active = if data & 0b0000_0001 != 0 {
            PinActive::ActiveHigh
        } else {
            PinActive::ActiveLow
        };
        let behavior = if data & 0b0000_0010 != 0 {
            PinBehavior::OpenDrain
        } else {
            PinBehavior::PushPull
        };
        Ok((active, behavior))
    }

    /// Write interrupt 3 configuration (0x16)
    /// This is the same register as interrupt 3.
    ///
    /// # Arguments
    ///
    /// - `active`: The pin active state
    /// - `behavior`: The pin behavior
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_conf_int3_write(
        &mut self,
        active: PinActive,
        behavior: PinBehavior,
    ) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_CONF as u8;
        let set = match active {
            PinActive::ActiveHigh => 0b0000_0001,
            PinActive::ActiveLow => 0b0000_0000,
        } | match behavior {
            PinBehavior::PushPull => 0b0000_0000,
            PinBehavior::OpenDrain => 0b0000_0010,
        };
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    /// Read interrupt 3 and 4 IO map (0x18)
    ///
    /// # Returns
    ///
    /// - `Ok(GyroDrdyMap)`: The pin mapping
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_drdy_map_read(&mut self) -> Result<GyroDrdyMap, Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_MAP as u8;
        let data = self.iface.read_register_gyro(reg)?;
        GyroDrdyMap::try_from(data).map_err(|_| Error::InvalidInputData)
    }

    /// Write interrupt 3 and 4 IO map (0x18)
    ///
    /// # Arguments
    ///
    /// - `map`: The pin mapping
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Write success
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_drdy_map_write(&mut self, map: GyroDrdyMap) -> Result<(), Error<CommE>> {
        let reg = GyroRegisters::GYRO_INT3_INT4_IO_MAP as u8;
        let set = map as u8;
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }
}
