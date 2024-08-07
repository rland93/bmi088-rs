use crate::{
    interface::{ReadData, WriteData},
    reg, types, Bmi088, Error,
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
    pub fn acc_chip_id_read(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_acc(reg::AccRegisters::ACC_CHIP_ID as u8)
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
    pub fn acc_err_reg(&mut self) -> Result<types::ErrCode, Error<CommE>> {
        let err = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_ERR_REG as u8)?;
        Ok(types::ErrCode::from_u8(err))
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
            .read_register_acc(reg::AccRegisters::ACC_STATUS as u8)?;

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
    pub fn acc_data(&mut self) -> Result<types::Sensor3DData, Error<CommE>> {
        let mut data = [0xFF; 6];
        let reg = reg::AccRegisters::ACC_X_LSB as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        Ok(types::Sensor3DData {
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
    pub fn sensor_time(&mut self) -> Result<u32, Error<CommE>> {
        let mut data = [0xFF; 4];
        let reg = reg::AccRegisters::SENSORTIME_0 as u8;
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
    pub fn acc_int_stat_1_read(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_INT_STAT_1 as u8)?;
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
    pub fn temp(&mut self) -> Result<f32, Error<CommE>> {
        let mut data = [0xFF, 0xFF];
        let reg = reg::AccRegisters::TEMP_LSB as u8;
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
    fn acc_conf_write(&mut self, conf: types::AccConf) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_CONF as u8;
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
    fn acc_conf_read(&mut self) -> Result<types::AccConf, Error<CommE>> {
        let mut data = [0xFF];
        let reg = reg::AccRegisters::ACC_CONF as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let conf = types::AccConf::try_from(data[0]).map_err(|_| Error::InvalidInputData)?;
        Ok(conf)
    }

    /// Read ACC_RANGE (0x41)
    ///
    /// # Returns
    ///
    /// - `Ok(AccRange)`: Range value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    fn acc_range_read(&mut self) -> Result<types::AccRange, Error<CommE>> {
        let mut data = [0xFF];
        let reg = reg::AccRegisters::ACC_RANGE as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let range = types::AccRange::try_from(data[0]).map_err(|_| Error::InvalidInputData)?;
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
    fn acc_range_write(&mut self, range: types::AccRange) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_RANGE as u8;
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
    pub fn acc_configuration_read(&mut self) -> Result<types::AccelerometerConfig, Error<CommE>> {
        let conf = self.acc_conf_read()?;
        let range = self.acc_range_read()?;
        Ok(types::AccelerometerConfig {
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
        config: types::AccelerometerConfig,
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
    pub fn acc_pwr_conf_read(&mut self) -> Result<types::AccWakeSuspend, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_PWR_CONF as u8)?;
        types::AccWakeSuspend::try_from(reg).map_err(|_| Error::InvalidInputData)
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
    pub fn acc_pwr_conf_write(&mut self, conf: types::AccWakeSuspend) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_PWR_CONF as u8;
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
    pub fn acc_pwr_ctrl_read(&mut self) -> Result<types::AccOffOn, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_PWR_CTRL as u8)?;
        types::AccOffOn::try_from(reg).map_err(|_| Error::InvalidInputData)
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
    pub fn acc_pwr_ctrl_write(&mut self, enable: types::AccOffOn) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_PWR_CTRL as u8;
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
    pub fn acc_softreset(&mut self) -> Result<(), Error<CommE>> {
        let reg = 0xB6;
        self.iface
            .write_register_acc(reg::AccRegisters::ACC_SOFTRESET as u8, reg)?;
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
    pub fn acc_init(
        &mut self,
        configuration: types::AccelerometerConfig,
    ) -> Result<(), Error<CommE>> {
        self.acc_configuration_write(configuration)?;
        self.acc_pwr_ctrl_write(types::AccOffOn::On)?;
        self.acc_pwr_conf_write(types::AccWakeSuspend::Active)?;
        Ok(())
    }

    /// Accelerometer drdy status. (0x1D) Clears drdy interrupt
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if an interrupt is active
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_int_stat_1_read_drdy(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    /// Write INT1 pin configuration
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
    pub fn acc_int1_io_ctrl_write(
        &mut self,
        conf: types::IntConfiguration,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::INT1_IO_CTRL as u8;
        let set = conf.into();
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Read configuration of INT1 pin.
    ///
    /// # Returns
    ///
    /// - `Ok(IntConfiguration)`: Interrupt configuration
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_int1_io_ctrl_read(&mut self) -> Result<types::IntConfiguration, Error<CommE>> {
        let reg = reg::AccRegisters::INT1_IO_CTRL as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::IntConfiguration::from(data))
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
    pub fn acc_int2_io_ctrl_write(
        &mut self,
        conf: types::IntConfiguration,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::INT2_IO_CTRL as u8;
        let set = conf.into();
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Read configuration of INT2 pin.
    ///
    /// # Returns
    ///
    /// - `Ok(IntConfiguration)`: Interrupt configuration
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn acc_int2_io_ctrl_read(&mut self) -> Result<types::IntConfiguration, Error<CommE>> {
        let reg = reg::AccRegisters::INT2_IO_CTRL as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::IntConfiguration::from(data))
    }

    /// Write Map data ready interrupt to output pin INT1 and/or INT2. (0x58)
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
    pub fn acc_int_map_data_write(&mut self, map: types::AccDrdyMap) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::INT_MAP_DATA as u8;
        let set = map as u8;
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    /// Read Map data ready interrupt to output pin INT1 and/or INT2. (0x58)
    ///
    /// # Returns
    ///
    /// - `Ok(AccDrdyMap)`: The interrupt configuration
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn acc_int_map_data_read(&mut self) -> Result<types::AccDrdyMap, Error<CommE>> {
        let reg = reg::AccRegisters::INT_MAP_DATA as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::AccDrdyMap::from(data))
    }

    ///////////////////////////////////////
    /*          GYRO REGISTERS           */
    ///////////////////////////////////////

    /// Read FIFO configuration register. (0x49)
    ///
    /// Stream - sampling continues when buffer is full (i.e. filled with 99
    /// frames); old is discarded
    ///
    /// Fifo - data collection stops once buffer is full (i.e. filled with 100
    /// frames)
    ///
    /// # Returns
    ///
    /// - `Ok(FifoModeConf)`: The FIFO configuration
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_fifo_config1_read(&mut self) -> Result<types::GyroFifoModeConf, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_1 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::GyroFifoModeConf::from(data))
    }

    /// Write FIFO configuration register. (0x49)
    ///
    /// Stream - sampling continues when buffer is full (i.e. filled with 99
    /// frames); old is discarded
    ///
    /// Fifo - data collection stops once buffer is full (i.e. filled with 100
    /// frames)
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Written successfully
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_fifo_config1_write(
        &mut self,
        set: types::GyroFifoModeConf,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_1 as u8;
        self.iface.write_register_gyro(reg, set as u8)?;
        Ok(())
    }

    /// Read FIFO configuration register. (0x48)
    ///
    /// # Returns
    ///
    /// - `Ok(u8)`: The FIFO configuration
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_fifo_config0_read(&mut self) -> Result<u8, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_0 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(data)
    }

    /// Write FIFO configuration register. (0x48)
    ///
    /// # Arguments
    ///
    /// - `set`: The FIFO configuration to write. If the requested watermark
    ///  level is greater than 128, the value is clamped to the maximum value of
    ///  128.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Written successfully
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_fifo_config0_write(&mut self, watermark: u8) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_0 as u8;

        if watermark > 128 {
            self.iface.write_register_gyro(reg, 128u8)?;
        } else {
            self.iface.write_register_gyro(reg, watermark)?;
        }

        Ok(())
    }

    /// Read FIFO external interrupt register. (0x34)
    ///
    /// # Returns
    ///
    /// - `Ok(FifoExtIntS)`: The FIFO interrupt source configuration
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_fifo_ext_int_source_read(&mut self) -> Result<types::FifoExtIntS, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_EXT_INT_S as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::FifoExtIntS::from(data))
    }

    /// Write FIFO external interrupt register. (0x34)
    ///
    /// # Arguments
    ///
    /// - `set`: The FIFO interrupt source configuration to write.
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Written successfully
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_fifo_ext_int_source_write(
        &mut self,
        set: types::FifoExtIntS,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_EXT_INT_S as u8;
        self.iface.write_register_gyro(reg, set.into())?;
        Ok(())
    }

    /// Read FIFO watermark enable register. (0x1E)
    ///
    /// # Returns
    ///
    /// - `Ok(bool)`: `true` if watermark interrupt is enabled
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_fifo_wm_enable_read(&mut self) -> Result<bool, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_WM_EN as u8;
        let data = self.iface.read_register_gyro(reg)?;
        match data {
            0x08 => Ok(false),
            0x88 => Ok(true),
            _ => Err(Error::InvalidOutputData),
        }
    }

    /// Write FIFO watermark enable register. (0x1E)
    ///
    /// # Arguments
    ///
    /// - `enable`: `true` to enable watermark interrupt, `false` to disable
    ///
    /// # Returns
    ///
    /// - `Ok(())`: Written successfully
    /// - `Err(Error<CommE>)`: Write failure
    ///
    pub fn gyro_fifo_wm_enable_write(&mut self, enable: bool) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_WM_EN as u8;
        let set = if enable { 0x88 } else { 0x08 };
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    /// Read FIFO watermark level register. (0x0E)
    ///
    /// If an overrun has occurred, the overrun bit is set until a write to
    /// FIFO_CONFIG_1 clears it.
    ///
    /// # Returns
    ///
    /// - `Ok((bool, u8))`: Tuple of whether overrun has occurred and the
    ///   current watermark level.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_fifo_status_read(&mut self) -> Result<(bool, u8), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_STATUS as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let watermark = (data & 0b1000_0000) != 0;
        let count = data & 0b0111_1111;
        Ok((watermark, count))
    }

    /// Read the gyro chip ID (0x00)
    ///
    /// # Returns
    ///
    /// - `Ok(u8)`: The chip ID value.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_chip_id_read(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_gyro(reg::GyroRegisters::GYRO_CHIP_ID as u8)
    }

    /// Read the gyro rate data (0x02-0x07)
    ///
    /// # Returns
    ///
    /// - `Ok(Sensor3DData)`: The 3D sensor data.
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_rate_read(&mut self) -> Result<types::Sensor3DData, Error<CommE>> {
        let mut data = [0xFF; 6];
        let reg = reg::GyroRegisters::RATE_X_LSB as u8;
        self.iface.read_data_gyro(reg, &mut data)?;
        Ok(types::Sensor3DData {
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
            .read_register_gyro(reg::GyroRegisters::GYRO_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    /// Read gyro range (0x0F)
    ///
    /// # Returns
    ///
    /// - `Ok(GyroRange)`: Range value
    /// - `Err(Error<CommE>)`: Read failure
    ///
    pub fn gyro_range_read(&mut self) -> Result<types::GyroRange, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_RANGE as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let range = types::GyroRange::try_from(data).map_err(|_| Error::InvalidInputData)?;
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
    pub fn gyro_range_write(&mut self, range: types::GyroRange) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_RANGE as u8;
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
    pub fn gyro_bandwidth_read(&mut self) -> Result<types::GyroBandwidth, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_BANDWIDTH as u8;
        let data = self.iface.read_register_gyro(reg)?;
        // Note: bit #7 is read-only and always ‚1‘, but has no function and can safely be ignored.
        let data_masked = data & 0b0111_1111;
        let bw =
            types::GyroBandwidth::try_from(data_masked).map_err(|_| Error::InvalidInputData)?;
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
    pub fn gyro_bandwidth_write(&mut self, bw: types::GyroBandwidth) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_BANDWIDTH as u8;
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
    pub fn gyro_power_mode_read(&mut self) -> Result<types::GyroPowerMode, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_LPM1 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let mode = types::GyroPowerMode::try_from(data).map_err(|_| Error::InvalidInputData)?;
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
    pub fn gyro_power_mode_write(
        &mut self,
        mode: types::GyroPowerMode,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_LPM1 as u8;
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
            .write_register_gyro(reg::GyroRegisters::GYRO_SOFTRESET as u8, reg)?;
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
    pub fn gyro_int_ctrl_enable(&mut self, enable: bool) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_INT_CTRL as u8;
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
    pub fn gyro_int4_io_conf_read(
        &mut self,
    ) -> Result<(types::PinActive, types::PinBehavior), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        let data = self.iface.read_register_gyro(reg)?;

        let active = if data & 0b0000_0100 != 0 {
            types::PinActive::ActiveHigh
        } else {
            types::PinActive::ActiveLow
        };
        let behavior = if data & 0b0000_1000 != 0 {
            types::PinBehavior::OpenDrain
        } else {
            types::PinBehavior::PushPull
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
    pub fn gyro_int4_io_conf_write(
        &mut self,
        active: types::PinActive,
        behavior: types::PinBehavior,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        let set = match active {
            types::PinActive::ActiveHigh => 0b0000_0100,
            types::PinActive::ActiveLow => 0b0000_0000,
        } | match behavior {
            types::PinBehavior::PushPull => 0b0000_0000,
            types::PinBehavior::OpenDrain => 0b0000_1000,
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
    pub fn gyro_int3_io_conf_read(
        &mut self,
    ) -> Result<(types::PinActive, types::PinBehavior), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let active = if data & 0b0000_0001 != 0 {
            types::PinActive::ActiveHigh
        } else {
            types::PinActive::ActiveLow
        };
        let behavior = if data & 0b0000_0010 != 0 {
            types::PinBehavior::OpenDrain
        } else {
            types::PinBehavior::PushPull
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
    pub fn gyro_int3_io_conf_write(
        &mut self,
        active: types::PinActive,
        behavior: types::PinBehavior,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        let set = match active {
            types::PinActive::ActiveHigh => 0b0000_0001,
            types::PinActive::ActiveLow => 0b0000_0000,
        } | match behavior {
            types::PinBehavior::PushPull => 0b0000_0000,
            types::PinBehavior::OpenDrain => 0b0000_0010,
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
    pub fn gyro_int3_int4_io_map_read(&mut self) -> Result<types::GyroDrdyMap, Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_MAP as u8;
        let data = self.iface.read_register_gyro(reg)?;
        types::GyroDrdyMap::try_from(data).map_err(|_| Error::InvalidInputData)
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
    pub fn gyro_int3_int4_io_map_write(
        &mut self,
        map: types::GyroDrdyMap,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_MAP as u8;
        let set = map as u8;
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }
}
