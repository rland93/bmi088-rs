use crate::{
    builder,
    interface::{ReadData, WriteData},
    reg, types,
    types::Error,
    Bmi088,
};

impl<DI, CommE> Bmi088<DI>
where
    DI: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
{
    pub fn acc_fifo_clear(&mut self) -> Result<(), Error<CommE>> {
        self.iface
            .write_register_acc(reg::AccRegisters::ACC_SOFTRESET as u8, 0xB0)?;
        Ok(())
    }

    pub fn acc_chip_id_read(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_acc(reg::AccRegisters::ACC_CHIP_ID as u8)
    }

    pub fn acc_err_reg(&mut self) -> Result<types::acc::ErrCode, Error<CommE>> {
        let err = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_ERR_REG as u8)?;
        Ok(types::acc::ErrCode::from_u8(err))
    }

    pub fn acc_status_read(&mut self) -> Result<bool, Error<CommE>> {
        let reg = reg::AccRegisters::ACC_STATUS as u8;
        let status = self.iface.read_register_acc(reg)?;

        Ok((status & 0b1000_0000) != 0)
    }

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

    pub fn sensor_time(&mut self) -> Result<u32, Error<CommE>> {
        let mut data = [0xFF; 4];
        let reg = reg::AccRegisters::SENSORTIME_0 as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let sensortime = [0x00, data[2], data[1], data[0]];
        Ok(u32::from_be_bytes(sensortime))
    }

    pub fn sensor_time_counts_to_us(counts: u32) -> u32 {
        let time = counts as u64 * 655360000 / ((1 << 23) - 1) as u64;
        time as u32
    }

    pub fn acc_int_stat_1_read(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

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

    pub fn acc_conf_write(
        &mut self,
        bandwidth: types::acc::AccBandwidth,
        data_rate: types::acc::AccDataRate,
    ) -> Result<(), Error<CommE>> {
        let bw = bandwidth as u8;
        let odr = data_rate as u8;
        self.iface
            .write_register_acc(reg::AccRegisters::ACC_CONF as u8, bw | odr)?;

        Ok(())
    }

    pub fn acc_conf_read(
        &mut self,
    ) -> Result<(types::acc::AccBandwidth, types::acc::AccDataRate), Error<CommE>> {
        let conf = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_CONF as u8)?;

        let bw = types::acc::AccBandwidth::try_from(conf).map_err(|_e| Error::InvalidOutputData)?;
        let odr = types::acc::AccDataRate::try_from(conf).map_err(|_e| Error::InvalidOutputData)?;
        Ok((bw, odr))
    }

    pub fn acc_range_read(&mut self) -> Result<types::acc::AccRange, Error<CommE>> {
        let mut data = [0xFF];
        let reg = reg::AccRegisters::ACC_RANGE as u8;
        self.iface.read_data_acc(reg, &mut data)?;
        let range = types::acc::AccRange::try_from(data[0]).map_err(|_| Error::InvalidInputData)?;
        Ok(range)
    }

    pub fn acc_range_write(&mut self, range: types::acc::AccRange) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_RANGE as u8;
        // [7:2] are reserved.
        let set = (range as u8) & 0b0000_0011; // [1:0]
        self.iface.write_register_acc(reg, set)?;

        Ok(())
    }

    pub fn acc_pwr_conf_read(&mut self) -> Result<types::acc::AccPowerConf, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_PWR_CONF as u8)?;
        types::acc::AccPowerConf::try_from(reg).map_err(|_| Error::InvalidInputData)
    }

    pub fn acc_pwr_conf_write(
        &mut self,
        conf: types::acc::AccPowerConf,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_PWR_CONF as u8;
        let set = conf as u8;
        self.iface.write_register_acc(reg, set)
    }

    pub fn acc_pwr_ctrl_read(&mut self) -> Result<types::acc::AccPowerCtrl, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_PWR_CTRL as u8)?;
        types::acc::AccPowerCtrl::try_from(reg).map_err(|_| Error::InvalidInputData)
    }

    pub fn acc_pwr_ctrl_write(
        &mut self,
        enable: types::acc::AccPowerCtrl,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::ACC_PWR_CTRL as u8;
        let set = enable as u8;
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    pub fn acc_softreset(
        &mut self,
        delay: &mut dyn embedded_hal::delay::DelayNs,
    ) -> Result<(), Error<CommE>> {
        let reg = 0xB6;
        self.iface
            .write_register_acc(reg::AccRegisters::ACC_SOFTRESET as u8, reg)?;
        delay.delay_ms(1);
        // first byte after reset is 0xFF so read that to flush.
        self.acc_chip_id_read()?;
        Ok(())
    }

    pub fn acc_int_stat_1_read_drdy(&mut self) -> Result<bool, Error<CommE>> {
        let reg = self
            .iface
            .read_register_acc(reg::AccRegisters::ACC_INT_STAT_1 as u8)?;
        Ok((reg & 0b1000_0000) != 0)
    }

    pub fn acc_int1_io_ctrl_write(
        &mut self,
        conf: types::acc::AccIntConfiguration,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::INT1_IO_CTRL as u8;
        let set = conf.into();
        let mut data = [reg, set];
        self.iface.write_data_acc(&mut data)?;
        Ok(())
    }

    pub fn acc_int1_io_ctrl_read(
        &mut self,
    ) -> Result<types::acc::AccIntConfiguration, Error<CommE>> {
        let reg = reg::AccRegisters::INT1_IO_CTRL as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(data.into())
    }

    pub fn acc_int2_io_ctrl_write(
        &mut self,
        conf: types::acc::AccIntConfiguration,
    ) -> Result<(), Error<CommE>> {
        self.iface
            .write_register_acc(reg::AccRegisters::INT2_IO_CTRL as u8, conf.into())?;
        Ok(())
    }

    pub fn acc_int2_io_ctrl_read(
        &mut self,
    ) -> Result<types::acc::AccIntConfiguration, Error<CommE>> {
        let reg = reg::AccRegisters::INT2_IO_CTRL as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(data.into())
    }

    pub fn acc_int1_int2_map_data_write(
        &mut self,
        map: types::acc::AccIntMap,
    ) -> Result<(), Error<CommE>> {
        self.iface
            .write_register_acc(reg::AccRegisters::INT_MAP_DATA as u8, map.into())?;
        Ok(())
    }

    pub fn acc_int1_int2_map_data_read(&mut self) -> Result<types::acc::AccIntMap, Error<CommE>> {
        let reg = reg::AccRegisters::INT_MAP_DATA as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::acc::AccIntMap::from(data))
    }

    pub fn acc_fifo_length_read(&mut self) -> Result<u16, Error<CommE>> {
        let l0 = reg::AccRegisters::FIFO_LENGTH_0 as u8;
        let l1 = reg::AccRegisters::FIFO_LENGTH_1 as u8;
        let l0_read = self.iface.read_register_acc(l0)?;
        let l1_read = self.iface.read_register_acc(l1)?;

        let fifo_length = ((l1_read as u16) << 8) | l0_read as u16;
        Ok(fifo_length)
    }

    pub fn acc_fifo_downs_write(&mut self, downs: u8) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_DOWNS as u8;
        let downs: u8 = ((downs & 0b111) << 4) | 0b1000_0000;
        self.iface.write_register_acc(reg, downs)
    }

    pub fn acc_fifo_downs_read(&mut self) -> Result<u8, Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_DOWNS as u8;
        let downs = self.iface.read_register_acc(reg)?;
        Ok((downs & 0b0111_0000) >> 4)
    }

    pub fn acc_fifo_wtm_write(&mut self, watermark: u16) -> Result<(), Error<CommE>> {
        // buffer[0] = (uint8_t) wml & 0xff;
        // buffer[1] = (uint8_t) (wml >> 8) & 0xff;

        let mut payload = [
            reg::AccRegisters::FIFO_WTM_0 as u8,
            //0xFF, // dummy
            watermark as u8,
            (watermark >> 8) as u8,
        ];

        self.iface.write_data_acc(&mut payload)?;

        // let watermark_lsb = watermark as u8;
        // let watermark_msb = (watermark >> 8) as u8;
        // let wtm0 = reg::AccRegisters::FIFO_WTM_0 as u8;
        // let wtm1 = reg::AccRegisters::FIFO_WTM_1 as u8;
        // self.iface.write_register_acc(wtm0, watermark_lsb)?;
        // self.iface.write_register_acc(wtm1, watermark_msb)?;
        Ok(())
    }

    pub fn acc_fifo_wtm_read(&mut self) -> Result<u16, Error<CommE>> {
        let wtm0 = reg::AccRegisters::FIFO_WTM_0 as u8;
        let wtm1 = reg::AccRegisters::FIFO_WTM_1 as u8;
        let wtm0_read = self.iface.read_register_acc(wtm0)?;
        let wtm1_read = self.iface.read_register_acc(wtm1)?;
        let watermark = ((wtm1_read as u16) << 8) | wtm0_read as u16;
        Ok(watermark)
    }

    pub fn acc_fifo_config1_read(&mut self) -> Result<types::acc::AccFifoConfig1, Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_CONFIG_1 as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::acc::AccFifoConfig1::from(data))
    }

    pub fn acc_fifo_config1_write(
        &mut self,
        set: types::acc::AccFifoConfig1,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_CONFIG_1 as u8;
        self.iface.write_register_acc(reg, u8::from(set))?;
        Ok(())
    }

    pub fn acc_fifo_mode_read(&mut self) -> Result<types::acc::AccFifoMode, Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_CONFIG_0 as u8;
        let data = self.iface.read_register_acc(reg)?;
        Ok(types::acc::AccFifoMode::from(data))
    }

    pub fn acc_fifo_mode_write(&mut self, watermark: u8) -> Result<(), Error<CommE>> {
        let reg = reg::AccRegisters::FIFO_CONFIG_0 as u8;

        if watermark > 128 {
            self.iface.write_register_acc(reg, 128u8)?;
        } else {
            self.iface.write_register_acc(reg, watermark)?;
        }

        Ok(())
    }

    /// blocking fifo data read. read out the entire buffer at &buf; incomplete
    /// frames are dropped. The FIFO queue is drained in proportion to the
    /// amount of data read.
    pub fn acc_fifo_data(&mut self, buf: &mut [u8]) -> Result<(), Error<CommE>> {
        self.iface
            .read_data_acc(reg::AccRegisters::FIFO_DATA as u8, buf)?;
        Ok(())
    }

    ///////////////////////////////////////
    /*          GYRO REGISTERS           */
    ///////////////////////////////////////
    ///

    pub fn gyro_fifo_data(&mut self, data: &mut [u8]) -> Result<(), Error<CommE>> {
        self.iface
            .read_data_gyro(reg::GyroRegisters::FIFO_DATA as u8, data)?;
        Ok(())
    }

    pub fn gyro_fifo_config1_read(
        &mut self,
    ) -> Result<types::gyro::GyroFifoModeConf, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_1 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::gyro::GyroFifoModeConf::from(data))
    }

    pub fn gyro_fifo_config1_write(
        &mut self,
        set: types::gyro::GyroFifoModeConf,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_1 as u8;
        self.iface.write_register_gyro(reg, set as u8)?;
        Ok(())
    }

    pub fn gyro_fifo_config0_read(&mut self) -> Result<u8, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_0 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(data)
    }

    pub fn gyro_fifo_config0_write(&mut self, watermark: u8) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_CONFIG_0 as u8;

        if watermark > 128 {
            self.iface.write_register_gyro(reg, 128u8)?;
        } else {
            self.iface.write_register_gyro(reg, watermark)?;
        }

        Ok(())
    }

    pub fn gyro_fifo_ext_int_s_read(&mut self) -> Result<types::gyro::GyroExtIntS, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_EXT_INT_S as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::gyro::GyroExtIntS::from(data))
    }

    pub fn gyro_fifo_ext_int_s_write(
        &mut self,
        set: types::gyro::GyroExtIntS,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_EXT_INT_S as u8;
        self.iface.write_register_gyro(reg, set.into())?;
        Ok(())
    }

    pub fn gyro_fifo_wm_enable_read(&mut self) -> Result<bool, Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_WM_EN as u8;
        let data = self.iface.read_register_gyro(reg)?;
        match data {
            0x08 => Ok(false),
            0x88 => Ok(true),
            _ => Err(Error::InvalidOutputData),
        }
    }

    pub fn gyro_fifo_wm_enable_write(&mut self, enable: bool) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_WM_EN as u8;
        let set = if enable { 0x88 } else { 0x08 };
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    pub fn gyro_fifo_status_read(&mut self) -> Result<(bool, u8), Error<CommE>> {
        let reg = reg::GyroRegisters::FIFO_STATUS as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let overrun = (data & 0b1000_0000) != 0;
        let count = data & 0b0111_1111;
        Ok((overrun, count))
    }

    pub fn gyro_chip_id_read(&mut self) -> Result<u8, Error<CommE>> {
        self.iface
            .read_register_gyro(reg::GyroRegisters::GYRO_CHIP_ID as u8)
    }

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

    /// returns a bool pair (drdy, fifo_int)
    pub fn gyro_drdy(&mut self) -> Result<(bool, bool), Error<CommE>> {
        let reg = self
            .iface
            .read_register_gyro(reg::GyroRegisters::GYRO_INT_STAT_1 as u8)?;
        let drdy = (reg & 0b1000_0000) != 0;
        let fifo_int = (reg & 0b0001_0000) != 0;
        Ok((drdy, fifo_int))
    }

    pub fn gyro_range_read(&mut self) -> Result<types::gyro::GyroRange, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_RANGE as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let range = types::gyro::GyroRange::try_from(data).map_err(|_| Error::InvalidInputData)?;
        Ok(range)
    }

    pub fn gyro_range_write(&mut self, range: types::gyro::GyroRange) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_RANGE as u8;
        let set = range as u8;
        self.iface.write_register_gyro(reg, set)?;

        Ok(())
    }

    pub fn gyro_bandwidth_read(&mut self) -> Result<types::gyro::GyroBandwidth, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_BANDWIDTH as u8;
        let data = self.iface.read_register_gyro(reg)?;
        // Note: bit #7 is read-only and always ‚1‘, but has no function and can safely be ignored.
        let data_masked = data & 0b0111_1111;
        let bw = types::gyro::GyroBandwidth::try_from(data_masked)
            .map_err(|_| Error::InvalidInputData)?;
        Ok(bw)
    }

    pub fn gyro_bandwidth_write(
        &mut self,
        bw: types::gyro::GyroBandwidth,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_BANDWIDTH as u8;
        let set = bw as u8;
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    pub fn gyro_lpm_read(&mut self) -> Result<types::gyro::GyroPowerConf, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_LPM1 as u8;
        let data = self.iface.read_register_gyro(reg)?;
        let mode =
            types::gyro::GyroPowerConf::try_from(data).map_err(|_| Error::InvalidInputData)?;
        Ok(mode)
    }

    pub fn gyro_lpm_write(&mut self, mode: types::gyro::GyroPowerConf) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_LPM1 as u8;
        let set = mode as u8;
        self.iface.write_register_gyro(reg, set)?;
        Ok(())
    }

    pub fn gyro_softreset(
        &mut self,
        delay: &mut dyn embedded_hal::delay::DelayNs,
    ) -> Result<(), Error<CommE>> {
        let reg = 0xB6;
        self.iface
            .write_register_gyro(reg::GyroRegisters::GYRO_SOFTRESET as u8, reg)?;
        delay.delay_ms(30);
        // first byte read after reset seems to be 0xFF so read that to flush.
        self.acc_chip_id_read()?;
        Ok(())
    }

    pub fn gyro_int_ctrl_enable_write(
        &mut self,
        enable: types::gyro::GyroIntEnable,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_INT_CTRL as u8;
        self.iface.write_register_gyro(reg, u8::from(enable))?;
        Ok(())
    }

    pub fn gyro_int_ctrl_enable_read(
        &mut self,
    ) -> Result<types::gyro::GyroIntEnable, Error<CommE>> {
        let reg = reg::GyroRegisters::GYRO_INT_CTRL as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::gyro::GyroIntEnable::from(data))
    }

    pub fn gyro_int3_int4_io_conf_read(
        &mut self,
    ) -> Result<types::gyro::GyroIntPinConfiguration, Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::gyro::GyroIntPinConfiguration::from(data))
    }

    pub fn gyro_int3_int4_io_conf_write(
        &mut self,
        conf: types::gyro::GyroIntPinConfiguration,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_CONF as u8;
        self.iface.write_register_gyro(reg, conf.into())?;
        Ok(())
    }

    pub fn gyro_int3_int4_io_map_read(
        &mut self,
    ) -> Result<types::gyro::GyroIntMapping, Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_MAP as u8;
        let data = self.iface.read_register_gyro(reg)?;
        Ok(types::gyro::GyroIntMapping::from(data))
    }

    pub fn gyro_int3_int4_io_map_write(
        &mut self,
        map: types::gyro::GyroIntMapping,
    ) -> Result<(), Error<CommE>> {
        let reg = reg::GyroRegisters::INT3_INT4_IO_MAP as u8;
        self.iface.write_register_gyro(reg, map.into())?;
        Ok(())
    }

    ///////////////////////////////////////
    /*          Utility Methods          */
    ///////////////////////////////////////

    /// Write the accelerometer configuration. This method combines every
    pub fn configure_accelerometer(
        &mut self,
        config: builder::AccConfiguration,
    ) -> Result<(), Error<CommE>> {
        if let Some(acc_int1) = config.int1 {
            self.acc_int1_io_ctrl_write(acc_int1)?;
        }

        if let Some(acc_int2) = config.int2 {
            self.acc_int2_io_ctrl_write(acc_int2)?;
        }

        if let Some(acc_int_map) = config.int_map {
            self.acc_int1_int2_map_data_write(acc_int_map)?;
        }

        if let Some((bw, odr)) = config.bandwidth {
            self.acc_conf_write(bw, odr)?;
        }

        if let Some(acc_range) = config.range {
            self.acc_range_write(acc_range)?;
        }

        if let Some(acc_power_conf) = config.power_conf {
            self.acc_pwr_conf_write(acc_power_conf)?;
        }

        if let Some(acc_power_ctrl) = config.power_ctrl {
            self.acc_pwr_ctrl_write(acc_power_ctrl)?;
        }

        if let Some(fifo_mode) = config.fifo_mode {
            self.acc_fifo_mode_write(fifo_mode as u8)?;
        }

        if let Some(fifo_conf1) = config.fifo_conf1 {
            self.acc_fifo_config1_write(fifo_conf1)?;
        }

        if let Some(fifo_downs) = config.fifo_downs {
            self.acc_fifo_downs_write(fifo_downs)?;
        }

        if let Some(fifo_wtm) = config.fifo_wtm {
            self.acc_fifo_wtm_write(fifo_wtm)?;
        }

        Ok(())
    }

    pub fn configure_gyro(
        &mut self,
        config: builder::GyroConfiguration,
    ) -> Result<(), Error<CommE>> {
        if let Some(interrupt) = config.interrupt {
            self.gyro_int_ctrl_enable_write(interrupt.enable)?;
            self.gyro_int3_int4_io_map_write(interrupt.map)?;
            self.gyro_int3_int4_io_conf_write(interrupt.conf)?;
        }

        if let Some(bandwidth) = config.bandwidth {
            self.gyro_bandwidth_write(bandwidth)?;
        }

        if let Some(range) = config.range {
            self.gyro_range_write(range)?;
        }

        if let Some(conf) = config.power_conf {
            self.gyro_lpm_write(conf)?;
        }

        if let Some(fifo_mode) = config.fifo_mode {
            self.gyro_fifo_config1_write(fifo_mode)?;
        }

        if let Some(ext_s) = config.ext_s {
            self.gyro_fifo_ext_int_s_write(ext_s)?;
        }

        if let Some(fifo_wtm) = config.fifo_wtm {
            self.gyro_fifo_config0_write(fifo_wtm & 0x7F)?;
        }

        Ok(())
    }
}
