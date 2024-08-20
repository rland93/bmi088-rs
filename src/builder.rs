use super::types;

#[derive(Default)]
pub struct AccConfiguration {
    pub int1: Option<types::acc::AccIntConfiguration>,
    pub int2: Option<types::acc::AccIntConfiguration>,
    pub bandwidth: Option<(types::acc::AccBandwidth, types::acc::AccDataRate)>,
    pub range: Option<types::acc::AccRange>,
    pub int_map: Option<types::acc::AccIntMap>,
    pub power_conf: Option<types::acc::AccPowerConf>,
    pub power_ctrl: Option<types::acc::AccPowerCtrl>,
    pub fifo_mode: Option<types::acc::AccFifoMode>,
    pub fifo_conf1: Option<types::acc::AccFifoConfig1>,
    pub fifo_downs: Option<u8>,
    pub fifo_wtm: Option<u16>,
}

pub struct AccConfigurationBuilder {
    config: AccConfiguration,
}

impl Default for AccConfigurationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl AccConfigurationBuilder {
    pub fn new() -> Self {
        Self {
            config: AccConfiguration::default(),
        }
    }

    pub fn acc_int1(mut self, value: types::acc::AccIntConfiguration) -> Self {
        self.config.int1 = Some(value);
        self
    }

    pub fn acc_int2(mut self, value: types::acc::AccIntConfiguration) -> Self {
        self.config.int2 = Some(value);
        self
    }

    pub fn acc_bandwidth(
        mut self,
        value: (types::acc::AccBandwidth, types::acc::AccDataRate),
    ) -> Self {
        self.config.bandwidth = Some(value);
        self
    }

    pub fn acc_range(mut self, value: types::acc::AccRange) -> Self {
        self.config.range = Some(value);
        self
    }

    pub fn acc_int_map(mut self, value: types::acc::AccIntMap) -> Self {
        self.config.int_map = Some(value);
        self
    }

    pub fn acc_power_conf(mut self, value: types::acc::AccPowerConf) -> Self {
        self.config.power_conf = Some(value);
        self
    }

    pub fn acc_power_ctrl(mut self, value: types::acc::AccPowerCtrl) -> Self {
        self.config.power_ctrl = Some(value);
        self
    }

    pub fn acc_fifo_mode(mut self, value: types::acc::AccFifoMode) -> Self {
        self.config.fifo_mode = Some(value);
        self
    }

    pub fn acc_fifo_conf1(mut self, value: types::acc::AccFifoConfig1) -> Self {
        self.config.fifo_conf1 = Some(value);
        self
    }

    pub fn acc_fifo_downs(mut self, value: u8) -> Self {
        self.config.fifo_downs = Some(value);
        self
    }

    pub fn acc_fifo_wtm(mut self, value: u16) -> Self {
        self.config.fifo_wtm = Some(value);
        self
    }

    pub fn build(self) -> AccConfiguration {
        self.config
    }
}

impl AccConfiguration {
    pub fn builder() -> AccConfigurationBuilder {
        AccConfigurationBuilder::new()
    }
}

#[derive(Default)]
pub struct GyroConfiguration {
    pub interrupt: Option<types::gyro::GyroIntConfiguration>,
    pub bandwidth: Option<types::gyro::GyroBandwidth>,
    pub range: Option<types::gyro::GyroRange>,
    pub power_conf: Option<types::gyro::GyroPowerConf>,
    pub fifo_mode: Option<types::gyro::GyroFifoModeConf>,
    pub ext_s: Option<types::gyro::GyroExtIntS>,
    pub fifo_wtm: Option<u8>,
}

impl GyroConfiguration {
    pub fn builder() -> GyroConfigurationBuilder {
        GyroConfigurationBuilder::new()
    }
}

pub struct GyroConfigurationBuilder {
    config: GyroConfiguration,
}

impl Default for GyroConfigurationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl GyroConfigurationBuilder {
    pub fn default() -> Self {
        Self::new()
    }

    pub fn new() -> Self {
        Self {
            config: GyroConfiguration::default(),
        }
    }

    pub fn interrupt(mut self, value: types::gyro::GyroIntConfiguration) -> Self {
        self.config.interrupt = Some(value);
        self
    }

    pub fn bandwidth(mut self, value: types::gyro::GyroBandwidth) -> Self {
        self.config.bandwidth = Some(value);
        self
    }

    pub fn range(mut self, value: types::gyro::GyroRange) -> Self {
        self.config.range = Some(value);
        self
    }

    pub fn power_conf(mut self, value: types::gyro::GyroPowerConf) -> Self {
        self.config.power_conf = Some(value);
        self
    }

    pub fn fifo_mode(mut self, value: types::gyro::GyroFifoModeConf) -> Self {
        self.config.fifo_mode = Some(value);
        self
    }

    pub fn ext_s(mut self, value: types::gyro::GyroExtIntS) -> Self {
        self.config.ext_s = Some(value);
        self
    }

    pub fn fifo_wtm(mut self, value: u8) -> Self {
        self.config.fifo_wtm = Some(value);
        self
    }

    pub fn build(self) -> GyroConfiguration {
        self.config
    }
}
