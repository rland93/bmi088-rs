/// BMI088 errors.
#[derive(Debug)]
pub enum Error<CommE> {
    /// Interface communication error
    Comm(CommE),
    /// Invalid input data
    InvalidInputData,
}

/// Sensor Power Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorPowerMode {
    /// Accelerometer Power Mode
    pub acc: AccPowerMode,
    /// Gyroscope Power Mode
    pub gyro: GyroPowerMode,
}

/// Accelerometer Power Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccPowerMode {
    /// Normal Mode
    Normal,
    /// Suspend Mode
    Suspend,
    /// Low Power Mode
    LowPower,
}

/// Gyroscope Power Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroPowerMode {
    /// Normal Mode
    Normal,
    /// Suspend Mode
    Suspend,
    /// Fast Start-up Mode
    FastStartUp,
}

/// Sensor Status
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Status {}
