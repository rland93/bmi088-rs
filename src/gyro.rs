use crate::{
    interface::{ReadData, WriteData},
    reg::Registers,
    Bmi088, Error,
};

impl<DI, CommE> Bmi088<DI>
where
    DI: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
{
    /// Get Accelerometer Chip ID
    pub fn gyro_chip_id(&mut self) -> Result<u8, Error<CommE>> {
        self.iface.read_register_gyro(Registers::GYRO_CHIP_ID as u8)
    }
}
