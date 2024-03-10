#![no_main]
#![no_std]

use bmi088::Bmi088;
use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal::{i2c::I2c1, prelude::*};

use defmt::{debug, info};

#[entry]
fn main() -> ! {
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    info!("i2c");
    let i2c1_scl = gpiob.pb6.into_alternate_open_drain();
    let i2c1_sda = gpiob.pb7.into_alternate_open_drain();
    let mut i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 400.kHz(), &clocks);

    // initialize the sensor
    let mut sensor = Bmi088::new_with_i2c(i2c1);

    loop {
        let id = sensor.acc_chip_id().unwrap();
        debug!("Chip ID: {:x}", id);
        cortex_m::asm::delay(8_000_000);
    }
}
