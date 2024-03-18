#![no_main]
#![no_std]

use bmi088::Bmi088;
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal::{i2c::I2c1, prelude::*};

#[entry]
fn main() -> ! {
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();
    let _gpioc = dp.GPIOC.split(); // for interrupts, when those are implemented
    let mut delay = dp.TIM2.delay_ms(&clocks);

    info!("i2c");
    let i2c1_scl = gpiob.pb6.into_alternate_open_drain();
    let i2c1_sda = gpiob.pb7.into_alternate_open_drain();
    let i2c1 = I2c1::new(dp.I2C1, (i2c1_scl, i2c1_sda), 100.kHz(), &clocks);

    // initialize the sensor
    let mut sensor = Bmi088::new_with_i2c(i2c1, false, false);

    // Perform resets
    info!("Accelerometer soft reset");
    sensor.acc_soft_reset().unwrap();
    delay.delay_ms(5);

    info!("Gyro soft reset");
    sensor.gyro_soft_reset().unwrap();
    delay.delay_ms(50);

    loop {
        // TEST: successful read of chip ID
        // --------------------------------
        info!("Reading chip ID");
        let id = sensor.acc_chip_id().unwrap();
        debug!("Chip ID: {:x}", id);

        // TEST: successful write and read back of configuration
        // ----------------------------------------------------
        info!("Writing and reading back configuration");

        // Configuration
        let configuration = bmi088::AccelerometerConfig {
            conf: bmi088::AccConf {
                acc_bwp: bmi088::AccBandwidth::X1,
                acc_odr: bmi088::AccDataRate::Hz100,
            },
            acc_range: bmi088::AccRange::G3,
        };
        sensor.acc_configuration_write(configuration).unwrap();

        // Read back configuration
        let readback_conf = sensor.acc_configuration_read().unwrap();

        // Success
        assert!(configuration == readback_conf);

        // Test: successful write and read of "enable" register
        // ----------------------------------------------------
        info!("Writing and reading back enable register");

        let enable = bmi088::AccPowerEnable::On;
        sensor.acc_enable_write(enable).unwrap();
        let readback_enable = sensor.acc_enable_read().unwrap();
        assert!(enable == readback_enable);

        // Test: successful write and read of "power" register
        // ---------------------------------------------------
        info!("Writing and reading back power register");

        let power = bmi088::AccPowerConf::Active;
        sensor.acc_wake_suspend_write(power).unwrap();
        let readback_power = sensor.acc_wake_suspend_read().unwrap();
        assert!(power == readback_power);

        // Test: successful read of values
        // -------------------------------
        info!("Reading accelerometer data");

        let values = sensor.acc_data().unwrap();
        debug!(
            "Accelerometer data: [{:?} {:?} {:?}]",
            values.x, values.y, values.z
        );

        // Test: successful read of positively incrementing sensortime
        // -----------------------------------------------------------
        info!("Reading sensortime");

        let sensortime24_0 = sensor.sensor_time_24bit().unwrap();
        debug!("Sensortime: {:?}", sensortime24_0);
        let sensortime24_1 = sensor.sensor_time_24bit().unwrap();
        debug!("Sensortime: {:?}", sensortime24_1);
        assert!(sensortime24_1 > sensortime24_0);

        // Test: successful read of temperature
        // ------------------------------------
        info!("Reading temperature");

        let temp = sensor.temperature().unwrap();
        debug!("Temperature: {:?}", temp);

        // Test: successful read of error register
        // ---------------------------------------
        info!("Reading error register");

        let err = sensor.acc_err_reg().unwrap();
        debug!(
            "Error register: code={:?}, fatal={:?}",
            err.error_code, err.fatal_error
        );

        // Test: successful read of status register
        // ----------------------------------------
        info!("Reading status register");

        let status = sensor.acc_status().unwrap();
        debug!("Status register: status={:?}", status);

        info!("Chip ID");
        let id = sensor.gyro_chipid().unwrap();
        debug!("Chip ID: {:x}", id);

        info!("Bandwidth");
        let bw = bmi088::GyroBandwidth::Hz32;
        sensor.gyro_bandwidth_write(bw).unwrap();
        let readback_bw = sensor.gyro_bandwidth_read().unwrap();
        assert!(bw == readback_bw);

        info!("Range");
        let range = bmi088::GyroRange::Dps2000;
        sensor.gyro_range_write(range).unwrap();
        let readback_range = sensor.gyro_range_read().unwrap();
        assert!(range == readback_range);

        info!("Power mode");
        let power = bmi088::GyroPowerMode::Normal;
        sensor.gyro_power_mode_write(power).unwrap();
        let readback_power = sensor.gyro_power_mode_read().unwrap();
        assert!(power == readback_power);

        info!("Data ready interrupt mapping");
        let pin_active = bmi088::PinActive::ActiveHigh;
        let pin_behavior = bmi088::PinBehavior::PushPull;

        sensor
            .gyro_conf_int3_write(pin_active, pin_behavior)
            .unwrap();
        let (rb_active, rb_beh) = sensor.gyro_conf_int3_read().unwrap();
        assert!(pin_active == rb_active);
        assert!(pin_behavior == rb_beh);

        info!("Reading gyroscope data");
        let values = sensor.gyro_read_rate().unwrap();
        debug!(
            "Gyroscope data: [{:?} {:?} {:?}]",
            values.x, values.y, values.z
        );
    }
}
