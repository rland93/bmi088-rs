#![no_main]
#![no_std]

use core::cell::RefCell;

use bmi088::Bmi088;
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use embedded_hal_bus::spi::RefCellDevice;
use panic_probe as _;
use stm32f4xx_hal::{prelude::*, spi, spi::Spi3};

#[entry]
fn main() -> ! {
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    let mut delay = dp.TIM2.delay_ms(&clocks);

    info!("spi");

    let spi = Spi3::new(
        dp.SPI3,
        (
            gpioc.pc10.into_alternate(),
            gpioc.pc11.into_alternate(),
            gpioc.pc12.into_alternate(),
        ),
        spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        },
        1.MHz(),
        &clocks,
    );

    let rc_spi = RefCell::new(spi);
    let gyro_cs = gpiob.pb5.into_push_pull_output();
    let acc_cs = gpiod.pd2.into_push_pull_output();

    let delay1 = dp.TIM10.delay_ms(&clocks);
    let delay2 = dp.TIM11.delay_ms(&clocks);

    let gyr_dev = RefCellDevice::new(&rc_spi, gyro_cs, delay1);
    let acc_dev = RefCellDevice::new(&rc_spi, acc_cs, delay2);

    // initialize the sensor
    let mut gyr_dev = Bmi088::new_with_spi(gyr_dev);
    let mut acc_dev = Bmi088::new_with_spi(acc_dev);

    // Perform resets
    info!("Accelerometer soft reset");
    gyr_dev.acc_soft_reset().unwrap();
    delay.delay_ms(5);

    info!("Gyro soft reset");
    gyr_dev.gyro_soft_reset().unwrap();
    delay.delay_ms(50);

    loop {
        // TEST: successful read of chip ID
        // --------------------------------
        info!("Reading acc chip ID");
        let id = acc_dev.acc_chipid().unwrap();
        debug!("Chip ID: {:02x}", id);

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
        acc_dev.acc_configuration_write(configuration).unwrap();

        // Read back configuration
        let readback_conf = acc_dev.acc_configuration_read().unwrap();

        // Success
        assert!(configuration == readback_conf);

        // Test: successful write and read of "enable" register
        // ----------------------------------------------------
        info!("Writing and reading back enable register");

        let enable = bmi088::AccPowerEnable::On;
        acc_dev.acc_enable_write(enable).unwrap();
        let readback_enable = acc_dev.acc_enable_read().unwrap();
        assert!(enable == readback_enable);

        // Test: successful write and read of "power" register
        // ---------------------------------------------------
        info!("Writing and reading back power register");

        let power = bmi088::AccPowerConf::Active;
        acc_dev.acc_wake_suspend_write(power).unwrap();
        let readback_power = acc_dev.acc_wake_suspend_read().unwrap();
        assert!(power == readback_power);

        // Test: successful read of values
        // -------------------------------
        info!("Reading accelerometer data");

        let values = acc_dev.acc_data().unwrap();
        debug!(
            "Accelerometer data: [{:?} {:?} {:?}]",
            values.x, values.y, values.z
        );

        // Test: successful read of positively incrementing sensortime
        // -----------------------------------------------------------
        info!("Reading sensortime");

        let sensortime24_0 = acc_dev.sensor_time_24bit().unwrap();
        debug!("Sensortime: {:?}", sensortime24_0);
        let sensortime24_1 = acc_dev.sensor_time_24bit().unwrap();
        debug!("Sensortime: {:?}", sensortime24_1);
        assert!(sensortime24_1 > sensortime24_0);

        // Test: successful read of temperature
        // ------------------------------------
        info!("Reading temperature");

        let temp = acc_dev.temperature().unwrap();
        debug!("Temperature: {:?}", temp);

        // Test: successful read of error register
        // ---------------------------------------
        info!("Reading error register");

        let err = acc_dev.acc_err_reg().unwrap();
        debug!(
            "Error register: code={:?}, fatal={:?}",
            err.error_code, err.fatal_error
        );

        // Test: successful read of status register
        // ----------------------------------------
        info!("Reading status register");

        let status = acc_dev.acc_status().unwrap();
        debug!("Status register: status={:?}", status);

        info!("Chip ID");
        let id = gyr_dev.gyro_chipid().unwrap();
        debug!("Chip ID: {:x}", id);

        info!("Bandwidth");
        let bw = bmi088::GyroBandwidth::Hz32;
        gyr_dev.gyro_bandwidth_write(bw).unwrap();
        let readback_bw = gyr_dev.gyro_bandwidth_read().unwrap();
        assert!(bw == readback_bw);

        info!("Range");
        let range = bmi088::GyroRange::Dps2000;
        gyr_dev.gyro_range_write(range).unwrap();
        let readback_range = gyr_dev.gyro_range_read().unwrap();
        assert!(range == readback_range);

        info!("Power mode");
        let power = bmi088::GyroPowerMode::Normal;
        gyr_dev.gyro_power_mode_write(power).unwrap();
        let readback_power = gyr_dev.gyro_power_mode_read().unwrap();
        assert!(power == readback_power);

        info!("Data ready interrupt mapping");
        let pin_active = bmi088::PinActive::ActiveHigh;
        let pin_behavior = bmi088::PinBehavior::PushPull;

        gyr_dev
            .gyro_conf_int3_write(pin_active, pin_behavior)
            .unwrap();
        let (rb_active, rb_beh) = gyr_dev.gyro_conf_int3_read().unwrap();
        assert!(pin_active == rb_active);
        assert!(pin_behavior == rb_beh);

        info!("Reading gyroscope data");
        let values = gyr_dev.gyro_read_rate().unwrap();
        debug!(
            "Gyroscope data: [{:?} {:?} {:?}]",
            values.x, values.y, values.z
        );
        delay.delay_ms(1000);
    }
}
