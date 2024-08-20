#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
/// In this example, we configure the BMI088 IMU to trigger an interrupt on the
/// DRDY pin when a new reading appears on the sensor. The interrupt is mapped
/// to the EXTI0 line on the STM32F401. The interrupt is configured to trigger
/// on a falling edge.
///
use defmt_rtt as _;
use embedded_hal_bus as ebus;
use panic_probe as _;
use stm32f4xx_hal::{gpio, pac, prelude::*, spi};

type GyroDev = ebus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'B', 5, gpio::Output>,
    ebus::spi::NoDelay,
>;
type AccelDev = ebus::spi::AtomicDevice<
    'static,
    spi::Spi<pac::SPI3>,
    gpio::Pin<'D', 2, gpio::Output>,
    ebus::spi::NoDelay,
>;
type Spi3Bus = ebus::util::AtomicCell<spi::Spi<pac::SPI3>>;
static mut SPI3BUS: Option<Spi3Bus> = None;

use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000);

use bmi088::interface;
use bmi088::Bmi088;
use rtic_monotonics::Monotonic;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        dev_accel: Bmi088<interface::SpiInterface<AccelDev>>,
        dev_gyro: Bmi088<interface::SpiInterface<GyroDev>>,
    }

    #[local]
    struct Local {
        imu_drdy: gpio::Pin<'C', 0, gpio::Input>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut dp = cx.device;
        let rcc = dp.RCC.constrain();
        let hse = 12.MHz();
        let sysclk = 64.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        Mono::start(cx.core.SYST, sysclk.to_Hz());

        let mut syscfg = dp.SYSCFG.constrain();

        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();

        defmt::info!("spi");
        let dev_spi: spi::Spi3 = spi::Spi3::new(
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
            8.MHz(),
            &clocks,
        );
        let mut acc_cs = gpiod.pd2.into_push_pull_output();
        let mut gyr_cs = gpiob.pb5.into_push_pull_output();
        acc_cs.set_high();
        gyr_cs.set_high();

        let bus = unsafe {
            SPI3BUS = Some(ebus::util::AtomicCell::new(dev_spi));
            SPI3BUS.as_ref().unwrap()
        };

        let mut dev_accel =
            Bmi088::new_with_spi(ebus::spi::AtomicDevice::new_no_delay(&bus, acc_cs).unwrap());
        let mut dev_gyro =
            Bmi088::new_with_spi(ebus::spi::AtomicDevice::new_no_delay(&bus, gyr_cs).unwrap());

        defmt::info!(
            "accel: {:x}, gyro: {:x}",
            dev_accel.acc_chip_id_read().unwrap(),
            dev_gyro.gyro_chip_id_read().unwrap()
        );

        // configure
        let acc_config = bmi088::AccConfiguration::builder()
            .acc_power_conf(bmi088::AccPowerConf::Active)
            .acc_power_ctrl(bmi088::AccPowerCtrl::On)
            .acc_bandwidth((bmi088::AccBandwidth::X4, bmi088::AccDataRate::Hz50))
            .acc_range(bmi088::AccRange::G3)
            .acc_int2(bmi088::AccIntConfiguration {
                int_pin: bmi088::IntPin::Output,
                int_od: bmi088::PinBehavior::PushPull,
                int_lvl: bmi088::PinActive::ActiveLow,
            })
            .acc_int_map(bmi088::AccIntMap {
                int1: bmi088::types::acc::IntMap::None,
                int2: bmi088::types::acc::IntMap::Drdy,
            })
            .build();

        let gyro_config = bmi088::GyroConfiguration::builder()
            .bandwidth(bmi088::types::gyro::GyroBandwidth::Hz64)
            .power_conf(bmi088::types::gyro::GyroPowerConf::Normal)
            .range(bmi088::types::gyro::GyroRange::Dps250)
            .build();

        dev_accel.configure_accelerometer(acc_config).unwrap();

        // print configuration values
        let mode = dev_accel.acc_conf_read().unwrap();
        defmt::debug!("acc mode CONF: {:?}", defmt::Debug2Format(&mode));
        let fifoconfig1 = dev_accel.acc_fifo_config1_read().unwrap();
        defmt::debug!("acc config1 CONF: {:?}", defmt::Debug2Format(&fifoconfig1));
        let downs = dev_accel.acc_fifo_downs_read().unwrap();
        defmt::debug!("acc downs CONF: {:?}", defmt::Debug2Format(&downs));
        let fifomode = dev_accel.acc_fifo_mode_read().unwrap();
        defmt::debug!("acc config0 CONF: {:?}", defmt::Debug2Format(&fifomode));
        let wtm = dev_accel.acc_fifo_wtm_read().unwrap();
        defmt::debug!("acc wtm CONF: {:?}", defmt::Debug2Format(&wtm));
        let int1 = dev_accel.acc_int1_io_ctrl_read().unwrap();
        defmt::debug!("acc int1 CONF: {:?}", defmt::Debug2Format(&int1));
        let int2 = dev_accel.acc_int2_io_ctrl_read().unwrap();
        defmt::debug!("acc int2 CONF: {:?}", defmt::Debug2Format(&int2));
        let int_map = dev_accel.acc_int1_int2_map_data_read().unwrap();
        defmt::debug!("acc int_map CONF: {:?}", defmt::Debug2Format(&int_map));
        let pwr_conf = dev_accel.acc_pwr_conf_read().unwrap();
        defmt::debug!("acc pwr_conf CONF: {:?}", defmt::Debug2Format(&pwr_conf));
        let pwr_ctrl = dev_accel.acc_pwr_ctrl_read().unwrap();
        defmt::debug!("acc pwr_ctrl CONF: {:?}", defmt::Debug2Format(&pwr_ctrl));
        let range = dev_accel.acc_range_read().unwrap();
        defmt::debug!("acc range CONF: {:?}", defmt::Debug2Format(&range));

        dev_gyro.configure_gyro(gyro_config).unwrap();

        let bandwith = dev_gyro.gyro_bandwidth_read().unwrap();
        defmt::debug!("gyro bandwith CONF: {:?}", defmt::Debug2Format(&bandwith));
        let power = dev_gyro.gyro_lpm_read().unwrap();
        defmt::debug!("gyro power CONF: {:?}", defmt::Debug2Format(&power));
        let range = dev_gyro.gyro_range_read().unwrap();
        defmt::debug!("gyro range CONF: {:?}", defmt::Debug2Format(&range));

        // set up interrupt
        let mut imu_drdy = gpioc.pc0.internal_pull_up(true);
        imu_drdy.make_interrupt_source(&mut syscfg);
        imu_drdy.trigger_on_edge(&mut dp.EXTI, gpio::Edge::Falling);
        imu_drdy.enable_interrupt(&mut dp.EXTI);

        defmt::info!("setup done");

        (
            Shared {
                dev_accel,
                dev_gyro,
            },
            Local { imu_drdy },
        )
    }

    // Interrupt
    #[task(binds = EXTI0, shared=[dev_accel, dev_gyro], local=[imu_drdy])]
    fn imu_drdy(cx: imu_drdy::Context) {
        cx.local.imu_drdy.clear_interrupt_pending_bit();
        sensor_process::spawn().ok();
    }

    //Task to read data from the IMU
    #[task(shared=[dev_accel, dev_gyro])]
    async fn sensor_process(mut cx: sensor_process::Context) {
        let acc_data = cx.shared.dev_accel.lock(|a| a.acc_data().unwrap());
        let gyro_data = cx.shared.dev_gyro.lock(|g| g.gyro_rate_read().unwrap());
        defmt::info!(
            "acc: {:?}, gyro: {:?}",
            defmt::Debug2Format(&acc_data),
            defmt::Debug2Format(&gyro_data)
        );
    }
}
