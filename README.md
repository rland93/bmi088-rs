# bmi088-rs

Embedded-hal driver for [Bosch Sensortec BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/).

[Sensor Datasheet (PDF)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

Thank you to [@eldruin](https://github.com/eldruin/bmi160-rs). The design of this driver is based very strongly on his work.

# Status

- Interfaces
  - [x] I2C
  - [x] SPI

- Accelerometer
  - [x] Error/Status Registers
  - [x] Data Registers
  - [x] Interrupt Registers
  - [x] Sensor Time Register
  - [x] Temperature Register
  - [x] Sensor Config Registers
  - [x] Interrupt Configuration (1/2)
  - [x] Power Configuration
  - [ ] Self Test
  - [x] Example Code

- Gyroscope
  - [x] Data Registers
  - [x] Sensor Config Registers
  - [x] Power Mode Registers
  - [x] Interrupt Configuration (3/4)
  - [x] Status Registers
  - [ ] Self Test
  - [x] Example Code

- Misc.
  - [ ] Publish on crates.io
  - [ ] Release versions
  - [ ] Usage documentation
 
## Example Code

To load the example code, create your `.cargo` configuration:

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
  # --- KEEP existing `link-arg` flags ---
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=--nmagic",
  # --- ADD following new flag ---
  "-C", "link-arg=-Tdefmt.x",
]

[build]
target = "thumbv7em-none-eabihf"

[env]
DEFMT_LOG = "trace"

[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "probe-run --chip STM32F401RCTx"
```

Replacing your board setup, of course. By default, the examples use SPI3:

- SCK: pc10
- MISO: pc11
- MOSI: pc12
- Acc CS: pd2
- Gyro CS: pb5

However, you can easily replace the pin and peripheral setup with your own. 

You should be able to compile and run the code examples by doing `cargo run`.