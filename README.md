# bmi088-rs

Embedded-hal driver for [Bosch Sensortec BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/).

[Sensor Datasheet (PDF)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

Thank you to [@eldruin](https://github.com/eldruin/bmi160-rs). The design of this driver is based very strongly on his work.

# Status

- Interfaces
  - [x] I2C
  - [ ] SPI

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

There is one example, in examples/stm32f411.rs. 

That is also the hardware test code.

The example code can be built on an STM32F411RETx "Black Pill" board: https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html. Connect the sensor to i2c1. 

Or any other STM32 hardware can be used with minor modifications to the target, memory.x, and probe-rs settings.
