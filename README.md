# bmi088-rs

Embedded-hal driver for [Bosch Sensortec BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/).

[Sensor Datasheet (PDF)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

Thank you to [@eldruin](https://github.com/eldruin/bmi160-rs). The design of this driver is based very strongly on his work.

## Example Code

There is one example, in examples/stm32f411.rs. 

That is also the hardware test platform.

The example code can be built on an STM32F411RETx "Black Pill" board: https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html. Or any other STM32 hardware, if minor modifications are made to the target, memory.x, and probe-rs --chip. 