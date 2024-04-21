MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 64K
}

/* 
You must modify the memory.x file depending on the MCU you are using.

for FLASH:
for the i2c example, use LENGTH = 512K; for the spi example, use LENGTH = 256K


for RAM:
for the i2c example, use LENGTH=128K; for the spi example, use LENGTH = 64K 
*/

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
