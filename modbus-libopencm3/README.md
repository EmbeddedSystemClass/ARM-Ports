Port of freeMODBUS to libopencm3
--------------------------------

This project describes a port of freeMODBUS to work with libopencm3. The files
that are adapted are portserial.c, in which the USART is initialised and ISRs
established to handle the serial input and output, and porttimer.c in which the
timer is initialised and an ISR established to handle timed operations. stm32.c
initialises the hardware clocks and the I/O ports if needed.

The makefile uses the load script for the ET-STM32F103 which is STM32F103RBT6
with 128kB FLASH memory and 20K RAM, so it is a medium density device.

The ET-ARM Stamp module also works. This has a STM32F103RET6 which is a
HD device with 512K FLASH, 64K SRAM.

This sets up a MODBUS slave on the ET-STM32F103 board.

Two examples are provided:

* modbus.c that uses timer polling.

* modbus-freertos.c that uses a scheduler.

More information is provided at:

http://www.jiggerjuice.info/electronics/projects/arm/modbus-stm32f103-port.html

(c) K. Sarkies 29/06/2015

