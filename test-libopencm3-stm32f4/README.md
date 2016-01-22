A series of tests for the STM32F407 on the STM32F4-Discovery Evaluation Board
=============================================================================

The makefiles use the load script for the STM32F4-Discovery which is STM32F407VG
with 1024kB FLASH memory and 128K RAM.

Compile these with
$ make -f Makefile-*name*

where *name* is the root name of the file.

**adc-injected-stm32f4discovery.c**
**adc-interrupt-stm32f4discovery.c**
**adc-poll-stm32f4discovery.c**
**adc-stm32f4discovery.c**
**blink-stm32f4discovery.c**
**test-dac-dma-stm32f4discovery.c**
**test-dac-polled-stm32f7discovery.c**
**test-dac-timer-stm32f4discovery.c**
**timer-interrupt-stm32f4discovery.c**

K. Sarkies
17/12/2015
