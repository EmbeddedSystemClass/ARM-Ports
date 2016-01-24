A series of tests for the STM32F407 on the STM32F4-Discovery Evaluation Board
=============================================================================

The makefiles use the load script for the STM32F4-Discovery which is STM32F407VG
with 1024kB FLASH memory and 128K RAM.

Compile these with
$ make -f Makefile-*name*

where *name* is the root name of the file.

* **adc--dma-stm32f4discovery.c**
Blink LED at a different rate with analogue control. LEDs on port D pins 12-15,
analogue signal into pin PA1 (ADC123 IN1), DMA used for data transfer.
* **adc-injected-stm32f4discovery.c**
* **adc-interrupt-stm32f4discovery.c**
Blink LED at a different rate with analogue control. LEDs on port D pins 12-15,
analogue signal into pin PA1 (ADC123 IN1), ADC interrupts when complete.
* **adc-poll-stm32f4discovery.c**
Blink LED at a different rate with analogue control. LEDs on port D pins 12-15,
analogue signal into pin PA1 (ADC123 IN1), poll for ADC complete.
* **blink-stm32f4discovery.c**
Basic Blink
* **test-dac-dma-stm32f4discovery.c**
DAC setup with timer 2 trigger using the OC1 output.
DMA used to move data from a predefined array in circular mode.
DMA ISR on transfer complete used to toggle a port for CRO trigger.
* **test-dac-polled-stm32f7discovery.c**
DAC setup with timer 2 to provide a timed output in timer ISR.
* **test-dac-timer-stm32f4discovery.c**
The DAC setup with timer 2 trigger on OC1 output.
DMA used to move data from a predefined array in circular mode.
DMA ISR on transfer complete used to toggle port C1 for CRO trigger.
* **timer-interrupt-stm32f4discovery.c**
LED is blinked in timer ISR. Timer is disabled in IRQ, then re-enabled after a
poll loop. Uses the update interrupt.

K. Sarkies
17/12/2015
