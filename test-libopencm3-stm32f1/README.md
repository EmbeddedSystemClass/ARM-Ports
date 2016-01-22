A series of tests for the STM32F103 on the ET STAMP and development board
=========================================================================

The makefiles use the load script for the ET-STM32F103 which is STM32F103RBT6
with 128kB FLASH memory and 20K RAM, so it is a medium density device.

The ET-ARM Stamp module also works. This has a STM32F103RET6 which is a
HD device with 512K FLASH, 64K SRAM.

Compile these with
$ make -f Makefile-*name*

where *name* is the root name of the file.

* **adc-dual-stm32f103.c**
    This converts a number of ADC channels using scan mode and dual conversion
    mode. Conversions are triggered by a timer. The results are put to memory
    using DMA, and are then transmitted by USART to an external terminal in
    an ASCII decimal form.
* **adc-poll-et-stm32f103.c**
    Read ADC1 on port PA1 and blink LEDs on PB8 and PB9 at a rate that changes
    with changes in the voltage on PA1.
* **blink-et-stm32f103.c**
    Toggles two LEDs on GPIO8 and GPIO9 (these are present on the ET-STM32F103.
* **dac-dma-dual-et-stamp-stm32f103.c**
    Outputs a funky waveform on DAC1 (PA4) and puts a CRO trigger signal on PC1.
* **flash-rw-et-stm32f103.c**
* **iwdg-et-stm32f103.c**
    The independent watchdog timer is set going and is reset for a period of
    time to prevent it triggering while the LED on GPIO8 blinks. Then the LED
    ojn GPIO9 is turned on and the program enters an infinite loop. The IWDT
    should then force a reset after its preset time period.
* **pwm-tim1.c**
    Set advanced timer 1 to PWM mode, centre aligned, 62.5kHz with a deadtime.
* **pwm-tim3.c**
    Set basic timer 3 to PWM mode, centre aligned, 62.5kHz with a deadtime.
* **sp2-dma-test.c**
    Based on the Lisa 2 spi-dma test in libopencm3-examples. Loops back the
    MISO and MOSI, and transmits the received data bytes via USART 1
* **spi1-test.c**
    Based on the Lisa 2 spi test in libopencm3-examples. Either loops back
    the MISO and MOSI, and transmits the received data byte via USART 1,
    or send a "reset" packet to an SD card and observe the response with CRO.
**spi2-test.c**
    Based on the Lisa 2 spi test in libopencm3-examples. Either loops back
    the MISO and MOSI, and transmits the received data byte via USART 1,
    or send a "reset" packet to an SD card and observe the response with CRO.
* **systick-et-stamp-stm32**
* **test-dac-dma-et-stamp-stm32f103.c**
* **test-dac-polled-dual-et-stamp-stm32f103.c**
* **test-dac-polled-et-stamp-stm32f103.c**
* **timer-interrupt-et-stm32f103.c**
    Test of timer IRQ. Timer 2 is set to 50Hz period. Uses update interrupt.
* **timer-interrupt-et-stm32f103**
    Test of timer IRQ. LED is blinked in timer ISR. Timer is disabled in IRQ,
    then re-enabled after a poll loop. Uses the update interrupt.
* **timer-interrupt-oc-et-stm32f103.c**
    Test of timer IRQ using Output Compare.

K. Sarkies
17/12/2015
