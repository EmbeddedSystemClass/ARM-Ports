STAMP Board for STM32F
----------------------

This project provides a STAMP board that is pin compatible with the ETT
ET-ARM-STAMP-STM32F103. This has the advantage that it can be built into a
prototype system to provide the basic processor functions without having to
commit to any particular processor nor produce PCBs until the system under
development has been proven.

The board takes certain STM32F processors that use the LQFP64 footprint, as
these are largely pin compatible amongst a number of different families within
the STM32F range. Examples are:

STM32F10x. This needs capacitors C15 and C16 to be shorted to ground.
STM32F2xx.
STM32F4xx.
STM32F15xxx.

STM32F01x is not compatible as additional GPIO pins are present that would be
connected to power on this board. For any particular processor, care must be
taken to ensure full compatibility.

The Intersil ICL3221 was selected for RS232 driver as it is has low power
consumption and automatically powers down when no RS232 signals are detected.
It provides a single Rx/Tx pair. It also has the option to allow an external
signal to force power down but this is not used here. This single RS232
interface is provided mainly for programming.

(c) K. Sarkies 26/12/2017

