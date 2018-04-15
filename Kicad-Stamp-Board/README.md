STAMP Board for STM32F
----------------------

This project provides a STAMP board that is pin compatible with the ETT
ET-ARM-STAMP-STM32F103. This has advantages that it can be built into a
prototype system to provide the basic processor functions without having to
commit to any particular processor nor produce PCBs until the system under
development has been proven. The board improves on the ETT board by allowing
a range of processor families.

The board takes certain STM32F processors that use the LQFP64 footprint, as
these are largely pin compatible amongst a number of different families within
the STM32 range. Examples are:

STM32F10x. J5, J6, C15, C16 must all be shorted.
STM32L15x. J5, J6, C15, C16 must all be shorted.
STM32F2xx. J5, J6 must be shorted. C15 and C16 are 2.2uF capacitors.
STM32F4xx. J5, J6 must be shorted. C15 and C16 are 2.2uF capacitors.
STM32F72x. J5, J6, C15, C16 must all be shorted.
STM32F05x. Only C16 must be shorted, C15 and J5, J6, J14 left open.
STM32F30x. J5, C15, C16 must all be shorted, J6 left open.

STM32F37x and STM32F73x are not compatible as pin order is different.

In the second revision of the board, J5 on pin 48 and J6 on pin 18 are provided
so that STM32F05x and STM32F30x can be accommodated.

For any particular processor family, care must be taken to ensure full
compatibility with the board.

The Intersil ICL3221 was selected for RS232 driver as it is has low power
consumption and automatically powers down when no RS232 signals are detected.
It provides a single Rx/Tx pair. It also has the option to allow an external
signal to force power down but this is not used here. This single RS232
interface is provided mainly for programming.

For testing, when the PC serial cable is inserted, the Tx input from that sits
at about -6V. This triggers the device charge pumps to work. The voltages
expected are:

Pin 1 Enable Inv            Gnd
Pin 2 C1+ (C10)            5.96V
Pin 3 V+  (C12)            6.12V
Pin 4 C1- (C10)            3.1V
Pin 5 C2- (C14)            5.78V
Pin 6 C2+ (C14)            -0.3V
Pin 7 V-  (C13)            -5.6V
Pin 8 Tx in                -6V

(c) K. Sarkies 15/02/2018

