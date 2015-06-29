Port of CANfestival to libopencm3
---------------------------------

This project describes a port of CANfestival libraries to work with libopencm3.

The example main program given here provides the basic timing interrupt but
the serial interface replacing the usual CAN Bus interface has not been
implemented.

main.c contains initialization code for the hardware. The port directory
contains the drivers for serial and timer initialization and ISR. CANfestival
itself has not been modified but must be compiled into the application as
indicated in the makefile.

More information is provided at:

http://www.jiggerjuice.info/electronics/projects/arm/canfestival-stm32f103-port.html

(c) K. Sarkies 30/06/2015

