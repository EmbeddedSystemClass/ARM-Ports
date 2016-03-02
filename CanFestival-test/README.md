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

Includes a sample Object Dictionary. gen_cfile.py is used to generate the code
for integration into the application.

CanFestival and libopencm3 must be provided and the makefile adapted to point
to these. The STM32 directory present here must be copied to the include
directory of CanFestival to provide the libopencm3 support for the STM32F ARM
processor. The libopencm3 code is provided in the port directory.

Latest CanFestival is version 3 on 04/08/2015. The authors do not seem to have
a version numbering system. The latest version can be accessed through the
CanFestival homepage from a Mercurial repository at dev.automforge.net.
Sourceforge has a very old version from 2007.

More information is provided at [Jiggerjuice](http://www.jiggerjuice.info/electronics/projects/arm/canfestival-stm32f103-port.html)

(c) K. Sarkies 22/01/2016

