/*	Circular Buffer Management

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

19 September 2012

Intended for libopencm3 but can be adapted to other libraries provided
data types defined.
*/

#ifndef BUFFER_H
#define BUFFER_H

#include <libopencm3/cm3/common.h>

void buffer_init(unsigned char buffer[], unsigned char size);
unsigned short buffer_get(unsigned char buffer[]);
unsigned short buffer_put(unsigned char buffer[], unsigned char data);
bool buffer_output_free(unsigned char buffer[]);
bool buffer_input_available(unsigned char buffer[]);

#endif 

