/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
STM32F103 Port: Ken Sarkies
Based on AVR Port: Andreas GLAUSER and Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "serial_stm32.h"
#include "canfestival.h"
#include <STM32/buffer.h>

#define BUFFER_SIZE 128

/* Globals */
UNS8 send_buffer[BUFFER_SIZE+3];
UNS8 receive_buffer[BUFFER_SIZE+3];
UNS8 message_temp[12];

volatile UNS8 msg_received = 0;
volatile UNS8 msg_recv_status = 0;

/******************************************************************************
Initialize the hardware to receive (USART based) CAN messages and start the
timer for the CANopen stack.
INPUT	bitrate		bitrate in kilobit (fixed at 115200)
OUTPUT	1 if successful	
******************************************************************************/
unsigned char canInit(unsigned int bitrate)
{
	msg_recv_status = 0;
	msg_received = 0;
/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
				    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN);
/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
/* Setup USART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
/* Enable USART1 receive interrupts. */
	usart_enable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
/* Finally enable the USART. */
	usart_enable(USART1);

/* Initialise the send and receive buffers */
	buffer_init(send_buffer,BUFFER_SIZE);
	buffer_init(receive_buffer,BUFFER_SIZE);

 	return 1;
}

/******************************************************************************
The driver send a CAN message passed from the CANopen stack
INPUT	CAN_PORT is not used (only use the defined in the ISR port)
	Message *m pointer to message to send
OUTPUT	1 if  hardware -> CAN frame

Message is a struct defined in can.h
This echoes the way the can_serial driver works.
******************************************************************************/
UNS8 count = 0;
unsigned char canSend(CAN_PORT notused, Message *m)
{
        unsigned char i;

/* Send the message as raw bytes (note little-endianness of Cortex M3) */
	buffer_put(send_buffer, (m->cob_id) & 0xFF);
	buffer_put(send_buffer, (m->cob_id) >> 8);
	buffer_put(send_buffer, (m->rtr));
	buffer_put(send_buffer, (m->len));
	for (i= 0; i < (m->len); i++)
		buffer_put(send_buffer, m->data[i]);
/* Start sending by enabling the interrupt */
	usart_enable_tx_interrupt(USART1);
        return 1;	// successful
}

/******************************************************************************
The driver passes a received CAN message to the stack
INPUT	Message *m pointer to received CAN message
OUTPUT	1 if a message received
******************************************************************************/
unsigned char canReceive(Message *m)
{
        unsigned char i;

  	if (msg_received == 0)
    	return 0;					/* Nothing received yet */

/* In this we are relying on the ISR to build up the message and shove to the
buffer before notifying us. Therefore we don't check unavailability of data in
the buffer or any other error conditions. */
	m->cob_id = buffer_get(receive_buffer) + (buffer_get(receive_buffer) << 8);
	m->rtr = buffer_get(receive_buffer);
	m->len = buffer_get(receive_buffer);
	for (i= 0; i < (m->len); i++)
		m->data[i] = buffer_get(receive_buffer);

	msg_received--;					/* Let the ISR know we are done */

        return 1;                  		/* message received */
}

/**************************************************************************
We won't do this as it is not CAN
***************************************************************************/
unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud)
{
	return 0;
}

/******************************************************************************
USART Interrupt
For the receiver, build a message first then put to buffer and tell main program.
******************************************************************************/
/* Find out what interrupted and get or send data as appropriate */

void usart1_isr(void)
{
	static UNS16 data;

/* Check if we were called because of RXNE. */
	if (usart_get_flag(USART1,USART_SR_RXNE))
	{
/* Put to temporary message buffer */
		data = (UNS8) usart_recv(USART1);
		message_temp[msg_recv_status] = (UNS8) data;
		msg_recv_status++;
/* At this point we should check for integrity of the message, but there is no
way to recover if the message length sent is wrong. So we'll do what we can and
hope we can catch up eventually. */
		if ((msg_recv_status > 3) && (message_temp[3] > 8))
		{
/* Bad message length - abort and restart a new message */
			msg_recv_status = 0;
		}
/* Check if we are finished (message length is in element 3) */
		else if (msg_recv_status == 4 + message_temp[3])
		{
			msg_recv_status = 0;
			msg_received++;			/* Signal another message arrived */
			UNS8 i;
/* Dump to buffer; if full we'll just have to drop it */
			for (i=0; i < 4 + message_temp[3]; i++)
				buffer_put(receive_buffer,message_temp[i]);
		}
	}
/* Check if we were called because of TXE. */
	else if (usart_get_flag(USART1,USART_SR_TXE))
	{
		data = buffer_get(send_buffer);
/* If buffer empty, disable the tx interrupt */
		if ((data & 0xFF00) > 0) usart_disable_tx_interrupt(USART1);
		else
        {
                usart_send(USART1, (data & 0xFF));
        }
	}
}

