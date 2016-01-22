/* STM32F1 CLI for tests - SD card using SPI

Terminal with baud rate 115200.

Initially tested with the ET-STM32F103 (STM32F103RBT6).

G will return a "hello" message
L will toggle GPIO8, first LED on the board.
S will return some status values for the inserted card.

Copyright K. Sarkies <ksarkies@internode.on.net>

3 September 2013
*/

/*
 * Copyright 2013 K. Sarkies <ksarkies@internode.on.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include "buffer.h"

/* Prototypes */

static void usart_print_int(int value);
static void usart_print_hex(uint16_t value);
static void usart_print_string(char *ch);
static void gpio_setup(void);
static void spi_setup(void);
static void usart_setup(void);
static void clock_setup(void);
static void parseCommand(char* line);
static uint8_t socketWriteProtected(void);
static uint8_t socketCardInserted(void);

#define BUFFER_SIZE 128
#define N_CONV 6
#define N_SAMPLES 16

/* Globals */
uint8_t send_buffer[BUFFER_SIZE+3];
uint8_t receive_buffer[BUFFER_SIZE+3];
char line[80];
uint8_t characterPosition;

/*--------------------------------------------------------------------------*/

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();
	spi_setup();
	buffer_init(send_buffer,BUFFER_SIZE);
	buffer_init(receive_buffer,BUFFER_SIZE);
	usart_enable_tx_interrupt(USART1);

/* Send a greeting message on USART1. */
	usart_print_string("SD Card SPI Mode Test\r\n");

	while (1)
	{
/* Command interface */
        if (buffer_input_available(receive_buffer))
        {
            char character = buffer_get(receive_buffer);
            if (character == 0x0D)
            {
                line[characterPosition] = 0;
                characterPosition = 0;
                parseCommand(line);
            }
            else line[characterPosition++] = character;
        }
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Parse a command line and act

*/

void parseCommand(char *line)
{
    if (line[0] == 'G')
    {
        usart_print_string("Hello\r\n");
    }
    else if (line[0] == 'L')
    {
        gpio_toggle(GPIOB,GPIO8);
    }
    else if (line[0] == 'S')
    {
        if (socketWriteProtected())
            usart_print_string("Write Protected\r\n");
        else
            usart_print_string("Writeable\r\n");
        if (socketCardInserted())
            usart_print_string("Card Present\r\n");
        else
            usart_print_string("No Card\r\n");
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Write Protect Test

*/

static uint8_t socketWriteProtected(void)
{
	return (gpio_get(GPIOC,GPIO6) > 0);
}

/*--------------------------------------------------------------------------*/
/** @brief Card Detect Test

*/

static uint8_t socketCardInserted(void)
{
	return (!socketWriteProtected() || (gpio_get(GPIOA,GPIO8) == 0));
}

/*--------------------------------------------------------------------------*/
/** @brief Clock Setup

The processor system clock is established and the necessary peripheral
clocks are turned on.
*/

void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/*--------------------------------------------------------------------------*/
/** @brief SPI Setup

This is for the SPI1 on the ET-STM32F103 card.
We are unable to use PA8 with the USART1.
*/

void spi_setup(void)
{
/* Enable GPIO clocks. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR,
                RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN);
/* PA6,8 inputs digital for SPI */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
			    GPIO6 | GPIO8);
/* PA4,5,7 outputs digital for SPI */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			    GPIO4 | GPIO5 | GPIO7);
/* PC6 input digital for write protect status */
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
			    GPIO6);
}

/*--------------------------------------------------------------------------*/
/** @brief GPIO Setup

Setup GPIO Ports needed.
*/

void gpio_setup(void)
{
/* Enable GPIO clocks. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR,
                RCC_APB2ENR_IOPBEN);
/* PB8-15 outputs digital for LEDs */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			    GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/
/** @brief USART Setup

USART 1 is configured for 115200 baud, no flow control, and interrupt.
*/

void usart_setup(void)
{
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
/* Setup UART parameters. */
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
}

/*--------------------------------------------------------------------------*/
/** @brief Print out the contents of a register (debug)

*/

void print_register(uint32_t reg)
{
	usart_print_hex((reg >> 16) & 0xFFFF);
	usart_print_hex((reg >> 00) & 0xFFFF);
	buffer_put(send_buffer, ' ');
    usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/** @brief Print out a value in ASCII decimal form (ack Thomas Otto)

*/

void usart_print_int(int value)
{
	uint8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0)
	{
		buffer_put(send_buffer, '-');
		value = value * -1;
	}
	if (value == 0) buffer[nr_digits++] = '0';
	else while (value > 0)
	{
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}
	for (i = nr_digits; i > 0; i--)
	{
		buffer_put(send_buffer, buffer[i-1]);
	}
//	buffer_put(send_buffer, ' ');
    usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/** @brief Print out a value in ASCII hex form

*/

void usart_print_hex(uint16_t value)
{
	uint8_t i;
	char buffer[25];

	for (i = 0; i < 4; i++)
	{
		buffer[i] = "0123456789ABCDEF"[value & 0xF];
		value >>= 4;
	}
	for (i = 4; i > 0; i--)
	{
		buffer_put(send_buffer, buffer[i-1]);
	}
	buffer_put(send_buffer, ' ');
    usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/** @brief Print a String

*/

void usart_print_string(char *ch)
{
  	while(*ch)
	{
     	buffer_put(send_buffer, *ch);
     	ch++;
  	}
    usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/** @brief USART Interrupt

Find out what interrupted and get or send data as appropriate.
*/

void usart1_isr(void)
{
	static uint16_t data;

/* Check if we were called because of RXNE. */
	if (usart_get_flag(USART1,USART_SR_RXNE))
	{
/* If buffer full we'll just drop it */
		buffer_put(receive_buffer, (uint8_t) usart_recv(USART1));
	}
/* Check if we were called because of TXE. */
	if (usart_get_flag(USART1,USART_SR_TXE))
	{
/* If buffer empty, disable the tx interrupt */
		data = buffer_get(send_buffer);
		if ((data & 0xFF00) > 0) usart_disable_tx_interrupt(USART1);
		else usart_send(USART1, (data & 0xFF));
	}
}

