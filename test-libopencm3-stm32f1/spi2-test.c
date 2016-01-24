/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2013 Stephen Dwyer <scdwyer@ualberta.ca>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include "buffer.h"

#ifndef USE_16BIT_TRANSFERS
#define USE_16BIT_TRANSFERS 1
#endif

static void clock_setup(void);
static void spi_setup(void);
static void usart_setup(void);
static void gpio_setup(void);
static void print_register(uint32_t reg);
static void usart_print_int(int value);
static void usart_print_hex(uint16_t value);
static void usart_print_string(char *ch);

#define BUFFER_SIZE 128

uint8_t send_buffer[BUFFER_SIZE+3];
uint8_t receive_buffer[BUFFER_SIZE+3];

/*--------------------------------------------------------------------------*/

int main(void)
{
	int counter = 0;
	uint16_t rx_value = 0x42;

/* Setup Rx/Tx buffers for USART */
	buffer_init(send_buffer,BUFFER_SIZE);
	buffer_init(receive_buffer,BUFFER_SIZE);

	clock_setup();
	gpio_setup();
	usart_setup();
    usart_print_string("SPI-DMA Test\n\r");
	spi_setup();

	/* Blink (PA1) on the board with every transmitted byte. */
	while (1) {
		/* LED on/off */
		gpio_toggle(GPIOA, GPIO1);

#ifdef LOOPBACK
		/* Print what is going to be sent on the SPI bus */
		usart_print_string("Sending  packet ");
        usart_print_int(counter);
        usart_print_string("\n\r");
		spi_send(SPI2, (uint8_t) counter);
		rx_value = spi_read(SPI2);
		usart_print_string("Received  packet ");
        usart_print_int(rx_value);
        usart_print_string("\n\r");
        counter++;
#else
/* This is a 1-byte "reset" command to SD card */
		spi_send(SPI2, 0x40);
		spi_send(SPI2, 0x00);
		spi_send(SPI2, 0x00);
		spi_send(SPI2, 0x00);
		spi_send(SPI2, 0x00);
		spi_send(SPI2, 0x95);
		/* Read the byte that just came in (use a loopback between MISO and MOSI
		 * to get the same byte back)
		 */
		rx_value = spi_read(SPI2);
#endif
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

}

/*--------------------------------------------------------------------------*/
static void gpio_setup(void)
{
/* Set GPIO1-3 (in GPIO port A) to 'output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1 | GPIO2 | GPIO3);
}

/*--------------------------------------------------------------------------*/
static void spi_setup(void) {

/* Enable SPI2 Periph */
	rcc_periph_clock_enable(RCC_SPI2);

/* Configure GPIOs: SS=PB12, SCK=PB13, MISO=PB14 and MOSI=PB15 */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO15 );
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,GPIO14);

/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI2);

/* Set up SPI in Master mode with:
* Clock baud rate: 1/128 of peripheral clock frequency
* Clock polarity: Idle High
* Clock phase: Data valid on 2nd clock pulse
* Data frame format: 8-bit
* Frame format: MSB First
*/
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_128, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

/*
* Set NSS management to software.
*
* Note:
* Setting nss high is very important, even if we are controlling the GPIO
* ourselves this bit needs to be at least set to 1, otherwise the spi
* peripheral will not send any data out.
*/
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);

/* Enable SPI2 periph. */
    spi_enable(SPI2);

/* Set the CS low (enabled) */
    gpio_clear(GPIOB, GPIO12);
}

/*--------------------------------------------------------------------------*/
/** @brief USART Setup

USART 1 is configured for 115200 baud, no flow control, and (no) interrupt.
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
        while (!buffer_output_free(send_buffer));
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
        while (!buffer_output_free(send_buffer));
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
        while (!buffer_output_free(send_buffer));
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
        while (!buffer_output_free(send_buffer));
     	buffer_put(send_buffer, *ch);
     	ch++;
  	}
	usart_enable_tx_interrupt(USART1);
}

/*-----------------------------------------------------------*/
/* USART ISR */
void usart1_isr(void)
{
/* Find out what interrupted and get or send data as appropriate */
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
		uint16_t data = buffer_get(send_buffer);
		if (data == BUFFER_EMPTY)
		{
			usart_disable_tx_interrupt(USART1);
		}
		else
		{
			usart_send(USART1, data);
		}
	}
}


