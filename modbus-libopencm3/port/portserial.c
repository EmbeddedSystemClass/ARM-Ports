/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include "port.h"

/* ----------------------- libopencm3 STM32F includes -------------------------------*/
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Enable USART interrupts -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    if( xRxEnable )
    {
		usart_enable_rx_interrupt(USART1);
    }
    else
    {
		usart_disable_rx_interrupt(USART1);
    }

    if( xTxEnable )
    {
		usart_enable_tx_interrupt(USART1);
    }
    else
    {
		usart_disable_tx_interrupt(USART1);
    }
}

/* ----------------------- Initialize USART ----------------------------------*/
/* Called with databits = 8 for RTU */

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
BOOL bStatus;
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
				    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN);
	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, ulBaudRate);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
    bStatus = TRUE;
    switch ( eParity )
    {
    case MB_PAR_NONE:
        usart_set_parity(USART1, USART_PARITY_NONE);
        break;
    case MB_PAR_ODD:
        usart_set_parity(USART1, USART_PARITY_ODD);
        break;
    case MB_PAR_EVEN:
        usart_set_parity(USART1, USART_PARITY_EVEN);
        break;
    default:
        bStatus = FALSE;
        break;
    }

/* Oddity of STM32F series: word length includes parity. 7 bits no parity
   not possible */
CHAR wordLength;
    switch ( ucDataBits )
    {
    case 8:
		if (eParity == MB_PAR_NONE)
			wordLength = 8;
		else
			wordLength = 9;
        usart_set_databits(USART1,wordLength);
        break;
    case 7:
		if (eParity == MB_PAR_NONE)
			bStatus = FALSE;
		else
        	usart_set_databits(USART1,8);
        break;
    default:
        bStatus = FALSE;
    }

    if( bStatus == TRUE )
    {
		/* Finally enable the USART. */
		usart_disable_rx_interrupt(USART1);
		usart_disable_tx_interrupt(USART1);
		usart_enable(USART1);
    }
    return bStatus;
}

/* -----------------------Send character  ----------------------------------*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	usart_send(USART1, ucByte);
    return TRUE;
}

/* ----------------------- Get character ----------------------------------*/
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = (CHAR) usart_recv(USART1);
    return TRUE;
}

/* ----------------------- Close Serial Port ----------------------------------*/
void
vMBPortSerialClose( void )
{
	nvic_disable_irq(NVIC_USART1_IRQ);
	usart_disable(USART1);
}

/* ----------------------- USART ISR ----------------------------------*/
/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/* Find out what interrupted and get or send data as appropriate */
void usart1_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (usart_get_interrupt_source(USART1,USART_SR_RXNE))
	{
	    pxMBFrameCBByteReceived(  );
	}
	/* Check if we were called because of TXE. */
	if (usart_get_interrupt_source(USART1,USART_SR_TXE))
	{
	    pxMBFrameCBTransmitterEmpty(  );
	}
}

