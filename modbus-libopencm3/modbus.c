/*
 * FreeModbus Libary: STM32F103 over FREERTOS
 * Copyright (C) 2012 Ken Sarkies <ksarkies@internode.on.net>
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: modbus.c,v 1.0 2012/09/26 Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include <mb.h>
#include <mbport.h>
#include "stm32f1.h"
#include <libopencm3/stm32/f1/gpio.h>

/* Prototypes */
void setupHardware( void );

/* ----------------------- Defines ------------------------------------------*/
/* Starting address of input registers */
#define REG_INPUT_START 				1000
/* Number of input registers */
#define REG_INPUT_NREGS 				4
#define REG_HOLDING_START               1
#define REG_HOLDING_NREGS               32

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
/* Buffer to hold the register values */
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- Start implementation -----------------------------*/
int
main( void )
{
    setupHardware();

    const UCHAR     ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
    eMBErrorCode    eStatus;

    for( ;; )
    {
        if((eStatus = eMBInit( MB_RTU, 0x0A, 1, 38400, MB_PAR_EVEN)) != MB_ENOERR)
        {
            /* Can not initialize. Add error handling code here. */
        }
        else
        {      
            if((eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3)) != MB_ENOERR)
            {
                /* Can not set slave id. Check arguments */
            }
            else if((eStatus = eMBEnable()) != MB_ENOERR)
            {
                /* Enable failed. */
            }
            else
            {      
                usRegHoldingBuf[0] = 1;
                do
                {
                    eMBPoll(  );
                    /* Here we simply count the number of poll cycles. */
                    usRegInputBuf[0]++;
                }
                while( usRegHoldingBuf[0] );
                eMBDisable(  );
                eMBClose(  );                
            }
        }
    }
	return -1;
}

/* ----------------------- Get values of registers -----------------------------*/

/* Callback function used if the value of a Input Register is required by the
protocol stack.
The starting register address is given by usAddress and the last register is given by
usAddress + usNRegs - 1. */

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
		gpio_toggle(GPIOB, GPIO8);
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
		gpio_toggle(GPIOB, GPIO9);
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/* ----------------------- Get from holding registers -----------------------------*/

/* Callback function used if an output Holding Register value is read or written by the
protocol stack.
The starting register address is given by usAddress and the last register is given by
usAddress + usNRegs - 1.  */

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/* ----------------------- Get/Put from Coil Register bits-----------------------------*/

/* Callback function used if a Coil Register value is read or written by the
protocol stack.
The starting register address is given by usAddress and the last register is given by
usAddress + usNRegs - 1.  */

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

/* ----------------------- Get from Input Discrete Register bits-----------------------------*/

/* Callback function used if an Input Discrete Register value is read by the
protocol stack.
The starting register address is given by usAddress and the last register is given by
usAddress + usNRegs - 1.  */

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
