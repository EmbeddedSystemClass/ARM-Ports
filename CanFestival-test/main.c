/*      Test of CANFestival drivers
*/

/*
This file is part of CanFestival, a library implementing CanOpen Stack.

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
#include <STM32/canfestival.h>
#include <STM32/serial_stm32.h>
#include <STM32/timerscfg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/dispatch/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include "ObjDict.h"
#include "ds401.h"

#define CAN_BAUDRATE 0

unsigned char timer_interrupt = 0;	   // Set if timer interrupt elapsed
unsigned char inputs;

// CAN
unsigned char nodeID;
unsigned char digital_input[1] = {0};
unsigned char digital_output[1] = {0};

static Message m = Message_Initializer;	        // contains a CAN message

void set_outputs(unsigned char data);
unsigned char get_inputs(void);
void sys_init();

int main(void)
{
    sys_init();                                 // Initialize system
    canInit(CAN_BAUDRATE);         		        // Initialize the CANopen bus
    initTimer();                                // Start timer for the CANopen stack
    nodeID = 0x04;				                // Read node ID first
    setNodeId(&ObjDict_Data, nodeID);
    setState(&ObjDict_Data, Initialisation);	// Initialise the state

    for(;;)		                                // forever loop
    {
        if (timer_interrupt == 1)               // Cycle timer, invoke action on every time slice
        {
            timer_interrupt = 0;	            // Reset the cycle timer
            digital_input[0] = get_inputs();
            digital_input_handler(&ObjDict_Data, digital_input, sizeof(digital_input));
            unsigned char error;
            error = digital_output_handler(&ObjDict_Data, digital_output, sizeof(digital_output));
            if (error = 0) EMCY_setError(&ObjDict_Data,1,1,45);
            else EMCY_errorRecovered(&ObjDict_Data,1);
            set_outputs(digital_output[0]);
        }

// a message was received pass it to the CANstack
        if (canReceive(&m))			            // a message received
        {
            canDispatch(&ObjDict_Data, &m);     // process it in the stack
        }
        else
        {
// Enter sleep mode
        }
      }
}

/******************************************************************************
Initialize the hardware, timer and process ports.
The timer is the tick timer set to 1ms intervals. The I/O is sampled every tick.
INPUT	void
OUTPUT	void
******************************************************************************/

void sys_init()
{
/* Clock setup to 72MHz */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
/* gpio setup */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
/* GPIO LED Ports */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
/* Timer Setup */
 	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);
	timer_reset(TIM2);
/* Timer global mode: - Divider 4, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	timer_set_prescaler(TIM2, 1440);
/* End timer value at 1ms. If this is reached an interrupt is generated. */
	timer_set_period(TIM2, 50);
/* Update interrupt enable to fire when counter reaches preset count. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
/* Start timer. */
	timer_enable_counter(TIM2);
}

/******************************************************************************
Write out the data byte to the LED port of the ET-STM32F103
******************************************************************************/

void set_outputs(unsigned char data)
{
/* Shift the data by 8 bits and output to LEDs */
    gpio_port_write(GPIOB, (uint) data << 8);
}

/******************************************************************************
Read in the data byte from the switch port of the ET-STM32F103
******************************************************************************/

unsigned char get_inputs(void)
{
    return 0;
}

/******************************************************************************
Set a flag for the main loop to activate a sample of I/O every clock tick.
******************************************************************************/

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF)) 
        timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous (buffered) write before leaving */
  	timer_interrupt = 1;	            // Set flag to indicate cycle timer tick
}
