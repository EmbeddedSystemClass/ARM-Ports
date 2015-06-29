/*      Port of CanFestival to STM32F103 using libopencm3

*/

/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
STM32F103 Port: Ken Sarkies, November 2012
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

/* STM32 implementation of the CANopen timer driver, uses Timer 3 (16 bit) */

/* Includes for the Canfestival driver */
#include <canfestival.h>
#include <timer.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

TIMEVAL timerAlarm = 0;

/************************** Module variables **********************************/
/* Store the last timer value to calculate the elapsed time */
static TIMEVAL last_time_set = TIMEVAL_MAX;     

/******************************************************************************
Initializes the timer, turn on the interrupt and put the interrupt time to zero
INPUT	void
OUTPUT	void

The timer is set to roll over at 0.5 second from an 8us clock, and the output
compare 1 is used to trigger an alarm. This is set progressively by the
CanFestival stack.
******************************************************************************/
void initTimer(void)
{
/* Set alarm back to zero */
  	timerAlarm = 0;
/* Enable TIM3 clock. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
/* Enable TIM3 interrupt. */
	nvic_enable_irq(NVIC_TIM3_IRQ);
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
/* Set prescaler to give 8us clock */
        timer_set_prescaler(TIM3, 576);
/* Set the period as 0.5 second */
	timer_set_period(TIM3, TIMEVAL_MAX);
/* Disable physical pin outputs. */
	timer_disable_oc_output(TIM3, TIM_OC1 | TIM_OC2 | TIM_OC3 | TIM_OC4);
/* Configure global mode of output channel 1, disabling the output. */
	timer_disable_oc_clear(TIM3, TIM_OC1);
	timer_disable_oc_preload(TIM3, TIM_OC1);
	timer_set_oc_slow_mode(TIM3, TIM_OC1);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FROZEN);
/* Set the initial compare value for OC1. */
	timer_set_oc_value(TIM3, TIM_OC1, timerAlarm);
/* Continous counting mode. */
	timer_continuous_mode(TIM3);
/* ARR reload disable. */
	timer_disable_preload(TIM3);
/* Counter enable. */
	timer_enable_counter(TIM3);
/* Enable compare match interrupt. */
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

/******************************************************************************
Set the timer for the next alarm.
INPUT	value TIMEVAL (unsigned long) 0...TIMEVAL_MAX
OUTPUT	void
******************************************************************************/
void setTimer(TIMEVAL value)
{
/* Add the desired time to the timer interrupt time for the next alarm */
/* NOTE: This is computing (timerAlarm + value) % TIMEVAL_MAX, but so that it can
handle possible (timerAlarm + value) beyond the integer range of TIMEVAL */
/* Just make certain that value is in range */
        if (value > TIMEVAL_MAX) value = TIMEVAL_MAX;
        if (timerAlarm < TIMEVAL_MAX - value)
  	        timerAlarm += value;
        else
                timerAlarm -= TIMEVAL_MAX - value;
	timer_set_oc_value(TIM3, TIM_OC1, (unsigned long) timerAlarm);
}

/******************************************************************************
Return the elapsed time to tell the Stack how much time is spent since last call.
INPUT	void
OUTPUT	value TIMEVAL (unsigned long) the elapsed time
******************************************************************************/
TIMEVAL getElapsedTime(void)
{
  	TIMEVAL timer = timer_get_counter(TIM3);	/* Read the value of the running timer */
  	if (timer > last_time_set)                      /* In case the timer value is higher than the last time. */
    	        return (timer - last_time_set);         /* Calculate the time difference */
  	else if (timer < last_time_set)                 /* Timer has cycled around */
    	        return (timer + (TIMEVAL_MAX - last_time_set)); /* Calculate the time difference */
  	else
    	        return TIMEVAL_MAX;                     /* timer has gone around a full cycle.*/
}

/******************************************************************************
Interruptserviceroutine Timer 3 Compare 1 for the CAN timer
Pull in the timer counter value to represent the last time an alarm was set.
******************************************************************************/
void tim3_isr(void)
{
/* Clear interrrupt flag. */
	if (timer_get_flag(TIM3, TIM_SR_CC1IF))
        {
                timer_clear_flag(TIM3, TIM_SR_CC1IF);
/* Reread to force the previous write before leaving (a side-effect of hardware pipelining)*/
	        timer_get_flag(TIM3, TIM_SR_CC1IF);
  	        last_time_set = timer_get_counter(TIM3);
/* Call the time handler of the stack to adapt the elapsed time */
  	        TimeDispatch();
        }
}

