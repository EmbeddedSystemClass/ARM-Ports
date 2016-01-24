/* STM32F1 Test of timer IRQ

Timer 2 is set to 50Hz period. Uses the update interrupt.

Tests:
Timer Interrupt.
GPIO basic output 50Hz on PA0.

The board used is the ET-STAMP-STM32

*/
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

/*--------------------------------------------------------------------------*/

void gpio_setup(void)
{
/* Enable GPIO clocks. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

/* Set GPIO15 (in GPIO port A) to 'output push-pull' */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		          GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
	gpio_clear(GPIOA, GPIO0);
}

/*--------------------------------------------------------------------------*/

void timer_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);
	timer_reset(TIM2);
/* Timer global mode: - No Divider, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
	/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	timer_set_prescaler(TIM2, 1440);
	/* End timer value. When this is reached an interrupt is generated. */
	timer_set_period(TIM2, 50);   /* Should be 1000 per second */
	/* Update interrupt enable. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	/* Start timer. */
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------------*/

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF))
        timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous write */
 	gpio_toggle(GPIOA, GPIO0);   /* Toggle port, frequency should be 50Hz. */
}

/*--------------------------------------------------------------------------*/

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gpio_setup();
	timer_setup();


	while (1);                       /* Halt. */

	return 0;
}
