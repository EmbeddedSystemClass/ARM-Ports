/* STM32F1 Test of timer IRQ

LED is blinked in timer ISR. Timer is disabled in IRQ, then re-enabled after a poll loop.
Uses the update interrupt.

Tests:
Timer Interrupt.
GPIO basic output

The board used is the stm32f4-discovery with LEDs on port D pins 12-15

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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/dispatch/nvic.h>

/*--------------------------------------------------------------------*/
void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

/*--------------------------------------------------------------------*/
void gpio_setup(void)
{
/* Port C are on AHB1 */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR,RCC_AHB1ENR_IOPCEN);
/* Digital Test output PC1 */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0 | GPIO1);
/* Signal output PD */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------*/
void timer_setup(void)
{
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
	/* End timer value. If this is reached an interrupt is generated. */
	timer_set_period(TIM2, 5000);
	/* Update interrupt enable. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	/* Start timer. */
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------*/
void tim2_isr(void)
{
	gpio_toggle(GPIOD, GPIO12);   /* LED2 on/off. */
	if (timer_get_flag(TIM2, TIM_SR_UIF)) timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous (buffered) write before leaving */
	timer_disable_irq(TIM2, TIM_DIER_UIE);
	timer_disable_counter(TIM2);
}

/*--------------------------------------------------------------------*/
int main(void)
{
	clock_setup();
	gpio_setup();
	timer_setup();

	while (1) /* Halt. */
	{
		/* Update interrupt enable. */
		timer_enable_irq(TIM2, TIM_DIER_UIE);

		/* Start timer. */
		timer_enable_counter(TIM2);

		gpio_toggle(GPIOD, GPIO13);	/* LED on/off */
		int i;
		for (i = 0; i < 6400000; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
