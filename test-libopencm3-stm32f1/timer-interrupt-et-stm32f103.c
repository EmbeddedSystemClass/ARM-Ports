/* STM32F1 Test of timer IRQ

LED is blinked in timer ISR. Timer is disabled in IRQ, then re-enabled after a
poll loop. Uses the update interrupt.

Tests:
Timer Interrupt.
GPIO basic output

The board used is the ET-STM32F103 with LEDs on port B pins 8-15

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

void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set GPIO8-15 (in GPIO port B) to 'output push-pull' for the LEDs. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_clear(GPIOB, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
               GPIO14 | GPIO15);
}

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
	timer_set_period(TIM2, 300);    /* Should be 166 per second (toggle 83Hz) */
//	timer_set_period(TIM2, 5000);   /* Should be 10 per second */
	/* Update interrupt enable. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	/* Start timer. */
	timer_enable_counter(TIM2);
}

void tim2_isr(void)
{
	gpio_toggle(GPIOB, GPIO8);              /* LED2 on/off. */
	if (timer_get_flag(TIM2, TIM_SR_UIF))
        timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous write */
	timer_disable_irq(TIM2, TIM_DIER_UIE);
	timer_disable_counter(TIM2);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gpio_setup();
	timer_setup();


	/*
	 * The goal is to let the LED2 glow for a second and then be
	 * off for a second.
	 */

	while (1) /* Halt. */
	{
		/* Update interrupt enable. */
		timer_enable_irq(TIM2, TIM_DIER_UIE);

		/* Start timer. */
		timer_enable_counter(TIM2);

//		gpio_toggle(GPIOB, GPIO9);	/* LED on/off */
/* Delay for 1 second LED flashes */
//		int i;
//		for (i = 0; i < 6400000; i++)	/* Wait a bit. */
//			__asm__("nop");
	}

	return 0;
}
