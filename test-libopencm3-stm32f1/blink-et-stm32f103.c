/* STM32F1 Test of GPIO function

Tests:
GPIO basic output

The board used is the ET-STM32F103 with LEDs on port B pins 8-15

The LEDs on ports GPIO8 and GPIO9 are flashed in unison.

*/
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/scb.h>

void gpio_setup(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		        GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
                GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

int main(void)
{
	int i,j=0;
    int period1 = 400000;
    int period2 = 1600000;

	gpio_setup();

	while (1) {
		gpio_toggle(GPIOB, GPIO8);	/* LED on/off */
		for (i = 0; i < period1; i++)	/* Wait a bit. */
			__asm__("nop");
		gpio_toggle(GPIOB, GPIO9);	/* LED on/off */
		for (i = 0; i < period1; i++)	/* Wait a bit. */
			__asm__("nop");
		gpio_toggle(GPIOB, GPIO8);	/* LED on/off */
		for (i = 0; i < period1; i++)	/* Wait a bit. */
			__asm__("nop");
		gpio_toggle(GPIOB, GPIO9);	/* LED on/off */
		for (i = 0; i < period2; i++)	/* Wait a bit. */
			__asm__("nop");
        if (j++ > 10)
        {
            period1 = 200000;
            period2 = 800000;
        }
        else
        {
            period1 = 400000;
            period2 = 1600000;
        }
        if (j > 20) scb_reset_system();
	}

	return 0;
}
