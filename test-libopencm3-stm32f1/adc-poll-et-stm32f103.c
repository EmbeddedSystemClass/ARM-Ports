/* STM32F1 Test of ADC function

Read ADC1 on port PA1 and blink LEDs on PB8 and PB9 at a different rate with
changes in an external analogue control.

Tests:
ADC basic direct conversion start and direct register read
GPIO basic output

The board used is the ET-STM32F103 with LEDs on port B pins 8-15

*/

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2012 Ken Sarkies ksarkies@internode.on.net
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/adc.h>

/*--------------------------------------------------------------------------*/

/* The processor system clock is established and the necessary peripheral
clocks are turned on */
void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/*--------------------------------------------------------------------------*/

void gpio_setup(void)
{
/* Enable AFIO, GPIOA, GPIOB and GPIOC clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
/* GPIO LED ports */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/

void adc_setup(void)
{
/* Set port PA1 for ADC1 to analogue input. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_ANALOG, GPIO1);
/* Enable the ADC1 clock on APB2 */
    rcc_periph_clock_enable(RCC_ADC1);
/* Setup the ADC */
    adc_power_on(ADC1);
    adc_calibration(ADC1);
	uint8_t channel[1] = { ADC_CHANNEL1 };
    adc_set_regular_sequence(ADC1, 1, channel);
}

/*--------------------------------------------------------------------------*/

int main(void)
{
	uint32_t i;

	clock_setup();
	gpio_setup();
	adc_setup();
/* Read ADC */
	while (1) {
        adc_start_conversion_direct(ADC1);
        while (!adc_eoc(ADC1));
        uint16_t value = adc_read_regular(ADC1);
//    }
/* Blink the LED (PB8, PB9) on the board rate proportional to adc output. */
//	while (1) {
		uint32_t count = 195*value;
		gpio_toggle(GPIOB, GPIO8);
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOB, GPIO9);
		for (i = 0; i < count; i++) __asm__("nop");
	}

	return 0;
}
