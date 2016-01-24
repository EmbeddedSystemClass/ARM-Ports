/* STM32F4 Test of ADC function

Blink LED at a different rate with analogue control.

Tests:
ADC basic direct conversion start and direct register read
ADC IRQ
GPIO basic output

The board used is the STM32F4-discovery with LEDs on port D pins 12-15
An analogue signal is inserted into pin PA1 (ADC123 IN1).

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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

uint16_t value = 500;

/*--------------------------------------------------------------------------*/

void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

/*--------------------------------------------------------------------------*/

void gpio_setup(void)
{
/* GPIO LED ports */
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/

void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
/* Set port PA1 for ADC1 to analogue mode. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
/* Setup the ADC */
	nvic_enable_irq(NVIC_ADC_IRQ);
	uint8_t channel[1] = { ADC_CHANNEL1 };
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
	adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_set_regular_sequence(ADC1, 1, channel);
	adc_enable_eoc_interrupt(ADC1);
    adc_power_on(ADC1);
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
        adc_start_conversion_regular(ADC1);
		uint32_t count = 195*value;
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOD, GPIO12);
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOD, GPIO13);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

void adc_isr(void)
{
        value = adc_read_regular(ADC1);
}

