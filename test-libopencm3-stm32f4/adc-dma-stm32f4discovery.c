/* STM32F4 Test of ADC function

Blink LED at a different rate with analogue control.

The ADC is setup for continuous conversion of a single channel. DMA is
setup in circular mode to fill an array. The first element of the array
is taken to pace the flashing of the LEDs.

STM32F4-Discovery board.
A variable voltage is placed at PA1 to adjust the analogue input.
Two LEDs D12 and D13 are flashed.
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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

uint32_t v[128];
uint16_t cntr;

/*--------------------------------------------------------------------*/
void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

/*--------------------------------------------------------------------*/
void gpio_setup(void)
{
/* Clocks on AHB1 for GPIO D (LEDs) and C (monitor) */
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOC);
/* Digital Test output PC1 */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0 | GPIO1);
/* GPIO LED ports */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------*/
void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_GPIOA);
	nvic_enable_irq(NVIC_ADC_IRQ);
/* Set port PA1 for ADC1 to analogue mode. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    adc_power_on(ADC1);
	uint8_t channel[1] = { ADC_CHANNEL1 };
    adc_set_regular_sequence(ADC1, 1, channel);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_3CYC);
	adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
	adc_set_dma_continue(ADC1);
	adc_enable_dma(ADC1);
	adc_enable_overrun_interrupt(ADC1);
}

/*--------------------------------------------------------------------*/
void dma_setup(void)
{
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
/* ADC1 uses DMA controller 2 Stream 0 channel 0. */
/* Enable DMA2 clock and IRQ */
	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
	dma_stream_reset(DMA2,DMA_STREAM0);
	dma_set_priority(DMA2,DMA_STREAM0,DMA_SxCR_PL_LOW);
	dma_set_peripheral_size(DMA2,DMA_STREAM0,DMA_SxCR_PSIZE_32BIT);
/* The register to target is the ADC regular data register */
	dma_set_peripheral_address(DMA2,DMA_STREAM0,(uint32_t) &ADC1_DR);
/* The array v[] is filled with the waveform data to be output */
	dma_set_memory_size(DMA2,DMA_STREAM0,DMA_SxCR_MSIZE_32BIT);
	dma_set_memory_address(DMA2,DMA_STREAM0,(uint32_t) v);
	dma_set_number_of_data(DMA2,DMA_STREAM0,64);
	dma_set_transfer_mode(DMA2,DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_enable_memory_increment_mode(DMA2,DMA_STREAM0);
	dma_enable_circular_mode(DMA2,DMA_STREAM0);
/* Don't use FIFO */
	dma_enable_direct_mode(DMA2,DMA_STREAM0);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
	dma_enable_stream(DMA2,DMA_STREAM0);
}

/*--------------------------------------------------------------------*/
int main(void)
{
	uint32_t i,j;

	cntr=4;
	clock_setup();
	gpio_setup();
	adc_setup();
	dma_setup();
/* Start of the ADC. Should continue indefinitely with DMA in circular mode */
    adc_start_conversion_regular(ADC1);
	while (1) {
/* Blink the LED (PB8, PB9) on the board. */
		uint32_t count = 500*v[1];
		gpio_toggle(GPIOD, GPIO12);
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOD, GPIO13);
		for (i = 0; i < count; i++) __asm__("nop");
	}

	return 0;
}

/*--------------------------------------------------------------------*/
void dma2_stream0_isr(void)
{
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF))
	{
		dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
	}
	if (++cntr > 63)
	{
		cntr=0;
	}
}

/*--------------------------------------------------------------------*/
/* Check for overrun errors */
void adc_isr(void)
{
	if (adc_get_overrun_flag(ADC1))
	{
		adc_clear_overrun_flag(ADC1);
	}
	gpio_set(GPIOD, GPIO15);
}

