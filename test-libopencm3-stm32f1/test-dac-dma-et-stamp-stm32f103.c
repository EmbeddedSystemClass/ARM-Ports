/* DAC test with DMA and timer 2 trigger

The DAC is setup with timer 2 trigger using the OC1 output.
DMA is used to move data from a predefined array in circular mode.
DMA ISR on transfer complete is used to toggle a port for CRO trigger.

Smaller STM32F103 devices as in the ET dev board do not have DAC.
*/

/*
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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>

#define PERIOD 1152

/* Globals */
uint8_t v[256];

/*--------------------------------------------------------------------*/
void clock_setup(void)
{
/* Setup the clock to 72MHz from the 8MHz external crystal */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/*--------------------------------------------------------------------*/
void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
/* Digital Test outputs PC0 and PC1 */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1);
}

/*--------------------------------------------------------------------*/
void timer_setup(void)
{
/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);
	timer_reset(TIM2);
/* Timer global mode: - No divider, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
	timer_set_period(TIM2, 1000);
	timer_disable_oc_output(TIM2, TIM_OC2 | TIM_OC3 | TIM_OC4);
	timer_enable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_clear(TIM2, TIM_OC1);
	timer_disable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_slow_mode(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_TOGGLE);
	timer_set_oc_value(TIM2, TIM_OC1, 500);
	timer_disable_preload(TIM2);
/* Set the timer trigger output (for the DAC) to the channel 1 output compare */
	timer_set_master_mode(TIM2, TIM_CR2_MMS_COMPARE_OC1REF);
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------*/
void dma_setup(void)
{
/* DAC channel 1 shares DMA controller 2 Channel 3. */
/* Enable DMA2 clock and IRQ */
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);
	nvic_enable_irq(NVIC_DMA2_CHANNEL3_IRQ);
	dma_channel_reset(DMA2,DMA_CHANNEL3);
	dma_set_priority(DMA2,DMA_CHANNEL3,DMA_CCR_PL_LOW);
	dma_set_memory_size(DMA2,DMA_CHANNEL3,DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA2,DMA_CHANNEL3,DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA2,DMA_CHANNEL3);
	dma_enable_circular_mode(DMA2,DMA_CHANNEL3);
	dma_set_read_from_memory(DMA2,DMA_CHANNEL3);
/* The register to target is the DAC1 8-bit right justified data register */
	dma_set_peripheral_address(DMA2,DMA_CHANNEL3,(uint32_t) &DAC_DHR8R1);
/* The array v[] is filled with the waveform data to be output */
	dma_set_memory_address(DMA2,DMA_CHANNEL3,(uint32_t) v);
	dma_set_number_of_data(DMA2,DMA_CHANNEL3,256);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_CHANNEL3);
	dma_enable_channel(DMA2,DMA_CHANNEL3);
}

/*--------------------------------------------------------------------*/
void dac_setup(void)
{
/* Enable the DAC clock on APB1 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
/* Set port PA4 for DAC1 output to 'alternate function'. Output driver mode is irrelevant. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);
/* Setup the DAC channel 1, with timer 2 as trigger source. Assume the DAC has
woken up by the time the first transfer occurs */
	dac_trigger_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_T2);
	dac_dma_enable(CHANNEL_1);
	dac_enable(CHANNEL_1);
}

/*--------------------------------------------------------------------*/
/* The ISR simply provides a test output for a CRO trigger */

void dma2_channel3_isr(void)
{
	if ((DMA2_ISR & DMA_ISR_TCIF3) != 0)
	{
		DMA2_IFCR |= DMA_IFCR_CTCIF3;
/* Toggle PC0 just to keep aware of activity and frequency. */
			gpio_toggle(GPIOC, GPIO1);
	}
}

/*--------------------------------------------------------------------*/
int main(void)
{
/* Fill the array with funky waveform data */
/* This is for dual channel 8-bit right aligned */
	uint32_t i, x;
	for (i=0; i<256; i++)
	{
		if (i<10) x=10;
		else if (i<121) x=10+((i*i)>>7);
		else if (i<128) x=128;
		else if (i<246) x=256-i;	
		else x=10;
		v[i] = x;
	}
	clock_setup();
	gpio_setup();
	timer_setup();
	dma_setup();
	dac_setup();

	while (1) {

	}

	return 0;
}
