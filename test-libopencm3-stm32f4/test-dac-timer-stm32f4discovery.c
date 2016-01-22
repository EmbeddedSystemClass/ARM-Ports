/* DAC test with DMA and timer 2 trigger

The DAC is setup with timer 2 trigger using the OC1 output.
DMA is used to move data from a predefined array in circular mode.
DMA ISR on transfer complete is used to toggle a port for CRO trigger.
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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/f4/dma.h>

#define PERIOD 1152

/* Globals */
u32 cntr;
u8 v[256];

/*--------------------------------------------------------------------*/
void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

/*--------------------------------------------------------------------*/
void gpio_setup(void)
{
/* Ports A and C are on AHB1 */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPCEN);
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
/* Enable TIM2 clock. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
	nvic_enable_irq(NVIC_TIM2_IRQ);
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
/* Set the timer trigger output TRGO (for the DAC) to the channel 1 output compare */
	timer_set_master_mode(TIM2, TIM_CR2_MMS_COMPARE_OC1REF);
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
	timer_set_dma_on_compare_event(TIM2);
	TIM2_DIER |= TIM_DIER_CC1DE;
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------*/
void dma_setup(void)
{
/* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
/* To use the timer, change to stream 5 channel 3, disable DMA in DAC and enable in timer */
/* Enable DMA1 clock and IRQ */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA1EN);
	nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
	dma_stream_reset(DMA1,DMA_STREAM5);
/* The register to target is the DAC1 8-bit right justified data register */
	dma_set_peripheral_address(DMA1,DMA_STREAM5,(u32) &DAC_DHR8R1);
	dma_set_memory_address(DMA1,DMA_STREAM5,(u32) v);
/* The array v[] has been filled with the waveform data to be output */
	dma_set_number_of_data(DMA1,DMA_STREAM5,256);
//	dma_channel_select(DMA1, DMA_STREAM5, DMA_SCR_CHSEL_7);	/* DAC1 */
	dma_channel_select(DMA1, DMA_STREAM5, DMA_SCR_CHSEL_3); /* TIM2 CC1 */
	dma_set_priority(DMA1,DMA_STREAM5,DMA_SCR_PL_LOW);
	dma_enable_fifo_mode(DMA1,DMA_STREAM5);	/* not needed, use direct mode OK */
	dma_set_dma_flow_control(DMA1,DMA_STREAM5);
	dma_set_memory_size(DMA1,DMA_STREAM5,DMA_SCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1,DMA_STREAM5,DMA_SCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1,DMA_STREAM5);
	dma_enable_circular_mode(DMA1,DMA_STREAM5);
	dma_set_transfer_mode(DMA1,DMA_STREAM5, DMA_SCR_DIR_MEM2PER);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
	dma_enable_stream(DMA1,DMA_STREAM5);
}

/*--------------------------------------------------------------------*/
void dac_setup(void)
{
/* Enable the DAC clock on APB1 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
/* Set port PA4 for DAC1 to analogue output. Output driver mode is ignored. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
/* Setup the DAC channel 1, with timer 2 as trigger source. Assume the DAC has
woken up by the time the first transfer occurs */
	dac_trigger_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_T2);
//	dac_dma_enable(CHANNEL_1);
	dac_enable(CHANNEL_1);
}

/*--------------------------------------------------------------------*/
/* The ISR simply provides a test output for a CRO trigger */

void dma1_stream5_isr(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_ISR_TCIF))
	{
		dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_ISR_TCIF);
/* Toggle PC0 just to keep aware of activity and frequency. */
		gpio_toggle(GPIOC, GPIO1);
	}
	gpio_toggle(GPIOD, GPIO12);
}

/*--------------------------------------------------------------------*/
void tim2_isr(void)
{
	gpio_toggle(GPIOD, GPIO13);   /* LED2 on/off. */
	if (timer_get_flag(TIM2, TIM_SR_CC1IF))
	{
		timer_clear_flag(TIM2, TIM_SR_CC1IF); /* Clear interrrupt flag. */
/* Set the next DAC output - saw-tooth.*/
		if (++cntr > 255)
		{
			cntr=0;
/* Toggle PC0 just to keep aware of activity and frequency. */
			gpio_toggle(GPIOC, GPIO0);
		}
	}
/* Reread to force the previous (buffered) write before leaving */
	timer_get_flag(TIM2, TIM_SR_CC1IF);
/* Load data buffer (no DMA) but rely on TRGO trigger to put it to the output. */
//	dac_load_data_buffer_single(v[cntr], RIGHT8, CHANNEL_1);
}

/*--------------------------------------------------------------------*/
int main(void)
{
/* Fill the array with funky waveform data */
/* This is for dual channel 8-bit right aligned */
	u32 i, x;
	for (i=0; i<256; i++)
	{
		if (i<10) x=10;
		else if (i<121) x=10+((i*i)>>7);
		else if (i<128) x=128;
		else if (i<246) x=256-i;	
		else x=10;
		v[i] = x;
	}
	cntr = 0;
	clock_setup();
	gpio_setup();
	timer_setup();
	dma_setup();
	dac_setup();

	while (1) {

	}

	return 0;
}
