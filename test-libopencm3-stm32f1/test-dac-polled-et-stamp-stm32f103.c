/* DAC test with DMA and timer 2 trigger

A Digital test output is provided on PC0 for triggering
Timer 2 is setup without output to provide a timed interrupt.
PA4 is provided for the DAC1 channel 1 output.

The DAC is setup with software trigger.
Timer 2 ISR is used to move preset array data to the DAC.

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

#define PERIOD 1152

/*--------------------------------------------------------------------------*/

/* Globals */
uint32_t cntr;
uint8_t v[256];

/*--------------------------------------------------------------------------*/

void clock_setup(void)
{
/* Setup the clock to 72MHz from the 8MHz external crystal */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/*--------------------------------------------------------------------------*/

void gpio_setup(void)
{
/* Digital Test output PC0 */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
}

/*--------------------------------------------------------------------------*/

void timer_setup(void)
{
/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);
/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_reset(TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
/* Continous mode. */
	timer_continuous_mode(TIM2);
	timer_set_period(TIM2, 1000);
/* Disable outputs. */
	timer_disable_oc_output(TIM2, TIM_OC1 | TIM_OC2 | TIM_OC3 | TIM_OC4);
/* Configure global mode of output channel 1, disabling the output. */
	timer_disable_oc_clear(TIM2, TIM_OC1);
	timer_disable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_slow_mode(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);
/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, 1000);
/* ARR reload disable. */
	timer_disable_preload(TIM2);
/* Counter enable. */
	timer_enable_counter(TIM2);
/* Enable commutation interrupt. */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

/*--------------------------------------------------------------------------*/

void dac_setup(void)
{
/* Set port PA4 for DAC1 to 'alternate function'. Output driver mode is ignored. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);
/* Enable the DAC clock on APB1 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
/* Setup the DAC, software trigger source. Assume the DAC has
woken up by the time the first interrupt occurs */
/*	dac_disable(CHANNEL_1);
	dac_set_waveform_characteristics(DAC_CR_MAMP1_2);
	dac_set_waveform_generation(DAC_CR_WAVE1_NOISE); */
	dac_enable(CHANNEL_1);
	dac_trigger_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
	dac_load_data_buffer_single(0, RIGHT8, CHANNEL_1);
}

/*--------------------------------------------------------------------------*/

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF))
	{
/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

/* Set the next DAC output - saw-tooth.*/
		if (++cntr > 255)
		{
			cntr=0;
/* Toggle PC0 just to keep aware of activity and frequency. */
			gpio_toggle(GPIOC, GPIO1);
		}
/* Trigger the DAC then load the next value */
		dac_software_trigger(CHANNEL_1);
		dac_load_data_buffer_single(v[cntr], RIGHT8, CHANNEL_1);
	}

}

/*--------------------------------------------------------------------------*/

int main(void)
{
	uint32_t i;
	for (i=0; i<256; i++)
	{
		if (i<10) v[i]=10;
		else if (i<121) v[i]=10+((i*i)>>7);
		else if (i<128) v[i]=128;
		else if (i<246) v[i]=256-i;	
		else v[i]=10;
	}
	cntr = 0;
	clock_setup();
	gpio_setup();
	timer_setup();
	dac_setup();

	while (1);

	return 0;
}
