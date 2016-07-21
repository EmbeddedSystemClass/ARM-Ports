/* DAC test using timer interrupt and software trigger.

A Digital test output is provided on PC0 for triggering
Timer 2 is setup without output to provide a timed interrupt and
move preset data to the DAC.

PA4 is provided for the DAC1 output on channel 1.

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
#include <libopencm3/stm32/adc.h>

#define PERIOD 1152

/*--------------------------------------------------------------------*/
/* Globals */
uint32_t cntr;
uint8_t v[256], y[256];

/*--------------------------------------------------------------------*/
void hardware_setup(void)
{
/* Setup the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

/* Enable GPIOA, GPIOB and GPIOC clocks.
   APB2 (High Speed Advanced Peripheral Bus) peripheral clock enable register (RCC_APB2ENR)
   Set RCC_APB2ENR_IOPBEN for port B, RCC_APB2ENR_IOPAEN for port A and RCC_APB2ENR_IOPAEN
   for Alternate Function clock */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

/* Digital Test output PC0 */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);

/* ----------------- Timer 2 Interrupt and DAC control*/

/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);
/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_reset(TIM2);
/* Timer global mode:
 * - No divider
 * - Alignment edge
 * - Direction up
 */
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

/* Set port PA4 for DAC1 to 'alternate function'. Output driver mode is ignored. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);

/* Enable the DAC clock on APB1 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);

/* Setup the DAC, software trigger source. Assume the DAC has
woken up by the time the first interrupt occurs */
	dac_trigger_enable(CHANNEL_D);
	dac_set_trigger_source(DAC_CR_TSEL1_SW | DAC_CR_TSEL2_SW);
	dac_enable(CHANNEL_D);
	dac_load_data_buffer_dual(0, 0, RIGHT8);

}

/*--------------------------------------------------------------------*/
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
			gpio_toggle(GPIOC, GPIO0);
		}
/* Trigger the DAC then load the next value */
		dac_software_trigger(CHANNEL_D);
		dac_load_data_buffer_single(v[cntr], RIGHT8, CHANNEL_D);
	}

}

/*--------------------------------------------------------------------*/
int main(void)
{
	uint32_t i, x;
	for (i=0; i<256; i++)
	{
		if (i<10) x=10;
		else if (i<121) x=10+((i*i)>>7);
		else if (i<128) x=128;
		else if (i<246) x=256-i;	
		else x=10;
		v[i] = x | (i << 8);
	}
	cntr = 0;
	hardware_setup();

	while (1) {

	}

	return 0;
}
