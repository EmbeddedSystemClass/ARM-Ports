/* STM32F1 Test of timer IRQ using Output Compare

The board used is the ET-STM32F103 with LEDs on port B pins 8-15
*/

/*
 * A Digital test output is provided on PC0 for triggering
 * Timer 3 is setup without output compare to provide a timed interrupt.
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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define PERIOD 1152

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
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
                GPIO12 | GPIO13 | GPIO14 | GPIO15);
/* Digital Test output PC0 */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
}

/*--------------------------------------------------------------------------*/
/* Set the timer with a 1us clock by setting the prescaler. The APB clock is
prescaled from the AHB clock which is itself prescaled from the system clock
(72MHz). Assume both prescales are 1. */

void timer_setup(void)
{
/* Enable TIM3 clock. */
	rcc_periph_clock_enable(RCC_TIM3);
/* Enable TIM3 interrupt. */
	nvic_enable_irq(NVIC_TIM3_IRQ);
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
/* Set prescaler to give 8us clock */
        timer_set_prescaler(TIM3, 576);
/* Set the period as 0.5 second */
	timer_set_period(TIM3, 62500);
/* Disable physical pin outputs. */
	timer_disable_oc_output(TIM3, TIM_OC1 | TIM_OC2 | TIM_OC3 | TIM_OC4);
/* Configure global mode of output channel 1, disabling the output. */
	timer_disable_oc_clear(TIM3, TIM_OC1);
	timer_disable_oc_preload(TIM3, TIM_OC1);
	timer_set_oc_slow_mode(TIM3, TIM_OC1);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FROZEN);
/* Set the compare value for OC1. */
	timer_set_oc_value(TIM3, TIM_OC1, 0);
/* Continous mode. */
	timer_continuous_mode(TIM3);
/* ARR reload disable. */
	timer_disable_preload(TIM3);
/* Counter enable. */
	timer_enable_counter(TIM3);
/* Enable commutation interrupt. */
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

/*--------------------------------------------------------------------------*/

void tim3_isr(void)
{
	if (timer_get_flag(TIM3, TIM_SR_CC1IF))
	{
/* Clear compare interrupt flag. */
		timer_clear_flag(TIM3, TIM_SR_CC1IF);
/* Toggle LED on PB8 just to keep aware of activity and frequency. */
		gpio_toggle(GPIOB, GPIO8);
	}
}

/*--------------------------------------------------------------------------*/

int main(void)
{
	cntr = 0;
	clock_setup();
	gpio_setup();
	timer_setup();

	while (1) {

	}

	return 0;
}
