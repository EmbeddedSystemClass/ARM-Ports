/* PWM test for SMPS work
 *
 * Set GPIO ports for advanced timer 3 to allow alternate function,
 * output, initially push-pull but eventually open drain.
 *
 * Set timer 3 to PWM mode, centre aligned, 62.5kHz. Set a deadtime.
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

#define PERIOD 1152

void hardware_setup(void)
{
/* Setup the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

/* Enable GPIOB and GPIOA clocks.
   APB2 (High Speed Advanced Peripheral Bus) peripheral clock enable register (RCC_APB2ENR)
   Set RCC_APB2ENR_IOPAEN for port A and RCC_APB2ENR_IOPAEN for Alternate Function clock */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);

/* Enable TIM3 clock. */
	rcc_periph_clock_enable(RCC_TIM3);

/* Set port PA6 to 'alternate function output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);

/* Reset TIM3 peripheral. */
	timer_reset(TIM3);

/* Set Timer global mode:
 * - No divider
 * - Edge mode
 * - Direction up (when centre mode is set it is read only, changes by hardware)
 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

/* Continuous mode. */
//	timer_continuous_mode(TIM3);

/* Set Timer output compare mode:
 * - Channel 1
 * - PWM mode 1 (output low when CNT < CCR1, high otherwise)
 */
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);

/* The ARR (auto-preload register) sets the PWM period to 62.5kHz from the
72 MHz clock.*/
	timer_enable_preload(TIM3);
	timer_set_period(TIM3, 1000);

	timer_enable_oc_output(TIM3, TIM_OC1);

/* The CCR1 (capture/compare register 1) sets the PWM duty cycle to default 50% */
	timer_enable_oc_preload(TIM3, TIM_OC1);
	timer_set_oc_value(TIM3, TIM_OC1, 500);

/* Counter enable. */
	timer_enable_counter(TIM3);

}

int main(void)
{
	hardware_setup();

	while (1) {
	}

	return 0;
}
