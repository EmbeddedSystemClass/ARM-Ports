/* STM32F1 Test of GPIO function

The IWDG is set going for 5 seconds, and for 10 cycles it is reset
with the first LED blinking. After that, the second LED is turned on
and an infinite loop is entered.

Tests:
IWDG timeout and reset function
GPIO basic output

The board used is the ET-STM32F103 with LEDs on port B pins 8-15

*/
/* This program is free software: you can redistribute it and/or modify
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
#include <libopencm3/stm32/iwdg.h>

/*--------------------------------------------------------------------*/
void hardware_setup(void)
{
/* Setup the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15);

	rcc_osc_on(LSI);
	rcc_wait_for_osc_ready(LSI);

	iwdg_set_period_ms(5000);
	iwdg_start();
}

/*--------------------------------------------------------------------*/
int main(void)
{
	hardware_setup();
	uint32_t i=0;
	for (; i<10; i++)
	{
		uint32_t d=0;
		for (; d<2000000; d++) __asm__("nop");
		gpio_toggle(GPIOB, GPIO8);
		iwdg_reset();
	}

	gpio_set(GPIOB, GPIO9);
	
	while (1) {
	}

	return 0;
}
