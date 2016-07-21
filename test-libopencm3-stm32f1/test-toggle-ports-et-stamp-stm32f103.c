/* Toggle some ports.

This allows basic port outputs to toggle to show that they are working.
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

/*--------------------------------------------------------------------*/
/* Globals */

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

/* Digital Test output PC0 */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
/* Digital Test output PA4 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
}

/*--------------------------------------------------------------------*/
int main(void)
{
	uint32_t i;
	hardware_setup();
    uint32_t count = 4096*192;

	while (1)
    {
		gpio_toggle(GPIOC, GPIO0);
		gpio_toggle(GPIOA, GPIO4);
		for (i = 0; i < count; i++) __asm__("nop");
	}

	return 0;
}
