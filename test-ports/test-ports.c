/* STM32F1 test of digital output ports

Initially tested with the ET-ARM-STAMP (STM32F103RET6).

Copyright K. Sarkies <ksarkies@internode.on.net>

10 February 2018
*/

/*
 * Copyright 2018 K. Sarkies <ksarkies@internode.on.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>

/* Prototypes */

static void gpio_setup(void);
static void clock_setup(void);
static void delay(uint32_t period);

/* Globals */

/*--------------------------------------------------------------------------*/

int main(void)
{
	clock_setup();
	gpio_setup();

	while (1)
	{
/* Start sequence by turning all on then off */
        gpio_set(GPIOA,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_set(GPIOB,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_set(GPIOC,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
        gpio_set(GPIOD,
		    GPIO2);
        delay(1600000);
        gpio_clear(GPIOA,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_clear(GPIOB,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_clear(GPIOC,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
        gpio_clear(GPIOD,
		    GPIO2);
        delay(1600000);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Delay

*/

void delay(uint32_t period)
{
    uint32_t i;
	for (i = 0; i < period; i++)	    /* Wait a bit. */
		__asm__("nop");
}

/*--------------------------------------------------------------------------*/
/** @brief Clock Setup

The processor system clock is established and the necessary peripheral
clocks are turned on.
*/

void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/*--------------------------------------------------------------------------*/
/** @brief GPIO Setup

Setup GPIO Ports A, B, C, D as outputs.
*/

void gpio_setup(void)
{
/* Enable all GPIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_AFIO);  /* Must enable this to allow remap */

/* Disable SWD and JTAG to allow full use of the ports PA13, PA14, PA15 */
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,0);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_clear(GPIOA,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_clear(GPIOB,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
    gpio_clear(GPIOC,
		    GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);

    gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		    GPIO2);
    gpio_clear(GPIOD, GPIO2);
}

