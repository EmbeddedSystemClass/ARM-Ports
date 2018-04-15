/* STM32F1 test of digital output ports

Ports are all set as outputs and pulsed to provide identification of pin
or soldering failure.

The ports alternate are set to inputs and the remaining outputs are pulsed with
a short pulse. The inputs should then show a single pulse while the outputs will
show a pulse followed by a shorter pulse. If a short exists between adjacent
pins then the input pin will show an apparent extra pulse.

This is based on the LQFP64 version of the STM32Fxxx pinout as used on the
STAMP board.

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
#include <libopencm3/stm32/pwr.h>
#include <stdint.h>
#include <stdbool.h>

/* Prototypes */

static void gpio_setup_inputs(void);
static void gpio_setup_outputs(void);
static void clock_setup(void);
static void delay(uint32_t period);

/* 24MHz PLL from 8MHz HSE */
const struct rcc_clock_scale clock_config_hse_24 = {
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL6,
	.pll_div = RCC_CFGR_PLLDIV_DIV2,
	.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
	.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
	.voltage_scale = PWR_SCALE1,
	.flash_waitstates = 1,
	.ahb_frequency	= 24000000,
	.apb1_frequency = 24000000,
	.apb2_frequency = 24000000,
};

/* 32MHz PLL from 8MHz HSE */
const struct rcc_clock_scale clock_config_hse_32 = {
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL12,
	.pll_div = RCC_CFGR_PLLDIV_DIV3,
	.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
	.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
	.voltage_scale = PWR_SCALE1,
	.flash_waitstates = 1,
	.ahb_frequency	= 32000000,
	.apb1_frequency = 32000000,
	.apb2_frequency = 32000000,
};

/* 64MHz PLL from 8MHz HSE */
const struct rcc_clock_scale clock_config_hse_64 = {
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL16,
	.pll_div = RCC_CFGR_PLLDIV_DIV2,        /* 64MHz (96MHz max) */
	.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_HCLK_DIV2,      /* 32MHz max */
	.ppre2 = RCC_CFGR_PPRE2_HCLK_DIV2,      /* 32MHz max */
	.voltage_scale = PWR_SCALE1,            /* Highest speed */
	.flash_waitstates = 1,
	.ahb_frequency	= 64000000,
	.apb1_frequency = 36000000,
	.apb2_frequency = 36000000,
};

/* 24MHz PLL from 16MHz HSI */
const struct rcc_clock_scale clock_config_hsi_24 = {
	.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL3,
	.pll_div = RCC_CFGR_PLLDIV_DIV2,
	.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
	.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
	.voltage_scale = PWR_SCALE1,
	.flash_waitstates = 1,
	.ahb_frequency	= 24000000,
	.apb1_frequency = 24000000,
	.apb2_frequency = 24000000,
};

/* 32MHz PLL from 16MHz HSI */
const struct rcc_clock_scale clock_config_hsi_32 = {
	.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL6,
	.pll_div = RCC_CFGR_PLLDIV_DIV3,
	.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
	.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
	.voltage_scale = PWR_SCALE1,
	.flash_waitstates = 1,
	.ahb_frequency	= 32000000,
	.apb1_frequency = 32000000,
	.apb2_frequency = 32000000,
};

/* Globals */

/*--------------------------------------------------------------------------*/

int main(void)
{
    clock_setup();

    while (1)
    {
/* Start sequence by turning all on then all off.
PA10 is not exercised as it is connected to the receive output of the serial
device. It is always set as input and will always show high during the test. */
        gpio_setup_outputs();
        gpio_set(GPIOA,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_set(GPIOB,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_set(GPIOC,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
        gpio_set(GPIOD,
            GPIO2);
        delay(3200000);
        gpio_clear(GPIOA,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_clear(GPIOB,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
        gpio_clear(GPIOC,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
        gpio_clear(GPIOD,
            GPIO2);
        delay(1600000);
/* Set some as inputs to check for cross leakage between pins.
Pulse remaining pins still set as outputs. */
        gpio_setup_inputs();
        gpio_set(GPIOA,
                GPIO1 | GPIO5 | GPIO7 |
                GPIO9 | GPIO11 | GPIO13 | GPIO14);
        gpio_set(GPIOB,
                GPIO1 | GPIO3 | GPIO5 | GPIO7 |
                GPIO9 | GPIO10 | GPIO13 | GPIO15);
        gpio_set(GPIOC,
                GPIO1 | GPIO3 | GPIO5 | GPIO7 |
                GPIO9 | GPIO10 | GPIO12);
        delay(800000);
        gpio_clear(GPIOA,
                GPIO1 | GPIO5 | GPIO7 |
                GPIO9 | GPIO11 | GPIO13 | GPIO14);
        gpio_clear(GPIOB,
                GPIO1 | GPIO3 | GPIO5 | GPIO7 |
                GPIO9 | GPIO10 | GPIO13 | GPIO15);
        gpio_clear(GPIOC,
                GPIO1 | GPIO3 | GPIO5 | GPIO7 |
                GPIO9 | GPIO10 | GPIO12);
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
    for (i = 0; i < period; i++)        /* Wait a bit. */
        __asm__("nop");
}

/*--------------------------------------------------------------------------*/
/** @brief Clock Setup

The processor system clock is established and the necessary peripheral
clocks are turned on.
*/

void clock_setup(void)
{
//    rcc_clock_setup_pll(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_24MHZ]);
    rcc_clock_setup_pll(&clock_config_hse_24);

/* Enable all GPIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
}

/*--------------------------------------------------------------------------*/
/** @brief GPIO Setup Outputs

Setup all available GPIO Ports A, B, C, D as outputs.
*/

void gpio_setup_outputs(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 |GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_clear(GPIOA,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_clear(GPIOB,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);
    gpio_clear(GPIOC,
            GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 |
            GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13);

    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO2);
    gpio_clear(GPIOD, GPIO2);
}

/*--------------------------------------------------------------------------*/
/** @brief GPIO Setup as Inputs

Setup GPIO Ports A, B, C, D as inputs on alternate pins to allow for checking
of leakage between pins. This is based on the LQFP64 version of the STM32Fxxx
*/

void gpio_setup_inputs(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO2 | GPIO3 | GPIO4 | GPIO6 |
            GPIO8 | GPIO10 | GPIO12 | GPIO15);

    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO2 | GPIO4 | GPIO6 |
            GPIO8 | GPIO11 | GPIO12 | GPIO14);

    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            GPIO0 | GPIO2 | GPIO4 | GPIO6 |
            GPIO8 | GPIO11 | GPIO13);

    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
            GPIO2);
}

