#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/dispatch/nvic.h>

u16 value = 500;

/*--------------------------------------------------------------------------*/

void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

/*--------------------------------------------------------------------------*/

void gpio_setup(void)
{
/* GPIO LED ports */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
		      GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/

void adc_setup(void)
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
/* Set port PA1 for ADC1 to analogue mode. */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
/* Setup the ADC */
    nvic_enable_irq(NVIC_ADC_IRQ);
    u8 channel[1] = { ADC_CHANNEL1 };
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR1_SMP_1DOT5CYC);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_set_injected_sequence(ADC1, 1, channel);
    adc_enable_eoc_interrupt_injected(ADC1);
    adc_power_on(ADC1);
}

/*--------------------------------------------------------------------------*/

int main(void)
{
	u32 i;

	clock_setup();
	gpio_setup();
	adc_setup();
/* Read ADC */
	while (1) {
        adc_start_conversion_injected(ADC1);
		u32 count = 195*value;
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOD, GPIO12);
		for (i = 0; i < count; i++) __asm__("nop");
		gpio_toggle(GPIOD, GPIO13);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

void adc_isr(void)
{
    ADC_SR(ADC1) &= ~ADC_SR_JEOC;
    value = adc_read_injected(ADC1, 4);
/* Clear Injected End Of Conversion (JEOC) */
		gpio_toggle(GPIOD, GPIO14);

}

