#include "nrf_delay.h"
#include "nrf_gpio.h"

void blink(void)
{
	nrf_gpio_cfg_output(10);
	nrf_gpio_pin_set(10);

	while (1)
	{
		nrf_gpio_pin_toggle(10);
		nrf_delay_ms(500);
	}
}
