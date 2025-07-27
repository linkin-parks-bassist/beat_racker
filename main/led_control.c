#include "driver/gpio.h"
#include "led_control.h"

int which_led;
int active;

int init_leds()
{
	which_led = 0;
	active = 1;
	
	gpio_set_direction(LED_PIN_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(LED_PIN_2, GPIO_MODE_OUTPUT);
	
	return 0;
}

void swap_leds()
{
	/* This looks silly; I should just write which_led = 1 - which_led,
	 * but I am accounting for the possibility that which_led has been
	 * inexplicably set to a value that is not either 0 or 1. */
	if (which_led)
		which_led = 0;
	else
		which_led = 1;
	
	update_leds();
}

void update_leds()
{
	gpio_set_level(LED_PIN_1, active && !which_led);
    gpio_set_level(LED_PIN_2, active &&  which_led);
}
