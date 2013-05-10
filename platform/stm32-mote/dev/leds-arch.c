#include "dev/models.h"
#include "dev/leds.h"

void
leds_arch_init(void)
{
	gpio_set_mode(LED_GPIO, GPIO_MODE_OUTPUT_2_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, LED_RED|LED_YELLOW|LED_GREEN);
}

unsigned char
leds_arch_get(void)
{
	return ((gpio_get(LED_GPIO, LED_RED)) ? 0 : LEDS_RED)
		| ((gpio_get(LED_GPIO, LED_GREEN)) ? 0 : LEDS_GREEN)
		| ((gpio_get(LED_GPIO, LED_YELLOW)) ? 0 : LEDS_YELLOW);
}

void
leds_arch_set(unsigned char leds)
{
	if(leds & LEDS_YELLOW) {
		gpio_set(LED_GPIO, LED_YELLOW);
	} else {
		gpio_clear(LED_GPIO, LED_YELLOW);
	}
	if(leds & LEDS_GREEN) {
		gpio_set(LED_GPIO, LED_GREEN);
	} else {
		gpio_clear(LED_GPIO, LED_GREEN);
	}
	if(leds & LEDS_RED) {
		gpio_set(LED_GPIO, LED_RED);
	} else {
		gpio_clear(LED_GPIO, LED_RED);
	}
}
