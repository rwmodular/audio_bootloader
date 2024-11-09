#include "pico/stdlib.h"

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

	while (true) {
		gpio_put(led_pins[cur_pin], 1);
		sleep_ms(200);
		gpio_put(led_pins[cur_pin], 0);
		sleep_ms(200);

		while(true) {
			for(int i=0; i<3; i++) {
				pico_set_led(true);
        		sleep_ms(200);
        		pico_set_led(false);
        		sleep_ms(200);
			}
			sleep_ms(3000);
		}
	}
}
