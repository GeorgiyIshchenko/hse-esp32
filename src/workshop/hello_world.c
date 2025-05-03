#include "workshop/hello_world.h"

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define BLINK_GPIO 12

void hello_world(){
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    bool state = 0;
    while (1) {
        gpio_set_level(BLINK_GPIO, state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
