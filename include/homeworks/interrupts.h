#include "freertos/FreeRTOS.h"

#ifndef XYZ_LEDS_H
#define XYZ_LEDS_H

#define GPIO_INT_1 17
#define GPIO_INT_2 18

#define PWM_FREQ 1000
#define OUTPUT_PIN 12

void init_esp32_interrupts();

void init_adxl345_interrupt_map();

void interrupts_test();

#endif // XYZ_LEDS_H