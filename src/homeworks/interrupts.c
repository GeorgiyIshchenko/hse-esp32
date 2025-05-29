#include "homeworks/interrupts.h"
#include "components/adxl345_i2c.h"

#include <freertos/FreeRTOS.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>

#include <string.h>

// static task
static TaskHandle_t s_dataTask;

static volatile bool data_ready = false;
static volatile bool tap_flag = false;
static int16_t x, y, z;

static IRAM_ATTR void read_xyz_task()
{
    // sleep until notifying
    int led_on = 0;
    int i = 0;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        adxl345_read_xyz(&x, &y, &z);
        data_ready = true;

        
        if (tap_flag)
        {
            uint8_t status;
            adxl345_read_val_from_reg(0x30, &status);
            tap_flag = false;
            printf("Single tap %i:\n", i++);
            led_on ^= 1;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, led_on ? 255 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static IRAM_ATTR void int1_interrupt()
{
    BaseType_t xHigherWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_dataTask, &xHigherWoken);
    if (xHigherWoken)
        portYIELD_FROM_ISR();
}

static IRAM_ATTR void int2_interrupt()
{
    tap_flag = true;
}

void init_esp32_interrupts()
{
    esp_intr_dump(NULL);
    gpio_reset_pin(GPIO_INT_1);
    gpio_set_direction(GPIO_INT_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_INT_1, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_INT_1, GPIO_INTR_POSEDGE);
    gpio_reset_pin(GPIO_INT_2);
    gpio_set_direction(GPIO_INT_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_INT_2, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_INT_2, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INT_1, int1_interrupt, NULL);
    gpio_isr_handler_add(GPIO_INT_2, int2_interrupt, NULL);
}

void init_adxl345_interrupt_map()
{
    // I want to on SINGLE_TAP and DATA_READY INTERRUPTS
    uint8_t DATA_READY = 1 << 7;
    uint8_t ACTIVITY = 1 << 6; // THIS IS SINGLE_TAP
    // uint8_t ACTIVITY = 1 << 4;
    // I USE ACTIVITY BOTH FOR ACTIVITY AND SINGLE_TAP

    adxl345_write_val_to_reg(0x2E, 0x00);

    // DATA_READY - INT1 -> SET TO 0
    // SINGLE_TAP - INT2 -> SET TO 1
    adxl345_write_val_to_reg(0x2F, ACTIVITY);

    // ACTIVITY SETTINGS
    // adxl345_write_val_to_reg(0x24, 0x20); // THRESH_ACT
    // adxl345_write_val_to_reg(0x27, 0x70); // ACT_INACT_CTL

    // TAP SETTINGS
    adxl345_write_val_to_reg(0x1D, 0x30); // THRESH_TAP
    adxl345_write_val_to_reg(0x21, 0x20); // DUR
    adxl345_write_val_to_reg(0x2A, 0x05); // TAP_AXES

    adxl345_write_val_to_reg(0x2E, DATA_READY | ACTIVITY);
}

static void init_ledc()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ch0_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = OUTPUT_PIN,
        .duty = 128,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ch0_config));
}

void interrupts_test()
{
    // Init task
    xTaskCreatePinnedToCore(read_xyz_task, "READ XYZ TASK", 2048, NULL, 1, &s_dataTask, tskNO_AFFINITY);

    // ledc_fade_func_install(0);
    init_ledc();

    i2c_init();
    adxl345_init();
    init_esp32_interrupts();
    init_adxl345_interrupt_map();

    while (1)
    {
        if (data_ready)
        {
            float xg = x * CONVERT_RAW_TO_G_COEF;
            float yg = y * CONVERT_RAW_TO_G_COEF;
            float zg = z * CONVERT_RAW_TO_G_COEF;

            // printf(">x:%f|g\n", xg);
            // printf(">y:%f|g\n", yg);
            // printf(">z:%f|g\n", zg);

            data_ready = false;
        }

        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
