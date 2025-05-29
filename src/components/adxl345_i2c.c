#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/i2c_types.h>
#include <driver/i2c_master.h>
#include <driver/i2c_slave.h>

#include "components/adxl345_i2c.h"

static i2c_master_dev_handle_t dev_handle;

void i2c_init()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = ADXL345_DEV_ADDR_LENGTH,
        .device_address = ADXL345_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void adxl345_init()
{
    uint8_t data_format = 0x01; // ±4g, 10-bit
    uint8_t power_ctl = 0x08;   // Measurement mode

    adxl345_write_val_to_reg(0x31, data_format);
    adxl345_write_val_to_reg(0x2D, power_ctl);
}

void adxl345_write_val_to_reg(uint8_t reg, uint8_t data){
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle,
                                        (uint8_t[]){reg, data}, 2, -1));
}

void adxl345_read_val_from_reg(uint8_t reg, uint8_t* data){
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, -1));
}

void adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t reg = 0x32; // DATA x0
    static int32_t DATA_LENGTH = 6;
    uint8_t data_rd[DATA_LENGTH];
    // Multibyte request
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data_rd, DATA_LENGTH, -1));
    *x = (int16_t)((data_rd[1] << 8) | data_rd[0]);
    *y = (int16_t)((data_rd[3] << 8) | data_rd[2]);
    *z = (int16_t)((data_rd[5] << 8) | data_rd[4]);
}

void adxl345_test_console_log()
{
    i2c_init();
    adxl345_init();

    int16_t x, y, z;

    while (1)
    {
        adxl345_read_xyz(&x, &y, &z);

        // printf(">x:%i|g\n", x); // Teleplot format
        // printf(">y:%i|g\n", y);
        // printf(">z:%i|g\n", z);

        // Convert to G
        float xg = x * CONVERT_RAW_TO_G_COEF;
        float yg = y * CONVERT_RAW_TO_G_COEF;
        float zg = z * CONVERT_RAW_TO_G_COEF;

        printf(">x:%f|g\n", xg); // Teleplot format
        printf(">y:%f|g\n", yg);
        printf(">z:%f|g\n", zg);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
