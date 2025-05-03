#include "workshop/accelometer.h"

#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include <string.h>

void accelometer()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = 19,
        .mosi_io_num = 23,
        .sclk_io_num = 18,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 8};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = 5,
        .queue_size = 1};

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_handle_t device;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &device));

    spi_transaction_t tr;
    memset(&tr, 0, sizeof(tr));
    tr.length = 16;
    tr.rxlength = 8;
    tr.tx_data[0] = 0b10000000 | 0x00;
    tr.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;

    spi_device_acquire_bus(device, portMAX_DELAY);
    ESP_ERROR_CHECK(spi_device_polling_transmit(device, &tr));
    spi_device_release_bus(device);

    printf("Result: 0x%X\n", tr.rx_data[1]);

    while (true)
    {
        vTaskDelay(10);
    }
}
