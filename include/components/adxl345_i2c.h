#ifndef ADXL345_I2C_H
#define ADXL345_I2C_H

#include <driver/i2c_types.h>

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define ADXL345_ADDR 0x53 // Requires SDO grounding

#define ADXL345_READ_ADDR 0xA6 // Requires SDO grounding
#define ADXL345_WRITE_ADDR 0xA7 // Requires SDO grounding

#define ADXL345_DEV_ADDR_LENGTH I2C_ADDR_BIT_10

// FROM DOC: "±4 g, 10-bit resolution 7.8 mg/LSB"
#define CONVERT_RAW_TO_G_COEF 0.0078f

void i2c_init();
void adxl345_init();
void adxl345_write_val_to_reg(uint8_t reg, uint8_t data);
void adxl345_read_val_from_reg(uint8_t reg, uint8_t* data);
void adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z);

void adxl345_test_console_log();

#endif // ADXL345_I2C_H