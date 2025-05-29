#include "workshop/hello_world.h"
#include "workshop/accelometer.h"

#include "components/adxl345_i2c.h"
#include "homeworks/interrupts.h"

void app_main(void)
{
    // hello_world();
    // accelometer();
    // lamp();

    adxl345_test_console_log();
    // interrupts_test();
}