#include "bma421.h"

#include "drivers/I2C.h"

uint16_t ptime_accel_init(struct bma4_dev *dev, mbed::I2C *i2c);
void ptime_accel_print_rslt(uint16_t rslt);
