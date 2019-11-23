#ifndef BMA421_ACCELEROMETER
#define BMA421_ACCELEROMETER

#include "bma4_defs.h"

#include "drivers/I2C.h"

// struct _

class BMA421_Accelerometer
{
  public:
    BMA421_Accelerometer(mbed::I2C *i2c);
    ~BMA421_Accelerometer();
    void enable();
    void set_config();

  protected:
  	struct bma4_dev bma;
  	mbed::I2C *_i2c;


    void delay_ms(uint32_t period);
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    void print_rslt(uint16_t rslt);
};

#endif