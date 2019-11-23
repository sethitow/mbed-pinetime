
#ifndef CST0XXTOUCHPAD_H
#define CST0XXTOUCHPAD_H

#include "drivers/I2C.h"
#include "platform/Callback.h"

class CST0xx_TouchPad
{
  public:
    enum event_type
    {
        EVENT_TYPE_TOUCH = 0x05,
        EVENT_TYPE_LONG_TOUCH = 0x0C,
        EVENT_TYPE_SWIPE_Y_POSITIVE = 0x01,
        EVENT_TYPE_SWIPE_Y_NEGATIVE = 0x02,
        EVENT_TYPE_SWIPE_X_NEGATIVE = 0x03,
        EVENT_TYPE_SWIPE_X_POSITIVE = 0x04
    };

    struct ts_event
    {
        enum event_type type;
        uint16_t x;
        uint16_t y;
    };

    CST0xx_TouchPad(mbed::I2C *i2c);
    CST0xx_TouchPad(mbed::I2C *i2c, mbed::Callback<void(struct ts_event)> touch_event_callback);
    void handle_interrupt();

  protected:
    mbed::I2C *_i2c;
    mbed::Callback<void(struct ts_event)> _touch_event_callback;
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
};

#endif // CST0XXTOUCHPAD_H
