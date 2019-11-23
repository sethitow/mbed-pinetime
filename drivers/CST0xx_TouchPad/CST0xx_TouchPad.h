
#ifndef CST0XXTOUCHPAD_H
#define CST0XXTOUCHPAD_H

#include "drivers/I2C.h"

#define HYN_MAX_POINTS                          10

class CST0xx_TouchPad
{
  public:
    struct ts_event
    {
        uint16_t au16_x[HYN_MAX_POINTS];         /* x coordinate */
        uint16_t au16_y[HYN_MAX_POINTS];         /* y coordinate */
        uint8_t au8_touch_event[HYN_MAX_POINTS]; /* touch event: 0 -- down; 1-- up; 2 -- contact */
        uint8_t au8_finger_id[HYN_MAX_POINTS];   /* touch ID */
        uint16_t pressure[HYN_MAX_POINTS];
        uint16_t area[HYN_MAX_POINTS];
        uint8_t touch_point;
        int touchs;
        uint8_t touch_point_num;
    };

    enum event_type
    {
        EVENT_TYPE_TOUCH = 0x05,
        EVENT_TYPE_LONG_TOUCH = 0x0C,
        EVENT_TYPE_SWIPE_Y_POSITIVE = 0x01,
        EVENT_TYPE_SWIPE_Y_NEGATIVE = 0x02,
        EVENT_TYPE_SWIPE_X_NEGATIVE = 0x03,
        EVENT_TYPE_SWIPE_X_POSITIVE = 0x04
    };

    CST0xx_TouchPad(mbed::I2C *i2c);
    void handle_interrupt();

  protected:
    mbed::I2C *_i2c;
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
};

#endif // CST0XXTOUCHPAD_H
