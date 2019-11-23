#include "CST0xx_TouchPad.h"

#include "platform/mbed_assert.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "TCHP"

#define CST0XX_I2C_ADDRESS 0x15

#define HYN_TOUCH_EVENT_TYPE_POS 1
#define HYN_TOUCH_X_H_POS 3
#define HYN_TOUCH_X_L_POS 4
#define HYN_TOUCH_Y_H_POS 5
#define HYN_TOUCH_Y_L_POS 6

void print_hex(uint8_t *string, uint32_t len)
{
    tr_info("Dumping hex buffer of size %lu", len);
    for (unsigned int i = 0; i < len; ++i)
    {
        printf("0x%02x \r\n", string[i]);
    }
    printf("\n");
    tr_info("End dump.");
}

CST0xx_TouchPad::CST0xx_TouchPad(mbed::I2C *i2c) : _i2c(i2c)
{
    // TODO: Check Chip ID and firmware version on initalization.
}

CST0xx_TouchPad::CST0xx_TouchPad(mbed::I2C *i2c,
                                 mbed::Callback<void(struct ts_event)> touch_event_callback)
    : _i2c(i2c), _touch_event_callback(touch_event_callback)
{
}

void CST0xx_TouchPad::handle_interrupt()
{
    uint8_t buf[7];
    uint16_t ret = i2c_reg_read(0x15, 0x00, buf, 7);
    if (ret)
    {
        tr_err("Touch event: I2C Read returned: %u", ret);
    }

    struct ts_event data = {};
    data.type = static_cast<enum event_type>(
        buf[HYN_TOUCH_EVENT_TYPE_POS]); // TODO: Handle invalid event type.
    data.x = (uint16_t)(buf[HYN_TOUCH_X_H_POS] & 0x0F) << 8 | (uint16_t)buf[HYN_TOUCH_X_L_POS];
    data.y = (uint16_t)(buf[HYN_TOUCH_Y_H_POS] & 0x0F) << 8 | (uint16_t)buf[HYN_TOUCH_Y_L_POS];

    tr_debug("Touch event: Type %u. X: %u Y: %u", data.type, data.x, data.y);

    if (_touch_event_callback)
    {
        _touch_event_callback(data);
    }
}

uint16_t CST0xx_TouchPad::i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                                        uint16_t length)
{
    /* Write to registers using I2C. Return 0 for a successful execution. */
    int ret = 0;
    const int TEMP_BUF_SIZE = 32;
    if (length > TEMP_BUF_SIZE)
    {
        return -2;
    }
    uint8_t tmp[TEMP_BUF_SIZE];
    tmp[0] = reg_addr;
    memcpy(tmp + 1, reg_data, length);

    ret = _i2c->write(i2c_addr << 1, (const char *)tmp, length + 1, false);

    if (!ret)
    {
        return 0;
    }
    tr_err("I2C Write Failed, returned %i", ret);
    return -1;
}

uint16_t CST0xx_TouchPad::i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                                       uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */

    int ret = 0;

    ret = _i2c->write(i2c_addr << 1, (const char *)&reg_addr, 1, true);
    if (!ret)
    {
        ret = _i2c->read(i2c_addr << 1, (char *)reg_data, length, false);
    }
    else
    {
        tr_err("I2C Read Failed writting address, returned %i.", ret);
    }

    if (!ret)
    {
        return 0;
    }
    tr_err("I2C Read Failed, returned %i.", ret);
    return -1;
}