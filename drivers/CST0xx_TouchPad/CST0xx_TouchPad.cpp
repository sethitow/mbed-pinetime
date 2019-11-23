#include "CST0xx_TouchPad.h"

#include "platform/mbed_assert.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "TCHP"

#define CST0XX_I2C_ADDRESS 0x15

#define CFG_MAX_TOUCH_POINTS 5

#define HYN_MAX_ID 0x0F
#define HYN_TOUCH_STEP 6
#define HYN_FACE_DETECT_POS 1
#define HYN_TOUCH_X_H_POS 3
#define HYN_TOUCH_X_L_POS 4
#define HYN_TOUCH_Y_H_POS 5
#define HYN_TOUCH_Y_L_POS 6
#define HYN_TOUCH_EVENT_POS 3
#define HYN_TOUCH_ID_POS 5
#define FT_TOUCH_POINT_NUM 2
#define HYN_TOUCH_XY_POS 7
#define HYN_TOUCH_MISC 8
#define POINT_READ_BUF (3 + HYN_TOUCH_STEP * HYN_MAX_POINTS)

void print_hex(uint8_t *string, uint32_t len)
{
    tr_info("Dumping hex buffer of size %lu", len);
    for (unsigned int i = 0; i < len; ++i)
    {
        printf("0x%02x ", string[i]);
    }
    printf("\n");
    tr_info("End dump.");
}

CST0xx_TouchPad::CST0xx_TouchPad(mbed::I2C *i2c) : _i2c(i2c)
{
    // uint8_t charbuf = 0;
    // i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_ID, &charbuf, 1);
    // MBED_ASSERT(charbuf == HRS3300_DEVICE_ID);

    // i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_RESOLUTION, &charbuf, 1);
    // tr_info("res: 0x%hu", charbuf);
    // i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_HGAIN, &charbuf, 1);
    // tr_info("gain: 0x%hu", charbuf);
}

void CST0xx_TouchPad::handle_interrupt()
{
    tr_info("Touch event");
    uint8_t buf[33];

    uint16_t ret = i2c_reg_read(0x15, 0x00, buf, 33);

    tr_info("I2C Read returned: %u", ret);

    print_hex(buf, sizeof(buf));

    struct ts_event data = {0};

    uint8_t pointid = HYN_MAX_ID;

    data.touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
    data.touch_point = 0;

    for (unsigned int i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
    {
        pointid = (buf[HYN_TOUCH_ID_POS + HYN_TOUCH_STEP * i]) >> 4;
        if (pointid >= HYN_MAX_ID)
            break;
        else
            data.touch_point++;
        data.au16_x[i] = (uint16_t)(buf[HYN_TOUCH_X_H_POS + HYN_TOUCH_STEP * i] & 0x0F) << 8 |
                         (uint16_t)buf[HYN_TOUCH_X_L_POS + HYN_TOUCH_STEP * i];
        data.au16_y[i] = (uint16_t)(buf[HYN_TOUCH_Y_H_POS + HYN_TOUCH_STEP * i] & 0x0F) << 8 |
                         (uint16_t)buf[HYN_TOUCH_Y_L_POS + HYN_TOUCH_STEP * i];
        data.au8_touch_event[i] = buf[HYN_TOUCH_EVENT_POS + HYN_TOUCH_STEP * i] >> 6;
        data.au8_finger_id[i] = (buf[HYN_TOUCH_ID_POS + HYN_TOUCH_STEP * i]) >> 4;

        data.pressure[i] = (buf[HYN_TOUCH_XY_POS + HYN_TOUCH_STEP * i]); // cannot constant value
        data.area[i] = (buf[HYN_TOUCH_MISC + HYN_TOUCH_STEP * i]) >> 4;
        if ((data.au8_touch_event[i] == 0 || data.au8_touch_event[i] == 2) &&
            (data.touch_point_num == 0))
            break;
    }

    tr_info("Num Points %u", data.touch_point_num);
    tr_info("X: %u Y: %u", data.au16_x[0], data.au16_y[0]);
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