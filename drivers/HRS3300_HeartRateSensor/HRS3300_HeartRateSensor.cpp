#include "HRS3300_HeartRateSensor.h"

#include "platform/mbed_assert.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "HRS"

HRS3300_HeartRateSensor::HRS3300_HeartRateSensor(mbed::I2C *i2c) : _i2c(i2c)
{
    uint8_t charbuf = 0;
    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_ID, &charbuf, 1);
    MBED_ASSERT(charbuf == HRS3300_DEVICE_ID);

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_RESOLUTION, &charbuf, 1);
    tr_info("res: 0x%hu", charbuf);
    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_HGAIN, &charbuf, 1);
    tr_info("gain: 0x%hu", charbuf);

    set_enable();
    set_adc_resolution();
    set_hrs_gain();
}

void HRS3300_HeartRateSensor::set_enable(enum HRS_ENABLE_DISABLE enable,
                                         enum HRS_WAIT_TIME wait_time, enum PDRIVE_CURRENT current,
                                         enum HRS_ENABLE_DISABLE p_on)
{
    uint8_t charbuf = 0;

    charbuf = charbuf | enable << 7;
    charbuf = charbuf | wait_time << 4;
    charbuf = charbuf | (current & (1 << 1)) << 2;
    i2c_reg_write(HRS3300_I2C_ADDRESS, HRS3300_REG_ENABLE, &charbuf, 1);

    charbuf = 0;
    charbuf = charbuf | (current & 1) << 6;
    charbuf = charbuf | p_on << 5;
    charbuf = charbuf | 1 << 3; // This bit is set by default. Not sure if it needs to be set.
    i2c_reg_write(HRS3300_I2C_ADDRESS, HRS3300_REG_PDRIVER, &charbuf, 1);
}

void HRS3300_HeartRateSensor::set_adc_resolution(enum ADC_RESOLUTION resolution)
{
    uint8_t charbuf = 0x60; // Default value is 0x66, not sure what the other bits do.
    charbuf = charbuf | resolution;
    i2c_reg_write(HRS3300_I2C_ADDRESS, HRS3300_REG_RESOLUTION, &charbuf, 1);
}

void HRS3300_HeartRateSensor::set_hrs_gain(enum HRS_GAIN gain)
{
    uint8_t charbuf = 0;
    charbuf = charbuf | gain << 2;
    i2c_reg_write(HRS3300_I2C_ADDRESS, HRS3300_REG_HGAIN, &charbuf, 1);
}

uint32_t HRS3300_HeartRateSensor::read_heart_rate_sensor()
{
    uint32_t ret_val = 0;
    uint8_t charbuf = 0;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C0DATAL, &charbuf, 1);
    ret_val = ret_val | charbuf;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C0DATAM, &charbuf, 1);
    ret_val = ret_val | charbuf << 8;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C0DATAH, &charbuf, 1);
    ret_val = ret_val | charbuf << 16;

    return ret_val;
}

uint32_t HRS3300_HeartRateSensor::read_ambient_light_sensor()
{
    uint32_t ret_val = 0;
    uint8_t charbuf = 0;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C1DATAL, &charbuf, 1);
    ret_val = ret_val | charbuf;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C1DATAM, &charbuf, 1);
    ret_val = ret_val | charbuf << 8;

    i2c_reg_read(HRS3300_I2C_ADDRESS, HRS3300_REG_C1DATAH, &charbuf, 1);
    ret_val = ret_val | charbuf << 16;

    return ret_val;
}

uint16_t HRS3300_HeartRateSensor::i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr,
                                                uint8_t *reg_data, uint16_t length)
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

uint16_t HRS3300_HeartRateSensor::i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr,
                                               uint8_t *reg_data, uint16_t length)
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