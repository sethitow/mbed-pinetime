#include "ptime_accel.h"

#include "bma421.h"

#include "drivers/I2C.h"
#include "rtos/ThisThread.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "PACL"

uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void delay_ms(uint32_t period);

mbed::I2C *i2c_instance;

uint16_t ptime_accel_init(struct bma4_dev *dev, mbed::I2C *i2c)
{
    i2c_instance = i2c;
    struct bma4_accel_config conf;
    int8_t rslt;

    /* Modify the parameters */
    dev->dev_addr = BMA4_I2C_ADDR_PRIMARY;
    dev->interface = BMA4_I2C_INTERFACE;
    dev->bus_read = i2c_reg_read;
    dev->bus_write = i2c_reg_write;
    dev->delay = delay_ms;
    dev->read_write_len = 8;

    // From Init Function
    dev->variant = BMA42X_VARIANT;
    dev->resolution = 12;

    tr_info("Accel Init");
    rslt = bma421_init(dev);
    ptime_accel_print_rslt(rslt);

    // tr_info("Write Feature Config Blob File");
    // rslt = bma421_write_config_file(&bma);
    // print_rslt(rslt);

    tr_info("Accel Enable");
    rslt = bma4_set_accel_enable(BMA4_ENABLE, dev);
    ptime_accel_print_rslt(rslt);

    conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    conf.range = BMA4_ACCEL_RANGE_2G;
    conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    conf.perf_mode = BMA4_CONTINUOUS_MODE;

    tr_info("Accel Set Config");
    rslt = bma4_set_accel_config(&conf, dev);
    ptime_accel_print_rslt(rslt);

    return rslt;
}

void ptime_accel_print_rslt(uint16_t rslt)
{
    switch (rslt)
    {
    case BMA4_OK:
        tr_info("Success.");
        break;
    case BMA4_E_NULL_PTR:
        tr_err("Error [%d] : Null pointer", rslt);
        break;
    case BMA4_E_OUT_OF_RANGE:
        tr_err("Error [%d] : Out of range", rslt);
        break;
    case BMA4_E_INVALID_SENSOR:
        tr_err("Error [%d] : Invalid sensor", rslt);
        break;
    case BMA4_E_CONFIG_STREAM_ERROR:
        tr_err("Error [%d] : Config stream error", rslt);
        break;
    case BMA4_E_SELF_TEST_FAIL:
        tr_err("Warning [%d] : Self test failed", rslt);
        break;
    default:
        tr_err("Error [%d] : Unknown error code", rslt);
        break;
    }
}

uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
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

    ret = i2c_instance->write(i2c_addr << 1, (const char *)tmp, length + 1, false);

    if (!ret)
    {
        return 0;
    }
    tr_err("I2C Write Failed, returned %i", ret);
    return -1;
}

uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */

    int ret = 0;

    ret = i2c_instance->write(i2c_addr << 1, (const char *)&reg_addr, 1, true);
    if (!ret)
    {
        ret = i2c_instance->read(i2c_addr << 1, (char *)reg_data, length, false);
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

void delay_ms(uint32_t period) { rtos::ThisThread::sleep_for(period); }