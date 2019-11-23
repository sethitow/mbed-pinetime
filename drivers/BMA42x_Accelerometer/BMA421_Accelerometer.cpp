#include "BMA421_Accelerometer.h"

#include "bma421.h"

#include "platform/Callback.h"
#include "rtos/ThisThread.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "ACEL"

BMA421_Accelerometer::BMA421_Accelerometer(mbed::I2C *i2c) : _i2c(i2c)
{
    /* Modify the parameters */
    bma.dev_addr = BMA4_I2C_ADDR_PRIMARY;
    bma.interface = BMA4_I2C_INTERFACE;
    // bma.bus_read = this->i2c_reg_read;
    // bma.bus_write = this->i2c_reg_write;
    // bma.delay = this->delay_ms;
    bma.read_write_len = 8;

    tr_info("Accel Init");
    int8_t rslt;
    rslt = bma421_init(&bma);
    print_rslt(rslt);

    // TODO: Get magic binary blob
    // tr_info("Write Feature Config Blob File");   
    // rslt = bma421_write_config_file(&bma);
    // print_rslt(rslt);
}

void BMA421_Accelerometer::enable()
{
    tr_info("Accel Enable");

    int8_t rslt;
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    print_rslt(rslt);
}

void BMA421_Accelerometer::set_config()
{
    struct bma4_accel_config conf;
    conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    conf.range = BMA4_ACCEL_RANGE_2G;
    conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    conf.perf_mode = BMA4_CONTINUOUS_MODE;

    tr_info("Accel Set Config");    
    int8_t rslt;
    rslt = bma4_set_accel_config(&conf, &bma);
    print_rslt(rslt);
}

void BMA421_Accelerometer::print_rslt(uint16_t rslt)
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

uint16_t BMA421_Accelerometer::i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr,
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

uint16_t BMA421_Accelerometer::i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr,
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

void delay_ms(uint32_t period) { rtos::ThisThread::sleep_for(period); }