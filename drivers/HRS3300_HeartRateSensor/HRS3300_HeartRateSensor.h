
#ifndef HRS3300HEARTRATESENSOR_H
#define HRS3300HEARTRATESENSOR_H

#include "drivers/I2C.h"

#define HRS3300_I2C_ADDRESS 0x44
#define HRS3300_DEVICE_ID 0x21

#define HRS3300_REG_ID 0x00
#define HRS3300_REG_ENABLE 0x01
#define HRS3300_REG_PDRIVER 0x0C

#define HRS3300_REG_C0DATAL 0x0F
#define HRS3300_REG_C0DATAM 0x09
#define HRS3300_REG_C0DATAH 0x0A

#define HRS3300_REG_C1DATAL 0x0E
#define HRS3300_REG_C1DATAM 0x08
#define HRS3300_REG_C1DATAH 0x0D

#define HRS3300_REG_RESOLUTION 0x16
#define HRS3300_REG_HGAIN 0x17

class HRS3300_HeartRateSensor
{
  public:
    enum HRS_WAIT_TIME
    {
        HRS_WAIT_TIME_800ms = 0,
        HRS_WAIT_TIME_400ms = 1,
        HRS_WAIT_TIME_200ms = 2,
        HRS_WAIT_TIME_100ms = 3,
        HRS_WAIT_TIME_75ms = 4,
        HRS_WAIT_TIME_50ms = 5,
        HRS_WAIT_TIME_12_5ms = 6,
        HRS_WAIT_TIME_0ms = 7
    };

    enum HRS_ENABLE_DISABLE
    {
        HRS_DISABLE = 0,
        HRS_ENABLE = 1

    };

    enum PDRIVE_CURRENT
    {
        PDRIVE_CURRENT_12_5mA = 0,
        PDRIVE_CURRENT_20mA = 1,
        PDRIVE_CURRENT_30mA = 2,
        PDRIVE_CURRENT_40mA = 3
    };

    enum ADC_RESOLUTION
    {
        ADC_RESOLUTION_8b = 0,
        ADC_RESOLUTION_9b = 1,
        ADC_RESOLUTION_10b = 2,
        ADC_RESOLUTION_11b = 3,
        ADC_RESOLUTION_12b = 4,
        ADC_RESOLUTION_13b = 5,
        ADC_RESOLUTION_14b = 6,
        ADC_RESOLUTION_15b = 7,
        ADC_RESOLUTION_16b = 8,
        ADC_RESOLUTION_17b = 9,
        ADC_RESOLUTION_18b = 10,
    };

    enum HRS_GAIN
    {
        HRS_GAIN_1x = 0,
        HRS_GAIN_2x = 1,
        HRS_GAIN_4x = 2,
        HRS_GAIN_8x = 3,
        HRS_GAIN_64x = 4
    };

    HRS3300_HeartRateSensor(mbed::I2C *i2c);
    ~HRS3300_HeartRateSensor();

    void set_enable(enum HRS_ENABLE_DISABLE enable = HRS_DISABLE,
                    enum HRS_WAIT_TIME wait_time = HRS_WAIT_TIME_12_5ms,
                    enum PDRIVE_CURRENT current = PDRIVE_CURRENT_40mA,
                    enum HRS_ENABLE_DISABLE p_on = HRS_ENABLE);
    void set_adc_resolution(enum ADC_RESOLUTION resolution = ADC_RESOLUTION_14b);
    void set_hrs_gain(enum HRS_GAIN gain = HRS_GAIN_64x);
    uint32_t read_heart_rate_sensor();
    uint32_t read_ambient_light_sensor();

  protected:
    mbed::I2C *_i2c;
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
};

#endif // HRS3300HEARTRATESENSOR_H
