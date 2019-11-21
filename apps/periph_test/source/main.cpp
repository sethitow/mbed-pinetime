#include "BlockDevice.h"
#include "drivers/AnalogIn.h"
#include "drivers/DigitalIn.h"
#include "drivers/DigitalOut.h"
#include "drivers/I2C.h"
#include "mbed-rtt.h"
#include "rtos/Mutex.h"
#include "rtos/ThisThread.h"

#include "HRS3300_HeartRateSensor.h"
#include "bma421_hack.h"

#include <bitset>

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "MAIN"

namespace mbed
{
FileHandle *mbed_override_console(int fd)
{
    static SeggerRTT rtt;
    return &rtt;
}
} // namespace mbed

void delay_ms(uint32_t period);
uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(uint16_t rslt);

mbed::DigitalIn main_button(p15, PullNone);
mbed::DigitalOut vibrator(PIN_VIBRATOR_OUT);
mbed::DigitalOut backlight_high(LCD_BACKLIGHT_HIGH);
mbed::DigitalOut backlight_mid(LCD_BACKLIGHT_MID);
mbed::DigitalOut backlight_low(LCD_BACKLIGHT_LOW);
mbed::AnalogIn battery_voltage(BATTERY_VOLTAGE);
mbed::DigitalIn charge_indication(PIN_CHARGE_INDICATION_IN);

mbed::DigitalOut lcd_cs(SPI_CS_LCD);

mbed::I2C i2c(I2C_SDA, I2C_SCL);

static rtos::Mutex trace_mutex;
static void trace_mutex_wait() { trace_mutex.lock(); }
static void trace_mutex_release() { trace_mutex.unlock(); }

int main()
{

    mbed_trace_mutex_wait_function_set(trace_mutex_wait);
    mbed_trace_mutex_release_function_set(trace_mutex_release);
    mbed_trace_config_set(TRACE_ACTIVE_LEVEL_ALL | TRACE_MODE_COLOR);
    mbed_trace_init();

    tr_info("Booting...");
    lcd_cs = true;   // Disable LCD SPI communications
    vibrator = true; // Turns the vibrator off

    // Turn the backlight off
    backlight_high = true;
    backlight_mid = true;
    backlight_low = true;

    mbed::BlockDevice *bd = mbed::BlockDevice::get_default_instance();
    tr_info("BD init returned %i", bd->init());
    tr_info("BD Size %lld.", bd->size());

    i2c.frequency(100000);

    HRS3300_HeartRateSensor hrs(&i2c);
    hrs.set_enable(HRS3300_HeartRateSensor::HRS_ENABLE,
                   HRS3300_HeartRateSensor::HRS_WAIT_TIME_400ms);

    struct bma4_dev bma;
    struct bma4_accel_config conf;
    struct bma4_accel data;
    int8_t rslt;

    /* Modify the parameters */
    bma.dev_addr = BMA4_I2C_ADDR_PRIMARY;
    bma.interface = BMA4_I2C_INTERFACE;
    bma.bus_read = i2c_reg_read;
    bma.bus_write = i2c_reg_write;
    bma.delay = delay_ms;
    bma.read_write_len = 8;

    // From Init Function
    bma.variant = BMA42X_VARIANT;
    bma.resolution = 12;

    tr_info("Accel Init");
    rslt = bma421_init(&bma);
    print_rslt(rslt);

    // tr_info("Write Feature Config Blob File");
    // rslt = bma421_write_config_file(&bma);
    // print_rslt(rslt);

    tr_info("Accel Enable");
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    print_rslt(rslt);

    conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    conf.range = BMA4_ACCEL_RANGE_2G;
    conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    conf.perf_mode = BMA4_CONTINUOUS_MODE;

    tr_info("Accel Set Config");
    rslt = bma4_set_accel_config(&conf, &bma);
    print_rslt(rslt);

    // printf("I2C Scan...\r\n");
    // for (int i = 0; i < 128; i++)
    // {
    //     i2c.start();
    //     int ret = i2c.write(i << 1);
    //     if (ret == 1)
    //     {
    //         printf("0x%x ACK \r\n", i); // Send command string
    //     }
    //     else
    //     {
    //     	printf("0x%x NO ACK, returned %i \r\n", i, ret); // Send command string
    //     }
    //     i2c.stop();
    // }
    // printf("I2C Scan End \r\n");

    while (true)
    {
        float battery_voltage_v = (battery_voltage.read() * 3.3) * 2;
        rslt = bma4_read_accel_xyz(&data, &bma);

        if (rslt)
        {
            print_rslt(rslt);
        }

        uint32_t hrs_val = hrs.read_heart_rate_sensor();
        uint32_t als_val = hrs.read_ambient_light_sensor();

        tr_info("Main Button: %d. Batt Volt: %f. Accel Data: %i, %i, %i. HRS: %lu. ALS: %lu.",
                main_button.read(), battery_voltage_v, data.x, data.y, data.z, hrs_val, als_val);

        rtos::ThisThread::sleep_for(500);
    }
}

void set_backlight(uint8_t light_level)
{
    std::bitset<3> bits(light_level);

    backlight_high = bits[2];
    backlight_mid = bits[1];
    backlight_low = bits[0];
}

void print_rslt(uint16_t rslt)
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

    ret = i2c.write(i2c_addr << 1, (const char *)tmp, length + 1, false);

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

    ret = i2c.write(i2c_addr << 1, (const char *)&reg_addr, 1, true);
    if (!ret)
    {
        ret = i2c.read(i2c_addr << 1, (char *)reg_data, length, false);
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
