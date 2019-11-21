#include "BlockDevice.h"
#include "drivers/AnalogIn.h"
#include "drivers/DigitalIn.h"
#include "drivers/DigitalOut.h"
#include "drivers/I2C.h"
#include "mbed-rtt.h"
#include "rtos/Mutex.h"
#include "rtos/ThisThread.h"
extern "C" {
#include "bma4.h"
}

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "main"

namespace mbed
{
FileHandle *mbed_override_console(int fd)
{
    static SeggerRTT rtt;
    return &rtt;
}
} // namespace mbed

void delay_ms(uint32_t period);
uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                     uint16_t length);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                    uint16_t length);
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

    mbed::BlockDevice *bd = mbed::BlockDevice::get_default_instance();
    tr_info("BD init returned %i", bd->init());
    tr_info("BD Size %lld.", bd->size());

    i2c.frequency(100000);

    struct bma4_dev bma;
    struct bma4_accel_config conf;
    struct bma4_accel data;
    int8_t rslt;

    // set_interface(BMA400_I2C_INTF, &bma);

    // rslt = bma4_init(&bma);
    // print_rslt(rslt);

    // rslt = bma4_soft_reset(&bma);
    // print_rslt(rslt);

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
	rslt = bma4_init(&bma);
	print_rslt(rslt);

	tr_info("Chip ID: %hu", bma.chip_id);

	conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;	
	conf.range = BMA4_ACCEL_RANGE_2G;
	conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
	conf.perf_mode = BMA4_CONTINUOUS_MODE;

	tr_info("Accel Set Config");
	rslt = bma4_set_accel_config(&conf, &bma);
	print_rslt(rslt);




    // /* Select the type of configuration to be modified */
    // conf.type = BMA400_ACCEL;

    // /* Get the accelerometer configurations which are set in the sensor */
    // rslt = bma400_get_sensor_conf(&conf, 1, &bma);
    // print_rslt(rslt);

    // /* Modify the desired configurations as per macros
    //  * available in bma400_defs.h file */
    // conf.param.accel.odr = BMA400_ODR_100HZ;
    // conf.param.accel.range = BMA400_2G_RANGE;
    // conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    // /* Set the desired configurations to the sensor */
    // rslt = bma400_set_sensor_conf(&conf, 1, &bma);
    // print_rslt(rslt);

    // rslt = bma400_set_power_mode(BMA400_LOW_POWER_MODE, &bma);
    // print_rslt(rslt);

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
        tr_info("Main Button: %d. Batt Volt: %f.", main_button.read(), battery_voltage_v);

        rslt = bma4_read_accel_xyz(&data, &bma);
        print_rslt(rslt);

        tr_info("Accel Data: %i, %i, %i.", data.x, data.y, data.z);

        tr_info("7");
        backlight_high = true;
        backlight_mid = true;
        backlight_low = true;
        rtos::ThisThread::sleep_for(500);

        // tr_info("6");
        // backlight_high = true;
        // backlight_mid = true;
        // backlight_low = false;
        // rtos::ThisThread::sleep_for(500);
        // tr_info("5");
        // backlight_high = true;
        // backlight_mid = false;
        // backlight_low = true;
        // rtos::ThisThread::sleep_for(500);

        // tr_info("4");
        // backlight_high = true;
        // backlight_mid = false;
        // backlight_low = false;
        // rtos::ThisThread::sleep_for(500);

        // tr_info("3");
        // backlight_high = false;
        // backlight_mid = true;
        // backlight_low = true;
        // rtos::ThisThread::sleep_for(500);

        // tr_info("2");
        // backlight_high = false;
        // backlight_mid = true;
        // backlight_low = false;
        // rtos::ThisThread::sleep_for(500);

        // tr_info("1");
        // backlight_high = false;
        // backlight_mid = false;
        // backlight_low = true;
        // rtos::ThisThread::sleep_for(500);

        // tr_info("0");
        // backlight_high = false;
        // backlight_mid = false;
        // backlight_low = false;
        // rtos::ThisThread::sleep_for(500);
        


    }
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
    // case BMA400_E_COM_FAIL:
    //     tr_err("Error [%d] : Communication failure", rslt);
    //     break;
    // case BMA400_E_DEV_NOT_FOUND:
    //     tr_err("Error [%d] : Device not found", rslt);
    //     break;
    case BMA4_E_INVALID_SENSOR:
        tr_err("Error [%d] : Invalid sensor", rslt);
        break;
    case BMA4_E_SELF_TEST_FAIL:
        tr_err("Warning [%d] : Self test failed", rslt);
        break;
    default:
        tr_err("Error [%d] : Unknown error code", rslt);
        break;
    }
}

uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
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

    ret = i2c.write(i2c_addr<<1, (const char *)tmp, length + 1, false);

    if (!ret)
    {
        return 0;
    }
    tr_err("I2C Write Failed, returned %i", ret);
    return -1;
}

uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                    uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */

    int ret = 0;

    ret = i2c.write(i2c_addr<<1, (const char *)&reg_addr, 1, true);
    if (!ret)
    {
        ret = i2c.read(i2c_addr<<1, (char *)reg_data, length, false);
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

// void set_interface(enum bma400_intf intf, struct bma400_dev *dev)
// {
//     switch (intf)
//     {
//     case BMA400_I2C_INTF:
//         dev->intf_ptr = NULL; /* To attach your interface device reference */
//         dev->delay_ms = delay_ms;
//         dev->dev_id = BMA400_I2C_ADDRESS_SDO_HIGH;
//         dev->read = i2c_reg_read;
//         dev->write = i2c_reg_write;
//         dev->intf = BMA400_I2C_INTF;
//         break;
//     default:
//         tr_err("Interface not supported.");
//     }
// }
