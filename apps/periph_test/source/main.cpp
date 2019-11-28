#include "BlockDevice.h"
#include "drivers/AnalogIn.h"
#include "drivers/DigitalIn.h"
#include "drivers/DigitalOut.h"
#include "drivers/I2C.h"
#include "drivers/InterruptIn.h"
#include "events/Event.h"
#include "events/EventQueue.h"
#include "platform/Callback.h"
#include "rtos/Mutex.h"
#include "rtos/ThisThread.h"

#include "Adafruit_ST7789.h"
#include "CST0xx_TouchPad.h"
#include "HRS3300_HeartRateSensor.h"
#include "ptime_accel.h"

#include "mbed-rtt.h"

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

void on_touch_event(struct CST0xx_TouchPad::ts_event event);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void set_backlight(uint8_t light_level);
void task_500ms(struct bma4_dev *dev);
void touchpad_interrupt_handler();
void main_button_on_rise();
void main_button_on_fall();

events::EventQueue event_queue;

mbed::DigitalOut vibrator(PIN_VIBRATOR_OUT);
mbed::DigitalOut backlight_high(LCD_BACKLIGHT_HIGH);
mbed::DigitalOut backlight_mid(LCD_BACKLIGHT_MID);
mbed::DigitalOut backlight_low(LCD_BACKLIGHT_LOW);
mbed::AnalogIn battery_voltage(BATTERY_VOLTAGE);
mbed::DigitalIn charge_indication(PIN_CHARGE_INDICATION_IN);
mbed::InterruptIn touchpad_interrupt(PIN_TOUCHPAD_INTERRUPT);
mbed::InterruptIn main_button_interrupt(PIN_PUSH_BUTTON_OUT);

mbed::DigitalOut spi_flash_cs(SPI_CS_FLASH);

mbed::I2C i2c(I2C_SDA, I2C_SCL);
HRS3300_HeartRateSensor hrs(&i2c);
CST0xx_TouchPad touch_pad(&i2c, callback(on_touch_event));

Adafruit_ST7789 display(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS_LCD, PIN_LCD_RS_OUT, PIN_LCD_RESET_OUT);

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
    vibrator = true;  // Turns the vibrator off
    set_backlight(6); // Turn the backlight on

    mbed::BlockDevice *bd = mbed::BlockDevice::get_default_instance();
    tr_info("BD init returned %i", bd->init());
    tr_info("BD Size %lld.", bd->size());

    i2c.frequency(400000);

    hrs.set_enable(HRS3300_HeartRateSensor::HRS_ENABLE,
                   HRS3300_HeartRateSensor::HRS_WAIT_TIME_400ms);

    struct bma4_dev bma;

    ptime_accel_init(&bma, &i2c);

    tr_info("display init");
    display.init();
    tr_info("fillScreen");
    display.fillScreen(0x07E0);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.setTextColor(0xFFFF, 0x0000);
    display.printf("SPIF Size: %lluB \r\n", bd->size());

    event_queue.call_every(500, task_500ms, &bma);
    touchpad_interrupt.fall(event_queue.event(&touch_pad, &CST0xx_TouchPad::handle_interrupt));
    main_button_interrupt.mode(PullNone);
    main_button_interrupt.rise(event_queue.event(main_button_on_rise));
    main_button_interrupt.fall(event_queue.event(main_button_on_fall));

    event_queue.dispatch_forever();
}

void task_500ms(struct bma4_dev *bma)
{
    float battery_voltage_v = (battery_voltage.read() * 3.3) * 2;
    struct bma4_accel data;
    int16_t rslt = bma4_read_accel_xyz(&data, bma);
    if (rslt)
    {
        ptime_accel_print_rslt(rslt);
    }

    display.setTextSize(2);
    display.setCursor(0, 40);
    display.setTextColor(0xFFFF, 0x0000);
    display.printf("Batt: %f \r\n", battery_voltage_v);

    if (charge_indication)
    {
        display.printf("Not Charging.\r\n");
    }
    else
    {
        display.printf("Charging.    \r\n");
    }

    uint32_t hrs_val = hrs.read_heart_rate_sensor();
    uint32_t als_val = hrs.read_ambient_light_sensor();

    display.printf("HRS: %li \r\n", hrs_val);
    display.printf("ALS: %li \r\n", als_val);

    display.printf("X: %i \r\n", data.x);
    display.printf("Y: %i \r\n", data.y);
    display.printf("Z: %i \r\n", data.z);

    tr_info("Batt Volt: %f. Accel Data: %i, %i, %i. HRS: %lu. ALS: %lu.", battery_voltage_v, data.x,
            data.y, data.z, hrs_val, als_val);
}

void set_backlight(uint8_t light_level)
{
    std::bitset<3> bits(light_level);

    backlight_high = bits[2];
    backlight_mid = bits[1];
    backlight_low = bits[0];
}

void i2c_scan(mbed::I2C &i2c)
{
    printf("I2C Scan...\r\n");
    for (int i = 0; i < 128; i++)
    {
        i2c.start();
        int ret = i2c.write(i << 1);
        if (ret == 1)
        {
            printf("0x%x ACK \r\n", i); // Send command string
        }
        else
        {
            printf("0x%x NO ACK, returned %i \r\n", i, ret); // Send command string
        }
        i2c.stop();
    }
    printf("I2C Scan End \r\n");
}

void on_touch_event(struct CST0xx_TouchPad::ts_event event)
{
    display.setTextSize(2);
    display.setCursor(0, 200);
    display.setTextColor(0xF800, 0xFFFF);
    switch (event.type)
    {
    case CST0xx_TouchPad::EVENT_TYPE_TOUCH:
        display.printf("TOUCH X: %u Y: %u", event.x, event.y);
        tr_info("TOUCH X: %u Y: %u", event.x, event.y);
        break;
    case CST0xx_TouchPad::EVENT_TYPE_LONG_TOUCH:
        display.printf("TOUCH LONG X: %u Y: %u", event.x, event.y);
        tr_info("TOUCH LONG X: %u Y: %u", event.x, event.y);
        break;
    case CST0xx_TouchPad::EVENT_TYPE_SWIPE_Y_POSITIVE:
        display.printf("SWIPE y+ X: %u Y: %u", event.x, event.y);
        tr_info("SWIPE y+ X: %u Y: %u", event.x, event.y);
        break;
    case CST0xx_TouchPad::EVENT_TYPE_SWIPE_X_POSITIVE:
        display.printf("SWIPE x+ X: %u Y: %u", event.x, event.y);
        tr_info("SWIPE x+ X: %u Y: %u", event.x, event.y);
        break;
    case CST0xx_TouchPad::EVENT_TYPE_SWIPE_Y_NEGATIVE:
        display.printf("SWIPE y- X: %u Y: %u", event.x, event.y);
        tr_info("SWIPE y- X: %u Y: %u", event.x, event.y);
        break;
    case CST0xx_TouchPad::EVENT_TYPE_SWIPE_X_NEGATIVE:
        display.printf("SWIPE x- X: %u Y: %u", event.x, event.y);
        tr_info("SWIPE x- X: %u Y: %u", event.x, event.y);
        break;
    }

    display.fillRect(event.x - 5, event.y - 5, 10, 10, 0xF800);
}

void main_button_on_rise() { tr_info("Main Button Rise"); }

void main_button_on_fall() { tr_info("Main Button Fall"); }
