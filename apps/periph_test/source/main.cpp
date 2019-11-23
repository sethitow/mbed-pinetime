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
#include "rtos/Thread.h"

#include "BMA421_Accelerometer.h"
#include "HRS3300_HeartRateSensor.h"
#include "CST0xx_TouchPad.h"
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

uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void set_backlight(uint8_t light_level);
void task_500ms();
void touchpad_interrupt_handler();
void main_button_on_rise();
void main_button_on_fall();

rtos::Thread event_queue_thread(osPriorityNormal, OS_STACK_SIZE, nullptr, "event_queue");
events::EventQueue event_queue;

mbed::DigitalOut vibrator(PIN_VIBRATOR_OUT);
mbed::DigitalOut backlight_high(LCD_BACKLIGHT_HIGH);
mbed::DigitalOut backlight_mid(LCD_BACKLIGHT_MID);
mbed::DigitalOut backlight_low(LCD_BACKLIGHT_LOW);
mbed::AnalogIn battery_voltage(BATTERY_VOLTAGE);
mbed::DigitalIn charge_indication(PIN_CHARGE_INDICATION_IN);
mbed::InterruptIn touchpad_interrupt(PIN_TOUCHPAD_INTERRUPT);
mbed::InterruptIn main_button_interrupt(PIN_PUSH_BUTTON_OUT);

mbed::DigitalOut lcd_cs(SPI_CS_LCD);

mbed::I2C i2c(I2C_SDA, I2C_SCL);
HRS3300_HeartRateSensor hrs(&i2c);
CST0xx_TouchPad touch_pad(&i2c);

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
    lcd_cs = true;    // Disable LCD SPI communications
    vibrator = true;  // Turns the vibrator off
    set_backlight(7); // Turn the backlight off

    mbed::BlockDevice *bd = mbed::BlockDevice::get_default_instance();
    tr_info("BD init returned %i", bd->init());
    tr_info("BD Size %lld.", bd->size());

    i2c.frequency(100000);

    hrs.set_enable(HRS3300_HeartRateSensor::HRS_DISABLE,
                   HRS3300_HeartRateSensor::HRS_WAIT_TIME_400ms);



    // BMA421_Accelerometer accel(&i2c);

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

    // event_queue.call_every(500, task_500ms);
    event_queue_thread.start(callback(&event_queue, &events::EventQueue::dispatch_forever));
    touchpad_interrupt.fall(event_queue.event(&touch_pad, &CST0xx_TouchPad::handle_interrupt));
    main_button_interrupt.mode(PullNone);
    main_button_interrupt.rise(event_queue.event(main_button_on_rise));
    main_button_interrupt.fall(event_queue.event(main_button_on_fall));

}

void task_500ms()
{
    float battery_voltage_v = (battery_voltage.read() * 3.3) * 2;
    struct bma4_accel data;
    // rslt = bma4_read_accel_xyz(&data, &bma);
    // if (rslt)
    // {
    //     print_rslt(rslt);
    // }

    uint32_t hrs_val = hrs.read_heart_rate_sensor();
    uint32_t als_val = hrs.read_ambient_light_sensor();

    tr_info("Batt Volt: %f. Accel Data: %i, %i, %i. HRS: %lu. ALS: %lu.", battery_voltage_v, data.x, data.y, data.z, hrs_val, als_val);
}

void set_backlight(uint8_t light_level)
{
    std::bitset<3> bits(light_level);

    backlight_high = bits[2];
    backlight_mid = bits[1];
    backlight_low = bits[0];
}

void main_button_on_rise()
{
    tr_info("Main Button Rise");
}

void main_button_on_fall()
{
    tr_info("Main Button Fall");
}




