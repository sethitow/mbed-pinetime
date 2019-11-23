APP_NAME = periph_test
MBED_TARGET = TARGET_PINETIME_DEV_KIT

PROFILE := $(MBED_PINETIME_PATH)/modules/mbed-os/tools/profiles/release.json

MBED_SOURCES += --source $(MBED_PINETIME_PATH)/modules/mbed-os/ \
	--source $(MBED_PINETIME_PATH)/targets/TARGET_NORDIC/TARGET_NRF5x/TARGET_NRF52/TARGET_MCU_NRF52832/TARGET_PINETIME_DEV_KIT/ \
	--source source/ \
	--source $(MBED_PINETIME_PATH)/drivers/SEGGER_RTT_V654c/RTT/ \
	--source $(MBED_PINETIME_PATH)/drivers/mbed-rtt \
	--source $(MBED_PINETIME_PATH)/drivers/BMA42x_Accelerometer \
	--source $(MBED_PINETIME_PATH)/drivers/HRS3300_HeartRateSensor \
	--source $(MBED_PINETIME_PATH)/drivers/CST0xx_TouchPad \
	--source $(MBED_PINETIME_PATH)/drivers/Adafruit_ST7789 \
	--source $(MBED_PINETIME_PATH)/drivers/Adafruit_GFX


include ../../Makefile