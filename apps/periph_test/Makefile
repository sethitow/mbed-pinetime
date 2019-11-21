APP_NAME = periph_test
MBED_TARGET = TARGET_PINETIME_DEV_KIT

MBED_SOURCES += --source $(MBED_PINETIME_PATH)/modules/mbed-os/ \
	--source $(MBED_PINETIME_PATH)/targets/TARGET_NORDIC/TARGET_NRF5x/TARGET_NRF52/TARGET_MCU_NRF52832/TARGET_PINETIME_DEV_KIT/ \
	--source source/ \
	--source $(MBED_PINETIME_PATH)/drivers/SEGGER_RTT_V654c/RTT/ \
	--source $(MBED_PINETIME_PATH)/drivers/mbed-rtt \
	--source $(MBED_PINETIME_PATH)/drivers/BMA423_Accelerometer \
	--source $(MBED_PINETIME_PATH)/drivers/HRS3300_HeartRateSensor

include ../../Makefile