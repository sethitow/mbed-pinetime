
MBED_PYTHON = python3

MBED_TOOLS_PATH = foobar
MBED_TOOLCHAIN = GCC_ARM


help:
	@echo "This is not the Makefile you're looking for."


build: FORCE
	$(MBED_PYTHON) $(MBED_PINETIME_PATH)/modules/mbed-os/tools/make.py \
	-t $(MBED_TOOLCHAIN) -m $(MBED_TARGET) \
	--custom-targets $(MBED_PINETIME_PATH)/targets \
	--profile $(MBED_PINETIME_PATH)/modules/mbed-os/tools/profiles/debug.json  \
	$(MBED_SOURCES) \
	--build $(MBED_PINETIME_PATH)/apps/$(APP_NAME)/build \
	--app-config mbed_app.json

bootstrap-python:
	pipenv install -r modules/mbed-os/requirements.txt
	pipenv install --skip-lock

FORCE: