BOARD ?= xiao_ble
BUILD_DIR ?= build
UF2_DIR ?=

.PHONY: all build flash menuconfig clean screen

all: build flash

build:
	@echo "==> Building Zephyr application for $(BOARD)"
	west build -b $(BOARD) -d $(BUILD_DIR) -- -DCONFIG_BOOT_DELAY=5000

flash:
	@echo "==> Flashing to $(BOARD)"
	cp $(BUILD_DIR)/zephyr/zephyr.uf2 $(UF2_DIR)

menuconfig:
	@echo "==> Launching menuconfig"
	west build -b $(BOARD) -d $(BUILD_DIR)
	west build -t menuconfig -d $(BUILD_DIR)

clean:
	@echo "==> Cleaning build directory"
	rm -rf $(BUILD_DIR)

screen:
	@echo "==> Launching Espressif monitor"
	minicom -D /dev/ttyACM0 -b 115200
