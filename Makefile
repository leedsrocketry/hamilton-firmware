BUILD_DIR := build

CFLAGS  ?=  -W -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -Wno-unknown-pragmas \
            -g3 -O0 -ffunction-sections -fdata-sections -Isrc -Isrc/include \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS) -g\
			-lm
LDFLAGS ?= -Tbuild/link.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/firmware.map
SOURCES ?=	src/main.c src/startup.c src/syscalls.c src/HAL/STM32_init.c src/drivers/MS5611_driver.c src/filters.c\
			src/drivers/ADXL375_driver.c src/frame_buffer.c src/drivers/LSM6DS3_driver.c\
			src/flight_manager.c src/sensors.c src/drivers/HC12_driver.c


# Ensure make clean is cross platform
ifeq ($(OS), Windows_NT)
	RM = del
else
	RM = rm -rf
endif

build: $(BUILD_DIR)/firmware.bin

$(BUILD_DIR)/firmware.elf: $(SOURCES) | $(BUILD_DIR)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

$(BUILD_DIR)/firmware.bin: $(BUILD_DIR)/firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: $(BUILD_DIR)/firmware.bin
	st-flash --reset write $< 0x8000000

clean:
	$(RM) $(BUILD_DIR)/firmware.*

debug-hardware:
	openocd -f "debug/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg"

unblock-write-protected:
	openocd -f "debug/OpenOCD/openocd/scripts/interface/stlink.cfg" \
          -f "debug/OpenOCD/openocd/scripts/target/stm32l4x.cfg" \
          -c "init; reset halt; stm32l4x unlock 0; stm32l4x mass_erase 0; program $(BUILD_DIR)/firmware.bin 0x08000000 verify reset; exit"

# Different targets for HFC/Nucleo
nucleo: build
nucleo-flash: nucleo flash

hfc: CFLAGS += -DFLIGHT_COMPUTER
hfc: build

hfc-flash: hfc flash

warnings:
	@(make clean && make hfc > $(BUILD_DIR)/make.log 2>&1) && grep "warning:" $(BUILD_DIR)/make.log | wc -l

analyse:
	cppcheck src/*

format:
	clang-format --style="Google" -i src/*.c src/*.h

check: analyse format

emulate: hfc
	./renode/run-emulator.sh renode/HFC_v2.resc

test-sensors: CFLAGS += -DSENSOR_TEST
test-sensors: CFLAGS += -DFLIGHT_COMPUTER
test-sensors: build
test-sensors: flash

calibrate: CFLAGS += -DCALIBRATE
calibrate: CFLAGS += -DFLIGHT_COMPUTER
calibrate: build
calibrate: flash

erase-NAND: CFLAGS += -DERASE_NAND
erase-NAND: CFLAGS += -DFLIGHT_COMPUTER
erase-NAND: build
erase-NAND: flash

read-NAND: CFLAGS += -DREAD_NAND
read-NAND: CFLAGS += -DFLIGHT_COMPUTER
read-NAND: build
read-NAND: flash

.PHONY: list
list:
	@LC_ALL=C $(MAKE) -pRrq -f $(firstword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/(^|\n)# Files(\n|$$)/,/(^|\n)# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | grep -E -v -e '^[^[:alnum:]]' -e '^$@$$'

.DEFAULT_GOAL := list





