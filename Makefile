BUILD_DIR := build

CFLAGS  ?=  -W -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -Wno-unknown-pragmas \
            -g3 -O0 -ffunction-sections -fdata-sections -Isrc -Isrc/include -Isegger-rtt/Config \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS) -g\
            -lm
LDFLAGS ?= -Tbuild/link.ld -u _printf_float -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/firmware.map
SOURCES ?=  src/main.c src/startup.c src/syscalls.c src/HAL/STM32_init.c src/drivers/MS5611_driver.c src/filters.c\
            src/drivers/ADXL375_driver.c src/drivers/LSM6DS3_driver.c\
            src/flight_manager.c src/sensors.c src/drivers/HC12_driver.c src/drivers/_driver_manager.c src/buffer.c segger-rtt/RTT/SEGGER_RTT.c segger-rtt/RTT/SEGGER_RTT_printf.c src/lib/log.c\

# Ensure make clean is cross platform
ifeq ($(OS), Windows_NT)
	RM = del
else
	RM = rm -rf
endif

build: clean
build: $(BUILD_DIR)/firmware.bin

$(BUILD_DIR)/firmware.elf: $(SOURCES) | $(BUILD_DIR)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

$(BUILD_DIR)/firmware.bin: $(BUILD_DIR)/firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: $(BUILD_DIR)/firmware.bin
	st-flash --reset write $< 0x8000000

flash-rs: $(BUILD_DIR)/firmware.elf
	probe-rs run --chip STM32L4R5ZITx $<

clean:
	$(RM) $(BUILD_DIR)/firmware.*

debug-hardware:
	openocd -f "debug/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg"

unblock-write-protected:
	openocd -f "debug/OpenOCD/openocd/scripts/interface/stlink.cfg" \
          -f "debug/OpenOCD/openocd/scripts/target/stm32l4x.cfg" \
          -c "init; reset halt; stm32l4x unlock 0; stm32l4x mass_erase 0; program $(BUILD_DIR)/firmware.bin 0x08000000 verify reset; exit"

#nucleo: build
#nucleo-flash: nucleo flash
#
#hfc: clean
#hfc: CFLAGS += -DFLIGHT_COMPUTER
#hfc: build
#
#hfc-flash: hfc flash-rs
#
#prod: CFLAGS += -DPROD

warnings:
	@(make clean && make hfc > $(BUILD_DIR)/make.log 2>&1) && grep "warning:" $(BUILD_DIR)/make.log | wc -l

analyse:
	cppcheck src/*

format:
	find src -type f \( -name "*.c" -o -name "*.h" \) | xargs clang-format -i

check: analyse format

emulate: hfc
	./renode/run-emulator.sh renode/HFC_v2.resc

test-sensors: CFLAGS += -DSENSOR_TEST
test-sensors: CFLAGS += -DFLIGHT_COMPUTER
test-sensors: clean
test-sensors: build
test-sensors: flash-rs

# Builds and flashes the routine to calibrate the ADXL375.
calibrate: CFLAGS += -DCALIBRATE
calibrate: CFLAGS += -DFLIGHT_COMPUTER
calibrate: clean
calibrate: build
calibrate: flash-rs

# Builds and flashes the routine to erase the NAND flash memory
# Warning: This will PERMANENTLY erase the NAND flash memory after a countdown
erase: CFLAGS += -DERASE_NAND
erase: CFLAGS += -DFLIGHT_COMPUTER
erase: clean
erase: build
erase: flash-rs

# Builds and flashes the routine to read the NAND flash memory
read: CFLAGS += -DREAD_NAND
read: CFLAGS += -DFLIGHT_COMPUTER
read: clean
read: build
read: flash-rs

.PHONY: list
list:
	@LC_ALL=C $(MAKE) -pRrq -f $(firstword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/(^|\n)# Files(\n|$$)/,/(^|\n)# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | grep -E -v -e '^[^[:alnum:]]' -e '^$@$$'

.DEFAULT_GOAL := list

hfc: CFLAGS += -DFLIGHT_COMPUTER
hfc: build

flight: CFLAGS += -DPROD
flight: hfc

dev: CFLAGS += -DFLIGHT_COMPUTER
dev: CFLAGS += -DLOGERROR
dev: CFLAGS += -DLOGWARN
dev: CFLAGS += -DLOGINFO
dev: CFLAGS += -DLOGDEBUG
dev: CFLAGS += -DQUIET
dev: hfc