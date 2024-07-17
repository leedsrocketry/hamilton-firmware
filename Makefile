BUILD_DIR := build

CFLAGS  ?=  -W -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -Wno-unknown-pragmas \
            -g3 -O0 -ffunction-sections -fdata-sections -Isrc -Isrc/include \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS) \
			-lm
LDFLAGS ?= -Tbuild/link.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/firmware.map
SOURCES ?=	src/main.c src/startup.c src/syscalls.c src/HAL/STM32_init.c src/drivers/MS5611_driver.c src/filters.c\
			src/drivers/ADXL375_driver.c src/tests/test_routines.c src/data_buffer.c src/drivers/LSM6DS3_driver.c\
			src/drivers/BME280_driver.c

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
	openocd -f "src/debug/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg"

unblock-write-protected:
	openocd -f "src/debug/OpenOCD/openocd/scripts/interface/stlink.cfg" \
          -f "src/debug/OpenOCD/openocd/scripts/target/stm32l4x.cfg" \
          -c "init; reset halt; stm32l4x unlock 0; stm32l4x mass_erase 0; program $(BUILD_DIR)/firmware.bin 0x08000000 verify reset; exit"

# Different targets for HFC/Nucleo
nucleo: build
nucleo-flash: nucleo flash

hfc: CFLAGS += -DFLIGHT_COMPUTER
hfc: build

hfc-flash: hfc flash

warnings:
	@(make clean && make hfc > $(BUILD_DIR)/make.log 2>&1) && grep "warning:" $(BUILD_DIR)/make.log | wc -l
