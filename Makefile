CFLAGS  ?=  -W -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -Wno-unknown-pragmas \
            -g3 -O0 -ffunction-sections -fdata-sections -I. -Iinclude \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS) \
			-lm
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES ?=	main.c startup.c syscalls.c STM32_init.c drivers/MS5611_driver.c filters.c\
			drivers/ADXL375_driver.c test_routines.c data_buffer.c drivers/LSM6DS3_driver.c\
			drivers/BME280_driver.c

# Ensure make clean is cross platform
ifeq ($(OS), Windows_NT)
	RM = del
else
	RM = rm
endif

build: firmware.bin

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x8000000

clean:
	$(RM) -rf firmware.*

debug-hardware:
	openocd -f "debug/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg"

unblock-write-protected:
	openocd -f "debug/OpenOCD/openocd/scripts/interface/stlink.cfg" \
          -f "debug/OpenOCD/openocd/scripts/target/stm32l4x.cfg" \
          -c "init; reset halt; stm32l4x unlock 0; stm32l4x mass_erase 0; program firmware.bin 0x08000000 verify reset; exit"

# Different targets for HFC/Nucleo
nucleo: build
nucleo-flash: nucleo flash

hfc: CFLAGS += -DFLIGHT_COMPUTER
hfc: build

hfc-flash: hfc flash