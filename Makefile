CFLAGS  ?=  -W -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -Wno-unknown-pragmas \
            -g3 -Os -ffunction-sections -fdata-sections -I. -Iinclude \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c STM32_init.c drivers/MS5611_driver.c

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

debug:
	openocd -f "$(PWD)/debug/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg

# Different targets for HFC/Nucleo
nucleo: build
nucleo-flash: nucleo flash

hfc: CFLAGS += -DFLIGHT_COMPUTER
hfc: build

hfc-flash: hfc flash