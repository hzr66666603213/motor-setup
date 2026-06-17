TOOLCHAIN ?= D:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin
CC := $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY := $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE := $(TOOLCHAIN)/arm-none-eabi-size

CFLAGS := -mcpu=cortex-m3 -mthumb -std=c11 -Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections
LDFLAGS := -mcpu=cortex-m3 -mthumb -TSTM32F103C8TX_FLASH.ld -Wl,--gc-sections -nostartfiles

BUILD := build
TARGET := $(BUILD)/c8t6_gpio_station

SRCS := startup_stm32f103c8tx.s main.c

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex
	$(SIZE) $(TARGET).elf

$(BUILD):
	mkdir $(BUILD)

$(TARGET).elf: $(SRCS) STM32F103C8TX_FLASH.ld | $(BUILD)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rmdir /S /Q $(BUILD)
