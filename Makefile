# Makefile
#
# PC 侧最小测试构建脚本。
# 这些目标只编译纯算法文件，不包含 STM32 HAL 后端。

ifeq ($(origin CC),default)
CC = gcc
endif

ifeq ($(OS),Windows_NT)
EXE := .exe
else
EXE :=
endif

CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Iinclude -Isrc -Isrc/control -Isrc/foc -Isrc/sim
LDFLAGS ?= -lm

FOC_MATH_TEST_SRCS := tests/foc_math_test.c \
	src/foc/foc_math.c

FOC_SIM_TEST_SRCS := tests/foc_sim_test.c \
	src/sim/foc_sim.c \
	src/control/current_controller.c \
	src/control/velocity_controller.c \
	src/foc/foc_math.c \
	src/foc/svpwm.c

FOC_PC_UNIT_TEST_SRCS := tests/main.c \
	src/sim/foc_sim.c \
	src/control/current_controller.c \
	src/control/velocity_controller.c \
	src/foc/foc_math.c \
	src/foc/svpwm.c

BOARD_ADC_SAMPLING_TEST_SRCS := tests/board_adc_sampling_test.c \
	src/board/board_odrive_v36.c \
	src/hal/mock/hal_adc_mock.c \
	src/hal/mock/hal_gpio_mock.c \
	src/hal/mock/hal_pwm_mock.c

.PHONY: test math_test sim_test pc_test clean build

test: math_test sim_test pc_test board_adc_sampling_test

math_test: build/foc_math_test$(EXE)
	./build/foc_math_test$(EXE)

sim_test: build/foc_sim_test$(EXE)
	./build/foc_sim_test$(EXE)

pc_test: build/foc_pc_unit_test$(EXE)
	./build/foc_pc_unit_test$(EXE)

board_adc_sampling_test: build/board_adc_sampling_test$(EXE)
	./build/board_adc_sampling_test$(EXE)

build:
	mkdir -p build

build/foc_math_test$(EXE): $(FOC_MATH_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_MATH_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_sim_test$(EXE): $(FOC_SIM_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_SIM_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_pc_unit_test$(EXE): $(FOC_PC_UNIT_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_PC_UNIT_TEST_SRCS) $(LDFLAGS) -o $@

build/board_adc_sampling_test$(EXE): $(BOARD_ADC_SAMPLING_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(BOARD_ADC_SAMPLING_TEST_SRCS) $(LDFLAGS) -o $@

clean:
	rm -rf build
