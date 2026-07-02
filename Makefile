# Makefile
#
# PC 侧最小测试构建脚本。
# 这些目标只编译纯算法文件，不包含 STM32 HAL 后端。

ifeq ($(origin CC),default)
CC = gcc
endif

ifeq ($(OS),Windows_NT)
EXE := .exe
MKDIR_BUILD := if not exist build mkdir build
CLEAN_BUILD := if exist build rmdir /S /Q build
else
EXE :=
MKDIR_BUILD := mkdir -p build
CLEAN_BUILD := rm -rf build
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

IDENTIFICATION_PIPELINE_TEST_SRCS := tests/identification_pipeline_test.c \
	src/current_sample_pipeline.c \
	src/motor_identification.c \
	src/drivers/drv8301.c \
	src/hal/mock/hal_spi_mock.c \
	src/hal/mock/hal_gpio_mock.c \
	src/hal/mock/hal_pwm_mock.c \
	src/board/board_odrive_v36.c \
	src/hal/mock/hal_adc_mock.c

CURRENT_CONTROLLER_TEST_SRCS := tests/current_controller_test.c \
	src/control/current_controller.c \
	src/control/fixed_rotor_current_test.c \
	src/foc/foc_math.c

.PHONY: test math_test sim_test pc_test board_adc_sampling_test identification_pipeline_test current_controller_test clean build

test: math_test sim_test pc_test board_adc_sampling_test identification_pipeline_test current_controller_test

math_test: build/foc_math_test$(EXE)
	./build/foc_math_test$(EXE)

sim_test: build/foc_sim_test$(EXE)
	./build/foc_sim_test$(EXE)

pc_test: build/foc_pc_unit_test$(EXE)
	./build/foc_pc_unit_test$(EXE)

board_adc_sampling_test: build/board_adc_sampling_test$(EXE)
	./build/board_adc_sampling_test$(EXE)

identification_pipeline_test: build/identification_pipeline_test$(EXE)
	./build/identification_pipeline_test$(EXE)

current_controller_test: build/current_controller_test$(EXE)
	./build/current_controller_test$(EXE)

build:
	$(MKDIR_BUILD)

build/foc_math_test$(EXE): $(FOC_MATH_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_MATH_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_sim_test$(EXE): $(FOC_SIM_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_SIM_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_pc_unit_test$(EXE): $(FOC_PC_UNIT_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(FOC_PC_UNIT_TEST_SRCS) $(LDFLAGS) -o $@

build/board_adc_sampling_test$(EXE): $(BOARD_ADC_SAMPLING_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(BOARD_ADC_SAMPLING_TEST_SRCS) $(LDFLAGS) -o $@

build/identification_pipeline_test$(EXE): $(IDENTIFICATION_PIPELINE_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(IDENTIFICATION_PIPELINE_TEST_SRCS) $(LDFLAGS) -o $@

build/current_controller_test$(EXE): $(CURRENT_CONTROLLER_TEST_SRCS) | build
	$(CC) $(CFLAGS) $(CURRENT_CONTROLLER_TEST_SRCS) $(LDFLAGS) -o $@

clean:
	$(CLEAN_BUILD)
