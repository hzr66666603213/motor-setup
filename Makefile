# Makefile
#
# PC 侧最小测试构建脚本。
# 这些目标只编译纯算法文件，不包含 STM32 HAL 后端。

TOOL_DIR := D:/Tools/w64devkit/bin

ifeq ($(OS),Windows_NT)
CC := $(TOOL_DIR)/gcc.exe
EXE := .exe
GCC_TOOLCHAIN_HINT := -B$(TOOL_DIR)/
else
CC := gcc
EXE :=
GCC_TOOLCHAIN_HINT :=
endif

CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Iinclude -Isrc -Isrc/control -Isrc/foc -Isrc/sim
LDFLAGS ?= -lm

FOC_MATH_TEST_SRCS := tests/foc_math_test.c \
	src/foc/foc_math.c

FOC_SIM_TEST_SRCS := tests/foc_sim_test.c \
	src/sim/foc_sim.c \
	src/control/current_controller.c \
	src/foc/foc_math.c \
	src/foc/svpwm.c

FOC_PC_UNIT_TEST_SRCS := tests/main.c \
	src/sim/foc_sim.c \
	src/control/current_controller.c \
	src/foc/foc_math.c \
	src/foc/svpwm.c

.PHONY: test math_test sim_test pc_test clean build

test: math_test sim_test pc_test

math_test: build/foc_math_test$(EXE)
	./build/foc_math_test$(EXE)

sim_test: build/foc_sim_test$(EXE)
	./build/foc_sim_test$(EXE)

pc_test: build/foc_pc_unit_test$(EXE)
	./build/foc_pc_unit_test$(EXE)

build:
	mkdir -p build

build/foc_math_test$(EXE): $(FOC_MATH_TEST_SRCS) | build
	$(CC) $(GCC_TOOLCHAIN_HINT) $(CFLAGS) $(FOC_MATH_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_sim_test$(EXE): $(FOC_SIM_TEST_SRCS) | build
	$(CC) $(GCC_TOOLCHAIN_HINT) $(CFLAGS) $(FOC_SIM_TEST_SRCS) $(LDFLAGS) -o $@

build/foc_pc_unit_test$(EXE): $(FOC_PC_UNIT_TEST_SRCS) | build
	$(CC) $(GCC_TOOLCHAIN_HINT) $(CFLAGS) $(FOC_PC_UNIT_TEST_SRCS) $(LDFLAGS) -o $@

clean:
	rm -rf build