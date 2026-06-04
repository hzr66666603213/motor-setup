CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Iinclude
LDFLAGS ?= -lm

# Makefile
#
# PC 侧最小测试构建脚本。
# 当前只编译 FOC 数学 smoke test，不包含 STM32 HAL 后端。
# 嵌入式工程移植时，可把 include/ 和 src/ 下的模块加入 CubeMX/Make/CMake 工程。

FOC_MATH_TEST_SRCS := tests/foc_math_test.c src/foc/foc_math.c

.PHONY: test clean

# 构建并运行 FOC 数学测试。
test: build/foc_math_test
	./build/foc_math_test

# 生成测试可执行文件；需要本机安装 gcc 或兼容编译器。
build/foc_math_test: $(FOC_MATH_TEST_SRCS)
	mkdir -p build
	$(CC) $(CFLAGS) $(FOC_MATH_TEST_SRCS) $(LDFLAGS) -o $@

# 清理 PC 侧构建产物，不影响固件源码。
clean:
	rm -rf build
