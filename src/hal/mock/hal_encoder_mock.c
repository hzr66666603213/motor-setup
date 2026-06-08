#include "hal/hal_encoder.h"

/*
 * hal_encoder_mock.c
 *
 * Encoder0 mock 后端。
 * 用于 PC 测试或没有真实 TIM3 的构建；真实固件应使用 STM32F405 后端。
 */

static int32_t s_mock_count = 0;
static bool s_mock_index_found = false;

bool hal_encoder0_init(void)
{
    s_mock_count = 0;
    s_mock_index_found = false;
    return true;
}

int32_t hal_encoder0_get_count(void)
{
    return s_mock_count;
}

void hal_encoder0_reset_count(void)
{
    s_mock_count = 0;
    s_mock_index_found = false;
}

bool hal_encoder0_index_found(void)
{
    return s_mock_index_found;
}
