#include "hal/hal_time.h"

/*
 * hal_time.c
 *
 * 时间 mock/stub 实现。
 * 当前返回固定 mock 计数；移植时应绑定 SysTick、DWT 或硬件定时器。
 */

static uint32_t s_mock_us = 0u;

uint32_t hal_time_micros(void)
{
    return s_mock_us;
}

uint32_t hal_time_millis(void)
{
    return s_mock_us / 1000u;
}
