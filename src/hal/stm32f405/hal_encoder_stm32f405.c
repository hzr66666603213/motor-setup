#include "hal/hal_encoder.h"

/*
 * hal_encoder_stm32f405.c
 *
 * ODrive v3.6 Encoder0 / MT6701 ABZ 的 TIM3 Encoder Mode 后端。
 *
 * 资源假设：
 * - Encoder0 A：PB4 / TIM3_CH1
 * - Encoder0 B：PB5 / TIM3_CH2
 * - Encoder0 Z：PC9，可选，第一版允许不用
 *
 * TIM3 是 16-bit 定时器，所以这里把 CNT 差分展开到 int32_t 软累计计数。
 */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

static uint16_t s_prev_tim3_cnt = 0u;
static int32_t s_encoder_count_accum = 0;
static bool s_index_found = false;

bool hal_encoder0_init(void)
{
    s_prev_tim3_cnt = 0u;
    s_encoder_count_accum = 0;
    s_index_found = false;
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    return HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) == HAL_OK;
}

int32_t hal_encoder0_get_count(void)
{
    /*
     * 16-bit CNT 差分展开。
     * int16_t 强制转换利用补码溢出特性，可以自然处理 0xffff -> 0x0000
     * 和 0x0000 -> 0xffff 的短周期跨界。
     */
    const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    const int16_t delta = (int16_t)(now - s_prev_tim3_cnt);
    s_prev_tim3_cnt = now;
    s_encoder_count_accum += (int32_t)delta;
    return s_encoder_count_accum;
}

void hal_encoder0_reset_count(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    s_prev_tim3_cnt = 0u;
    s_encoder_count_accum = 0;
    s_index_found = false;
}

bool hal_encoder0_index_found(void)
{
    return s_index_found;
}
