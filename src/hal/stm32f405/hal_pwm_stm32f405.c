#include "hal/hal_pwm.h"

/*
 * hal_pwm_stm32f405.c
 *
 * ODrive v3.6 Axis0 TIM1 互补 PWM 真实后端。
 *
 * 资源假设：
 * - TIM1_CH1/2/3：PA8/PA9/PA10，高边 PWM；
 * - TIM1_CH1N/2N/3N：PB13/PB14/PB15，低边互补 PWM；
 * - CubeMX/LL 负责配置中心对齐、死区、break/off-state、PWM 频率 20 kHz。
 *
 * 安全语义：
 * - hal_pwm_disable() 不用 duty=0 表示安全，而是关闭 TIM1 主输出 MOE 并停止
 *   CH1/2/3 和 CH1N/2N/3N；
 * - fault/idle 时还应由 board/DRV 层拉低 EN_GATE；
 * - hal_pwm_set_all_low() 在真实后端中等价于“清 CCR 并关闭主输出”，不依赖
 *   低边全开这种高风险状态。
 */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;

static bool s_pwm_enabled = false;

static float clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static uint32_t duty_to_ccr(float duty)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    return (uint32_t)(clamp01(duty) * (float)arr + 0.5f);
}

bool hal_pwm_init(void)
{
    hal_pwm_disable();
    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    return true;
}

void hal_pwm_enable(void)
{
    /*
     * 先启动普通通道和互补通道，再打开 MOE。
     * 如果 CubeMX 已经配置 Break/Deadtime，这里会使用对应 BDTR 配置。
     */
    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);
    s_pwm_enabled = true;
}

void hal_pwm_disable(void)
{
    /*
     * 这是故障路径可重复调用的安全关断：
     * 1. 关闭 TIM1 主输出 MOE；
     * 2. 停止 3 路主 PWM 和 3 路互补 PWM；
     * 3. 清 CCR，避免下次启动前残留旧 duty。
     */
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    s_pwm_enabled = false;
}

void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
    /*
     * 20 kHz ISR 内调用，必须只写寄存器，不阻塞。
     * TIM1 互补输出由定时器硬件和死区单元生成，软件只更新 CCR1/2/3。
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_to_ccr(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_to_ccr(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_to_ccr(duty_w));
}

void hal_pwm_set_all_low(void)
{
    /*
     * 对第一次 bring-up，安全态采用“PWM 主输出关闭 + EN_GATE 关闭”，
     * 不把“低边全开”当作默认安全态。
     */
    hal_pwm_disable();
}

bool hal_pwm_is_enabled(void)
{
    return s_pwm_enabled;
}
