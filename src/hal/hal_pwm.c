#include "hal/hal_pwm.h"

/*
 * hal_pwm.c
 *
 * PWM mock/stub 实现。
 * 当前只保存状态和 duty，不访问真实定时器。
 * 移植 STM32 时，应替换为 TIM1/TIM8 等高级定时器的 HAL/LL 实现。
 */

static bool s_pwm_enabled = false;
static float s_duty_u = 0.5f;
static float s_duty_v = 0.5f;
static float s_duty_w = 0.5f;

bool hal_pwm_init(void)
{
    /* 上电默认保持 PWM 禁止，中点 duty 便于后续安全启动。 */
    s_pwm_enabled = false;
    s_duty_u = 0.5f;
    s_duty_v = 0.5f;
    s_duty_w = 0.5f;
    return true;
}

void hal_pwm_enable(void)
{
    s_pwm_enabled = true;
}

void hal_pwm_disable(void)
{
    s_pwm_enabled = false;
}

void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
    /* mock 只记录 duty；真实后端应写 CCR 寄存器并考虑互补输出极性。 */
    s_duty_u = duty_u;
    s_duty_v = duty_v;
    s_duty_w = duty_w;
}

void hal_pwm_set_all_low(void)
{
    /* 真实硬件中应结合高低桥臂驱动逻辑定义 all low 的安全状态。 */
    s_duty_u = 0.0f;
    s_duty_v = 0.0f;
    s_duty_w = 0.0f;
}

bool hal_pwm_is_enabled(void)
{
    return s_pwm_enabled;
}
