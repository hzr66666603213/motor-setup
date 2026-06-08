#include "hal/hal_pwm.h"

/*
 * hal_pwm_mock.c
 *
 * PWM mock 后端。
 * 只保存 enable 状态和 duty，不访问 TIM1。真实固件应使用 STM32F405 后端。
 */

static bool s_pwm_enabled = false;
static float s_duty_u = 0.5f;
static float s_duty_v = 0.5f;
static float s_duty_w = 0.5f;

bool hal_pwm_init(void)
{
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

void hal_pwm_start_adc_trigger_only(void)
{
    s_pwm_enabled = false;
    s_duty_u = 0.5f;
    s_duty_v = 0.5f;
    s_duty_w = 0.5f;
}

void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
    s_duty_u = duty_u;
    s_duty_v = duty_v;
    s_duty_w = duty_w;
}

void hal_pwm_set_all_low(void)
{
    s_duty_u = 0.0f;
    s_duty_v = 0.0f;
    s_duty_w = 0.0f;
}

bool hal_pwm_is_enabled(void)
{
    return s_pwm_enabled;
}
