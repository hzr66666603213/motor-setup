#include "hal/hal_pwm.h"

/*
 * hal_pwm_stm32f405.c
 *
 * ODrive v3.6 Axis0 TIM1 互补 PWM 真实后端。
 *
 * 资源假设：
 * - TIM1_CH1/2/3：PA8/PA9/PA10，高边 PWM；
 * - TIM1_CH1N/2N/3N：PB13/PB14/PB15，低边互补 PWM；
 * - TIM1_CH4：建议作为 injected ADC 采样触发点；
 * - CubeMX/LL 负责配置中心对齐、死区、break/off-state、PWM 频率 20 kHz。
 *
 * 安全语义：
 * - hal_pwm_disable() 关闭 MOE，并停止三相 CH/CHN；
 * - fault/idle 还必须由 board/DRV 层拉低 EN_GATE；
 * - hal_pwm_start_adc_trigger_only() 只运行 TIM1/CC4 触发 ADC，不启动三相功率通道。
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
     * 真正使能功率级 PWM。
     * 调用前 board 层必须已经确认 fault、VBUS、ADC、编码器、DRV 状态满足准入条件。
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
     * 故障/IDLE 的全关断：停止三相功率 PWM 和 ADC 触发。
     * current_offset_calibration 不应调用这个函数，因为它需要 TIM1 继续触发 ADC。
     */
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_Base_Stop(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    s_pwm_enabled = false;
}

void hal_pwm_start_adc_trigger_only(void)
{
    /*
     * ADC trigger-only 模式：
     * - 入口先关闭 MOE；
     * - 不启动 CH1/2/3 和 CH1N/2N/3N；
     * - 默认只启动 TIM1 base + OC4，让 TIM1_CC4 触发 ADC injected conversion；
     * - 如果你的 CubeMX 使用 TIM1 TRGO/update 触发 ADC，可以保留 Base_Start 并按实物配置调整。
     */
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_to_ccr(0.5f));

    (void)HAL_TIM_Base_Start(&htim1);
    (void)HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

    __HAL_TIM_MOE_DISABLE(&htim1);
    s_pwm_enabled = false;
}

void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
    /*
     * 20 kHz ISR 内调用，必须只写寄存器，不阻塞。
     * TIM1 互补输出由硬件和死区单元生成，软件只更新 CCR1/2/3。
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_to_ccr(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_to_ccr(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_to_ccr(duty_w));
}

void hal_pwm_set_all_low(void)
{
    /*
     * 第一阶段 bring-up 默认安全态是“MOE=0 + EN_GATE=0”，不把低边全开当作安全态。
     */
    hal_pwm_disable();
}

bool hal_pwm_is_enabled(void)
{
    return s_pwm_enabled;
}
