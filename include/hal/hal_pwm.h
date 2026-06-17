#ifndef HAL_PWM_H
#define HAL_PWM_H

/*
 * hal_pwm.h
 *
 * PWM / TIM1 硬件抽象接口。
 *
 * 重要区分：
 * - hal_pwm_enable()：允许三相互补 PWM 输出，用于真实功率级驱动；
 * - hal_pwm_disable()：停止 PWM 输出和 TIM 触发，故障/IDLE 安全关断用；
 * - hal_pwm_start_adc_trigger_only()：只让 TIM1 继续产生 ADC 同步触发，
 *   但保持 MOE=0、EN_GATE=0，不驱动 MOS，用于电流零偏等无功率级采样。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hal_pwm_init(void);
void hal_pwm_enable(void);
void hal_pwm_disable(void);
void hal_pwm_start_adc_trigger_only(void);
void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w);
void hal_pwm_set_all_low(void);
bool hal_pwm_is_enabled(void);

typedef struct {
    uint32_t base_start_status;
    uint32_t oc4_start_status;
    uint32_t start_count;
} HalPwmDiagnostics;

void hal_pwm_get_diagnostics(HalPwmDiagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* HAL_PWM_H */
