#ifndef HAL_PWM_H
#define HAL_PWM_H

/*
 * hal_pwm.h
 *
 * PWM 硬件抽象接口。
 * FOC 和控制模块只能调用本接口，不能直接访问 STM32 TIM/HAL/LL。
 * 实际移植时应在 src/hal/hal_pwm.c 中绑定高级定时器、互补 PWM 和死区配置。
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 PWM 后端，配置完成但默认不输出功率。 */
bool hal_pwm_init(void);
/* 允许 PWM 输出；调用前应确保 gate driver 和保护条件正常。 */
void hal_pwm_enable(void);
/* 禁止 PWM 输出；故障和 IDLE 状态必须可立即调用。 */
void hal_pwm_disable(void);
/* 设置 U/V/W 三相 duty，范围应为 0..1。 */
void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w);
/* 将三相输出置为安全低电平；具体含义由硬件后端实现。 */
void hal_pwm_set_all_low(void);
/* 查询 mock/后端当前是否认为 PWM 已使能。 */
bool hal_pwm_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_PWM_H */
