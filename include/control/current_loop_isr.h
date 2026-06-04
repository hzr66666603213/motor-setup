#ifndef CURRENT_LOOP_ISR_H
#define CURRENT_LOOP_ISR_H

/*
 * current_loop_isr.h
 *
 * 20 kHz PWM/ADC 同步中断入口。
 * 该函数只描述控制流程，不绑定 STM32 HAL。
 * 实际工程中通常从 ADC 转换完成中断或 PWM 更新中断调用。
 */

#include "control/current_controller.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 20 kHz PWM ISR 入口：读取采样、执行 FOC、电流 PI、SVPWM，并做快速保护。 */
void pwm_current_loop_isr(Axis *axis, CurrentController *current_controller, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_LOOP_ISR_H */
