#ifndef AXIS0_CURRENT_LOOP_ISR_H
#define AXIS0_CURRENT_LOOP_ISR_H

/*
 * axis0_current_loop_isr.h
 *
 * ODrive v3.6 Axis0 20kHz FOC 电流环 ISR 骨架。
 * ISR 写：Axis0Context.rt 中的电流、电角度、电压、duty、vbus。
 * 后台/1kHz 写：Axis0Context.cmd、requested_state、配置参数。
 * 共享数据最终建议使用双缓冲或临界区保护。
 */

#include "app/axis0_types.h"
#include "control/current_controller.h"
#include "drivers/current_sensor.h"
#include "drivers/drv8301.h"
#include "drivers/encoder_mt6701_abz.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Axis0Context *axis;
    CurrentController *current_controller;
    CurrentSensorConfig *current_sensor;
    EncoderMt6701AbzState *encoder;
    Drv8301 *drv0;               /* M0/Axis0 DRV8301 */
    Drv8301 *drv1;               /* M1/Axis1 DRV8301；Axis0-only 也要初始化并监控 */
} Axis0IsrContext;

void axis0_current_loop_isr(Axis0IsrContext *ctx, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_CURRENT_LOOP_ISR_H */
