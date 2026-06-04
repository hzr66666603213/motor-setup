#ifndef AXIS_STATE_MACHINE_H
#define AXIS_STATE_MACHINE_H

/*
 * axis_state_machine.h
 *
 * 单轴状态机接口。
 * 运行频率：建议 1 kHz 或后台周期任务。
 * 状态机负责“是否允许闭环”和故障安全关断；PWM ISR 只执行快速控制和快速保护。
 */

#include <stdbool.h>
#include "calibration/motor_calibration.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    MotorCalibrationContext motor_calib;       /* 当前校准流程上下文 */
    MotorCalibrationConfig motor_calib_config; /* 校准限幅和时间配置 */
} AxisStateMachineContext;

/* 初始化状态机，默认进入 BOOT 并保持安全输出。 */
void axis_sm_init(Axis *axis, AxisStateMachineContext *ctx);
/* 请求状态切换；实际切换会在 axis_sm_update 中检查安全条件。 */
void axis_sm_request_state(Axis *axis, AxisStateId requested_state);
/* 周期推进状态机；禁止阻塞。 */
void axis_sm_update(Axis *axis, AxisStateMachineContext *ctx, float dt_s);
/* 清除故障并请求回到 IDLE。 */
void axis_sm_clear_faults(Axis *axis);

/* 以下 enter/update/exit 函数便于后续按状态扩展动作。 */
void axis_sm_enter_boot(Axis *axis);
void axis_sm_update_boot(Axis *axis);
void axis_sm_exit_boot(Axis *axis);
void axis_sm_enter_idle(Axis *axis);
void axis_sm_update_idle(Axis *axis);
void axis_sm_exit_idle(Axis *axis);
void axis_sm_enter_motor_calibration(Axis *axis, AxisStateMachineContext *ctx);
void axis_sm_update_motor_calibration(Axis *axis, AxisStateMachineContext *ctx, float dt_s);
void axis_sm_exit_motor_calibration(Axis *axis);
void axis_sm_enter_encoder_offset_calibration(Axis *axis, AxisStateMachineContext *ctx);
void axis_sm_update_encoder_offset_calibration(Axis *axis, AxisStateMachineContext *ctx, float dt_s);
void axis_sm_exit_encoder_offset_calibration(Axis *axis);
void axis_sm_enter_closed_loop(Axis *axis);
void axis_sm_update_closed_loop(Axis *axis);
void axis_sm_exit_closed_loop(Axis *axis);
void axis_sm_enter_fault(Axis *axis);
void axis_sm_update_fault(Axis *axis);
void axis_sm_exit_fault(Axis *axis);

#ifdef __cplusplus
}
#endif

#endif /* AXIS_STATE_MACHINE_H */
