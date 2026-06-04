#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/*
 * control_task.h
 *
 * 慢速控制任务接口。
 * 运行频率：建议 1 kHz。
 * 职责：根据 control_mode 将输入指令转换为电流环可消费的力矩/iq 目标。
 */

#include "control/position_controller.h"
#include "control/velocity_controller.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1 kHz 控制任务；最终工程中建议写入双缓冲命令，再由 20 kHz ISR 读取。 */
void control_task_1khz(Axis *axis,
                       VelocityController *velocity_controller,
                       PositionController *position_controller,
                       float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_TASK_H */
