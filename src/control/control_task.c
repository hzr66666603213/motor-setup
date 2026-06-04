#include "control/control_task.h"
#include "foc/foc_math.h"

/*
 * control_task.c
 *
 * 该任务运行在 1 kHz 左右，负责外环控制：
 * - TORQUE_CONTROL：力矩直接换算为 iq。
 * - VELOCITY_CONTROL：速度 PI 输出 iq。
 * - POSITION_CONTROL：位置 P 输出速度目标，再由速度 PI 输出 iq。
 * 当前骨架将最终 iq 重新写回 input.torque_nm，供 ISR 统一换算。
 * 工程化版本建议单独建立 CurrentCommand 双缓冲结构。
 */

void control_task_1khz(Axis *axis,
                       VelocityController *velocity_controller,
                       PositionController *position_controller,
                       float dt_s)
{
    if (axis->axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        /* 非闭环状态下清速度积分，避免重新闭环时历史误差造成冲击。 */
        velocity_controller_reset(velocity_controller);
        return;
    }

    float iq_target_a = 0.0f;
    if (axis->axis.control_mode == CONTROL_MODE_TORQUE) {
        iq_target_a = axis->input.torque_nm / axis->motor.torque_constant_nm_per_a;
    } else if (axis->axis.control_mode == CONTROL_MODE_VELOCITY) {
        iq_target_a = velocity_controller_update(velocity_controller,
                                                 axis->input.velocity_rad_s,
                                                 axis->motor_state.mechanical_velocity_rad_s,
                                                 dt_s);
    } else if (axis->axis.control_mode == CONTROL_MODE_POSITION) {
        /* 位置阶跃先变为受限速度目标，再进入速度环。 */
        const float velocity_target = position_controller_update(position_controller,
                                                                 axis->input.position_rad,
                                                                 axis->motor_state.mechanical_angle_rad);
        iq_target_a = velocity_controller_update(velocity_controller,
                                                 velocity_target,
                                                 axis->motor_state.mechanical_velocity_rad_s,
                                                 dt_s);
    } else {
        iq_target_a = 0.0f;
    }

    iq_target_a = foc_clamp(iq_target_a, -axis->motor.current_limit_a, axis->motor.current_limit_a);
    /* 临时命令交接：把 iq 转回力矩，ISR 统一 torque/kt 得到 iq。 */
    axis->input.torque_nm = iq_target_a * axis->motor.torque_constant_nm_per_a;
}
