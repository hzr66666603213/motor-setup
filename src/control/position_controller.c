#include "control/position_controller.h"
#include "foc/foc_math.h"

/*
 * position_controller.c
 *
 * 位置环第一版只做 P 控制，并输出速度目标。
 * 这样可以避免位置阶跃直接变成电流阶跃，冲击由速度环和电流限幅继续约束。
 */

void position_controller_init(PositionController *controller,
                              float kp,
                              float velocity_limit_rad_s,
                              float position_min_rad,
                              float position_max_rad)
{
    controller->kp = kp;
    controller->velocity_limit_rad_s = velocity_limit_rad_s;
    controller->position_min_rad = position_min_rad;
    controller->position_max_rad = position_max_rad;
}

void position_controller_set_gain(PositionController *controller, float kp)
{
    controller->kp = kp;
}

float position_controller_update(PositionController *controller,
                                 float position_target_rad,
                                 float position_measured_rad)
{
    /* 先做位置软限位，避免指令越过机械安全范围。 */
    const float target = foc_clamp(position_target_rad,
                                   controller->position_min_rad,
                                   controller->position_max_rad);
    const float error = target - position_measured_rad;

    /* P 位置环输出速度目标，再由速度 PI 转成 iq。 */
    const float velocity_target = controller->kp * error;
    return foc_clamp(velocity_target,
                     -controller->velocity_limit_rad_s,
                     controller->velocity_limit_rad_s);
}
