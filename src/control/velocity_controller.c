#include "control/velocity_controller.h"
#include "foc/foc_math.h"

/*
 * velocity_controller.c
 *
 * 速度环比电流环慢，第一版采用 PI。
 * 输出是 iq 目标电流，而不是直接输出 PWM 或电压。
 */

void velocity_controller_init(VelocityController *controller, float kp, float ki, float current_limit_a, float velocity_limit_rad_s)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->integrator_a = 0.0f;
    controller->integrator_limit_a = current_limit_a;
    controller->velocity_limit_rad_s = velocity_limit_rad_s;
    controller->current_limit_a = current_limit_a;
}

void velocity_controller_reset(VelocityController *controller)
{
    controller->integrator_a = 0.0f;
}

void velocity_controller_set_gains(VelocityController *controller, float kp, float ki)
{
    controller->kp = kp;
    controller->ki = ki;
}

float velocity_controller_update(VelocityController *controller,
                                 float velocity_target_rad_s,
                                 float velocity_measured_rad_s,
                                 float dt_s)
{
    /* 先限制速度目标，避免通信写入异常值导致速度环直接打满。 */
    const float target = foc_clamp(velocity_target_rad_s,
                                   -controller->velocity_limit_rad_s,
                                   controller->velocity_limit_rad_s);
    const float error = target - velocity_measured_rad_s;

    /* 积分项单位为 A，用于消除稳态速度误差。 */
    controller->integrator_a += controller->ki * error * dt_s;
    controller->integrator_a = foc_clamp(controller->integrator_a,
                                         -controller->integrator_limit_a,
                                         controller->integrator_limit_a);

    /* 输出 iq 电流目标，后续由 20 kHz 电流环跟踪。 */
    float iq_target_a = controller->kp * error + controller->integrator_a;
    iq_target_a = foc_clamp(iq_target_a, -controller->current_limit_a, controller->current_limit_a);
    return iq_target_a;
}
