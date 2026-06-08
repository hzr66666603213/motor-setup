#include "control/current_controller.h"

#include "foc/foc_math.h"

/*
 * current_controller.c
 *
 * FOC 电流环是最快的闭环，通常与 PWM 同频运行。
 *
 * 控制目标：
 * - d 轴通常控制为 0A，除非后续加入弱磁/MTPA；
 * - q 轴对应力矩电流；
 * - 输出 vd/vq 必须受母线电压和用户配置 voltage_limit 约束。
 *
 * 为什么需要电压限幅和抗积分饱和：
 * - 逆变器可合成的电压矢量有限，PI 输出超过这个范围时无法被硬件实现；
 * - 如果积分器继续累加，退出饱和后会带来明显电流冲击；
 * - 因此限幅后把“不可实现的电压差”回灌到积分器，降低 windup。
 */

void current_controller_init(CurrentController *controller, float kp, float ki, float max_voltage_v)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->integrator_d_v = 0.0f;
    controller->integrator_q_v = 0.0f;
    controller->integrator_limit_v = max_voltage_v;
    controller->max_voltage_v = max_voltage_v;
}

void current_controller_reset(CurrentController *controller)
{
    controller->integrator_d_v = 0.0f;
    controller->integrator_q_v = 0.0f;
}

void current_controller_set_gains(CurrentController *controller, float kp, float ki)
{
    controller->kp = kp;
    controller->ki = ki;
}

void current_controller_update(CurrentController *controller,
                               float id_target_a,
                               float iq_target_a,
                               float id_measured_a,
                               float iq_measured_a,
                               float vbus_v,
                               float dt_s,
                               float *vd_v,
                               float *vq_v)
{
    const float id_error_a = id_target_a - id_measured_a;
    const float iq_error_a = iq_target_a - iq_measured_a;

    const float bus_limited_v = 0.57735026919f * vbus_v;
    const float max_voltage_v = (controller->max_voltage_v < bus_limited_v) ?
                                controller->max_voltage_v :
                                bus_limited_v;

    controller->integrator_d_v += controller->ki * id_error_a * dt_s;
    controller->integrator_q_v += controller->ki * iq_error_a * dt_s;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);

    float vd_cmd_v = controller->kp * id_error_a + controller->integrator_d_v;
    float vq_cmd_v = controller->kp * iq_error_a + controller->integrator_q_v;
    const float vd_pre_limit_v = vd_cmd_v;
    const float vq_pre_limit_v = vq_cmd_v;

    foc_limit_voltage(&vd_cmd_v, &vq_cmd_v, max_voltage_v);

    controller->integrator_d_v += vd_cmd_v - vd_pre_limit_v;
    controller->integrator_q_v += vq_cmd_v - vq_pre_limit_v;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);

    *vd_v = vd_cmd_v;
    *vq_v = vq_cmd_v;
}

void current_controller_tune_from_rl(CurrentController *controller,
                                     float phase_resistance_ohm,
                                     float phase_inductance_h,
                                     float bandwidth_hz,
                                     float voltage_limit_v)
{
    if (controller == 0 ||
        phase_resistance_ohm <= 0.0f ||
        phase_inductance_h <= 0.0f ||
        bandwidth_hz <= 0.0f ||
        voltage_limit_v <= 0.0f) {
        return;
    }

    const float wc_rad_s = FOC_TWO_PI_F * bandwidth_hz;
    controller->kp = phase_inductance_h * wc_rad_s;
    controller->ki = phase_resistance_ohm * wc_rad_s;
    controller->max_voltage_v = voltage_limit_v;
    controller->integrator_limit_v = voltage_limit_v;
    current_controller_reset(controller);
}
