#include "control/current_controller.h"
#include "foc/foc_math.h"

/*
 * current_controller.c
 *
 * 电流环是 FOC 的最快闭环，通常与 PWM 同频运行。
 * 这里实现最小可用的 d/q 轴 PI：
 * - d 轴通常控制到 0 A，除非后续加入弱磁或 MTPA。
 * - q 轴对应力矩电流。
 * - 输出 vd/vq 必须受母线电压和配置电压限制约束。
 */

void current_controller_init(CurrentController *controller, float kp, float ki, float max_voltage_v)
{
    /* 固定分配控制器对象，不在运行期申请内存。 */
    controller->kp = kp;
    controller->ki = ki;
    controller->integrator_d_v = 0.0f;
    controller->integrator_q_v = 0.0f;
    controller->integrator_limit_v = max_voltage_v;
    controller->max_voltage_v = max_voltage_v;
}

void current_controller_reset(CurrentController *controller)
{
    /* 防止重新使能闭环时带着历史积分量产生电流冲击。 */
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
    /* 误差单位为 A，PI 输出单位为 V。 */
    const float id_error_a = id_target_a - id_measured_a;
    const float iq_error_a = iq_target_a - iq_measured_a;

    /*
     * SVPWM 在线性区可用相电压矢量约为 vbus/sqrt(3)。
     * 同时还要服从用户配置的 max_voltage_v。
     */
    const float bus_limited_v = 0.57735026919f * vbus_v;
    const float max_voltage_v = (controller->max_voltage_v < bus_limited_v) ? controller->max_voltage_v : bus_limited_v;

    /* 先积分再限幅，避免积分项本身无限增长。 */
    controller->integrator_d_v += controller->ki * id_error_a * dt_s;
    controller->integrator_q_v += controller->ki * iq_error_a * dt_s;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v, -controller->integrator_limit_v, controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v, -controller->integrator_limit_v, controller->integrator_limit_v);

    float vd_cmd_v = controller->kp * id_error_a + controller->integrator_d_v;
    float vq_cmd_v = controller->kp * iq_error_a + controller->integrator_q_v;
    const float vd_pre_limit_v = vd_cmd_v;
    const float vq_pre_limit_v = vq_cmd_v;

    /*
     * 逆变器只能合成有限电压矢量。
     * 圆形限幅保持 vd/vq 方向，尽量不破坏电流控制解耦关系。
     */
    foc_limit_voltage(&vd_cmd_v, &vq_cmd_v, max_voltage_v);

    /*
     * 抗积分饱和：
     * 如果 PI 输出被电压限幅截断，说明当前积分项包含了不可实现的电压需求。
     * 将限幅前后的差值回灌到积分器，可以减少退出饱和时的电流冲击。
     */
    controller->integrator_d_v += vd_cmd_v - vd_pre_limit_v;
    controller->integrator_q_v += vq_cmd_v - vq_pre_limit_v;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v, -controller->integrator_limit_v, controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v, -controller->integrator_limit_v, controller->integrator_limit_v);

    *vd_v = vd_cmd_v;
    *vq_v = vq_cmd_v;
}
