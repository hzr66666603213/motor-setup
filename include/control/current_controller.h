#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

/*
 * current_controller.h
 *
 * d/q 轴电流 PI 控制器。
 *
 * 调用频率：
 * - current_controller_update()：20 kHz PWM ISR；
 * - tune/set/reset：后台、状态切换或校准完成后调用。
 *
 * 单位：
 * - 电流 A；
 * - 电压 V；
 * - 时间 s；
 * - 电阻 ohm；
 * - 电感 H。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                 /* 比例增益，V/A */
    float ki;                 /* 积分增益，V/(A*s) */
    float integrator_d_v;     /* d 轴积分项，V */
    float integrator_q_v;     /* q 轴积分项，V */
    float integrator_limit_v; /* 积分项限幅，V */
    float max_voltage_v;      /* 电压矢量最大幅值，V */
} CurrentController;

void current_controller_init(CurrentController *controller, float kp, float ki, float max_voltage_v);
void current_controller_reset(CurrentController *controller);
void current_controller_set_gains(CurrentController *controller, float kp, float ki);

void current_controller_update(CurrentController *controller,
                               float id_target_a,
                               float iq_target_a,
                               float id_measured_a,
                               float iq_measured_a,
                               float vbus_v,
                               float dt_s,
                               float *vd_v,
                               float *vq_v);

/*
 * 根据电机 R/L 和目标电流环带宽估算 PI 参数。
 *
 * 经验公式：
 * - Kp ≈ L * wc
 * - Ki ≈ R * wc
 * - wc = 2*pi*bandwidth_hz
 *
 * 第一次上真实 2804 小电机建议 bandwidth_hz 先取 300~800 Hz，
 * 并配合很低的 current_limit / voltage_limit。
 */
void current_controller_tune_from_rl(CurrentController *controller,
                                     float phase_resistance_ohm,
                                     float phase_inductance_h,
                                     float bandwidth_hz,
                                     float voltage_limit_v);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_CONTROLLER_H */
