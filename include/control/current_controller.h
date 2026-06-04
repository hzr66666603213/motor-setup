#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

/*
 * current_controller.h
 *
 * d/q 轴电流 PI 控制器接口。
 * 运行上下文：20 kHz PWM ISR。
 * 输入：id/iq 目标电流、id/iq 实测电流、母线电压、dt。
 * 输出：vd/vq 电压指令。
 * 注意：ISR 中不得调用阻塞函数，也不得使用动态内存。
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

/* 初始化控制器参数，并清零积分器。 */
void current_controller_init(CurrentController *controller, float kp, float ki, float max_voltage_v);

/* 清零积分器；退出闭环、故障恢复或切换模式时应调用。 */
void current_controller_reset(CurrentController *controller);

/* 在线更新 PI 参数；建议在慢速任务中写入，然后安全同步到 ISR。 */
void current_controller_set_gains(CurrentController *controller, float kp, float ki);

/* 执行一次 d/q 电流 PI 更新，适合 20 kHz ISR 调用。 */
void current_controller_update(CurrentController *controller,
                               float id_target_a,
                               float iq_target_a,
                               float id_measured_a,
                               float iq_measured_a,
                               float vbus_v,
                               float dt_s,
                               float *vd_v,
                               float *vq_v);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_CONTROLLER_H */
