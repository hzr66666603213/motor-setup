#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

/*
 * velocity_controller.h
 *
 * 速度环 PI 控制器。
 * 运行频率：建议 1 kHz。
 * 输入：目标机械速度 rad/s、实测机械速度 rad/s。
 * 输出：q 轴电流目标 A。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                  /* 比例增益，A/(rad/s) */
    float ki;                  /* 积分增益，A/rad */
    float integrator_a;        /* 速度环积分项，A */
    float integrator_limit_a;  /* 积分限幅，A */
    float velocity_limit_rad_s;/* 速度限幅，rad/s */
    float current_limit_a;     /* 输出电流限幅，A */
} VelocityController;

/* 初始化速度 PI，限幅来自 MotorConfig。 */
void velocity_controller_init(VelocityController *controller, float kp, float ki, float current_limit_a, float velocity_limit_rad_s);

/* 清零积分器；退出闭环或切换模式时调用。 */
void velocity_controller_reset(VelocityController *controller);

/* 在线更新速度环增益。 */
void velocity_controller_set_gains(VelocityController *controller, float kp, float ki);

/* 执行一次速度 PI，返回 iq_target_A。 */
float velocity_controller_update(VelocityController *controller,
                                 float velocity_target_rad_s,
                                 float velocity_measured_rad_s,
                                 float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_CONTROLLER_H */
