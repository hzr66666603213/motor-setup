#ifndef FOC_SIM_H
#define FOC_SIM_H

/*
 * foc_sim.h
 *
 * Simulink/PC 侧 FOC 仿真入口。
 *
 * 设计目标：
 * - 只依赖纯算法模块：foc_math、current_controller、svpwm。
 * - 不包含 stm32f4xx_hal.h，不调用 hal_pwm/hal_adc/hal_spi/hal_gpio。
 * - 可被普通 gcc 编译，也可被 Simulink C Caller 直接调用。
 * - 所有物理量使用 SI 单位：
 *   电流 A，电压 V，角度 rad，速度 rad/s，时间 s。
 *
 * 调用频率：
 * - 建议与电流环一致，例如 20 kHz。
 * - 每次调用 foc_sim_step() 等价于真实固件里一次 FOC 电流环计算。
 */

#include <stdint.h>

#include "control/current_controller.h"
#include "control/velocity_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 仿真输出量。
 *
 * 该结构体用于 context 版本接口；Simulink C Caller 若不方便处理结构体，
 * 可以直接调用下方标量指针版本 foc_sim_step()。
 */
typedef struct {
    float id_a;        /* d 轴实测电流，A */
    float iq_a;        /* q 轴实测电流，A */
    float vd_v;        /* d 轴电压指令，V */
    float vq_v;        /* q 轴电压指令，V */
    float v_alpha_v;   /* alpha 轴电压指令，V */
    float v_beta_v;    /* beta 轴电压指令，V */
    float duty_u;      /* U/A 相 PWM duty，0.0..1.0 */
    float duty_v;      /* V/B 相 PWM duty，0.0..1.0 */
    float duty_w;      /* W/C 相 PWM duty，0.0..1.0 */
} FocSimOutput;

/*
 * 外部传入的仿真 context。
 *
 * 如果只仿真单个电机，可以直接用 foc_sim_init()/foc_sim_step() 的静态 context。
 * 如果需要在同一个进程里仿真多台电机或多组参数，请使用 context 版本接口。
 */
typedef struct {
    float current_kp;              /* 电流环比例增益，V/A */
    float current_ki;              /* 电流环积分增益，V/(A*s) */
    float max_voltage_v;           /* 电压矢量限幅，V */
    float last_mechanical_angle_rad;/* 最近一次机械角，rad，便于外部调试 */
    float last_mechanical_velocity_rad_s; /* 最近一次机械速度，rad/s */
    uint8_t initialized;           /* 1 表示 context 已初始化 */

    /*
     * CurrentController 是纯算法结构体，不含硬件句柄。
     * 它保存 d/q 电流 PI 的积分器状态，因此必须跨步保存。
     */
    CurrentController current_controller;
} FocSimContext;

/*
 * 速度环仿真 context。
 *
 * 该 context 与电流环 FOC context 分开保存，便于 Simulink 单独验证速度 PI。
 */
typedef struct {
    VelocityController velocity_controller;
    float last_velocity_target_rad_s;   /* 最近一次速度目标，rad/s */
    float last_velocity_measured_rad_s; /* 最近一次实测速度，rad/s */
    uint8_t initialized;                /* 1 表示速度环 context 已初始化 */
} FocSimVelocityContext;

/*
 * 初始化外部 context。
 *
 * 参数：
 * - current_kp: 电流环 Kp，V/A。
 * - current_ki: 电流环 Ki，V/(A*s)。
 * - max_voltage_v: 仿真电压矢量限幅，V；通常先设为低于 vbus/sqrt(3) 的安全值。
 */
void foc_sim_context_init(FocSimContext *ctx,
                          float current_kp,
                          float current_ki,
                          float max_voltage_v);

/* 清零 PI 积分器，适合切换工况、复位仿真或故障恢复后调用。 */
void foc_sim_context_reset(FocSimContext *ctx);

/*
 * 使用外部 context 执行一次 FOC 仿真步。
 *
 * 返回值：
 * - 0：计算成功；
 * - -1：ctx 或 output 为空；
 * - -2：dt/vbus/pole_pairs 等输入无效，输出会被置为安全零电压/50% duty。
 */
int foc_sim_context_step(FocSimContext *ctx,
                         float ia_a,
                         float ib_a,
                         float ic_a,
                         float mechanical_angle_rad,
                         float mechanical_velocity_rad_s,
                         float vbus_v,
                         float id_target_a,
                         float iq_target_a,
                         float dt_s,
                         uint8_t pole_pairs,
                         float encoder_offset_rad,
                         FocSimOutput *output);

/*
 * Simulink C Caller 友好的静态 context 接口。
 *
 * foc_sim_init() 使用保守默认值：
 * - Kp = 0.05 V/A
 * - Ki = 100.0 V/(A*s)
 * - max_voltage = 3.0 V
 *
 * 如需覆盖默认 PI 参数，可在 foc_sim_init() 后调用 foc_sim_set_current_controller()。
 */
void foc_sim_init(void);

void foc_sim_set_current_controller(float current_kp,
                                    float current_ki,
                                    float max_voltage_v);

void foc_sim_reset(void);

/*
 * 静态 context 版本的一步 FOC 计算。
 *
 * 该函数刻意使用“标量输入 + 输出指针”，避免 Simulink C Caller 对结构体解析的额外配置。
 * 输出指针允许传 NULL；传 NULL 的输出项会被跳过。
 */
int foc_sim_step(float ia_a,
                 float ib_a,
                 float ic_a,
                 float mechanical_angle_rad,
                 float mechanical_velocity_rad_s,
                 float vbus_v,
                 float id_target_a,
                 float iq_target_a,
                 float dt_s,
                 uint8_t pole_pairs,
                 float encoder_offset_rad,
                 float *id_a,
                 float *iq_a,
                 float *vd_v,
                 float *vq_v,
                 float *v_alpha_v,
                 float *v_beta_v,
                 float *duty_u,
                 float *duty_v,
                 float *duty_w);

/*
 * Simulink C Caller 推荐入口。
 *
 * 与 foc_sim_step() 的区别：
 * - 不需要外部调用 foc_sim_init()，wrapper 内部会自动初始化静态 context。
 * - 输入/输出使用 double，匹配 Simulink 默认信号类型。
 * - pole_pairs 使用 double 输入，避免 C Caller 侧处理 uint8。
 *
 * pole_pairs_double 会在内部检查：
 * - 必须有限；
 * - 必须 >= 1；
 * - 必须 <= 255；
 * - 会四舍五入到最近的整数极对数。
 */
int foc_sim_step_wrapper(double ia_a,
                         double ib_a,
                         double ic_a,
                         double mechanical_angle_rad,
                         double mechanical_velocity_rad_s,
                         double vbus_v,
                         double id_target_a,
                         double iq_target_a,
                         double dt_s,
                         double pole_pairs_double,
                         double encoder_offset_rad,
                         double *id_a,
                         double *iq_a,
                         double *vd_v,
                         double *vq_v,
                         double *v_alpha_v,
                         double *v_beta_v,
                         double *duty_u,
                         double *duty_v,
                         double *duty_w);

/*
 * Simulink C Caller 速度环仿真入口。
 *
 * 输入/输出全部使用 double，内部调用 velocity_controller_update()。
 * reset_integrator 非 0 时会重置速度环积分器，适合仿真开始或切换工况时使用。
 */
int foc_sim_velocity_step_wrapper(double velocity_target_rad_s,
                                  double velocity_measured_rad_s,
                                  double dt_s,
                                  double velocity_kp,
                                  double velocity_ki,
                                  double current_limit_a,
                                  double velocity_limit_rad_s,
                                  double reset_integrator,
                                  double *iq_target_a);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SIM_H */
