#ifndef FOC_MATH_H
#define FOC_MATH_H

/*
 * foc_math.h
 *
 * FOC 数学基础模块。
 * 运行上下文：可在 20 kHz PWM ISR 中调用。
 * 约束：
 * - 不使用动态内存。
 * - 不访问硬件外设。
 * - 不阻塞，不打印。
 * - 所有角度使用 rad，电压使用 V，电流使用 A。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TWO_PI_F 6.2831853071795864769f

typedef struct {
    float alpha; /* alpha 轴分量，单位由调用者决定 */
    float beta;  /* beta 轴分量，单位由调用者决定 */
} AlphaBeta;

typedef struct {
    float d; /* d 轴分量，单位由调用者决定 */
    float q; /* q 轴分量，单位由调用者决定 */
} Dq;

typedef struct {
    float u; /* U 相 PWM 占空比，0..1 */
    float v; /* V 相 PWM 占空比，0..1 */
    float w; /* W 相 PWM 占空比，0..1 */
} PhaseDuty;

/* 将任意角度归一化到 [0, 2*pi)，输入/输出单位 rad。 */
float foc_wrap_0_2pi(float angle_rad);

/* 将角度归一化到 [-pi, pi)，常用于位置误差计算。 */
float foc_wrap_minuspi_pi(float angle_rad);

/* 由机械角、极对数和零偏计算电角度，输出归一化到 [0, 2*pi)。 */
float foc_electrical_angle(float mechanical_angle_rad, uint8_t pole_pairs, float offset_rad);

/* ODrive v3.6 Axis0 用：加入编码器方向符号后的电角度计算。 */
float foc_electrical_angle_dir(float mechanical_angle_rad,
                               uint8_t pole_pairs,
                               int direction,
                               float encoder_offset_rad);

/* 兼容用户需求命名：normalize_0_2pi 等价于 foc_wrap_0_2pi。 */
float normalize_0_2pi(float angle_rad);

/* 兼容用户需求命名：normalize_minuspi_pi 等价于 foc_wrap_minuspi_pi。 */
float normalize_minuspi_pi(float angle_rad);

/* Clarke 变换：三相静止坐标系电流 -> alpha/beta 静止坐标系电流。 */
void foc_clarke(float ia_a, float ib_a, float ic_a, float *i_alpha_a, float *i_beta_a);

/* Park 变换：alpha/beta 电流 -> d/q 旋转坐标系电流。 */
void foc_park(float i_alpha_a, float i_beta_a, float electrical_angle_rad, float *id_a, float *iq_a);

/* 反 Park 变换：d/q 电压指令 -> alpha/beta 电压指令。 */
void foc_inv_park(float vd_v, float vq_v, float electrical_angle_rad, float *v_alpha_v, float *v_beta_v);

/* 对 vd/vq 做圆形电压矢量限幅，保持方向，限制幅值。 */
void foc_limit_voltage(float *vd_v, float *vq_v, float max_voltage_v);

/* SVPWM/零序注入：alpha/beta 电压和母线电压 -> 三相 duty，范围 0..1。 */
void foc_svpwm(float v_alpha_v, float v_beta_v, float vbus_v, float *duty_u, float *duty_v, float *duty_w);

/* 一阶低通滤波：alpha 为 0..1，越大越接近当前输入。 */
float foc_lpf(float previous, float input, float alpha);

/* 通用限幅工具，ISR 可用。 */
float foc_clamp(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
